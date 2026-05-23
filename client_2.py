"""
client_3.py - Cliente estereo con rectificacion + RAFT-Stereo en GPU
=====================================================================
Flujo por frame:
  raw frame  ->  split izq/der  ->  remap (rectificacion)
             ->  RAFT-Stereo GPU  ->  mapa de disparidad
             ->  mostrar + guardar (frames rectificados + disparidad)

Controles: W/A/S/D = mover carrito | Q = frenar | X = salir
"""

import socket
import cv2
import struct
import threading
import sys
import os
import time
import csv
import json
import queue
from pathlib import Path
from datetime import datetime

import numpy as np
import torch
from visual_odometry import VisualOdometry  # clase ORB-SLAM2-like para comparar con RAFT-Stereo

if os.name == 'nt':
    import msvcrt
else:
    import select
    import tty
    import termios

# ==================================================================
#  CONFIGURACION - ajusta paths si cambias de maquina
# ==================================================================
PI_IP          = '10.42.0.146'
CAL_JSON       = Path('/home/alex/Escritorio/6tosemestre/Tridimensional/carrito/parac/estereo_cal.json')
RAFT_DIR       = Path('/home/alex/Escritorio/6tosemestre/Tridimensional/carrito/corrercudapytorch/RAFT-Stereo')
RAFT_MODEL     = RAFT_DIR / 'models/raftstereo-middlebury.pth'
SAVE_ROOT      = Path('captured_frames')
SAVE_FRAMES    = True
MAX_FRAME_SIZE = 50 * 1024 * 1024
INFER_ITERS    = 20   # iteraciones RAFT (mas = mas calidad, mas lento)

# -- RAFT-Stereo path setup ----------------------------------------
sys.path.insert(0, str(RAFT_DIR))
sys.path.insert(0, str(RAFT_DIR / 'core'))
from raft_stereo import RAFTStereo

# -- Estado global -------------------------------------------------
stop_event     = threading.Event()
command_socket = None
command_lock   = threading.Lock()
infer_queue    = queue.Queue(maxsize=2)        # cola entre video e inferencia
latest_disp    = {'img': None, 'lock': threading.Lock()}


# ==================================================================
#  UTILIDADES DE RED
# ==================================================================

def recv_exact(sock, size):
    """Lee exactamente `size` bytes del socket."""
    data = b''
    while len(data) < size and not stop_event.is_set():
        try:
            packet = sock.recv(size - len(data))
        except Exception:
            return None
        if not packet:
            return None
        data += packet
    return data if len(data) == size else None


def read_one_frame(sock):
    """
    Lee un frame completo del socket de video.
    Retorna (frame_bgr, frame_bytes) o (None, None) si hay error.
    """
    payload_size = struct.calcsize('<L')
    packed = recv_exact(sock, payload_size)
    if packed is None:
        return None, None

    msg_size = struct.unpack('<L', packed)[0]
    if msg_size <= 0 or msg_size > MAX_FRAME_SIZE:
        return None, None

    frame_data = recv_exact(sock, msg_size)
    if frame_data is None:
        return None, None

    frame_np = np.frombuffer(frame_data, dtype=np.uint8)
    frame    = cv2.imdecode(frame_np, cv2.IMREAD_COLOR)
    return frame, frame_data


# ==================================================================
#  CALIBRACION Y RECTIFICACION
# ==================================================================

def load_rectification(cal_path, lens_w, lens_h):
    """
    Carga el JSON de calibracion y calcula los mapas de rectificacion
    escalados a la resolucion real del stream.

    El JSON fue generado a 1280x720 por lente.
    Si el server transmite a menor resolucion, se escalan las matrices K.

    Retorna:
      maps_izq  - (map1, map2) para cv2.remap del lente izquierdo
      maps_der  - (map1, map2) para cv2.remap del lente derecho
      Q         - matriz de reproyeccion disparidad->3D (para point cloud)
    """
    with open(cal_path) as f:
        cal = json.load(f)

    # Resolucion con la que se calibro (por lente)
    cal_w, cal_h = cal['lente_izquierdo']['resolucion']   # [1280, 720]
    sx = lens_w / cal_w
    sy = lens_h / cal_h

    def scale_K(K_list):
        K = np.array(K_list, dtype=np.float64)
        K[0, 0] *= sx   # fx
        K[1, 1] *= sy   # fy
        K[0, 2] *= sx   # cx
        K[1, 2] *= sy   # cy
        return K

    K_izq  = scale_K(cal['lente_izquierdo']['matriz_camara'])
    D_izq  = np.array(cal['lente_izquierdo']['coef_distorsion'], dtype=np.float64)
    K_der  = scale_K(cal['lente_derecho']['matriz_camara'])
    D_der  = np.array(cal['lente_derecho']['coef_distorsion'], dtype=np.float64)
    R      = np.array(cal['estereo']['R'], dtype=np.float64)
    T      = np.array(cal['estereo']['T'], dtype=np.float64)
    # T esta en mm (espacio 3D real), no se escala

    size = (lens_w, lens_h)
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        K_izq, D_izq, K_der, D_der, size, R, T, alpha=0
    )

    maps_izq = cv2.initUndistortRectifyMap(K_izq, D_izq, R1, P1, size, cv2.CV_32FC1)
    maps_der = cv2.initUndistortRectifyMap(K_der, D_der, R2, P2, size, cv2.CV_32FC1)

    print(f"  Resolucion por lente: {lens_w}x{lens_h}")
    print(f"  Escala vs calibracion: sx={sx:.3f}  sy={sy:.3f}")
    if sx < 0.5:
        print("  AVISO  Resolucion baja vs calibracion - sube resolucion en server_2.py")
        print("     para mejor calidad de disparidad.")

    # P1[:3,:3] es la K exacta del lente izquierdo rectificado
    K_rect = P1[:3, :3].copy()
    return maps_izq, maps_der, Q, K_rect


# ==================================================================
#  RAFT-Stereo
# ==================================================================

def build_model_args():
    import argparse
    return argparse.Namespace(
        corr_implementation='reg',
        corr_levels=4,
        corr_radius=4,
        dropout=0.0,
        mixed_precision=False,
        n_downsample=2,
        n_gru_layers=3,
        shared_backbone=False,
        slow_fast_gru=False,
        valid_iters=INFER_ITERS,
        hidden_dims=[128, 128, 128],
        context_norm='batch',
    )


def load_raft_model():
    """Carga RAFT-Stereo en GPU y retorna (model, args)."""
    args  = build_model_args()
    model = RAFTStereo(args)

    state = torch.load(str(RAFT_MODEL), map_location='cpu')
    # Quitar prefijo 'module.' si el checkpoint viene de DataParallel
    clean_state = {k.replace('module.', ''): v for k, v in state.items()}
    model.load_state_dict(clean_state)

    model = model.cuda().eval()
    print(f"  GPU: {torch.cuda.get_device_name(0)}")
    return model, args


def pad32(img):
    """Pad imagen para que H y W sean multiplos de 32 (requerido por RAFT)."""
    h, w   = img.shape[:2]
    pad_h  = (32 - h % 32) % 32
    pad_w  = (32 - w % 32) % 32
    padded = cv2.copyMakeBorder(img, 0, pad_h, 0, pad_w, cv2.BORDER_REFLECT)
    return padded, h, w


def to_tensor(img_bgr):
    """Convierte imagen BGR numpy -> tensor CUDA (1, 3, H, W) float."""
    rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    t   = torch.from_numpy(rgb).permute(2, 0, 1).float().unsqueeze(0)
    return t.cuda()


@torch.no_grad()
def run_raft(model, args, img_izq, img_der):
    """
    Calcula mapa de disparidad con RAFT-Stereo.
    Retorna array numpy (H, W) con valores de disparidad en pixeles (>= 0).
    """
    orig_h, orig_w = img_izq.shape[:2]

    izq_pad, h, w = pad32(img_izq)
    der_pad, _, _ = pad32(img_der)

    t_izq = to_tensor(izq_pad)
    t_der = to_tensor(der_pad)

    _, flow = model(t_izq, t_der, iters=args.valid_iters, test_mode=True)

    # RAFT devuelve flujo izq->der (negativo), lo invertimos a disparidad positiva
    disp = -flow.squeeze().cpu().numpy()
    disp = np.clip(disp[:orig_h, :orig_w], 0, None)
    return disp


def disp_to_color(disp):
    """Convierte mapa de disparidad a imagen BGR coloreada (TURBO colormap)."""
    if disp.max() > 0:
        norm = (disp / disp.max() * 255).astype(np.uint8)
    else:
        norm = np.zeros(disp.shape, dtype=np.uint8)
    return cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)


# ==================================================================
#  SESION DE CAPTURA
# ==================================================================

def create_session():
    ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
    sd   = SAVE_ROOT / f'session_{ts}'
    dirs = {k: sd / k for k in ('full', 'left_rect', 'right_rect', 'disparity')}
    for d in dirs.values():
        d.mkdir(parents=True, exist_ok=True)

    csv_file = open(sd / 'frames.csv', 'w', newline='', encoding='utf-8')
    writer   = csv.writer(csv_file)
    writer.writerow([
        'frame_index', 'timestamp_unix', 'timestamp_iso',
        'width', 'height',
        'full_path', 'left_rect_path', 'right_rect_path', 'disparity_path',
    ])

    print(f"  Sesion: {sd.resolve()}")
    return {'sd': sd, 'dirs': dirs, 'csv': csv_file, 'writer': writer}


def close_session(sess):
    if sess is None:
        return
    try:
        sess['csv'].flush()
        sess['csv'].close()
    except Exception:
        pass


def save_frame(sess, idx, raw_bytes, frame, rect_izq, rect_der, disp_color):
    if sess is None:
        return

    now = time.time()
    ts  = datetime.fromtimestamp(now).isoformat(timespec='milliseconds')
    h, w = frame.shape[:2]
    d    = sess['dirs']

    p_full = d['full']       / f'frame_{idx:06d}.jpg'
    p_izq  = d['left_rect']  / f'left_{idx:06d}.jpg'
    p_der  = d['right_rect'] / f'right_{idx:06d}.jpg'
    p_disp = d['disparity']  / f'disp_{idx:06d}.jpg'

    with open(p_full, 'wb') as f:
        f.write(raw_bytes)
    cv2.imwrite(str(p_izq),  rect_izq)
    cv2.imwrite(str(p_der),  rect_der)
    cv2.imwrite(str(p_disp), disp_color)

    sess['writer'].writerow([
        idx, f'{now:.6f}', ts, w, h,
        str(p_full), str(p_izq), str(p_der), str(p_disp),
    ])
    if idx % 30 == 0:
        sess['csv'].flush()


# ==================================================================
#  HILO DE INFERENCIA RAFT-Stereo
# ==================================================================

def inference_worker(model, args, sess, vo):
    """
    Corre en hilo separado.
    Consume frames de infer_queue, ejecuta RAFT-Stereo y actualiza latest_disp.
    El hilo de video no se bloquea esperando la GPU.
    """
    while not stop_event.is_set():
        try:
            item = infer_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        idx, raw_bytes, frame, rect_izq, rect_der = item
        try:
            # -- Disparidad RAFT-Stereo --------------------------------
            disp       = run_raft(model, args, rect_izq, rect_der)
            disp_color = disp_to_color(disp)

            with latest_disp['lock']:
                latest_disp['img'] = disp_color.copy()

            # -- Odometría Visual (Fase 3) -----------------------------
            result = vo.update(rect_izq, disp)

            if result is not None:
                R, t = result
                # t está en las mismas unidades que T de calibración (mm)
                print(f"  [VO] frame {idx:05d} | "
                      f"dx={t[0]:+7.1f}mm  dy={t[1]:+7.1f}mm  dz={t[2]:+7.1f}mm")

            if SAVE_FRAMES:
                save_frame(sess, idx, raw_bytes, frame,
                           rect_izq, rect_der, disp_color)

        except Exception as e:
            print(f"  AVISO Error inferencia frame {idx}: {e}")

        infer_queue.task_done()


# ==================================================================
#  COMANDOS (WASD)
# ==================================================================

def send_car_command(cmd):
    with command_lock:
        if command_socket is None:
            return
        try:
            command_socket.sendall(cmd.encode())
        except Exception:
            pass


def request_shutdown():
    already = stop_event.is_set()
    stop_event.set()
    if not already:
        send_car_command('q')
        send_car_command('x')


def handle_commands(cmd_sock):
    """Hilo que lee teclado y manda comandos al carrito."""
    global command_socket
    old_settings = None
    fd = None

    try:
        with command_lock:
            command_socket = cmd_sock

        print("    W/A/S/D = mover | Q = frenar | X = salir")

        if os.name != 'nt':
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            tty.setraw(fd)

        while not stop_event.is_set():
            if os.name == 'nt':
                if msvcrt.kbhit():
                    key = msvcrt.getch()
                    if key in {b'\x00', b'\xe0'}:
                        if msvcrt.kbhit():
                            msvcrt.getch()
                        continue
                    key = key.decode(errors='ignore').lower()
                else:
                    time.sleep(0.02)
                    continue
            else:
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not ready:
                    continue
                key = sys.stdin.read(1).lower()

            if key in {'w', 'a', 's', 'd', 'q', 'x'}:
                send_car_command(key)
            if key == 'x':
                request_shutdown()
                break

    except Exception as e:
        print(f"  Error comandos: {e}")
        request_shutdown()
    finally:
        if old_settings is not None and fd is not None:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            except Exception:
                pass
        with command_lock:
            command_socket = None


# ==================================================================
#  LOOP DE VIDEO
# ==================================================================

def video_loop(vid_sock, maps_izq, maps_der, sess, first_frame=None):
    """
    Recibe frames del servidor, aplica rectificacion y los encola
    para que el hilo de inferencia calcule la disparidad.
    Muestra en tiempo real: lente izq rectificado, lente der rectificado
    y el ultimo mapa de disparidad disponible.
    """
    map1_izq, map2_izq = maps_izq
    map1_der, map2_der = maps_der
    frame_index = 0

    def process(frame, frame_data):
        nonlocal frame_index
        h, w = frame.shape[:2]

        raw_izq = frame[:, :w // 2]
        raw_der = frame[:, w // 2:]

        rect_izq = cv2.remap(raw_izq, map1_izq, map2_izq, cv2.INTER_LINEAR)
        rect_der = cv2.remap(raw_der, map1_der, map2_der, cv2.INTER_LINEAR)

        # Mostrar frames rectificados
        cv2.imshow('Izquierdo rectificado', rect_izq)
        cv2.imshow('Derecho rectificado',   rect_der)

        # Mostrar ultimo mapa de disparidad disponible (no bloquea)
        with latest_disp['lock']:
            if latest_disp['img'] is not None:
                cv2.imshow('Disparidad RAFT-Stereo', latest_disp['img'])

        # Encolar para inferencia - si la GPU no alcanza, descartamos el frame
        try:
            infer_queue.put_nowait(
                (frame_index, frame_data, frame, rect_izq, rect_der)
            )
        except queue.Full:
            pass  # GPU ocupada, frame descartado, continuamos sin bloquear

        frame_index += 1

    try:
        # El primer frame ya fue leido en main() para detectar resolucion
        if first_frame is not None:
            frame, frame_data = first_frame
            if frame is not None:
                process(frame, frame_data)

        while not stop_event.is_set():
            frame, frame_data = read_one_frame(vid_sock)
            if frame is None:
                print("  Conexion de video cerrada")
                break

            process(frame, frame_data)

            if cv2.waitKey(1) & 0xFF == ord('x'):
                request_shutdown()
                break

    except Exception as e:
        print(f"  Error en video: {e}")
        request_shutdown()
    finally:
        request_shutdown()
        cv2.destroyAllWindows()
        print(f"  Frames recibidos: {frame_index}")


# ==================================================================
#  MAIN
# ==================================================================

def main():
    print("=" * 58)
    print("  client_3.py - Stereo SLAM con RAFT-Stereo + GPU")
    print("=" * 58)

    # -- 1. Cargar RAFT-Stereo en GPU -----------------------------
    print("\n[1/4] Cargando RAFT-Stereo en GPU...")
    model, args = load_raft_model()

    # -- 2. Conectar al servidor ----------------------------------
    # El servidor acepta: primero comandos (9999), luego video (9998)
    # El cliente debe conectar en ese mismo orden
    print(f"\n[2/4] Conectando a Pi ({PI_IP})...")
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cmd_sock.connect((PI_IP, 9999))
    print("  OK Comandos conectados (puerto 9999)")

    vid_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    vid_sock.connect((PI_IP, 9998))
    print("  OK Video conectado (puerto 9998)")

    # -- 3. Leer primer frame para detectar resolucion real -------
    print("\n[3/4] Detectando resolucion del stream...")
    first_frame, first_data = read_one_frame(vid_sock)
    if first_frame is None:
        print("  ERROR No se pudo leer el primer frame. Verifica server_2.py.")
        cmd_sock.close()
        vid_sock.close()
        return

    h, w       = first_frame.shape[:2]
    lens_w     = w // 2
    lens_h     = h

    # -- 4. Calcular mapas de rectificacion -----------------------
    print("\n[4/4] Calculando mapas de rectificacion estereo...")
    maps_izq, maps_der, Q , K_rect = load_rectification(CAL_JSON, lens_w, lens_h)
    print("  OK Mapas listos")

    # -- Instanciar odometría visual ----------------------------------
    vo = VisualOdometry(Q, K_rect)
    print("  OK VisualOdometry lista")

    # -- Crear sesion de captura -----------------------------------
    print("\nCreando sesion de captura...")
    sess = create_session() if SAVE_FRAMES else None

    # -- Hilo de inferencia RAFT-Stereo ----------------------------
    t_infer = threading.Thread(
        target=inference_worker,
        args=(model, args, sess, vo),
        daemon=True,
        name='RAFT-Inference',
    )
    t_infer.start()

    # -- Hilo de comandos ------------------------------------------
    t_cmd = threading.Thread(
        target=handle_commands,
        args=(cmd_sock,),
        daemon=True,
        name='Commands',
    )
    t_cmd.start()

    print("\n> Stream iniciado.\n")

    # -- Loop de video en hilo principal --------------------------
    try:
        video_loop(
            vid_sock, maps_izq, maps_der, sess,
            first_frame=(first_frame, first_data),
        )
    finally:
        stop_event.set()
        t_infer.join(timeout=5)
        close_session(sess)
        try:
            cmd_sock.close()
        except Exception:
            pass
        try:
            vid_sock.close()
        except Exception:
            pass
        print("\nOK Sesion cerrada correctamente.")


if __name__ == '__main__':
    main()
