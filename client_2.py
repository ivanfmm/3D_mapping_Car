"""
client_2.py - Cliente estereo con SLAM offline
===============================================
Flujo en dos fases:

FASE 1 - Captura (en vivo):
  video_loop (hilo) -> rectifica -> save_queue -> save_worker (disco)
  hilo principal    -> GUI loop (imshow / waitKey)
  hilo comandos     -> WASD -> carrito

FASE 2 - SLAM offline (al cerrar):
  lector_disco -> raft_queue -> RAFT GPU -> vo_queue -> VO CPU
  build_point_cloud (nube.py) -> mapa_final.ply + mapa_paredes.ply

Controles: W/A/S/D = mantener presionado para mover | Q = frenar | X = salir y procesar
(funcionan con el foco en la terminal o en las ventanas de vista previa)
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
import shutil
from pathlib import Path
from datetime import datetime

import numpy as np
import torch
from visual_odometry import VisualOdometry
from rectificacion import load_rectification
from nube import build_point_cloud, SAVE_ROOT

if os.name == 'nt':
    import msvcrt
else:
    import select
    import tty
    import termios

# ==================================================================
#  CONFIGURACION
# ==================================================================
BASE_DIR       = Path(__file__).resolve().parent
PI_IP          = os.environ.get('PI_IP', '10.42.0.146')
CAL_JSON       = Path(os.environ.get('CAL_JSON', BASE_DIR / 'Calibracion_Camara' / 'estereo_cal.json'))
DEFAULT_RAFT_DIR = BASE_DIR / 'RAFT-Stereo'
LEGACY_RAFT_DIR  = Path('/home/alex/Escritorio/6tosemestre/Tridimensional/carrito/corrercudapytorch/RAFT-Stereo')
RAFT_DIR       = Path(os.environ.get(
    'RAFT_DIR',
    DEFAULT_RAFT_DIR if DEFAULT_RAFT_DIR.exists() else LEGACY_RAFT_DIR
))
RAFT_MODEL     = Path(os.environ.get('RAFT_MODEL', RAFT_DIR / 'models/raftstereo-middlebury.pth'))
SAVE_FRAMES    = True
MAX_FRAME_SIZE = 50 * 1024 * 1024
INFER_ITERS    = 20
PREVIEW_SCALE  = 0.5
HEARTBEAT_S    = 0.25   # cada cuanto se manda latido al watchdog del Pi
# Angulo de la camara respecto al piso (grados, + = hacia arriba).
# Se usa para forzar movimiento plano. None = intentar detectar el piso.
INCLINACION_CAMARA = None

# -- RAFT-Stereo path setup ----------------------------------------
sys.path.insert(0, str(RAFT_DIR))
sys.path.insert(0, str(RAFT_DIR / 'core'))
try:
    from raft_stereo import RAFTStereo
except Exception as e:
    raise RuntimeError(
        f"No se pudo importar RAFT-Stereo desde {RAFT_DIR}. "
        "Configura la variable de entorno RAFT_DIR apuntando a la carpeta RAFT-Stereo."
    ) from e

# -- Estado global -------------------------------------------------
stop_event     = threading.Event()
command_socket = None
command_lock   = threading.Lock()
save_queue     = queue.Queue(maxsize=30)
latest_preview = {'izq': None, 'der': None, 'lock': threading.Lock()}


# ==================================================================
#  UTILIDADES DE RED
# ==================================================================

def recv_exact(sock, size):
    buf  = bytearray(size)
    view = memoryview(buf)
    pos  = 0
    while pos < size and not stop_event.is_set():
        try:
            n = sock.recv_into(view[pos:], size - pos)
        except Exception:
            return None
        if not n:
            return None
        pos += n
    return buf if pos == size else None


def read_one_frame(sock):
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
    if frame is None:
        print("  ERROR Frame JPEG corrupto o no decodificable")
        return None, None
    return frame, frame_data



# ==================================================================
#  RAFT-Stereo
# ==================================================================

def build_model_args():
    import argparse
    return argparse.Namespace(
        corr_implementation='alt',
        corr_levels=4,
        corr_radius=4,
        dropout=0.0,
        mixed_precision=True,
        n_downsample=2,
        n_gru_layers=3,
        shared_backbone=False,
        slow_fast_gru=False,
        valid_iters=INFER_ITERS,
        hidden_dims=[128, 128, 128],
        context_norm='batch',
    )


def load_raft_model():
    args  = build_model_args()
    model = RAFTStereo(args)
    state = torch.load(str(RAFT_MODEL), map_location='cpu')
    clean_state = {k.replace('module.', ''): v for k, v in state.items()}
    model.load_state_dict(clean_state)
    model = model.cuda().eval()
    print(f"  GPU: {torch.cuda.get_device_name(0)}")
    return model, args


def pad32(img):
    h, w   = img.shape[:2]
    pad_h  = (32 - h % 32) % 32
    pad_w  = (32 - w % 32) % 32
    return cv2.copyMakeBorder(img, 0, pad_h, 0, pad_w, cv2.BORDER_REFLECT), h, w


def to_tensor(img_bgr):
    rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    return torch.from_numpy(rgb).permute(2, 0, 1).float().unsqueeze(0).cuda()


@torch.no_grad()
def run_raft(model, args, img_izq, img_der):
    orig_h, orig_w = img_izq.shape[:2]
    izq_pad, _, _  = pad32(img_izq)
    der_pad, _, _  = pad32(img_der)

    _, flow = model(to_tensor(izq_pad), to_tensor(der_pad),
                    iters=args.valid_iters, test_mode=True)

    flow_sq = flow.squeeze()
    if flow_sq.ndim == 1:
        flow_sq = flow_sq.reshape(orig_h, orig_w)
    elif flow_sq.ndim > 2:
        flow_sq = flow_sq[0]

    disp = (-flow_sq[:orig_h, :orig_w]).clamp(min=0).cpu().numpy()

    if disp.ndim != 2:
        raise RuntimeError(f"run_raft: disp shape inesperado {disp.shape}")
    return disp


def disp_to_color(disp):
    if disp.max() > 0:
        norm = (disp / disp.max() * 255).astype(np.uint8)
    else:
        norm = np.zeros(disp.shape, dtype=np.uint8)
    return cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)


# ==================================================================
#  SESION DE CAPTURA
# ==================================================================

def create_session():
    ts  = datetime.now().strftime('%Y%m%d_%H%M%S')
    sd  = SAVE_ROOT / f'session_{ts}'
    dirs = {k: sd / k for k in ('full', 'left_rect', 'right_rect')}
    for d in dirs.values():
        d.mkdir(parents=True, exist_ok=True)

    csv_file = open(sd / 'frames.csv', 'w', newline='', encoding='utf-8')
    writer   = csv.writer(csv_file)
    writer.writerow([
        'frame_index', 'timestamp_unix', 'timestamp_iso',
        'width', 'height',
        'full_path', 'left_rect_path', 'right_rect_path',
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


# ==================================================================
#  HILO DE GUARDADO EN DISCO
# ==================================================================

def save_worker(sess):
    while True:
        item = save_queue.get()
        try:
            if item is None:
                return
            try:
                _save_frame(sess, *item)
            except Exception as e:
                frame_id = item[0] if isinstance(item, tuple) and item else '?'
                print(f"  AVISO Error guardado frame {frame_id}: {e}")
        finally:
            save_queue.task_done()


def _save_frame(sess, idx, raw_bytes, frame, rect_izq, rect_der):
    if sess is None:
        return

    now = time.time()
    ts  = datetime.fromtimestamp(now).isoformat(timespec='milliseconds')
    h, w = frame.shape[:2]
    d    = sess['dirs']

    p_full = d['full']       / f'frame_{idx:06d}.jpg'
    p_izq  = d['left_rect']  / f'left_{idx:06d}.jpg'
    p_der  = d['right_rect'] / f'right_{idx:06d}.jpg'

    with open(p_full, 'wb') as f:
        f.write(raw_bytes)
    cv2.imwrite(str(p_izq),  rect_izq)
    cv2.imwrite(str(p_der),  rect_der)

    sess['writer'].writerow([
        idx, f'{now:.6f}', ts, w, h,
        str(p_full), str(p_izq), str(p_der),
    ])
    if idx % 30 == 0:
        sess['csv'].flush()


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
    global command_socket
    old_settings = None
    fd = None

    try:
        with command_lock:
            command_socket = cmd_sock

        print("    W/A/S/D = mantener presionado para mover | Q = frenar | X = salir y procesar")

        if os.name != 'nt':
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            tty.setraw(fd)

        last_hb = 0.0
        while not stop_event.is_set():
            # latido: si esta PC se congela, el Pi deja de recibirlo y frena
            if time.monotonic() - last_hb > HEARTBEAT_S:
                send_car_command('h')
                last_hb = time.monotonic()

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
    map1_izq, map2_izq = maps_izq
    map1_der, map2_der = maps_der
    frame_index = 0

    def process(frame, frame_data):
        nonlocal frame_index
        h, w = frame.shape[:2]
        if w % 2 != 0:
            raise RuntimeError(f"Ancho de frame impar ({w}); no se puede dividir en estereo.")

        raw_izq = frame[:, :w // 2]
        raw_der = frame[:, w // 2:]

        if raw_izq.shape[:2] != map1_izq.shape[:2] or raw_der.shape[:2] != map1_der.shape[:2]:
            raise RuntimeError(
                f"Tamano de lente cambio durante ejecucion: "
                f"izq={raw_izq.shape[:2]}, der={raw_der.shape[:2]}, "
                f"mapas={map1_izq.shape[:2]}"
            )

        rect_izq = cv2.remap(raw_izq, map1_izq, map2_izq, cv2.INTER_LINEAR)
        rect_der = cv2.remap(raw_der, map1_der, map2_der, cv2.INTER_LINEAR)

        with latest_preview['lock']:
            latest_preview['izq'] = rect_izq.copy()
            latest_preview['der'] = rect_der.copy()

        if SAVE_FRAMES:
            item = (frame_index, bytes(frame_data), frame, rect_izq, rect_der)
            while not stop_event.is_set():
                try:
                    save_queue.put(item, timeout=0.5)
                    break
                except queue.Full:
                    print("  AVISO Guardado atrasado: esperando a que se vacie la cola...")

        frame_index += 1

    try:
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

    except Exception as e:
        print(f"  Error en video: {e}")
        request_shutdown()
    finally:
        request_shutdown()
        print(f"  Frames recibidos: {frame_index}")


# ==================================================================
#  SLAM OFFLINE (Fase 2)
# ==================================================================

def slam_offline(sess, model, args, Q, K_rect):
    if sess is None or not SAVE_FRAMES:
        print("  [SLAM] Sin sesion de captura.")
        return None

    sd       = sess['sd']
    left_dir = sd / 'left_rect'
    rder_dir = sd / 'right_rect'

    left_files = sorted(left_dir.glob('left_*.jpg'))

    if not left_files:
        print("  [SLAM] No se encontraron frames en disco.")
        print(f"         Buscando en: {left_dir.resolve()}")
        return None

    pairs = []
    for lp in left_files:
        idx_str = lp.stem.replace('left_', '')
        rp = rder_dir / f'right_{idx_str}.jpg'
        if rp.exists():
            pairs.append((int(idx_str), lp, rp))

    n_total = len(pairs)
    print(f"\n{'='*58}")
    print(f"  SLAM OFFLINE — {n_total} pares de frames")
    print(f"{'='*58}")

    disp_raw_dir = sd / 'disparity_raw'
    disp_raw_dir.mkdir(exist_ok=True)

    raft_queue_off = queue.Queue(maxsize=4)
    vo_queue_off   = queue.Queue(maxsize=4)
    vo             = VisualOdometry(Q, K_rect, inclinacion_deg=INCLINACION_CAMARA)

    def lector_worker():
        for frame_idx, lp, rp in pairs:
            izq = cv2.imread(str(lp))
            der = cv2.imread(str(rp))
            if izq is None or der is None:
                continue
            raft_queue_off.put((frame_idx, izq, der))
        raft_queue_off.put(None)

    raft_t = [0.0]   # segundos acumulados de RAFT

    def raft_worker():
        processed = 0
        while True:
            item = raft_queue_off.get()
            if item is None:
                vo_queue_off.put(None)
                break
            frame_idx, izq, der = item
            try:
                t0   = time.perf_counter()
                disp = run_raft(model, args, izq, der)
                raft_t[0] += time.perf_counter() - t0
                np.save(str(disp_raw_dir / f'disp_{frame_idx:06d}.npy'), disp)
                vo_queue_off.put((frame_idx, izq, disp))
                processed += 1
                if processed % 20 == 0:
                    pct = 100 * processed / n_total
                    print(f"  [RAFT] {processed}/{n_total} ({pct:.0f}%)...")
            except Exception as e:
                print(f"  AVISO RAFT frame {frame_idx}: {e}")

    vo_done = threading.Event()
    validos = [0]
    total   = [0]

    def vo_worker_off():
        while True:
            item = vo_queue_off.get()
            if item is None:
                break
            frame_idx, izq, disp = item
            total[0] += 1
            try:
                result = vo.update(izq, disp, frame_idx=frame_idx)
                if result is not None:
                    validos[0] += 1
                    if validos[0] % 10 == 0:
                        R, t = result
                        print(f"  [VO]  frame {frame_idx:05d} | "
                              f"dx={t[0]:+7.1f}  dy={t[1]:+7.1f}  dz={t[2]:+7.1f} mm"
                              f"  ({validos[0]}/{total[0]} validos)")
            except Exception as e:
                print(f"  AVISO VO frame {frame_idx}: {e}")
        vo_done.set()

    t_lector = threading.Thread(target=lector_worker, daemon=True, name='Lector')
    t_raft   = threading.Thread(target=raft_worker,   daemon=True, name='RAFT-off')
    t_vo     = threading.Thread(target=vo_worker_off, daemon=True, name='VO-off')

    t_lector.start()
    t_raft.start()
    t_vo.start()

    t_lector.join()
    t_raft.join()
    vo_done.wait()

    print(f"\n  [SLAM] Completo: {validos[0]}/{total[0]} frames con pose valida")
    if total[0] > 0:
        print(f"  [TIEMPO] RAFT: {raft_t[0]:.1f} s total, "
              f"{1000*raft_t[0]/total[0]:.0f} ms por frame")
    if total[0] > 0:
        pct = 100 * validos[0] / total[0]
        print(f"  [SLAM] Cobertura: {pct:.1f}%"
              + (" — buena" if pct > 60 else " — baja"))

    return vo


# ==================================================================
#  MAIN
# ==================================================================

def main():
    print("=" * 58)
    print("  client_2.py - Stereo SLAM offline")
    print("=" * 58)

    print("\n[1/4] Cargando RAFT-Stereo en GPU...")
    model, args = load_raft_model()

    print(f"\n[2/4] Conectando a Pi ({PI_IP})...")
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cmd_sock.connect((PI_IP, 9999))
    print("  OK Comandos conectados (puerto 9999)")

    vid_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    vid_sock.connect((PI_IP, 9998))
    print("  OK Video conectado (puerto 9998)")

    print("\n[3/4] Detectando resolucion del stream...")
    first_frame, first_data = read_one_frame(vid_sock)
    if first_frame is None:
        print("  ERROR No se pudo leer el primer frame.")
        cmd_sock.close()
        vid_sock.close()
        return

    h, w   = first_frame.shape[:2]
    if w % 2 != 0:
        print(f"  ERROR Ancho de frame impar ({w}); no se puede dividir en estereo.")
        request_shutdown()
        cmd_sock.close()
        vid_sock.close()
        return
    lens_w = w // 2
    lens_h = h
    print(f"  Frame completo recibido: {w}x{h}")
    print(f"  Cada lente recibido: {lens_w}x{lens_h}")

    print("\n[4/4] Calculando mapas de rectificacion estereo...")
    maps_izq, maps_der, Q, K_rect = load_rectification(CAL_JSON, lens_w, lens_h)
    print("  OK Mapas listos")

    print("\nCreando sesion de captura...")
    sess = create_session() if SAVE_FRAMES else None
    if sess is not None:
        try:
            with open(CAL_JSON, 'r', encoding='utf-8') as f:
                cal_info = json.load(f)
            cal_res = cal_info['lente_izquierdo']['resolucion']
            session_info = {
                'pi_ip': PI_IP,
                'frame_completo_recibido': [w, h],
                'lente_izquierdo_recibido': [lens_w, lens_h],
                'lente_derecho_recibido': [lens_w, lens_h],
                'calibracion_por_lente': cal_res,
                'escala_vs_calibracion': [lens_w / cal_res[0], lens_h / cal_res[1]],
                'cal_json': str(CAL_JSON),
                'cal_json_sesion': 'estereo_cal.json',
                'raft_dir': str(RAFT_DIR),
                'raft_model': str(RAFT_MODEL),
                'inclinacion_camara_deg': INCLINACION_CAMARA,
            }
            with open(sess['sd'] / 'session_info.json', 'w', encoding='utf-8') as f:
                json.dump(session_info, f, indent=2)
            # Copia de la calibracion usada: si despues se recalibra, esta sesion
            # se puede reprocesar con la misma Q con la que se rectificaron sus frames.
            shutil.copyfile(CAL_JSON, sess['sd'] / 'estereo_cal.json')
        except Exception as e:
            print(f"  AVISO No se pudo escribir session_info.json / copiar calibracion: {e}")

    t_save = threading.Thread(
        target=save_worker, args=(sess,), daemon=True, name='Save-IO'
    )
    t_save.start()

    t_cmd = threading.Thread(
        target=handle_commands, args=(cmd_sock,), daemon=True, name='Commands'
    )
    t_cmd.start()

    t_video = threading.Thread(
        target=video_loop,
        args=(vid_sock, maps_izq, maps_der, sess, (first_frame, first_data)),
        daemon=True,
        name='Video',
    )
    t_video.start()

    print("\n> Stream iniciado.")
    print("> Fase 1: grabando frames a disco")
    print("> W/A/S/D = mantener presionado para mover | Q = frenar | X = salir y procesar\n")

    try:
        while not stop_event.is_set():
            with latest_preview['lock']:
                izq = latest_preview['izq']
                der = latest_preview['der']

            if izq is not None:
                cv2.imshow('Izquierdo rectificado',
                           cv2.resize(izq, (0, 0), fx=PREVIEW_SCALE, fy=PREVIEW_SCALE))
            if der is not None:
                cv2.imshow('Derecho rectificado',
                           cv2.resize(der, (0, 0), fx=PREVIEW_SCALE, fy=PREVIEW_SCALE))

            # Las teclas tambien se atienden aqui: si el foco esta en la ventana
            # de vista previa, la terminal no las recibe.
            key = chr(cv2.waitKey(33) & 0xFF).lower()
            if key == 'x':
                request_shutdown()
                break
            if key in {'w', 'a', 's', 'd', 'q'}:
                send_car_command(key)

        cv2.destroyAllWindows()

    finally:
        # request_shutdown (y no solo stop_event.set) para que el Pi reciba
        # 'q' y 'x' tambien cuando se sale con Ctrl+C o por una excepcion
        request_shutdown()
        t_video.join(timeout=5)
        if SAVE_FRAMES:
            save_queue.put(None)
            save_queue.join()
            t_save.join(timeout=5)
        t_cmd.join(timeout=3)

        print("\n> Fase 2: SLAM offline (puede tardar varios minutos)...")
        vo = slam_offline(sess, model, args, Q, K_rect)

        if vo is not None:
            print("\n> Fase 3: Generando point cloud...")
            build_point_cloud(sess, vo, Q)

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
