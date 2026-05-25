"""
client_3.py - Cliente estereo con rectificacion + RAFT-Stereo en GPU
=====================================================================
Flujo por frame:
  raw frame  ->  split izq/der  ->  remap (rectificacion)
             ->  RAFT-Stereo GPU  ->  mapa de disparidad  ->  vo_worker CPU
             ->  mostrar + guardar (frames rectificados + disparidad)

Arquitectura de hilos:
  video_loop (hilo)  -> infer_queue -> inference_worker (GPU)
                                    -> vo_queue          -> vo_worker (CPU)
  hilo principal     -> GUI loop (imshow / waitKey) a ~30fps

Controles: W/A/S/D = mover carrito | Q = frenar | X = salir

FIXES aplicados:
  - run_raft: squeeze defensivo + assert 2D antes de retornar disp
  - _deproject en VisualOdometry: squeeze al recibir disp
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
from visual_odometry import VisualOdometry

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
INFER_ITERS    = 10   # iteraciones RAFT (mas = mas calidad, mas lento)
PREVIEW_SCALE  = 0.5  # escala del preview (1.0 = sin escala)

# -- RAFT-Stereo path setup ----------------------------------------
sys.path.insert(0, str(RAFT_DIR))
sys.path.insert(0, str(RAFT_DIR / 'core'))
from raft_stereo import RAFTStereo

# -- Estado global -------------------------------------------------
stop_event     = threading.Event()
command_socket = None
command_lock   = threading.Lock()
infer_queue    = queue.Queue(maxsize=2)   # video -> RAFT (GPU)
vo_queue       = queue.Queue(maxsize=2)   # RAFT  -> VO   (CPU)
save_queue     = queue.Queue(maxsize=10)  # RAFT  -> disk (hilo I/O)
latest_disp    = {'img': None, 'lock': threading.Lock()}
latest_preview = {'izq': None, 'der': None, 'lock': threading.Lock()}


# ==================================================================
#  UTILIDADES DE RED
# ==================================================================

def recv_exact(sock, size):
    """Lee exactamente `size` bytes del socket."""
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

    Retorna:
      maps_izq  - (map1, map2) para cv2.remap del lente izquierdo
      maps_der  - (map1, map2) para cv2.remap del lente derecho
      Q         - matriz de reproyeccion disparidad->3D
      K_rect    - matriz K exacta del lente izquierdo rectificado (P1[:3,:3])
    """
    with open(cal_path) as f:
        cal = json.load(f)

    cal_w, cal_h = cal['lente_izquierdo']['resolucion']
    sx = lens_w / cal_w
    sy = lens_h / cal_h

    def scale_K(K_list):
        K = np.array(K_list, dtype=np.float64)
        K[0, 0] *= sx
        K[1, 1] *= sy
        K[0, 2] *= sx
        K[1, 2] *= sy
        return K

    K_izq  = scale_K(cal['lente_izquierdo']['matriz_camara'])
    D_izq  = np.array(cal['lente_izquierdo']['coef_distorsion'], dtype=np.float64)
    K_der  = scale_K(cal['lente_derecho']['matriz_camara'])
    D_der  = np.array(cal['lente_derecho']['coef_distorsion'], dtype=np.float64)
    R      = np.array(cal['estereo']['R'], dtype=np.float64)
    T      = np.array(cal['estereo']['T'], dtype=np.float64)

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

    K_rect = P1[:3, :3].copy()
    return maps_izq, maps_der, Q, K_rect


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
    padded = cv2.copyMakeBorder(img, 0, pad_h, 0, pad_w, cv2.BORDER_REFLECT)
    return padded, h, w


def to_tensor(img_bgr):
    rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    t   = torch.from_numpy(rgb).permute(2, 0, 1).float().unsqueeze(0)
    return t.cuda()


@torch.no_grad()
def run_raft(model, args, img_izq, img_der):
    orig_h, orig_w = img_izq.shape[:2]

    izq_pad, h, w = pad32(img_izq)
    der_pad, _, _ = pad32(img_der)

    t_izq = to_tensor(izq_pad)
    t_der = to_tensor(der_pad)

    _, flow = model(t_izq, t_der, iters=args.valid_iters, test_mode=True)

    # Squeeze completo en GPU, luego crop y clamp
    flow_sq = flow.squeeze()

    # Asegurar que sea 2D (H, W) — RAFT a veces deja dims extra
    if flow_sq.ndim == 1:
        # Caso degenrado: reshape usando las dimensiones originales
        flow_sq = flow_sq.reshape(orig_h, orig_w)
    elif flow_sq.ndim > 2:
        flow_sq = flow_sq[0]

    disp = (-flow_sq[:orig_h, :orig_w]).clamp(min=0).cpu().numpy()

    # Garantia final antes de salir de esta funcion
    if disp.ndim != 2:
        raise RuntimeError(
            f"run_raft: disp debe ser 2D pero tiene shape {disp.shape}. "
            f"flow original shape={flow.shape}"
        )

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
    ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
    sd   = SAVE_ROOT / f'session_{ts}'
    dirs = {k: sd / k for k in ('full', 'left_rect', 'right_rect', 'disparity', 'disparity_raw')}
    for d in dirs.values():
        d.mkdir(parents=True, exist_ok=True)

    csv_file = open(sd / 'frames.csv', 'w', newline='', encoding='utf-8')
    writer   = csv.writer(csv_file)
    writer.writerow([
        'frame_index', 'timestamp_unix', 'timestamp_iso',
        'width', 'height',
        'full_path', 'left_rect_path', 'right_rect_path',
        'disparity_path', 'disparity_raw_path',
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
#  HILO DE GUARDADO EN DISCO  (I/O)
# ==================================================================

def save_worker(sess):
    """
    Consume de save_queue y escribe frames/CSV a disco.
    Al correr en hilo separado, el hilo GPU nunca bloquea por I/O.
    """
    while not stop_event.is_set():
        try:
            item = save_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        idx, raw_bytes, frame, rect_izq, rect_der, disp_color, disp_raw = item
        try:
            save_frame(sess, idx, raw_bytes, frame, rect_izq, rect_der, disp_color, disp_raw)
        except Exception as e:
            print(f"  AVISO Error guardado frame {idx}: {e}")

        save_queue.task_done()

    # Vaciar lo que quede en la cola al cerrar
    while not save_queue.empty():
        try:
            item = save_queue.get_nowait()
        except queue.Empty:
            break
        idx, raw_bytes, frame, rect_izq, rect_der, disp_color, disp_raw = item
        try:
            save_frame(sess, idx, raw_bytes, frame, rect_izq, rect_der, disp_color, disp_raw)
        except Exception:
            pass
        save_queue.task_done()


def save_frame(sess, idx, raw_bytes, frame, rect_izq, rect_der, disp_color, disp_raw):
    if sess is None:
        return

    now = time.time()
    ts  = datetime.fromtimestamp(now).isoformat(timespec='milliseconds')
    h, w = frame.shape[:2]
    d    = sess['dirs']

    p_full     = d['full']          / f'frame_{idx:06d}.jpg'
    p_izq      = d['left_rect']     / f'left_{idx:06d}.jpg'
    p_der      = d['right_rect']    / f'right_{idx:06d}.jpg'
    p_disp     = d['disparity']     / f'disp_{idx:06d}.jpg'
    p_disp_raw = d['disparity_raw'] / f'disp_{idx:06d}.npy'

    with open(p_full, 'wb') as f:
        f.write(raw_bytes)
    cv2.imwrite(str(p_izq),  rect_izq)
    cv2.imwrite(str(p_der),  rect_der)
    cv2.imwrite(str(p_disp), disp_color)
    np.save(str(p_disp_raw), disp_raw)

    sess['writer'].writerow([
        idx, f'{now:.6f}', ts, w, h,
        str(p_full), str(p_izq), str(p_der), str(p_disp), str(p_disp_raw),
    ])
    if idx % 30 == 0:
        sess['csv'].flush()


# ==================================================================
#  HILO DE INFERENCIA RAFT-Stereo  (GPU)
# ==================================================================

def inference_worker(model, args, sess):
    """
    Consume frames de infer_queue, ejecuta RAFT-Stereo (GPU)
    y encola el resultado en vo_queue para el hilo de VO (CPU).
    """
    while not stop_event.is_set():
        try:
            item = infer_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        idx, raw_bytes, frame, rect_izq, rect_der = item
        try:
            # -- Disparidad RAFT-Stereo (GPU) -------------------------
            disp       = run_raft(model, args, rect_izq, rect_der)
            disp_color = disp_to_color(disp)

            with latest_disp['lock']:
                latest_disp['img'] = disp_color.copy()

            # -- Guardar frames (hilo I/O separado) ------------------
            if SAVE_FRAMES:
                try:
                    save_queue.put_nowait(
                        (idx, raw_bytes, frame, rect_izq, rect_der, disp_color, disp)
                    )
                except queue.Full:
                    pass

            # -- Encolar para VO (CPU) --------------------------------
            try:
                vo_queue.put_nowait((idx, rect_izq, disp))
            except queue.Full:
                pass

        except Exception as e:
            print(f"  AVISO Error inferencia frame {idx}: {e}")

        infer_queue.task_done()


# ==================================================================
#  HILO DE ODOMETRIA VISUAL  (CPU)
# ==================================================================

def vo_worker(vo):
    """
    Corre en hilo separado del de RAFT.
    Consume de vo_queue y ejecuta VO (ORB + solvePnPRansac) en CPU.
    """
    _vo_print_every = 10
    while not stop_event.is_set():
        try:
            item = vo_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        idx, rect_izq, disp = item
        try:
            result = vo.update(rect_izq, disp, frame_idx=idx)

            if result is not None and idx % _vo_print_every == 0:
                R, t = result
                print(f"  [VO] frame {idx:05d} | "
                      f"dx={t[0]:+7.1f}mm  dy={t[1]:+7.1f}mm  dz={t[2]:+7.1f}mm")

        except Exception as e:
            print(f"  AVISO Error VO frame {idx}: {e}")

        vo_queue.task_done()


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

        # Guardar para el hilo de preview (sin bloquear aqui)
        with latest_preview['lock']:
            latest_preview['izq'] = rect_izq
            latest_preview['der'] = rect_der

        try:
            infer_queue.put_nowait(
                (frame_index, frame_data, frame, rect_izq, rect_der)
            )
        except queue.Full:
            pass

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
#  GENERACION DE NUBE DE PUNTOS  (fase post-sesion)
# ==================================================================
def build_point_cloud(sess, vo, Q):
    """
    Post-sesion: proyecta disparidades guardadas a 3D con poses optimizadas,
    verifica unidades, exporta trayectoria, aplica ICP entre frames consecutivos
    con gate de fitness, y exporta mapa_final.ply.
    """
    try:
        import open3d as o3d
    except ImportError:
        print("  [PC] open3d no disponible - instala con: pip install open3d")
        return

    if sess is None or not SAVE_FRAMES:
        return

    # -- Parametros (distancias en mm) --------------------------------
    DISP_MIN      = 2.0     # disparidad minima de confianza
    Z_MIN         = 100.0   # profundidad minima (mm)
    Z_MAX         = 8000.0  # profundidad maxima (mm)
    VOXEL_SIZE    = 20.0    # voxel por frame y final (mm)
    STAT_K        = 20      # vecinos para outlier estadistico por frame
    STAT_STD      = 2.0     # umbral sigma por frame
    RAD_POINTS    = 16      # vecinos minimos outlier radial final
    RAD_RADIUS    = 50.0    # radio outlier radial final (mm)
    ICP_MAX_DIST  = 100.0   # distancia maxima de correspondencia ICP (mm)
    ICP_MIN_FIT   = 0.15    # fitness minimo — descartar frame si es menor
    ICP_MAX_ITER  = 30      # iteraciones maximas ICP
    FRAME_STRIDE  = 2       # usar 1 de cada N frames

    sd       = sess['sd']
    raw_dir  = sd / 'disparity_raw'
    left_dir = sd / 'left_rect'

    poses = vo.get_all_poses()
    if not poses:
        print("  [PC] Sin poses disponibles")
        return

    # -- 1. Exportar trayectoria poses.txt ----------------------------
    poses_path = sd / 'poses.txt'
    with open(str(poses_path), 'w') as f:
        f.write("frame_idx  X_mm        Y_mm        Z_mm\n")
        for fid in sorted(poses.keys()):
            x, y, z = poses[fid][:3, 3]
            f.write(f"{fid:06d}    {x:12.3f}  {y:12.3f}  {z:12.3f}\n")
    print(f"  [PC] Trayectoria: {poses_path}  ({len(poses)} poses)")

    all_frame_ids = sorted(poses.keys())
    frame_ids     = all_frame_ids[::FRAME_STRIDE]
    print(f"\n[PC] Sesion: {len(all_frame_ids)} frames totales -> "
          f"{len(frame_ids)} a procesar (stride={FRAME_STRIDE})...")

    pcd_combined  = o3d.geometry.PointCloud()
    prev_pcd      = None
    prev_pos      = None   # traslacion del ultimo frame aceptado
    processed     = 0
    skipped       = 0
    skipped_pose  = 0
    first_done    = False

    for frame_idx in frame_ids:
        disp_path = raw_dir  / f'disp_{frame_idx:06d}.npy'
        left_path = left_dir / f'left_{frame_idx:06d}.jpg'

        if not disp_path.exists() or not left_path.exists():
            continue

        disp = np.load(str(disp_path)).astype(np.float32)
        img  = cv2.imread(str(left_path))
        if img is None:
            continue

        # -- 1b. Saltar frame si la pose no se movio >= 5mm -----------
        cur_pos = poses[frame_idx][:3, 3]
        if prev_pos is not None:
            if np.linalg.norm(cur_pos - prev_pos) < 5.0:
                skipped_pose += 1
                continue

        pts3d = cv2.reprojectImageTo3D(disp, Q)   # (H, W, 3) en coords camara

        # -- 2. Verificacion de unidades en el primer frame -----------
        if not first_done:
            z_all  = pts3d[:, :, 2]
            vmask  = (disp > DISP_MIN) & np.isfinite(z_all) & (z_all > 0)
            sample = pts3d[vmask][:10]
            print(f"  [PC] Q:\n{np.array2string(Q, precision=4, suppress_small=True)}")
            print("  [PC] Primeros 10 puntos 3D validos (verifica unidades Z):")
            for p in sample:
                print(f"       X={p[0]:8.1f}  Y={p[1]:8.1f}  Z={p[2]:8.1f}")
            first_done = True

        # -- Filtrar por confianza y rango de profundidad -------------
        z    = pts3d[:, :, 2]
        mask = ((disp > DISP_MIN) & np.isfinite(z) & (z > Z_MIN) & (z < Z_MAX))

        pts  = pts3d[mask].astype(np.float64)
        cols = img[mask][:, ::-1].astype(np.float64) / 255.0

        if len(pts) < STAT_K + 1:
            continue

        # -- Transformar a coordenadas mundo con la pose VO -----------
        T_pose = poses[frame_idx]
        pts_h  = np.hstack([pts, np.ones((len(pts), 1))])
        pts_w  = (T_pose @ pts_h.T).T[:, :3]

        # -- Filtrar puntos detrás del plano mundo (Z negativo) -------
        valid_w = pts_w[:, 2] > 0
        pts_w   = pts_w[valid_w]
        cols    = cols[valid_w]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts_w)
        pcd.colors = o3d.utility.Vector3dVector(cols)

        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=STAT_K, std_ratio=STAT_STD)
        pcd    = pcd.voxel_down_sample(VOXEL_SIZE)

        if len(pcd.points) < 50:
            continue

        # -- 3. ICP contra el ultimo frame aceptado -------------------
        if prev_pcd is not None:
            result = o3d.pipelines.registration.registration_icp(
                source=pcd,
                target=prev_pcd,
                max_correspondence_distance=ICP_MAX_DIST,
                init=np.eye(4),
                estimation_method=(
                    o3d.pipelines.registration.TransformationEstimationPointToPoint()
                ),
                criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=ICP_MAX_ITER
                ),
            )

            # -- 4. Descartar frame si fitness < ICP_MIN_FIT ----------
            if result.fitness < ICP_MIN_FIT:
                skipped += 1
                continue   # prev_pcd no se actualiza: proximo frame se alinea al ultimo bueno

            pcd.transform(result.transformation)

        # Frame aceptado: actualizar referencia y acumular
        prev_pcd      = pcd
        prev_pos      = cur_pos
        pcd_combined += pcd
        processed    += 1

        if processed % 10 == 0:
            print(f"  [PC] {processed} acumulados, {skipped} ICP + {skipped_pose} pose descartados...")

    print(f"  [PC] Resumen: {processed} acumulados, {skipped} ICP + "
          f"{skipped_pose} pose < 5mm descartados")

    if len(pcd_combined.points) == 0:
        print("  [PC] Nube vacia, no se genera archivo")
        return

    n_bruto = len(pcd_combined.points)

    pcd_combined = pcd_combined.voxel_down_sample(VOXEL_SIZE)
    pcd_tmp, _   = pcd_combined.remove_radius_outlier(
        nb_points=RAD_POINTS, radius=RAD_RADIUS
    )

    # -- DBSCAN: conservar solo el cluster principal ----------------
    # Elimina islas creadas por saltos de odometria que sobrevivieron
    # al filtrado de radio (pueden ser densas localmente).
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Error):
        labels = np.array(pcd_tmp.cluster_dbscan(
            eps=60.0, min_points=20, print_progress=False
        ))

    if labels.max() >= 0:
        counts    = np.bincount(labels[labels >= 0])
        main_lbl  = int(counts.argmax())
        pcd_final = pcd_tmp.select_by_index(np.where(labels == main_lbl)[0])
        n_noise   = int(np.sum(labels < 0))
        n_other   = int(np.sum((labels >= 0) & (labels != main_lbl)))
        print(f"  [PC] DBSCAN: {len(counts)} clusters -> "
              f"principal={len(pcd_final.points):,}pts  "
              f"ruido={n_noise}  otros={n_other}")
    else:
        pcd_final = pcd_tmp
        print("  [PC] DBSCAN: sin clusters detectados, usando nube completa")

    n_clean = len(pcd_final.points)
    print(f"  [PC] Bruto: {n_bruto:,} -> voxel: {len(pcd_combined.points):,} "
          f"-> radius: {len(pcd_tmp.points):,} -> dbscan: {n_clean:,}")

    # -- Detectar plano del suelo ------------------------------------
    floor_inliers = None
    R_align       = np.eye(3)

    try:
        plane_model, inliers = pcd_final.segment_plane(
            distance_threshold=30.0,
            ransac_n=3,
            num_iterations=1000,
        )
        floor_inliers = inliers
        a, b, c, d = plane_model
        normal = np.array([a, b, c], dtype=np.float64)
        normal /= np.linalg.norm(normal)

        # Orientar la normal hacia arriba (Y positivo en Open3D = arriba)
        if normal[1] < 0:
            normal = -normal

        target = np.array([0.0, 1.0, 0.0])
        v      = np.cross(normal, target)
        s      = np.linalg.norm(v)
        cos_a  = np.dot(normal, target)

        if s > 1e-6:
            vx = np.array([
                [0,    -v[2],  v[1]],
                [v[2],  0,    -v[0]],
                [-v[1], v[0],  0   ],
            ])
            R_align = np.eye(3) + vx + vx @ vx * ((1 - cos_a) / (s * s))

        print(f"  [PC] Plano suelo: [{a:.4f}, {b:.4f}, {c:.4f}, {d:.4f}]  "
              f"inliers={len(inliers)}")
        print(f"  [PC] Normal suelo: {normal}  -> rotacion aplicada")

    except Exception as e:
        print(f"  [PC] segment_plane fallo: {e}  (exportando sin alineacion)")

    # -- Separar suelo y paredes -------------------------------------
    if floor_inliers is not None:
        pcd_floor  = pcd_final.select_by_index(floor_inliers)
        pcd_walls  = pcd_final.select_by_index(floor_inliers, invert=True)
    else:
        pcd_walls  = pcd_final
        pcd_floor  = o3d.geometry.PointCloud()

    # -- Aplicar rotacion de alineacion a ambas nubes ----------------
    center = pcd_final.get_center()
    pcd_final.rotate(R_align, center=center)
    pcd_walls.rotate(R_align, center=center)

    # -- Exportar dos PLY --------------------------------------------
    out_final  = str(sd / 'mapa_final.ply')
    out_walls  = str(sd / 'mapa_paredes.ply')

    o3d.io.write_point_cloud(out_final, pcd_final)
    o3d.io.write_point_cloud(out_walls, pcd_walls)

    print(f"  [PC] mapa_final.ply   -> {len(pcd_final.points):,} puntos  ({out_final})")
    print(f"  [PC] mapa_paredes.ply -> {len(pcd_walls.points):,} puntos  ({out_walls})")


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

    h, w   = first_frame.shape[:2]
    lens_w = w // 2
    lens_h = h

    # -- 4. Calcular mapas de rectificacion -----------------------
    print("\n[4/4] Calculando mapas de rectificacion estereo...")
    maps_izq, maps_der, Q, K_rect = load_rectification(CAL_JSON, lens_w, lens_h)
    print("  OK Mapas listos")
    print(f"  Matriz Q:\n{np.array2string(Q, precision=4, suppress_small=True)}")

    # -- Instanciar odometria visual ------------------------------
    vo = VisualOdometry(Q, K_rect)
    print("  OK VisualOdometry lista")

    # -- Crear sesion de captura ----------------------------------
    print("\nCreando sesion de captura...")
    sess = create_session() if SAVE_FRAMES else None

    # -- Hilo RAFT-Stereo (GPU) -----------------------------------
    t_infer = threading.Thread(
        target=inference_worker,
        args=(model, args, sess),
        daemon=True,
        name='RAFT-GPU',
    )
    t_infer.start()

    # -- Hilo Odometria Visual (CPU) ------------------------------
    t_vo = threading.Thread(
        target=vo_worker,
        args=(vo,),
        daemon=True,
        name='VO-CPU',
    )
    t_vo.start()

    # -- Hilo de guardado en disco (I/O) -------------------------
    t_save = threading.Thread(
        target=save_worker,
        args=(sess,),
        daemon=True,
        name='Save-IO',
    )
    t_save.start()

    # -- Hilo de comandos -----------------------------------------
    t_cmd = threading.Thread(
        target=handle_commands,
        args=(cmd_sock,),
        daemon=True,
        name='Commands',
    )
    t_cmd.start()

    print("\n> Stream iniciado.")
    print("> Arquitectura: video(hilo) -> RAFT(GPU) -> VO(CPU) | GUI en hilo principal\n")

    # -- Loop de video en hilo separado ---------------------------
    t_video = threading.Thread(
        target=video_loop,
        args=(vid_sock, maps_izq, maps_der, sess, (first_frame, first_data)),
        daemon=True,
        name='Video',
    )
    t_video.start()

    # -- GUI loop en hilo principal (cv2.imshow/waitKey requieren hilo principal) --
    try:
        while not stop_event.is_set():
            with latest_preview['lock']:
                izq = latest_preview['izq']
                der = latest_preview['der']
            with latest_disp['lock']:
                disp_img = latest_disp['img']

            if izq is not None:
                cv2.imshow('Izquierdo rectificado', cv2.resize(izq, (0, 0), fx=PREVIEW_SCALE, fy=PREVIEW_SCALE))
            if der is not None:
                cv2.imshow('Derecho rectificado', cv2.resize(der, (0, 0), fx=PREVIEW_SCALE, fy=PREVIEW_SCALE))
            if disp_img is not None:
                cv2.imshow('Disparidad RAFT-Stereo', cv2.resize(disp_img, (0, 0), fx=PREVIEW_SCALE, fy=PREVIEW_SCALE))

            key = cv2.waitKey(33) & 0xFF  # ~30 fps
            if key == ord('x'):
                request_shutdown()
                break
        cv2.destroyAllWindows()
    finally:
        stop_event.set()
        t_video.join(timeout=5)
        t_infer.join(timeout=5)
        t_vo.join(timeout=5)
        t_save.join(timeout=10)
        t_cmd.join(timeout=3)
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
