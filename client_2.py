"""
client_3.py - Cliente estereo con SLAM offline
===============================================
Flujo en dos fases:

FASE 1 - Captura (en vivo):
  video_loop (hilo) -> rectifica -> save_queue -> save_worker (disco)
  hilo principal    -> GUI loop (imshow / waitKey)
  hilo comandos     -> WASD -> carrito

FASE 2 - SLAM offline (al cerrar):
  lector_disco -> raft_queue -> RAFT GPU -> vo_queue -> VO CPU
  build_point_cloud -> mapa_final.ply + mapa_paredes.ply

Controles: W/A/S/D = mover carrito | Q = frenar | X = salir y procesar
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
#  CONFIGURACION
# ==================================================================
BASE_DIR       = Path(__file__).resolve().parent
PI_IP          = os.environ.get('PI_IP', '10.42.0.146')
CAL_JSON       = Path(os.environ.get('CAL_JSON', '/home/alex/Escritorio/6tosemestre/Tridimensional/carrito/parac/estereo_cal.json'))
DEFAULT_RAFT_DIR = BASE_DIR / 'RAFT-Stereo'
LEGACY_RAFT_DIR  = Path('/home/alex/Escritorio/6tosemestre/Tridimensional/carrito/corrercudapytorch/RAFT-Stereo')
RAFT_DIR       = Path(os.environ.get(
    'RAFT_DIR',
    DEFAULT_RAFT_DIR if DEFAULT_RAFT_DIR.exists() else LEGACY_RAFT_DIR
))
RAFT_MODEL     = Path(os.environ.get('RAFT_MODEL', RAFT_DIR / 'models/raftstereo-middlebury.pth'))
SAVE_ROOT      = BASE_DIR / 'captured_frames'
SAVE_FRAMES    = True
MAX_FRAME_SIZE = 50 * 1024 * 1024
INFER_ITERS    = 20
PREVIEW_SCALE  = 0.5

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
#  CALIBRACION Y RECTIFICACION
# ==================================================================

def load_rectification(cal_path, lens_w, lens_h):
    with open(cal_path) as f:
        cal = json.load(f)

    cal_w, cal_h = cal['lente_izquierdo']['resolucion']
    sx = lens_w / cal_w
    sy = lens_h / cal_h

    def scale_K(K_list):
        K = np.array(K_list, dtype=np.float64)
        K[0, 0] *= sx;  K[1, 1] *= sy
        K[0, 2] *= sx;  K[1, 2] *= sy
        return K

    K_izq = scale_K(cal['lente_izquierdo']['matriz_camara'])
    D_izq = np.array(cal['lente_izquierdo']['coef_distorsion'], dtype=np.float64)
    K_der = scale_K(cal['lente_derecho']['matriz_camara'])
    D_der = np.array(cal['lente_derecho']['coef_distorsion'], dtype=np.float64)
    R     = np.array(cal['estereo']['R'], dtype=np.float64)
    T     = np.array(cal['estereo']['T'], dtype=np.float64)

    size = (lens_w, lens_h)
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        K_izq, D_izq, K_der, D_der, size, R, T, alpha=0
    )

    maps_izq = cv2.initUndistortRectifyMap(K_izq, D_izq, R1, P1, size, cv2.CV_32FC1)
    maps_der = cv2.initUndistortRectifyMap(K_der, D_der, R2, P2, size, cv2.CV_32FC1)

    print(f"  Calibracion por lente: {cal_w}x{cal_h}")
    print(f"  Resolucion recibida por lente: {lens_w}x{lens_h}")
    print(f"  Escala vs calibracion: sx={sx:.3f}  sy={sy:.3f}")
    if (lens_w, lens_h) != (cal_w, cal_h):
        print("  AVISO Resolucion de uso distinta a la calibracion.")
        print("        Se recalculan K, mapas y Q para esta resolucion,")
        print("        pero para mejor precision conviene capturar o recalibrar con la misma resolucion.")
    if abs(sx - sy) > 0.05:
        print("  AVISO Escalado no uniforme entre X/Y; la profundidad puede perder precision.")

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

        print("    W/A/S/D = mover | Q = frenar | X = salir y procesar")

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
    vo             = VisualOdometry(Q, K_rect)

    def lector_worker():
        for frame_idx, lp, rp in pairs:
            izq = cv2.imread(str(lp))
            der = cv2.imread(str(rp))
            if izq is None or der is None:
                continue
            raft_queue_off.put((frame_idx, izq, der))
        raft_queue_off.put(None)

    def raft_worker():
        processed = 0
        while True:
            item = raft_queue_off.get()
            if item is None:
                vo_queue_off.put(None)
                break
            frame_idx, izq, der = item
            try:
                disp = run_raft(model, args, izq, der)
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
        pct = 100 * validos[0] / total[0]
        print(f"  [SLAM] Cobertura: {pct:.1f}%"
              + (" — buena" if pct > 60 else " — baja"))

    return vo


# ==================================================================
#  BUILD POINT CLOUD (Fase 3)
# ==================================================================

def build_point_cloud(sess, vo, Q):
    try:
        import open3d as o3d
    except ImportError:
        print("  [PC] open3d no disponible - pip install open3d")
        return

    if sess is None or not SAVE_FRAMES:
        return

    # ------------------------------------------------------------------
    # PARAMETROS  (ajustados para mayor densidad de puntos)
    # ------------------------------------------------------------------
    DISP_MIN     = 2.0    # disparidad mínima confiable
    Z_MIN        = 100.0  # profundidad mínima en mm (10cm)
    Z_MAX        = 8000.0 # profundidad máxima en mm (8m)
    VOXEL_SIZE   = 8.0   # tamaño del cubo de fusión en mm
    STAT_K       = 20     # vecinos para outlier estadístico
    STAT_STD     = 2.0    # umbral sigma
    RAD_POINTS   = 8      # vecinos mínimos para outlier radial
    RAD_RADIUS   = 40.0   # radio de búsqueda en mm
    ICP_MIN_FIT  = 0.15   # fitness mínimo para aceptar alineación
    FRAME_STRIDE = 1      # usar todos los frames
    MIN_POSE_MOV = 5.0    # movimiento mínimo entre frames en mm
    # ------------------------------------------------------------------

    sd       = sess['sd']
    raw_dir  = sd / 'disparity_raw'
    left_dir = sd / 'left_rect'

    poses = vo.get_all_poses()

    poses_path = sd / 'poses.txt'
    with open(str(poses_path), 'w') as f:
        f.write("frame_idx  X_mm        Y_mm        Z_mm\n")
        for fid in sorted(poses.keys()):
            x, y, z = poses[fid][:3, 3]
            f.write(f"{fid:06d}    {x:12.3f}  {y:12.3f}  {z:12.3f}\n")
    print(f"  [PC] Trayectoria: {poses_path}  ({len(poses)} poses)")

    all_frame_ids = sorted(poses.keys())
    frame_ids     = all_frame_ids[::FRAME_STRIDE]
    print(f"\n  [PC] Generando nube ({len(frame_ids)} frames, stride={FRAME_STRIDE})...")

    pcd_combined = o3d.geometry.PointCloud()
    prev_pcd     = None
    prev_pos     = None
    processed    = 0
    skipped_icp  = 0
    skipped_pose = 0
    first_done   = False

    for frame_idx in frame_ids:
        disp_path = raw_dir  / f'disp_{frame_idx:06d}.npy'
        left_path = left_dir / f'left_{frame_idx:06d}.jpg'

        if not disp_path.exists() or not left_path.exists():
            continue

        disp = np.load(str(disp_path)).astype(np.float32)
        img  = cv2.imread(str(left_path))
        if img is None:
            continue

        cur_pos = poses[frame_idx][:3, 3]
        if prev_pos is not None:
            if np.linalg.norm(cur_pos - prev_pos) < MIN_POSE_MOV:
                print(f"Frame {frame_idx:06d} descartada porque o no se movio o fue muy poco")
                skipped_pose += 1
                continue

        pts3d = cv2.reprojectImageTo3D(disp, Q)

        if not first_done:
            z_all  = pts3d[:, :, 2]
            vmask  = (disp > DISP_MIN) & np.isfinite(z_all) & (z_all > 0)
            sample = pts3d[vmask][:10]
            print(f"  [PC] Q:\n{np.array2string(Q, precision=4, suppress_small=True)}")
            print("  [PC] Primeros 10 puntos 3D validos:")
            for p in sample:
                print(f"       X={p[0]:8.1f}  Y={p[1]:8.1f}  Z={p[2]:8.1f}")
            first_done = True

        z    = pts3d[:, :, 2]
        # mask = (disp > DISP_MIN) & np.isfinite(z) & (z > Z_MIN) & (z < Z_MAX)
        total_px = disp.size  # total de píxeles en el frame
        # Filtro 1 — disparidad mínima
        m1 = disp > DISP_MIN
        # Filtro 2 — valores finitos
        m2 = np.isfinite(z)
        # Filtro 3 — profundidad mínima
        m3 = z > Z_MIN
        # Filtro 4 — profundidad máxima
        m4 = z < Z_MAX

        # Combinado final
        mask = m1 & m2 & m3 & m4

        if processed % 10 == 0:
            print(f"\n  [PC] frame {frame_idx:05d} — {total_px:,} píxeles totales")
            print(f"    disp < {DISP_MIN}  (sin info):  {(~m1).sum():>8,} eliminados  ({100*(~m1).sum()/total_px:.1f}%)")
            print(f"    infinitos/NaN:           {(~m2).sum():>8,} eliminados  ({100*(~m2).sum()/total_px:.1f}%)")
            print(f"    z < {Z_MIN}mm (muy cerca): {(~m3).sum():>8,} eliminados  ({100*(~m3).sum()/total_px:.1f}%)")
            print(f"    z > {Z_MAX}mm (muy lejos): {(~m4).sum():>8,} eliminados  ({100*(~m4).sum()/total_px:.1f}%)")
            print(f"    QUEDAN:                  {mask.sum():>8,} puntos     ({100*mask.sum()/total_px:.1f}%)")

        pts  = pts3d[mask].astype(np.float64)
        cols = img[mask][:, ::-1].astype(np.float64) / 255.0

        if len(pts) < STAT_K + 1:
            continue

        T_pose = poses[frame_idx]
        pts_h  = np.hstack([pts, np.ones((len(pts), 1))])
        pts_w  = (T_pose @ pts_h.T).T[:, :3]

        valid_w = np.all(np.isfinite(pts_w), axis=1)
        pts_w   = pts_w[valid_w]
        cols    = cols[valid_w]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts_w)
        pcd.colors = o3d.utility.Vector3dVector(cols)

        # Filtro estadistico por frame (mantener, es util para ruido local)
        # pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=STAT_K, std_ratio=STAT_STD)
        
        n_antes = len(pcd.points)
        pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=STAT_K, std_ratio=STAT_STD)
        n_despues = len(pcd.points)

        if processed % 10 == 0:
            eliminados = n_antes - n_despues
            print(f"    outlier estadístico:     {eliminados:>8,} eliminados  "
                f"({100*eliminados/n_antes:.1f}%)  "
                f"quedan={n_despues:,}")


        # ELIMINADO: voxel_down_sample por frame
        # Antes se aplicaba aqui con VOXEL_SIZE=20, lo que tiraba puntos
        # prematuramente. Ahora solo se aplica una vez al combinado final.

        if len(pcd.points) < 50:
            continue

        prev_pcd      = pcd
        prev_pos      = cur_pos
        pcd_combined += pcd
        processed    += 1

        if processed % 10 == 0:
            print(f"  [PC] {processed} acumulados, "
                  f"{skipped_icp} ICP + {skipped_pose} pose descartados...")

    print(f"  [PC] Resumen: {processed} acumulados, "
          f"{skipped_icp} ICP + {skipped_pose} pose < {MIN_POSE_MOV}mm descartados")

    if len(pcd_combined.points) == 0:
        print("  [PC] Nube vacia, no se genera archivo")
        return

    n_bruto = len(pcd_combined.points)

    # Voxel downsample una sola vez sobre el combinado completo
    # pcd_combined = pcd_combined.voxel_down_sample(VOXEL_SIZE)
    # pcd_tmp, _   = pcd_combined.remove_radius_outlier(nb_points=RAD_POINTS, radius=RAD_RADIUS)

    n_antes = len(pcd_combined.points)
    pcd_combined = pcd_combined.voxel_down_sample(VOXEL_SIZE)
    print(f"\n  [PC] Voxel downsample:   {n_antes:,} → {len(pcd_combined.points):,} "
        f"(-{n_antes - len(pcd_combined.points):,} puntos)")

    # Radius outlier
    n_antes = len(pcd_combined.points)
    pcd_tmp, _ = pcd_combined.remove_radius_outlier(nb_points=RAD_POINTS, radius=RAD_RADIUS)
    print(f"  [PC] Radius outlier:     {n_antes:,} → {len(pcd_tmp.points):,} "
        f"(-{n_antes - len(pcd_tmp.points):,} puntos)")


    # with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Error):
    #     labels = np.array(pcd_tmp.cluster_dbscan(
    #         eps=60.0, min_points=20, print_progress=False
    #     ))

    # if labels.max() >= 0:
    #     counts    = np.bincount(labels[labels >= 0])
    #     main_lbl  = int(counts.argmax())
    #     pcd_final = pcd_tmp.select_by_index(np.where(labels == main_lbl)[0])
    #     print(f"  [PC] DBSCAN: {len(counts)} clusters -> "
    #           f"principal={len(pcd_final.points):,}pts  "
    #           f"ruido={int(np.sum(labels < 0))}  "
    #           f"otros={int(np.sum((labels >= 0) & (labels != main_lbl)))}")
    # else:
    #     pcd_final = pcd_tmp

    n_antes = len(pcd_tmp.points)

    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Error):
        labels = np.array(pcd_tmp.cluster_dbscan(
            eps=60.0, min_points=20, print_progress=False
        ))

    if labels.max() >= 0:
        counts   = np.bincount(labels[labels >= 0])
        main_lbl = int(counts.argmax())
        pcd_final = pcd_tmp.select_by_index(np.where(labels == main_lbl)[0])

        n_ruido  = int(np.sum(labels < 0))
        n_otros  = int(np.sum((labels >= 0) & (labels != main_lbl)))
        n_principal = len(pcd_final.points)

        print(f"\n  [PC] DBSCAN:")
        print(f"    clusters encontrados:  {len(counts)}")
        print(f"    cluster principal:     {n_principal:>8,} puntos  ({100*n_principal/n_antes:.1f}%)")
        print(f"    ruido (sin cluster):   {n_ruido:>8,} puntos  ({100*n_ruido/n_antes:.1f}%)")
        print(f"    otros clusters:        {n_otros:>8,} puntos  ({100*n_otros/n_antes:.1f}%)")
        print(f"    eliminados total:      {n_antes - n_principal:>8,} puntos  "
            f"({100*(n_antes - n_principal)/n_antes:.1f}%)")

        # Si hay muchos clusters, listar los más grandes
        if len(counts) > 1:
            print(f"    tamaño de clusters:")
            for i, c in enumerate(sorted(counts, reverse=True)[:5]):
                marca = " ← principal" if i == 0 else ""
                print(f"      cluster {i+1}: {c:,} puntos{marca}")
    else:
        pcd_final = pcd_tmp
        print("  [PC] DBSCAN: sin clusters detectados, usando nube completa")

    # Guardar raw antes de alineacion
    pcd_raw_for_export = pcd_final.select_by_index(list(range(len(pcd_final.points))))
    out_raw = str(sd / 'mapa_final_raw.ply')
    o3d.io.write_point_cloud(out_raw, pcd_raw_for_export)
    print(f"  [PC] Guardado raw sin alineacion: {out_raw}")

    # Detectar suelo y alinear
    floor_inliers = None
    R_align       = np.eye(3)

    try:
        plane_model, inliers = pcd_final.segment_plane(
            distance_threshold=30.0, ransac_n=3, num_iterations=1000
        )
        floor_inliers = inliers
        a, b, c, d   = plane_model
        normal        = np.array([a, b, c], dtype=np.float64)
        normal       /= np.linalg.norm(normal)
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

        print(f"  [PC] Suelo detectado: normal={np.round(normal,3)}, inliers={len(inliers)}")

    except Exception as e:
        print(f"  [PC] segment_plane fallo: {e}")

    if floor_inliers is not None:
        pcd_walls = pcd_final.select_by_index(floor_inliers, invert=True)
    else:
        pcd_walls = pcd_final

    out_walls_raw = str(sd / 'mapa_paredes_raw.ply')
    o3d.io.write_point_cloud(out_walls_raw, pcd_walls)
    print(f"  [PC] Guardado paredes raw sin alineacion: {out_walls_raw}")

    center = pcd_final.get_center()
    pcd_final.rotate(R_align, center=center)
    pcd_walls.rotate(R_align, center=center)

    out_final = str(sd / 'mapa_final.ply')
    out_walls = str(sd / 'mapa_paredes.ply')

    o3d.io.write_point_cloud(out_final, pcd_final)
    o3d.io.write_point_cloud(out_walls, pcd_walls)

    print(f"\n  [PC] Bruto: {n_bruto:,} -> final: {len(pcd_final.points):,} puntos")
    print(f"  [PC] Guardado: {out_final}")
    print(f"  [PC] Guardado: {out_walls}")


# ==================================================================
#  MAIN
# ==================================================================

def main():
    print("=" * 58)
    print("  client_3.py - Stereo SLAM offline")
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
                'raft_dir': str(RAFT_DIR),
                'raft_model': str(RAFT_MODEL),
            }
            with open(sess['sd'] / 'session_info.json', 'w', encoding='utf-8') as f:
                json.dump(session_info, f, indent=2)
        except Exception as e:
            print(f"  AVISO No se pudo escribir session_info.json: {e}")

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
    print("> W/A/S/D = mover | Q = frenar | X = salir y procesar\n")

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

            key = cv2.waitKey(33) & 0xFF
            if key == ord('x'):
                request_shutdown()
                break

        cv2.destroyAllWindows()

    finally:
        stop_event.set()
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
