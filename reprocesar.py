"""
reprocesar.py - Regenera poses y nube de puntos de una sesion ya capturada,
sin el carrito y sin volver a correr RAFT (usa disparity_raw/*.npy guardados).

Mismas fases que el pipeline SfM:
  Fase 1  validar capturas  -> descarta frames borrosos (varianza del Laplaciano)
  Fase 2  sparse            -> odometria visual + nube sparse (puntos ORB)
  Fase 3  densa (opcional)  -> build_point_cloud con todos los pixeles

Uso:
  python reprocesar.py                       -> ultima sesion, sparse + densa
  python reprocesar.py --sparse              -> solo sparse (rapido, para ver la forma)
  python reprocesar.py captured_frames/session_XXXX --sparse
  python reprocesar.py --blur 0.8            -> mas estricto con frames borrosos
  python reprocesar.py --blur 0              -> no descartar frames borrosos
  python reprocesar.py --max-frames 100      -> solo los primeros N frames
  python reprocesar.py --icp                 -> prender el refinamiento ICP (para comparar)
  python reprocesar.py --loops               -> prender loop closure + PGO (experimental, ver visual_odometry.py)
  python reprocesar.py --sin-plano           -> odometria 6D, sin restriccion de piso (para comparar)
  python reprocesar.py --inclinacion 12      -> camara inclinada 12 grados hacia arriba
                                               (define el piso para el movimiento plano)

Salidas en la carpeta de la sesion:
  nube_sparse.ply, trayectoria.ply, poses.txt   (siempre)
  mapa_final.ply, mapa_paredes.ply              (solo sin --sparse)

Ver:  python ver_nube.py <sesion>/nube_sparse.ply <sesion>/trayectoria.ply
"""
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

from client_2 import load_rectification, build_point_cloud, SAVE_ROOT
import visual_odometry
from visual_odometry import VisualOdometry, guardar_poses

# Profundidad util para la nube sparse (mm). Con baseline de 6 cm,
# arriba de ~3 m el error por pixel de disparidad pasa de 17 cm.
# (2 m es mas preciso, pero en estas sesiones ~90% de los keypoints
# estan entre 2.1 y 2.6 m; se puede cambiar con --zmax)
SPARSE_Z_MIN = 100.0

# Puntos en bordes de profundidad: si la disparidad cambia mas de esto
# en una ventana 5x5 alrededor del keypoint, el punto se descarta
# (ORB detecta esquinas, que muchas veces caen justo en un borde).
EDGE_MAX_DISP = 2.0   # px

# Keyframes para la nube sparse: solo se agregan los puntos de un frame
# si la camara se movio o giro lo suficiente desde el ultimo keyframe.
# Sin esto, cuando el carrito esta quieto el mismo punto se agrega cientos
# de veces con ruido de profundidad y se ven "rayos" saliendo de la camara.
KF_MOV_MM  = 50.0
KF_ROT_DEG = 5.0


def nitidez(gray):
    # Varianza del Laplaciano: bordes nitidos = valor alto
    return cv2.Laplacian(gray, cv2.CV_64F).var()


def guardar_ply(ruta, pts, cols):
    import open3d as o3d
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
    pcd.colors = o3d.utility.Vector3dVector(cols.astype(np.float64))
    o3d.io.write_point_cloud(str(ruta), pcd)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('sesion', nargs='?', help='carpeta de la sesion (default: la ultima)')
    ap.add_argument('--sparse', action='store_true', help='solo nube sparse, sin densa')
    ap.add_argument('--blur', type=float, default=0.6,
                    help='descarta frames con nitidez < blur * mediana de la sesion (0 = no filtrar)')
    ap.add_argument('--max-frames', type=int, default=0, help='procesar solo N frames (0 = todos)')
    ap.add_argument('--icp', action='store_true', help='prender el refinamiento ICP de la VO')
    ap.add_argument('--loops', action='store_true', help='prender loop closure + PGO (experimental)')
    ap.add_argument('--inclinacion', type=float, default=None,
                    help='grados de la camara respecto al piso (+ = arriba); '
                         'default: el de session_info.json, o detectar el piso')
    ap.add_argument('--sin-plano', action='store_true',
                    help='no forzar movimiento plano (odometria de 6 grados de libertad)')
    ap.add_argument('--zmax', type=float, default=3000.0,
                    help='profundidad maxima (mm) de los puntos sparse')
    ap.add_argument('--todos-los-frames', action='store_true',
                    help='nube sparse con todos los frames, sin keyframes (para comparar)')
    a = ap.parse_args()
    visual_odometry.USE_ICP = a.icp
    visual_odometry.USE_LOOP_CLOSURE = a.loops
    visual_odometry.USE_PLANAR = not a.sin_plano

    sesiones = sorted(SAVE_ROOT.glob('session_*'))
    sd = Path(a.sesion) if a.sesion else (sesiones[-1] if sesiones else None)
    if sd is None or not (sd / 'session_info.json').exists():
        print(f"No encontre la sesion: {sd}")
        print("Sesiones disponibles:")
        for x in sesiones:
            print(f"  {x}")
        return
    print(f"Sesion: {sd}")

    # misma calibracion y resolucion que se usaron al capturar
    info = json.load(open(sd / 'session_info.json', encoding='utf-8'))
    lens_w, lens_h = info['lente_izquierdo_recibido']
    _, _, Q, K_rect = load_rectification(Path(info['cal_json']), lens_w, lens_h)

    disp_files = sorted((sd / 'disparity_raw').glob('disp_*.npy'))
    if a.max_frames > 0:
        disp_files = disp_files[:a.max_frames]

    # ------------------------------------------------------------------
    # Fase 1: nitidez de cada frame
    # Umbral relativo a la mediana de la sesion: un umbral fijo (ej. 100)
    # no sirve porque la nitidez cambia mucho con la luz y la camara.
    # ------------------------------------------------------------------
    t0 = time.perf_counter()
    nitideces = {}
    for dp in disp_files:
        idx = int(dp.stem.replace('disp_', ''))
        gray = cv2.imread(str(sd / 'left_rect' / f'left_{idx:06d}.jpg'), cv2.IMREAD_GRAYSCALE)
        if gray is not None:
            nitideces[idx] = nitidez(gray)
    valores = np.array(list(nitideces.values()))
    umbral = a.blur * np.median(valores)
    borrosos = int((valores < umbral).sum())
    print(f"\n[Fase 1] Nitidez: min={valores.min():.1f}  mediana={np.median(valores):.1f}  "
          f"max={valores.max():.1f}  -> umbral={umbral:.1f}")
    print(f"         {borrosos}/{len(valores)} frames descartados por borrosos (--blur {a.blur:g})")
    print(f"[TIEMPO] nitidez: {time.perf_counter() - t0:.1f} s")

    # ------------------------------------------------------------------
    # Fase 2: odometria visual + nube sparse
    # ------------------------------------------------------------------
    inclinacion = a.inclinacion
    if inclinacion is None:
        inclinacion = info.get('inclinacion_camara_deg')
    vo = VisualOdometry(Q, K_rect, inclinacion_deg=inclinacion)
    sparse = {}          # {frame_idx: (pts_cam Nx3, colores Nx3)}
    n_usados = validos = 0
    t_lectura = t_vo = 0.0
    primero = True

    for dp in disp_files:
        idx = int(dp.stem.replace('disp_', ''))

        if idx not in nitideces or nitideces[idx] < umbral:
            continue
        t0 = time.perf_counter()
        izq = cv2.imread(str(sd / 'left_rect' / f'left_{idx:06d}.jpg'))
        disp = np.load(str(dp))
        n_usados += 1
        t_lectura += time.perf_counter() - t0

        t0 = time.perf_counter()
        ok = vo.update(izq, disp, frame_idx=idx) is not None
        t_vo += time.perf_counter() - t0
        validos += ok

        # Solo frames con pose confiable (si VO fallo, su pose es la anterior
        # y sus puntos quedarian en el lugar equivocado)
        if ok or primero:
            uv, pts = vo.get_frame_points(idx)
            u = np.clip(uv[:, 0].astype(int), 0, izq.shape[1] - 1)
            v = np.clip(uv[:, 1].astype(int), 0, izq.shape[0] - 1)
            # rango de disparidad en 5x5 = max - min (dilate - erode)
            k = np.ones((5, 5), np.uint8)
            rango = cv2.dilate(disp, k) - cv2.erode(disp, k)
            z = pts[:, 2]
            m = (z > SPARSE_Z_MIN) & (z < a.zmax) & (rango[v, u] < EDGE_MAX_DISP)
            sparse[idx] = (pts[m], izq[v[m], u[m]][:, ::-1] / 255.0)   # BGR -> RGB
            primero = False

    print(f"[Fase 2] VO: {validos}/{n_usados} frames con pose valida")
    if a.sin_plano:
        print("         Movimiento plano: desactivado (--sin-plano)")
    elif vo.floor_info is None:
        print("         Movimiento plano: NO se encontro el piso, se uso 6D "
              "(usa --inclinacion GRADOS)")
    else:
        fi = vo.floor_info
        print(f"         Movimiento plano: {fi['origen']}, camara inclinada "
              f"{fi['inclinacion_deg']:+.1f} grados")
    print(f"[TIEMPO] lectura de imagen + disparidad: {t_lectura:.1f} s")
    print(f"[TIEMPO] VO (ORB + PnP{' + ICP' if a.icp else ''} + loops): {t_vo:.1f} s "
          f"({1000 * t_vo / max(n_usados, 1):.0f} ms por frame)")

    # ------------------------------------------------------------------
    # Nube sparse con las poses finales (despues de PGO)
    # ------------------------------------------------------------------
    t0 = time.perf_counter()
    poses = vo.get_all_poses()
    if not poses:
        print("Sin poses: revisa el umbral --blur")
        return
    guardar_poses(sd / 'poses.txt', poses)

    # keyframes con las poses finales
    kfs, T_kf = [], None
    for idx in sorted(sparse):
        T = poses[idx]
        if T_kf is not None and not a.todos_los_frames:
            rel = np.linalg.inv(T_kf) @ T
            mov = np.linalg.norm(rel[:3, 3])
            ang = np.degrees(np.linalg.norm(cv2.Rodrigues(rel[:3, :3])[0]))
            if mov < KF_MOV_MM and ang < KF_ROT_DEG:
                continue
        kfs.append(idx)
        T_kf = T

    pts_w, cols_w = [], []
    for idx in kfs:
        pts, cols = sparse[idx]
        T = poses[idx]
        pts_w.append(pts @ T[:3, :3].T + T[:3, 3])   # camara -> mundo
        cols_w.append(cols)
    pts_w = np.vstack(pts_w) if pts_w else np.zeros((0, 3))
    cols_w = np.vstack(cols_w) if cols_w else np.zeros((0, 3))
    guardar_ply(sd / 'nube_sparse.ply', pts_w, cols_w)

    # trayectoria: centro de cada camara como punto rojo
    centros = np.array([poses[i][:3, 3] for i in sorted(poses)])
    guardar_ply(sd / 'trayectoria.ply', centros, np.tile([1.0, 0.0, 0.0], (len(centros), 1)))

    recorrido = np.linalg.norm(np.diff(centros, axis=0), axis=1).sum() if len(centros) > 1 else 0

    # Sugerencia de inclinacion: plano que mejor ajusta a la trayectoria (PCA).
    # Solo tiene sentido en 6D y si el carro se movio en 2 direcciones (con giro).
    if vo.floor_info is None and len(centros) > 10:
        c = centros - centros.mean(axis=0)
        _, S, Vt = np.linalg.svd(c, full_matrices=False)
        if S[1] > 0.2 * S[0]:
            n = Vt[2] if Vt[2][1] > 0 else -Vt[2]
            print(f"[Fase 2] La trayectoria sugiere: --inclinacion {np.degrees(np.arcsin(-n[2])):.0f} "
                  f"(alabeo {np.degrees(np.arcsin(n[0])):+.0f} grados)")
        else:
            print("[Fase 2] Trayectoria casi recta: no se puede estimar la inclinacion (falta un giro)")
    print(f"[Fase 2] Nube sparse: {len(pts_w):,} puntos de {len(kfs)} keyframes "
          f"(de {len(sparse)} frames con pose)")
    print(f"         Recorrido VO: {recorrido / 1000:.2f} m   "
          f"desplazamiento neto: {np.linalg.norm(centros[-1] - centros[0]) / 1000:.2f} m")
    print(f"[TIEMPO] nube sparse: {time.perf_counter() - t0:.1f} s")
    print(f"         -> {sd / 'nube_sparse.ply'}")

    if a.sparse:
        return

    # ------------------------------------------------------------------
    # Fase 3: nube densa (todos los pixeles)
    # ------------------------------------------------------------------
    t0 = time.perf_counter()
    build_point_cloud({'sd': sd}, vo, Q)
    print(f"[TIEMPO] nube densa total: {time.perf_counter() - t0:.1f} s")


if __name__ == '__main__':
    main()
