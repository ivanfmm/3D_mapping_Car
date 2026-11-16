"""
nube.py - Nube de puntos densa a partir de disparidades + poses (Fase 3).

No depende de torch ni de RAFT: lo usan client_2.py y reprocesar.py.

Salidas en la carpeta de la sesion:
  poses.txt                  trayectoria (sistema de la primera camara)
  mapa_final_raw.ply         nube filtrada, SIN alinear (mismo sistema que poses.txt,
                             trayectoria.ply y nube_sparse.ply)
  mapa_paredes_raw.ply       igual, sin el piso
  mapa_final.ply             nube alineada al piso (Y = normal del piso)
  mapa_paredes.ply           igual, sin el piso
  trayectoria_alineada.ply   trayectoria en el mismo sistema que mapa_final.ply
  alineacion.json            rotacion aplicada (mundo -> alineado)
"""
import json
import time
from pathlib import Path

import cv2
import numpy as np

from visual_odometry import guardar_poses

SAVE_ROOT = Path(__file__).resolve().parent / 'captured_frames'

# ------------------------------------------------------------------
# PARAMETROS  (ajustados para mayor densidad de puntos)
# ------------------------------------------------------------------
DISP_MIN     = 2.0    # disparidad minima confiable
Z_MIN        = 100.0  # profundidad minima en mm (10 cm)
Z_MAX        = 3000.0 # profundidad maxima en mm (arriba de 3 m el error pasa de 17 cm por px)
VOXEL_SIZE   = 8.0    # tamano del cubo de fusion en mm
STAT_K       = 20     # vecinos para outlier estadistico
STAT_STD     = 2.0    # umbral sigma
RAD_POINTS   = 8      # vecinos minimos para outlier radial
RAD_RADIUS   = 40.0   # radio de busqueda en mm
FRAME_STRIDE = 1      # usar todos los frames
# Keyframes: un frame entra a la nube solo si se movio o giro lo suficiente
# desde el ultimo aceptado (si el carro esta quieto no se duplican puntos)
MIN_POSE_MOV = 50.0   # mm
MIN_POSE_ROT = 5.0    # grados
# Cada cuantos frames aceptados se compacta el acumulado con el mismo voxel
# del final. Sin esto el acumulado guarda ~900 mil puntos por frame hasta el
# final y se queda sin memoria en recorridos largos.
COMPACT_EVERY = 20
DBSCAN_EPS    = 60.0       # mm
DBSCAN_MIN_PTS = 20
DBSCAN_MAX    = 2_000_000  # arriba de esto DBSCAN se queda sin memoria y tumba el proceso
# Se conservan todos los clusters con al menos esta fraccion del principal.
# Antes solo se conservaba el principal y se perdia cualquier pared separada
# mas de DBSCAN_EPS del resto.
CLUSTER_MIN_FRAC = 0.02
# Plano del piso
FLOOR_DIST_MM   = 30.0  # distancia maxima al plano para ser inlier
FLOOR_MIN_COS   = 0.9   # |cos| minimo entre el plano y el piso de la VO (~25 grados)
FLOOR_MIN_NY    = 0.7   # sin piso de la VO: la normal debe ser casi el eje Y de la camara


def _rot_entre(a, b):
    """Rotacion minima que lleva el vector unitario a sobre b (Rodrigues)."""
    v = np.cross(a, b)
    s = np.linalg.norm(v)
    c = float(np.dot(a, b))
    if s < 1e-6:
        return np.eye(3)
    vx = np.array([
        [0,    -v[2],  v[1]],
        [v[2],  0,    -v[0]],
        [-v[1], v[0],  0   ],
    ])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / (s * s))


def build_point_cloud(sess, vo, Q):
    try:
        import open3d as o3d
    except ImportError:
        print("  [PC] open3d no disponible - pip install open3d")
        return

    if sess is None:
        return

    sd       = sess['sd']
    raw_dir  = sd / 'disparity_raw'
    left_dir = sd / 'left_rect'

    poses = vo.get_all_poses()

    poses_path = sd / 'poses.txt'
    guardar_poses(poses_path, poses)
    print(f"  [PC] Trayectoria: {poses_path}  ({len(poses)} poses)")

    all_frame_ids = sorted(poses.keys())
    frame_ids     = all_frame_ids[::FRAME_STRIDE]
    print(f"\n  [PC] Generando nube ({len(frame_ids)} frames, stride={FRAME_STRIDE})...")

    t_pc = time.perf_counter()
    pcd_combined = o3d.geometry.PointCloud()
    prev_T       = None
    processed    = 0
    skipped_pose = 0
    n_bruto      = 0
    first_done   = False

    for frame_idx in frame_ids:
        disp_path = raw_dir  / f'disp_{frame_idx:06d}.npy'
        left_path = left_dir / f'left_{frame_idx:06d}.jpg'

        if not disp_path.exists() or not left_path.exists():
            continue

        cur_T = poses[frame_idx]
        if prev_T is not None:
            # movimiento relativo respecto al ultimo frame aceptado
            rel = np.linalg.inv(prev_T) @ cur_T
            mov = np.linalg.norm(rel[:3, 3])
            ang = np.degrees(np.linalg.norm(cv2.Rodrigues(rel[:3, :3])[0]))
            # se descarta solo si casi no se traslado NI giro
            if mov < MIN_POSE_MOV and ang < MIN_POSE_ROT:
                skipped_pose += 1
                continue

        disp = np.load(str(disp_path)).astype(np.float32)
        img  = cv2.imread(str(left_path))
        if img is None:
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

        z        = pts3d[:, :, 2]
        total_px = disp.size
        m1 = disp > DISP_MIN       # disparidad minima
        m2 = np.isfinite(z)        # valores finitos
        m3 = z > Z_MIN             # profundidad minima
        m4 = z < Z_MAX             # profundidad maxima
        mask = m1 & m2 & m3 & m4

        detalle = processed % 10 == 0
        if detalle:
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

        pts_w = pts @ cur_T[:3, :3].T + cur_T[:3, 3]   # camara -> mundo

        valid_w = np.all(np.isfinite(pts_w), axis=1)
        pts_w   = pts_w[valid_w]
        cols    = cols[valid_w]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts_w)
        pcd.colors = o3d.utility.Vector3dVector(cols)

        # Filtro estadistico por frame (util para ruido local)
        n_antes = len(pcd.points)
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=STAT_K, std_ratio=STAT_STD)
        n_despues = len(pcd.points)

        if detalle:
            eliminados = n_antes - n_despues
            print(f"    outlier estadístico:     {eliminados:>8,} eliminados  "
                  f"({100*eliminados/n_antes:.1f}%)  "
                  f"quedan={n_despues:,}")

        if n_despues < 50:
            continue

        prev_T        = cur_T
        pcd_combined += pcd
        n_bruto      += n_despues
        processed    += 1

        # Compactar el acumulado con el mismo voxel del final (ver COMPACT_EVERY)
        if processed % COMPACT_EVERY == 0:
            pcd_combined = pcd_combined.voxel_down_sample(VOXEL_SIZE)

        if processed % 10 == 0:
            print(f"  [PC] {processed} acumulados ({len(pcd_combined.points):,} puntos en memoria), "
                  f"{skipped_pose} descartados por pose...")

    print(f"  [TIEMPO] frames -> nube (incluye outlier por frame): "
          f"{time.perf_counter() - t_pc:.1f} s")
    t_pc = time.perf_counter()
    print(f"  [PC] Resumen: {processed} acumulados, "
          f"{skipped_pose} descartados por moverse < {MIN_POSE_MOV}mm y girar < {MIN_POSE_ROT} grados")

    if len(pcd_combined.points) == 0:
        print("  [PC] Nube vacia, no se genera archivo")
        return

    # Voxel downsample final sobre el combinado completo
    n_antes = len(pcd_combined.points)
    pcd_combined = pcd_combined.voxel_down_sample(VOXEL_SIZE)
    print(f"\n  [PC] Voxel downsample:   {n_bruto:,} brutos -> {len(pcd_combined.points):,} "
          f"(ultimo paso: -{n_antes - len(pcd_combined.points):,} puntos)")

    # Radius outlier
    n_antes = len(pcd_combined.points)
    pcd_tmp, _ = pcd_combined.remove_radius_outlier(nb_points=RAD_POINTS, radius=RAD_RADIUS)
    print(f"  [PC] Radius outlier:     {n_antes:,} -> {len(pcd_tmp.points):,} "
          f"(-{n_antes - len(pcd_tmp.points):,} puntos)")

    n_antes = len(pcd_tmp.points)

    print(f"  [TIEMPO] voxel + radius outlier: {time.perf_counter() - t_pc:.1f} s")
    t_pc = time.perf_counter()

    if n_antes > DBSCAN_MAX:
        # nube demasiado grande: se salta DBSCAN para no crashear
        print(f"  [PC] DBSCAN omitido: {n_antes:,} puntos > {DBSCAN_MAX:,}")
        labels = np.array([-1])
    elif n_antes == 0:
        labels = np.array([-1])
    else:
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Error):
            labels = np.array(pcd_tmp.cluster_dbscan(
                eps=DBSCAN_EPS, min_points=DBSCAN_MIN_PTS, print_progress=False
            ))

    if labels.max() >= 0:
        counts   = np.bincount(labels[labels >= 0])
        main_lbl = int(counts.argmax())
        # clusters que se conservan: el principal y los que no son mucho mas chicos
        keep_lbls = np.where(counts >= CLUSTER_MIN_FRAC * counts[main_lbl])[0]
        keep_mask = np.isin(labels, keep_lbls)
        pcd_final = pcd_tmp.select_by_index(np.where(keep_mask)[0])

        n_ruido     = int(np.sum(labels < 0))
        n_principal = int(counts[main_lbl])
        n_conserva  = len(pcd_final.points)
        n_chicos    = n_antes - n_ruido - n_conserva

        print(f"\n  [PC] DBSCAN:")
        print(f"    clusters encontrados:  {len(counts)}")
        print(f"    cluster principal:     {n_principal:>8,} puntos  ({100*n_principal/n_antes:.1f}%)")
        print(f"    clusters conservados:  {len(keep_lbls)}  "
              f"(>= {100*CLUSTER_MIN_FRAC:.0f}% del principal) = {n_conserva:,} puntos")
        print(f"    ruido (sin cluster):   {n_ruido:>8,} puntos  ({100*n_ruido/n_antes:.1f}%)")
        print(f"    clusters chicos:       {n_chicos:>8,} puntos  ({100*n_chicos/n_antes:.1f}%)")
        print(f"    eliminados total:      {n_antes - n_conserva:>8,} puntos  "
              f"({100*(n_antes - n_conserva)/n_antes:.1f}%)")

        if len(counts) > 1:
            print(f"    tamaño de clusters:")
            for i, c in enumerate(sorted(counts, reverse=True)[:5]):
                marca = " <- principal" if i == 0 else ""
                print(f"      cluster {i+1}: {c:,} puntos{marca}")
    else:
        pcd_final = pcd_tmp
        print("  [PC] DBSCAN: sin clusters detectados, usando nube completa")

    print(f"  [TIEMPO] DBSCAN: {time.perf_counter() - t_pc:.1f} s")

    if len(pcd_final.points) == 0:
        print("  [PC] Nube vacia tras los filtros, no se genera archivo")
        return

    # Guardar raw antes de alineacion (mismo sistema que poses.txt)
    out_raw = str(sd / 'mapa_final_raw.ply')
    o3d.io.write_point_cloud(out_raw, pcd_final)
    print(f"  [PC] Guardado raw sin alineacion: {out_raw}")

    # ------------------------------------------------------------------
    # Piso: detectar el plano y comprobar que SI es el piso.
    # segment_plane devuelve el plano mas grande, que puede ser una pared.
    # Referencia: el piso que ya encontro la odometria (vo.A); el mundo es
    # el sistema de la primera camara, asi que su normal vale tal cual.
    # ------------------------------------------------------------------
    A_piso = getattr(vo, 'A', None)        # filas = ejes del piso (Y = normal, hacia abajo)
    floor_inliers = None
    normal_plano  = None

    try:
        plane_model, inliers = pcd_final.segment_plane(
            distance_threshold=FLOOR_DIST_MM, ransac_n=3, num_iterations=1000
        )
        normal  = np.array(plane_model[:3], dtype=np.float64)
        normal /= np.linalg.norm(normal)
        if normal[1] < 0:
            normal = -normal

        if A_piso is not None:
            cos_piso = abs(float(normal @ A_piso[1]))
            es_piso  = cos_piso > FLOOR_MIN_COS
            criterio = f"|cos| con el piso de la VO = {cos_piso:.2f}"
        else:
            es_piso  = normal[1] > FLOOR_MIN_NY
            criterio = f"componente Y de la normal = {normal[1]:.2f}"

        if es_piso:
            floor_inliers = inliers
            normal_plano  = normal
            print(f"  [PC] Suelo detectado: normal={np.round(normal,3)}, "
                  f"inliers={len(inliers)} ({criterio})")
        else:
            print(f"  [PC] El plano mas grande NO es el piso ({criterio}): "
                  f"normal={np.round(normal,3)}. No se quita de mapa_paredes.")

    except Exception as e:
        print(f"  [PC] segment_plane fallo: {e}")

    if floor_inliers is not None:
        pcd_walls = pcd_final.select_by_index(floor_inliers, invert=True)
    else:
        pcd_walls = pcd_final.select_by_index(np.arange(len(pcd_final.points)))

    out_walls_raw = str(sd / 'mapa_paredes_raw.ply')
    o3d.io.write_point_cloud(out_walls_raw, pcd_walls)
    print(f"  [PC] Guardado paredes raw sin alineacion: {out_walls_raw}")

    # Rotacion mundo -> alineado (Y = normal del piso)
    if A_piso is not None:
        R_align, origen_align = np.array(A_piso, dtype=np.float64), 'piso de la odometria'
    elif normal_plano is not None:
        R_align, origen_align = _rot_entre(normal_plano, np.array([0.0, 1.0, 0.0])), 'plano de la nube'
    else:
        R_align, origen_align = np.eye(3), 'ninguna (no se encontro el piso)'
    print(f"  [PC] Alineacion: {origen_align}")

    # Se rota alrededor del ORIGEN (la primera camara), no del centro de la nube:
    # asi la misma rotacion sirve para la trayectoria y todo queda en un solo sistema.
    origen = np.zeros(3)
    pcd_final.rotate(R_align, center=origen)
    pcd_walls.rotate(R_align, center=origen)

    out_final = str(sd / 'mapa_final.ply')
    out_walls = str(sd / 'mapa_paredes.ply')
    out_tray  = str(sd / 'trayectoria_alineada.ply')

    o3d.io.write_point_cloud(out_final, pcd_final)
    o3d.io.write_point_cloud(out_walls, pcd_walls)

    centros = np.array([poses[i][:3, 3] for i in sorted(poses)]) @ R_align.T
    tray = o3d.geometry.PointCloud()
    tray.points = o3d.utility.Vector3dVector(centros)
    tray.colors = o3d.utility.Vector3dVector(np.tile([1.0, 0.0, 0.0], (len(centros), 1)))
    o3d.io.write_point_cloud(out_tray, tray)

    with open(sd / 'alineacion.json', 'w', encoding='utf-8') as f:
        json.dump({'origen': origen_align, 'R_mundo_a_alineado': R_align.tolist()}, f, indent=2)

    print(f"\n  [PC] Bruto: {n_bruto:,} -> final: {len(pcd_final.points):,} puntos")
    print(f"  [PC] Guardado: {out_final}")
    print(f"  [PC] Guardado: {out_walls}")
    print(f"  [PC] Guardado: {out_tray}  (mismo sistema que mapa_final.ply)")
