"""
visual_odometry.py - Odometria Visual Estereo con Loop Closure
==============================================================
Usa ORB + BFMatcher + solvePnPRansac para estimar movimiento frame a frame.
Detecta loop closures con FAISS (descriptores binarios ORB, distancia Hamming)
y optimiza el grafo de poses con scipy.optimize (L-BFGS-B).

API publica:
    vo     = VisualOdometry(Q, K_rect)
    result = vo.update(frame, disp, frame_idx=None)  -> (R, t) o None
    poses  = vo.get_all_poses()                       -> {frame_idx: 4x4}

FIXES originales preservados:
  - _deproject: squeeze defensivo para disp con dims extra (1,H,W) etc.
  - inliers: ravel + cast bool para evitar "too many indices"
"""

import cv2
import numpy as np
import faiss
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix


# ==================================================================
#  Constantes de loop closure y PGO
# ==================================================================
MIN_LOOP_SEP      = 80    # frames minimos de separacion para loop closure \eran 15
MIN_LOOP_VOTES    = 8    # votos FAISS minimos para considerar candidato \era 12
MIN_LOOP_INLIERS  = 6     # inliers PnP para verificacion geometrica de loop \era 8
LOOP_CHECK_EVERY  = 1     # buscar loop closure en CADA frame
LOOP_PGO_COOLDOWN = 30    # frames minimos entre corridas consecutivas de PGO
PGO_WEIGHT_LOOP   = 5.0   # peso de edges de loop vs edges secuenciales
PGO_MAX_ITER      = 50    # evaluaciones maximas de least_squares
PGO_ROT_SIGMA_DEG = 1.0   # error de rotacion 'normal' por arista (grados)
PGO_TRANS_SIGMA   = 10.0  # error de traslacion 'normal' por arista (mm)

# Deteccion de saltos de odometria
JUMP_MAX_TRANS  = 300.0   # mm  - maximo desplazamiento aceptable por frame
JUMP_MAX_ROT    = 30.0    # grados - maxima rotacion aceptable por frame
MAX_FALLOS      = 5       # frames fallidos seguidos (salto, PnP, matches...) antes de reiniciar la referencia

# Parametros de matching y PnP
LOWE_RATIO      = 0.75    # ratio test de Lowe para filtrar matches ORB
PNP_ITERATIONS  = 200     # iteraciones RANSAC de solvePnPRansac
PNP_REPROJ_ERR  = 2.0     # error de reproyeccion en pixeles (640x360)
MIN_PNP_MATCHES = 12      # matches minimos tras ratio test para intentar PnP
MIN_PNP_INLIERS = 12      # inliers minimos para aceptar resultado de PnP

# Loop closure - filtro de calidad FAISS
LC_HAMMING_MAX  = 80      # distancia Hamming maxima para contar voto valido \ era 64

# Loop closure APAGADO por default (18-oct-2026).
# Antes nunca se disparaba: FAISS busca solo el vecino mas cercano (k=1) y casi
# siempre caia en un frame reciente (< MIN_LOOP_SEP), que se descartaba. En
# session_20261018_114813: 476k huellas en el indice y 0 loops.
# Al meter solo keyframes al indice si empezo a dispararse, y salieron loops
# FALSOS (ej. 85->180 con rotacion de 142 grados, imposible si ambas vistas
# comparten escena con ~72 grados de campo de vision). Con 6 inliers PnP no
# basta para verificar y el PGO sin perdida robusta se desarma (recorrido de
# 4 m -> 90 m). Prender solo para experimentar: reprocesar.py --loops
USE_LOOP_CLOSURE = False

# Loop closure - solo los keyframes entran al indice FAISS.
# Antes se agregaban las ~1000 huellas de TODOS los frames: el indice crecia sin
# limite (costo O(F^2)) y, con el carro quieto, se llenaba de copias casi iguales.
LC_KF_MOV_MM    = 50.0    # mm que debe avanzar el carro desde el ultimo keyframe agregado
LC_KF_ROT_DEG   = 5.0     # o grados que debe girar

# ICP frame a frame
# Apagado por default: con el carrito QUIETO (session_20261018_092006,
# frames 0-39) la VO con ICP marco 0.79 m de recorrido falso y sin ICP 0.04 m.
# El piso domina la nube y el ICP "patina" sobre el plano. Ademas era el 80%
# del tiempo de VO (220 ms -> 45 ms por frame sin ICP).
USE_ICP       = False

# Restriccion de movimiento plano (el carrito va sobre el piso)
# El carro solo puede avanzar, moverse de lado y girar (3 grados de libertad).
# Subir/bajar, cabecear y alabear se fuerzan a cero en cada paso.
USE_PLANAR        = True
FLOOR_BAND        = 0.35    # franja de abajo de la imagen donde se busca el piso
FLOOR_Z_MAX       = 2000.0  # mm - solo piso cercano (mas preciso)
FLOOR_RANSAC_MM   = 10.0    # distancia maxima al plano para ser inlier
FLOOR_MIN_RATIO   = 0.4     # fraccion minima de inliers para aceptar el plano
FLOOR_MAX_TRIES   = 30      # frames en los que se intenta detectar el piso
ICP_MAX_DIST  = 100.0   # mm - distancia maxima de correspondencia
ICP_MAX_PTS   = 4000    # puntos por nube tras submuestreo (velocidad)
ICP_MIN_FIT   = 0.25    # fitness minimo para aceptar resultado de ICP
ICP_MAX_ITER  = 30      # iteraciones maximas de ICP
ICP_Z_MIN     = 100.0   # mm - profundidad minima para ICP
ICP_Z_MAX     = 8000.0  # mm - profundidad maxima para ICP
ICP_DISP_MIN  = 2.0     # disparidad minima para incluir punto en ICP


# ==================================================================
#  Utilidades de poses SE(3)
# ==================================================================

def guardar_poses(ruta, poses):
    """poses.txt: posicion (mm) y rotacion como vector de Rodrigues (grados)."""
    with open(str(ruta), 'w') as f:
        f.write("frame_idx  X_mm  Y_mm  Z_mm  rx_deg  ry_deg  rz_deg\n")
        for fid in sorted(poses):
            x, y, z = poses[fid][:3, 3]
            r = np.degrees(cv2.Rodrigues(poses[fid][:3, :3])[0].ravel())
            f.write(f"{fid:06d} {x:12.3f} {y:12.3f} {z:12.3f} "
                    f"{r[0]:9.3f} {r[1]:9.3f} {r[2]:9.3f}\n")


def mat_to_vec(T):
    """4x4 homogenea -> [rx, ry, rz, tx, ty, tz] (Rodrigues + traslacion)."""
    rvec, _ = cv2.Rodrigues(T[:3, :3])
    return np.concatenate([rvec.ravel(), T[:3, 3]])


def vec_to_mat(v):
    """[rx, ry, rz, tx, ty, tz] -> 4x4 homogenea."""
    R, _ = cv2.Rodrigues(v[:3].reshape(3, 1))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = v[3:]
    return T


# ==================================================================
#  Detector de loop closure con FAISS (IndexBinaryFlat)
# ==================================================================

class _LoopDetector:
    """
    Mantiene un indice FAISS de descriptores ORB binarios (256 bits = 32 bytes).
    Usa distancia Hamming para encontrar descriptores similares entre el frame
    actual y frames almacenados con separacion >= MIN_LOOP_SEP.
    """

    def __init__(self):
        self.index     = faiss.IndexBinaryFlat(256)
        self._frame_of = []   # posicion en index -> frame_idx

    def add(self, frame_idx, descriptors):
        """Agrega todos los descriptores de un frame al indice."""
        if descriptors is None or len(descriptors) == 0:
            return
        des = np.ascontiguousarray(descriptors, dtype=np.uint8)
        self.index.add(des)
        self._frame_of.extend([frame_idx] * len(des))

    def query(self, descriptors, current_idx):
        """
        Busca el frame pasado mas similar al actual por voto de descriptores.

        Retorna (best_frame_idx, vote_count) o (None, 0) si no hay candidato
        con suficiente separacion.
        """
        if descriptors is None or len(descriptors) == 0:
            return None, 0
        if self.index.ntotal == 0:
            return None, 0

        des = np.ascontiguousarray(descriptors, dtype=np.uint8)
        D, I = self.index.search(des, 1)   # D: distancias Hamming, I: indices

        votes = {}
        for (dist,), (nn_id,) in zip(D, I):
            if nn_id < 0:
                continue
            if int(dist) > LC_HAMMING_MAX:
                continue   # match de baja calidad: distancia > 64/256 bits
            fid = self._frame_of[nn_id]
            if current_idx - fid < MIN_LOOP_SEP:
                continue
            votes[fid] = votes.get(fid, 0) + 1

        if not votes:
            return None, 0
        best = max(votes, key=votes.get)
        return best, votes[best]


# ==================================================================
#  Optimizacion del grafo de poses con scipy
# ==================================================================

def _optimize_pose_graph(all_poses, seq_edges, loop_edges):
    """
    Optimiza el grafo de poses con minimos cuadrados (scipy least_squares).

    Cada arista aporta 6 residuos:
      - error de rotacion (rad) / PGO_ROT_SIGMA
      - error de traslacion (mm) / PGO_TRANS_SIGMA
    Al dividir entre la sigma ambos quedan sin unidades, asi los mm ya no
    dominan a los radianes.

    jac_sparsity le dice a scipy que cada arista solo depende de 2 poses,
    asi el jacobiano numerico cuesta ~12-18 evaluaciones y no 6*N.

    all_poses  : {frame_idx: 4x4}
    seq_edges  : [(i, j, T_rel_4x4), ...]   peso 1.0
    loop_edges : [(i, j, T_rel_4x4), ...]   peso PGO_WEIGHT_LOOP
    La primera pose queda fija (no es variable).
    Retorna {frame_idx: 4x4} optimizado.
    """
    frame_ids = sorted(all_poses.keys())
    if len(frame_ids) < 2:
        return all_poses

    fixed_id = frame_ids[0]
    var_ids  = frame_ids[1:]
    col_of   = {fid: 6 * k for k, fid in enumerate(var_ids)}  # columna de cada pose
    T_fixed  = all_poses[fixed_id]

    # (i, j, inv(T_meas), sqrt(peso)); se precalcula la inversa una vez
    edges = []
    for lst, w in ((seq_edges, 1.0), (loop_edges, PGO_WEIGHT_LOOP)):
        for i, j, T_meas in lst:
            if i in all_poses and j in all_poses:
                edges.append((i, j, np.linalg.inv(T_meas), np.sqrt(w)))
    if not edges:
        return all_poses

    rot_sigma = np.radians(PGO_ROT_SIGMA_DEG)

    def pose_of(x, fid):
        if fid == fixed_id:
            return T_fixed
        c = col_of[fid]
        return vec_to_mat(x[c:c+6])

    def residuals(x):
        r = np.empty(6 * len(edges))
        for e, (i, j, T_meas_inv, sw) in enumerate(edges):
            # error = medido^-1 * estimado; si coinciden es la identidad
            T_err = T_meas_inv @ np.linalg.inv(pose_of(x, i)) @ pose_of(x, j)
            rvec, _ = cv2.Rodrigues(T_err[:3, :3])
            r[6*e   : 6*e+3] = sw * rvec.ravel() / rot_sigma
            r[6*e+3 : 6*e+6] = sw * T_err[:3, 3] / PGO_TRANS_SIGMA
        return r

    # Patron de no-ceros del jacobiano: la arista e solo depende de i y j
    sparsity = lil_matrix((6 * len(edges), 6 * len(var_ids)), dtype=np.int8)
    for e, (i, j, _, _) in enumerate(edges):
        for fid in (i, j):
            if fid != fixed_id:
                c = col_of[fid]
                sparsity[6*e : 6*e+6, c : c+6] = 1

    x0 = np.concatenate([mat_to_vec(all_poses[fid]) for fid in var_ids])

    res = least_squares(
        residuals, x0,
        jac_sparsity=sparsity,
        method='trf',
        x_scale='jac',     # escala cada variable segun su efecto (rad vs mm)
        max_nfev=PGO_MAX_ITER,
    )

    out = {fixed_id: T_fixed.copy()}
    for fid in var_ids:
        c = col_of[fid]
        out[fid] = vec_to_mat(res.x[c:c+6])
    return out


# ==================================================================
#  Clase principal
# ==================================================================

class VisualOdometry:
    def __init__(self, Q, K, inclinacion_deg=None):
        """
        Q : matriz de reproyeccion disparidad->3D (de cv2.stereoRectify)
        K : matriz intrinseca del lente izquierdo rectificado (P1[:3,:3])
        inclinacion_deg : angulo de la camara respecto al piso (+ = hacia arriba).
            Si se da, define el plano del piso para el movimiento plano.
            Si es None, se intenta detectar el piso en las primeras imagenes.
        """
        self.Q = Q
        self.K = K

        # nlevels=6: para 640x360 la piramide de 8 niveles llega a ~80px (inutil)
        # fastThreshold=10: detecta mas esquinas en zonas de poca textura
        self.orb = cv2.ORB_create(
            nfeatures    = 1000,
            scaleFactor  = 1.2,
            nlevels      = 6,
            edgeThreshold= 19,
            patchSize    = 19,
            fastThreshold= 10,
        )
        # Sin crossCheck: usamos ratio test de Lowe con knnMatch (mas discriminativo)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

        self.prev_frame = None
        self.prev_kp    = None
        self.prev_des   = None
        self.prev_disp  = None

        # Pose acumulada en el mundo (matriz 4x4 homogenea)
        self.pose = np.eye(4)

        # Grafo de poses
        self.all_poses  = {}   # {frame_idx: 4x4}
        self.seq_edges  = []   # [(i, j, T_rel_4x4), ...]
        self.loop_edges = []   # [(i, j, T_rel_4x4), ...]

        # Historial de keyframes para verificacion geometrica de loops
        # Solo descriptores + puntos 3D de los keypoints (~45 KB por frame).
        # Antes se guardaba la disparidad completa (~3.7 MB por frame).
        self._kf_data = {}   # {frame_idx: {'des', 'pts3d'}}

        self._loop_detector  = _LoopDetector()
        self._prev_idx       = None
        self._internal_cnt   = 0
        self._last_pgo_frame = 0
        self._fallos         = 0     # frames fallidos seguidos desde el ultimo bueno
        self._last_lc_pose   = None  # pose del ultimo keyframe agregado al indice de loops

        # Piso: A = rotacion camara -> sistema del piso (eje Y perpendicular al piso)
        self.A           = None
        self.floor_info  = None   # {'origen', 'inclinacion_deg', 'alabeo_deg', ...}
        self._floor_tries = 0
        if USE_PLANAR and inclinacion_deg is not None:
            # camara inclinada th hacia arriba, sin alabeo: abajo = (0, cos th, -sin th)
            th = np.radians(inclinacion_deg)
            self._set_floor(np.array([0.0, np.cos(th), -np.sin(th)]), {'origen': 'parametro'})

    # ------------------------------------------------------------------
    #  API publica
    # ------------------------------------------------------------------

    def update(self, frame, disp, frame_idx=None):
        """
        Estima la transformacion R, t respecto al frame anterior y detecta
        loop closures con optimizacion de grafo de poses.

        Parametros
        ----------
        frame     : np.ndarray  imagen rectificada izquierda (BGR o grayscale)
        disp      : np.ndarray  mapa de disparidad de RAFT; admite (H,W),
                                (1,H,W) o (1,1,H,W), se normaliza internamente
        frame_idx : int opcional; si es None se usa un contador interno

        Retorna (R, t) si hay suficientes inliers, None en caso contrario.
        """
        if frame_idx is None:
            frame_idx = self._internal_cnt
        self._internal_cnt += 1

        # -- Preprocesar entrada ----------------------------------------
        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame

        disp = self._ensure_2d(disp)

        # -- Detectar el piso (solo hasta encontrarlo) ------------------
        if USE_PLANAR and self.A is None and self._floor_tries < FLOOR_MAX_TRIES:
            self._floor_tries += 1
            self._detect_floor(disp, frame_idx)

        # -- Extraer features -------------------------------------------
        kp, des = self.orb.detectAndCompute(gray, None)

        # Guardar datos del keyframe antes de cualquier retorno anticipado
        if kp:
            uv_kp    = np.float32([k.pt for k in kp])
            pts3d_kp = self._deproject(uv_kp, disp)
        else:
            uv_kp    = np.zeros((0, 2), dtype=np.float32)
            pts3d_kp = np.zeros((0, 3), dtype=np.float32)
        self._kf_data[frame_idx] = {'des': des, 'pts3d': pts3d_kp, 'uv': uv_kp}
        self.all_poses[frame_idx] = self.pose.copy()

        # -- Primer frame (o referencia sin descriptores): solo guardamos estado --
        if self.prev_frame is None or self.prev_des is None:
            self._save_state(gray, kp, des, disp)
            self._prev_idx = frame_idx
            self._add_loop_keyframe(frame_idx, des)
            return None

        if des is None or len(des) < 8:
            print(f"Frame  failed des < 8")
            self._fallo(gray, kp, des, disp, frame_idx)
            return None

        # -- Ratio test de Lowe (reemplaza crossCheck) ----------------
        raw = self.matcher.knnMatch(self.prev_des, des, k=2)
        matches = [
            pair[0] for pair in raw
            if len(pair) == 2 and pair[0].distance < LOWE_RATIO * pair[1].distance
        ]
        if len(matches) < MIN_PNP_MATCHES:
            print(f"Frame  failed matches < MIN_PNP_MATCHES")
            self._fallo(gray, kp, des, disp, frame_idx)
            return None

        # -- Coordenadas 2D de correspondencias -----------------------
        pts_prev_2d = np.float32([self.prev_kp[m.queryIdx].pt for m in matches])
        pts_curr_2d = np.float32([kp[m.trainIdx].pt           for m in matches])

        # -- Proyectar puntos anteriores a 3D usando su disparidad ----
        pts3d = self._deproject(pts_prev_2d, self.prev_disp)
        valid = pts3d[:, 2] > 0
        if valid.sum() < MIN_PNP_INLIERS:
            print(f"Frame  failed valid.sum() < MIN_PNP_INLIERS")
            self._fallo(gray, kp, des, disp, frame_idx)
            return None

        pts3d       = pts3d[valid]
        pts_curr_2d = pts_curr_2d[valid]

        # -- PnP RANSAC -----------------------------------------------
        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            pts3d, pts_curr_2d, self.K, None,
            iterationsCount=PNP_ITERATIONS,
            reprojectionError=PNP_REPROJ_ERR,
        )

        if not success or inliers is None or len(inliers) < MIN_PNP_INLIERS:
            print(f"Frame  failed not success or no inliers or inliers < MIN_PNP_INLIERS")
            self._fallo(gray, kp, des, disp, frame_idx)
            return None

        R, _ = cv2.Rodrigues(rvec)
        tvec_raw  = tvec.flatten()
        tvec_mag  = float(np.linalg.norm(tvec_raw))
        angle_deg = float(np.degrees(np.linalg.norm(rvec.flatten())))

        # -- Deteccion de saltos de odometria -------------------------
        if tvec_mag > JUMP_MAX_TRANS or angle_deg > JUMP_MAX_ROT:
            print(f"  [VO] SALTO frame {frame_idx}: "
                  f"|t|={tvec_mag:.0f}mm rot={angle_deg:.1f}deg "
                  f"({self._fallos + 1} fallos seguidos)")
            self._fallo(gray, kp, des, disp, frame_idx)
            return None

        self._fallos = 0

        # -- Construir T y refinar con ICP (prev->curr) ---------------
        T = np.eye(4)
        T[:3, :3] = R
        T[:3,  3] = tvec_raw
        if USE_ICP:
            T = self._icp_refine(T, self.prev_disp, disp)
        if USE_PLANAR and self.A is not None:
            # T mapea prev->curr; el movimiento de la camara es su inversa
            T = np.linalg.inv(self._to_planar(np.linalg.inv(T)))

        # -- Acumular pose (T mapea prev_cam->curr_cam, pose es cam->world) ---
        self.pose = self.pose @ np.linalg.inv(T)
        self.all_poses[frame_idx] = self.pose.copy()

        # Edge secuencial entre frame previo y actual
        if self._prev_idx is not None and self._prev_idx in self.all_poses:
            T_rel = np.linalg.inv(self.all_poses[self._prev_idx]) @ self.pose
            self.seq_edges.append((self._prev_idx, frame_idx, T_rel))

        self._add_loop_keyframe(frame_idx, des)

        # -- Loop closure (cada LOOP_CHECK_EVERY frames) --------------
        if USE_LOOP_CLOSURE and self._internal_cnt % LOOP_CHECK_EVERY == 0:
            self._check_loop_closure(frame_idx, kp, des, disp)

        self._save_state(gray, kp, des, disp)
        self._prev_idx = frame_idx
        return R, tvec.flatten()

    def get_frame_points(self, frame_idx):
        """
        Puntos ORB de un frame (nube sparse):
          uv    : (N, 2) pixeles en la imagen izquierda rectificada
          pts3d : (N, 3) mm en coordenadas de la camara de ese frame
                  (z = 0 si no hubo disparidad valida)
        """
        kf = self._kf_data.get(frame_idx)
        if kf is None:
            return np.zeros((0, 2), np.float32), np.zeros((0, 3), np.float32)
        return kf['uv'], kf['pts3d']

    # ------------------------------------------------------------------
    #  Movimiento plano
    # ------------------------------------------------------------------

    def _detect_floor(self, disp, frame_idx):
        """
        Ajusta un plano (RANSAC) a los puntos cercanos de la franja de abajo.
        Si es piso (normal casi vertical y suficientes inliers) guarda self.A.
        """
        h, w = disp.shape
        y0 = int(h * (1 - FLOOR_BAND))
        d  = disp[y0::4, ::4].astype(np.float32).ravel()
        vs, us = np.mgrid[y0:h:4, 0:w:4]
        uvd1 = np.stack([us.ravel(), vs.ravel(), d, np.ones_like(d)], axis=1)
        uvd1 = uvd1[d > 2.0]
        if len(uvd1) < 500:
            return False
        P = (self.Q @ uvd1.T).T
        pts = P[:, :3] / P[:, 3:4]
        pts = pts[(pts[:, 2] > 100) & (pts[:, 2] < FLOOR_Z_MAX)]
        if len(pts) < 500:
            return False

        # RANSAC: el plano con mas puntos a menos de FLOOR_RANSAC_MM
        rng = np.random.default_rng(0)
        best = None
        for _ in range(200):
            p = pts[rng.choice(len(pts), 3, replace=False)]
            n = np.cross(p[1] - p[0], p[2] - p[0])
            if np.linalg.norm(n) < 1e-6:
                continue
            n /= np.linalg.norm(n)
            inl = np.abs((pts - p[0]) @ n) < FLOOR_RANSAC_MM
            if best is None or inl.sum() > best.sum():
                best = inl
        if best is None or best.mean() < FLOOR_MIN_RATIO:
            return False

        # Refinar con todos los inliers: normal = vector singular mas chico
        q  = pts[best]
        c  = q.mean(axis=0)
        n  = np.linalg.svd(q - c)[2][2]
        if n[1] < 0:
            n = -n                       # que apunte hacia abajo (como el eje Y de la camara)
        if n[1] < 0.7:
            return False                 # no es piso: plano muy inclinado (>45 grados)

        self._set_floor(n, {'origen': f'piso detectado en frame {frame_idx}',
                            'altura_mm': float(abs(n @ c))})
        return True

    def _set_floor(self, n, info):
        """n: normal del piso apuntando hacia abajo, en coords de la camara."""
        n = n / np.linalg.norm(n)
        # Sistema del piso: Y = normal (abajo), Z = frente de la camara sobre el piso
        y_f = n
        z_f = np.array([0.0, 0.0, 1.0]) - n[2] * n
        z_f /= np.linalg.norm(z_f)
        x_f = np.cross(y_f, z_f)
        self.A = np.stack([x_f, y_f, z_f])   # filas = ejes -> camara a piso

        info['inclinacion_deg'] = float(np.degrees(np.arcsin(-n[2])))  # + = hacia arriba
        info['alabeo_deg']      = float(np.degrees(np.arcsin(n[0])))
        self.floor_info = info
        print(f"  [VO] Movimiento plano ({info['origen']}): inclinacion "
              f"{info['inclinacion_deg']:+.1f} grados, alabeo {info['alabeo_deg']:+.1f} grados")

    def _to_planar(self, M):
        """
        M: pose de la camara actual en coords de la camara anterior (4x4).
        Lo pasa al sistema del piso, se queda solo con el giro sobre la
        normal (yaw) y la traslacion sobre el piso, y lo regresa.
        """
        A = self.A
        R = A @ M[:3, :3] @ A.T
        t = A @ M[:3, 3]
        # giro sobre Y: R_y = [[c,0,s],[0,1,0],[-s,0,c]]
        yaw = np.arctan2(R[0, 2] - R[2, 0], R[0, 0] + R[2, 2])
        c, s = np.cos(yaw), np.sin(yaw)
        R_p = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        t_p = np.array([t[0], 0.0, t[2]])   # sin subir ni bajar
        out = np.eye(4)
        out[:3, :3] = A.T @ R_p @ A
        out[:3, 3]  = A.T @ t_p
        return out

    def get_all_poses(self):
        """Retorna copia del dict {frame_idx: 4x4} con todas las poses."""
        return {k: v.copy() for k, v in self.all_poses.items()}

    # ------------------------------------------------------------------
    #  Loop closure
    # ------------------------------------------------------------------

    def _check_loop_closure(self, current_idx, kp_curr, des_curr, disp_curr):
        candidate_idx, votes = self._loop_detector.query(des_curr, current_idx)
        if candidate_idx is None or votes < MIN_LOOP_VOTES:
            return

        kf = self._kf_data.get(candidate_idx)
        if kf is None or kf['des'] is None:
            return

        # Evitar loops casi duplicados: mismo candidato detectado hace menos
        # de LOOP_PGO_COOLDOWN frames no aporta informacion nueva
        if any(
            e[0] == candidate_idx and
            abs(e[1] - current_idx) < LOOP_PGO_COOLDOWN
            for e in self.loop_edges
        ):
            return

        T_rel = self._verify_loop(
            kf['des'], kf['pts3d'],
            kp_curr, des_curr,
        )
        if T_rel is None:
            return

        print(f"  [LC] Loop closure: frame {candidate_idx} <-> "
              f"{current_idx} ({votes} votos)")

        self.loop_edges.append((candidate_idx, current_idx, T_rel))

        # Cooldown: no ejecutar PGO mas de una vez cada LOOP_PGO_COOLDOWN frames
        if self._internal_cnt - self._last_pgo_frame >= LOOP_PGO_COOLDOWN:
            self._last_pgo_frame = self._internal_cnt
            self._run_pgo()

    def _verify_loop(self, des_a, pts3d_a, kp_b, des_b):
        """
        Verificacion geometrica del loop: matchea descriptores entre el frame
        candidato (a) y el actual (b), luego verifica con PnP RANSAC.
        Retorna T_rel 4x4 en la misma convencion que los edges secuenciales
        (inv(P_a) @ P_b, es decir mapea cam_b -> cam_a) o None si falla.
        """
        if des_a is None or des_b is None:
            return None

        raw = self.matcher.knnMatch(des_a, des_b, k=2)
        matches = [
            pair[0] for pair in raw
            if len(pair) == 2 and pair[0].distance < LOWE_RATIO * pair[1].distance
        ]
        if len(matches) < MIN_LOOP_INLIERS:
            return None

        # pts3d_a ya viene deproyectado (mismo orden que des_a)
        pts3d    = pts3d_a[[m.queryIdx for m in matches]]
        pts_b_2d = np.float32([kp_b[m.trainIdx].pt for m in matches])

        valid = pts3d[:, 2] > 0
        if valid.sum() < MIN_LOOP_INLIERS:
            return None

        pts3d    = pts3d[valid]
        pts_b_2d = pts_b_2d[valid]

        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            pts3d, pts_b_2d, self.K, None,
            iterationsCount=150,
            reprojectionError=2.0,
        )
        if not success or inliers is None or len(inliers) < MIN_LOOP_INLIERS:
            return None

        R_lc, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R_lc
        T[:3,  3] = tvec.flatten()
        # PnP da a->b; el grafo usa inv(P_a) @ P_b (b->a), por eso se invierte
        M = np.linalg.inv(T)
        if USE_PLANAR and self.A is not None:
            M = self._to_planar(M)
        return M

    def _run_pgo(self):
        print(f"  [PGO] Optimizando {len(self.all_poses)} poses, "
              f"{len(self.loop_edges)} loop(s)...")
        optimized  = _optimize_pose_graph(
            self.all_poses, self.seq_edges, self.loop_edges
        )
        self.all_poses = optimized
        # Sincronizar pose acumulada con la del ultimo frame optimizado
        if self.all_poses:
            self.pose = self.all_poses[max(self.all_poses)].copy()
        print("  [PGO] Listo")

    # ------------------------------------------------------------------
    #  ICP frame a frame
    # ------------------------------------------------------------------

    def _icp_refine(self, T_init, disp_prev, disp_curr):
        """
        Refina T_init (prev->curr) con ICP point-to-point entre las nubes
        de puntos generadas desde los mapas de disparidad consecutivos.

        source = pcd_prev (en frame de camara anterior)
        target = pcd_curr (en frame de camara actual)
        init   = T_init   (estimado por PnP, mapea prev->curr)

        Retorna T_refined 4x4 (prev->curr) o T_init si ICP falla/diverge.
        """
        try:
            import open3d as o3d
        except ImportError:
            return T_init

        def _disp_to_pcd(disp):
            pts3d = cv2.reprojectImageTo3D(
                disp.astype(np.float32), self.Q
            )                                          # (H, W, 3)
            z    = pts3d[:, :, 2]
            mask = (
                (disp > ICP_DISP_MIN) &
                np.isfinite(z)        &
                (z > ICP_Z_MIN)       &
                (z < ICP_Z_MAX)
            )
            pts = pts3d[mask].astype(np.float64)
            if len(pts) > ICP_MAX_PTS:
                idx = np.random.choice(len(pts), ICP_MAX_PTS, replace=False)
                pts = pts[idx]
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts)
            return pcd

        pcd_prev = _disp_to_pcd(disp_prev)
        pcd_curr = _disp_to_pcd(disp_curr)

        if len(pcd_prev.points) < 50 or len(pcd_curr.points) < 50:
            return T_init

        result = o3d.pipelines.registration.registration_icp(
            source=pcd_prev,
            target=pcd_curr,
            max_correspondence_distance=ICP_MAX_DIST,
            init=T_init,
            estimation_method=(
                o3d.pipelines.registration.TransformationEstimationPointToPoint()
            ),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=ICP_MAX_ITER
            ),
        )

        if result.fitness < ICP_MIN_FIT:
            return T_init

        return np.asarray(result.transformation)

    # ------------------------------------------------------------------
    #  Helpers privados
    # ------------------------------------------------------------------

    def _save_state(self, gray, kp, des, disp):
        self.prev_frame = gray
        self.prev_kp    = kp
        self.prev_des   = des
        self.prev_disp  = disp

    def _fallo(self, gray, kp, des, disp, frame_idx):
        """
        Frame sin pose confiable. NO se vuelve la referencia: el siguiente frame
        se compara contra el ultimo frame bueno, asi el movimiento de este frame
        no se pierde.
        Si se juntan MAX_FALLOS seguidos, el ultimo bueno ya quedo muy lejos (casi
        no comparten esquinas) y se reinicia la referencia aqui. Solo en ese caso
        se pierde el movimiento del hueco, igual que antes.
        """
        self._fallos += 1
        if self._fallos >= MAX_FALLOS:
            print(f"  [VO] {MAX_FALLOS} fallos seguidos: nueva referencia en frame {frame_idx}")
            self._fallos = 0
            self._save_state(gray, kp, des, disp)
            self._prev_idx = frame_idx
            # conserva su pose (la ultima conocida) para no romper la cadena del grafo
        else:
            # sin pose propia: que no entre a la nube ni al grafo con una pose vieja
            self.all_poses.pop(frame_idx, None)

    def _add_loop_keyframe(self, frame_idx, des):
        """Agrega las huellas al indice de loops solo si el carro se movio o giro lo suficiente."""
        if not USE_LOOP_CLOSURE or des is None or len(des) == 0:
            return
        if self._last_lc_pose is not None:
            rel = np.linalg.inv(self._last_lc_pose) @ self.pose
            mov = np.linalg.norm(rel[:3, 3])
            ang = np.degrees(np.linalg.norm(cv2.Rodrigues(rel[:3, :3])[0]))
            if mov < LC_KF_MOV_MM and ang < LC_KF_ROT_DEG:
                return
        self._loop_detector.add(frame_idx, des)
        self._last_lc_pose = self.pose.copy()

    @staticmethod
    def _ensure_2d(disp):
        """
        Acepta disp con cualquier cantidad de dims extra y devuelve (H, W).
        Cubre los casos comunes de RAFT: (H,W), (1,H,W), (1,1,H,W).
        """
        disp = np.squeeze(disp)
        if disp.ndim == 2:
            return disp
        if disp.ndim == 1:
            raise ValueError(
                f"_ensure_2d: disp llego como 1D con shape {disp.shape}. "
                "Verifica que run_raft devuelva un array 2D."
            )
        while disp.ndim > 2:
            disp = disp[0]
        return disp

    def _deproject(self, pts2d, disp):
        """
        Convierte puntos 2D + mapa de disparidad a coordenadas 3D
        usando la matriz de reproyeccion Q.
        """
        disp = self._ensure_2d(disp)
        uv   = pts2d.astype(int)
        us   = uv[:, 0]
        vs   = uv[:, 1]
        oob  = (
            (us < 0) | (us >= disp.shape[1]) |
            (vs < 0) | (vs >= disp.shape[0])
        )
        us_c = np.clip(us, 0, disp.shape[1] - 1)
        vs_c = np.clip(vs, 0, disp.shape[0] - 1)
        ds   = disp[vs_c, us_c].astype(np.float32)
        ds[oob] = 0.0
        ones  = np.ones(len(uv), dtype=np.float32)
        pts_h = np.stack(
            [us.astype(np.float32), vs.astype(np.float32), ds, ones], axis=1
        )
        vecs  = (self.Q @ pts_h.T).T        # (N, 4)
        w     = vecs[:, 3:4]
        pts3d = vecs[:, :3] / np.where(np.abs(w) > 1e-6, w, 1.0)
        pts3d[ds < ICP_DISP_MIN] = 0.0   # consistente con ICP_DISP_MIN=2.0
        return pts3d.astype(np.float32)
