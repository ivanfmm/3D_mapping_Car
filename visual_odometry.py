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
from scipy.optimize import minimize


# ==================================================================
#  Constantes de loop closure y PGO
# ==================================================================
MIN_LOOP_SEP      = 10    # frames minimos de separacion para loop closure \eran 15
MIN_LOOP_VOTES    = 8    # votos FAISS minimos para considerar candidato \era 12
MIN_LOOP_INLIERS  = 6     # inliers PnP para verificacion geometrica de loop \era 8
LOOP_CHECK_EVERY  = 1     # buscar loop closure en CADA frame
LOOP_PGO_COOLDOWN = 30    # frames minimos entre corridas consecutivas de PGO
PGO_WEIGHT_LOOP   = 5.0   # peso de edges de loop vs edges secuenciales
PGO_MAX_ITER      = 300   # iteraciones maximas de L-BFGS-B

# Deteccion de saltos de odometria
JUMP_MAX_TRANS  = 300.0   # mm  - maximo desplazamiento aceptable por frame
JUMP_MAX_ROT    = 30.0    # grados - maxima rotacion aceptable por frame
JUMP_RECOVER_N  = 5       # rechazos consecutivos antes de forzar recovery

# Parametros de matching y PnP
LOWE_RATIO      = 0.75    # ratio test de Lowe para filtrar matches ORB
PNP_ITERATIONS  = 200     # iteraciones RANSAC de solvePnPRansac
PNP_REPROJ_ERR  = 2.0     # error de reproyeccion en pixeles (640x360)
MIN_PNP_MATCHES = 12      # matches minimos tras ratio test para intentar PnP
MIN_PNP_INLIERS = 12      # inliers minimos para aceptar resultado de PnP

# Loop closure - filtro de calidad FAISS
LC_HAMMING_MAX  = 80      # distancia Hamming maxima para contar voto valido \ era 64

# ICP frame a frame
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
    Minimiza el error cuadratico de las restricciones de poses.

    Parametros
    ----------
    all_poses  : {frame_idx: 4x4}
    seq_edges  : [(i, j, T_rel_4x4), ...]   peso 1.0
    loop_edges : [(i, j, T_rel_4x4), ...]   peso PGO_WEIGHT_LOOP

    La primera pose se fija como identidad (no se optimiza).
    Retorna {frame_idx: 4x4} optimizado.
    """
    frame_ids = sorted(all_poses.keys())
    n         = len(frame_ids)
    if n < 2:
        return all_poses

    id_to_k = {fid: k for k, fid in enumerate(frame_ids)}

    # Vector inicial con las poses actuales convertidas a Rodrigues
    x0 = np.zeros(6 * n)
    for fid, T in all_poses.items():
        k = id_to_k[fid]
        x0[6*k : 6*k+6] = mat_to_vec(T)

    def cost(x):
        total = 0.0
        for edges, w in ((seq_edges, 1.0), (loop_edges, PGO_WEIGHT_LOOP)):
            for i, j, T_meas in edges:
                if i not in id_to_k or j not in id_to_k:
                    continue
                ki, kj = id_to_k[i], id_to_k[j]
                T_i   = vec_to_mat(x[6*ki : 6*ki+6])
                T_j   = vec_to_mat(x[6*kj : 6*kj+6])
                T_est = np.linalg.inv(T_i) @ T_j
                err   = np.linalg.inv(T_meas) @ T_est - np.eye(4)
                total += w * float(np.sum(err * err))
        return total

    # Fijar la primera pose con bounds identicos al valor inicial
    bounds = [(None, None)] * (6 * n)
    for k in range(6):
        bounds[k] = (float(x0[k]), float(x0[k]))

    res = minimize(
        cost, x0,
        method='L-BFGS-B',
        bounds=bounds,
        options={'maxiter': PGO_MAX_ITER, 'ftol': 1e-9},
    )

    x_opt = res.x
    return {
        fid: vec_to_mat(x_opt[6*id_to_k[fid] : 6*id_to_k[fid]+6])
        for fid in frame_ids
    }


# ==================================================================
#  Clase principal
# ==================================================================

class VisualOdometry:
    def __init__(self, Q, K):
        """
        Q : matriz de reproyeccion disparidad->3D (de cv2.stereoRectify)
        K : matriz intrinseca del lente izquierdo rectificado (P1[:3,:3])
        """
        self.Q = Q
        self.K = K

        # nlevels=6: para 640x360 la piramide de 8 niveles llega a ~80px (inutil)
        # fastThreshold=10: detecta mas esquinas en zonas de poca textura
        self.orb = cv2.ORB_create(
            nfeatures    = 500,
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
        self._kf_data = {}   # {frame_idx: {'kp', 'des', 'disp'}}

        self._loop_detector  = _LoopDetector()
        self._prev_idx       = None
        self._internal_cnt   = 0
        self._last_pgo_frame = 0
        self._consec_reject  = 0   # rechazos consecutivos por salto de odometria

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

        # -- Extraer features -------------------------------------------
        kp, des = self.orb.detectAndCompute(gray, None)

        # Guardar datos del keyframe antes de cualquier retorno anticipado
        self._kf_data[frame_idx] = {'kp': kp, 'des': des, 'disp': disp}
        self._loop_detector.add(frame_idx, des)
        self.all_poses[frame_idx] = self.pose.copy()

        # -- Primer frame: solo guardamos estado -----------------------
        if self.prev_frame is None or self.prev_des is None:
            self._save_state(gray, kp, des, disp)
            self._prev_idx = frame_idx
            return None

        if des is None or len(des) < 8:
            self._save_state(gray, kp, des, disp)
            self._prev_idx = frame_idx
            return None

        # -- Ratio test de Lowe (reemplaza crossCheck) ----------------
        raw = self.matcher.knnMatch(self.prev_des, des, k=2)
        matches = [
            pair[0] for pair in raw
            if len(pair) == 2 and pair[0].distance < LOWE_RATIO * pair[1].distance
        ]
        if len(matches) < MIN_PNP_MATCHES:
            self._save_state(gray, kp, des, disp)
            self._prev_idx = frame_idx
            return None

        # -- Coordenadas 2D de correspondencias -----------------------
        pts_prev_2d = np.float32([self.prev_kp[m.queryIdx].pt for m in matches])
        pts_curr_2d = np.float32([kp[m.trainIdx].pt           for m in matches])

        # -- Proyectar puntos anteriores a 3D usando su disparidad ----
        pts3d = self._deproject(pts_prev_2d, self.prev_disp)
        valid = pts3d[:, 2] > 0
        if valid.sum() < MIN_PNP_INLIERS:
            self._save_state(gray, kp, des, disp)
            self._prev_idx = frame_idx
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
            self._save_state(gray, kp, des, disp)
            self._prev_idx = frame_idx
            return None

        R, _ = cv2.Rodrigues(rvec)
        tvec_raw  = tvec.flatten()
        tvec_mag  = float(np.linalg.norm(tvec_raw))
        angle_deg = float(np.degrees(np.linalg.norm(rvec.flatten())))

        # -- Deteccion de saltos de odometria -------------------------
        if tvec_mag > JUMP_MAX_TRANS or angle_deg > JUMP_MAX_ROT:
            self._consec_reject += 1
            print(f"  [VO] SALTO frame {frame_idx}: "
                  f"|t|={tvec_mag:.0f}mm rot={angle_deg:.1f}deg "
                  f"({self._consec_reject} consec.)")
            if self._consec_reject >= JUMP_RECOVER_N:
                # La camara se movio genuinamente lejos: forzar recovery
                print(f"  [VO] Recovery forzada tras {JUMP_RECOVER_N} rechazos")
                self._consec_reject = 0
                self._save_state(gray, kp, des, disp)
                self._prev_idx = frame_idx
            # No acumular pose en ninguno de los dos casos
            return None

        self._consec_reject = 0

        # -- Construir T y refinar con ICP (prev->curr) ---------------
        T = np.eye(4)
        T[:3, :3] = R
        T[:3,  3] = tvec_raw
        T = self._icp_refine(T, self.prev_disp, disp)

        # -- Acumular pose (T mapea prev_cam->curr_cam, pose es cam->world) ---
        self.pose = self.pose @ np.linalg.inv(T)
        self.all_poses[frame_idx] = self.pose.copy()

        # Edge secuencial entre frame previo y actual
        if self._prev_idx is not None and self._prev_idx in self.all_poses:
            T_rel = np.linalg.inv(self.all_poses[self._prev_idx]) @ self.pose
            self.seq_edges.append((self._prev_idx, frame_idx, T_rel))

        # -- Loop closure (cada LOOP_CHECK_EVERY frames) --------------
        if self._internal_cnt % LOOP_CHECK_EVERY == 0:
            self._check_loop_closure(frame_idx, kp, des, disp)

        self._save_state(gray, kp, des, disp)
        self._prev_idx = frame_idx
        return R, tvec.flatten()

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
            kf['kp'], kf['des'], kf['disp'],
            kp_curr, des_curr, disp_curr,
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

    def _verify_loop(self, kp_a, des_a, disp_a, kp_b, des_b, disp_b):
        """
        Verificacion geometrica del loop: matchea descriptores entre el frame
        candidato (a) y el actual (b), luego verifica con PnP RANSAC.
        Retorna T_rel 4x4 o None si la verificacion falla.
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

        pts_a_2d = np.float32([kp_a[m.queryIdx].pt for m in matches])
        pts_b_2d = np.float32([kp_b[m.trainIdx].pt for m in matches])

        pts3d = self._deproject(pts_a_2d, disp_a)
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
        return T

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
