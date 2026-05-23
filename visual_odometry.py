import cv2
import numpy as np

class VisualOdometry:
    def __init__(self, Q, K):
        self.Q = Q          # matriz de reproyección disparidad->3D
        self.K = K          # matriz intrínseca del lente izquierdo rectificado (P1[:3,:3])
        self.orb = cv2.ORB_create(nfeatures=1000)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        self.prev_frame = None
        self.prev_kp    = None
        self.prev_des   = None
        self.prev_disp  = None
        
        # Pose acumulada (posición en el mundo)
        self.pose = np.eye(4)  # matriz 4x4 homogénea

    def update(self, frame_gray, disp):
        """
        frame_gray : imagen rectificada izquierda (grayscale o BGR)
        disp       : mapa de disparidad numpy (H, W) de RAFT
        Retorna la transformación R, t respecto al frame anterior.
        """
        if len(frame_gray.shape) == 3:
            gray = cv2.cvtColor(frame_gray, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame_gray

        kp, des = self.orb.detectAndCompute(gray, None)

        if self.prev_frame is None:
            # Primer frame, solo guardamos
            self.prev_frame = gray
            self.prev_kp    = kp
            self.prev_des   = des
            self.prev_disp  = disp
            return None

        # --- Matching de descriptores ORB ---
        matches = self.matcher.match(self.prev_des, des)
        matches = sorted(matches, key=lambda x: x.distance)[:200]

        # --- Obtener coordenadas 3D de los puntos anteriores ---
        pts_prev_2d = np.float32([self.prev_kp[m.queryIdx].pt for m in matches])
        pts_curr_2d = np.float32([kp[m.trainIdx].pt           for m in matches])

        # Proyectar puntos del frame anterior a 3D usando disparidad
        pts3d = self._deproject(pts_prev_2d, self.prev_disp)

        # Filtrar puntos con profundidad válida
        valid = pts3d[:, 2] > 0
        if valid.sum() < 6:
            return None
        pts3d        = pts3d[valid]
        pts_curr_2d  = pts_curr_2d[valid]

        # --- PnP: estimar movimiento del carrito ---
        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            pts3d, pts_curr_2d, self.K, None,
            iterationsCount=100, reprojectionError=2.0
        )

        if not success:
            return None

        R, _ = cv2.Rodrigues(rvec)

        # Actualizar estado
        self.prev_frame = gray
        self.prev_kp    = kp
        self.prev_des   = des
        self.prev_disp  = disp

        return R, tvec.flatten()

    def _deproject(self, pts2d, disp):
        """Convierte puntos 2D + disparidad a coordenadas 3D usando la matriz Q."""
        pts3d = []
        for (u, v) in pts2d.astype(int):
            d = disp[v, u] if 0 <= v < disp.shape[0] and 0 <= u < disp.shape[1] else 0
            if d > 1.0:
                vec = self.Q @ np.array([u, v, d, 1.0])
                pts3d.append(vec[:3] / vec[3])
            else:
                pts3d.append([0, 0, 0])  # profundidad inválida
        return np.array(pts3d, dtype=np.float32)