"""
rectificacion.py - Carga la calibracion estereo y calcula los mapas de rectificacion.

No depende de torch ni de RAFT: lo usan client_2.py y reprocesar.py.
"""
import json

import cv2
import numpy as np


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
