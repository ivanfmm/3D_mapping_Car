"""
Calibración Estéreo para cámara USB sincrónica (MMlove 1200P 60FPS)
=====================================================================
La cámara entrega UN solo stream donde el lente izquierdo ocupa la mitad
izquierda del frame y el derecho la mitad derecha.

MODOS:
  captura   → Captura pares de imágenes en vivo desde la cámara estéreo.
  calibrar  → Calibra ambos lentes + relación estéreo (distancia base).
  imagenes  → Igual que calibrar pero desde pares ya guardados en disco.
  rectificar → Aplica la calibración a un par de imágenes y muestra el resultado.

FLUJO RECOMENDADO:
  1. python calibracion_estereo.py captura  --cam 0
  2. python calibracion_estereo.py calibrar --carpeta capturas/
  3. python calibracion_estereo.py rectificar --izq img_izq.jpg --der img_der.jpg

Requiere:
  pip install opencv-python numpy
"""

import cv2 as cv
import numpy as np
import glob
import os
import json
import argparse # para que funcione a base de comandos el script
from pathlib import Path


# ═══════════════════════════════════════════════════════════════
#  CONFIGURACIÓN  — ajusta según tu tablero físico impreso
# ═══════════════════════════════════════════════════════════════
COLS_ESQUINAS  = 10      # esquinas internas horizontales  (columnas de cuadros - 1)
FILAS_ESQUINAS = 7      # esquinas internas verticales    (filas de cuadros - 1)
TAMANO_CUADRO  = 20.0   # tamaño real de cada cuadro en mm  (mide el tuyo con regla)
MIN_PARES      = 10     # mínimo de pares válidos para calibrar
CARPETA_CAP    = "capturas"       # donde se guardan los pares capturados
ARCHIVO_CAL    = "estereo_cal.json"  # resultado final

# Criterio de refinamiento de esquinas
CRITERIO = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Flags para stereoCalibrate:
#   CALIB_FIX_INTRINSIC  → usa las matrices intrínsecas ya calculadas (más estable)
FLAGS_ESTEREO = cv.CALIB_FIX_INTRINSIC


# ───────────────────────────────────────────────────────────────
#  UTILIDADES COMUNES
# ───────────────────────────────────────────────────────────────

def preparar_objp():
    """Puntos 3-D del tablero en el plano Z=0 (en mm)."""
    objp = np.zeros((COLS_ESQUINAS * FILAS_ESQUINAS, 3), np.float32)
    objp[:, :2] = np.mgrid[0:COLS_ESQUINAS, 0:FILAS_ESQUINAS].T.reshape(-1, 2)
    objp *= TAMANO_CUADRO
    return objp


def detectar_esquinas(gray):
    """
    Busca las esquinas del tablero en una imagen en gris.
    Devuelve (ok, esquinas_refinadas).
    """
    flags = cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv.findChessboardCorners(gray, (COLS_ESQUINAS, FILAS_ESQUINAS), flags)
    if ok:
        corners = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), CRITERIO)
    return ok, corners


def dividir_frame_estereo(frame):
    """
    La MMlove entrega un frame panorámico donde:
      - mitad izquierda  → lente izquierdo
      - mitad derecha    → lente derecho
    Devuelve (izq, der) como imágenes separadas BGR.
    """
    h, w = frame.shape[:2]
    mid = w // 2
    return frame[:, :mid], frame[:, mid:]


def calibrar_mono(objpoints, imgpoints, shape_gray, nombre=""):
    """
    Calibración intrínseca de un lente individual.
    Devuelve (mtx, dist, datos_dict).
    """
    rms, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
        objpoints, imgpoints, shape_gray[::-1], None, None
    )

    # Error de reproyección
    error_total = 0.0
    for i in range(len(objpoints)):
        pts2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error_total += cv.norm(imgpoints[i], pts2, cv.NORM_L2) / len(pts2)
    err_medio = error_total / len(objpoints)

    datos = {
        "lente":             nombre,
        "rms":               float(rms),
        "error_reproyeccion": float(err_medio),
        "matriz_camara":     mtx.tolist(),
        "coef_distorsion":   dist.tolist(),
        "num_imagenes":      len(objpoints),
        "resolucion":        list(shape_gray[::-1]),
    }
    return mtx, dist, datos


def mostrar_resumen_mono(datos):
    lente = datos.get("lente", "?").upper()
    print(f"\n  ── Lente {lente} ──")
    print(f"     Imágenes usadas    : {datos['num_imagenes']}")
    print(f"     Error RMS          : {datos['rms']:.4f} px")
    print(f"     Error reproyección : {datos['error_reproyeccion']:.4f} px")
    print(f"     Resolución         : {datos['resolucion']}")
    K = np.array(datos["matriz_camara"])
    print(f"     fx={K[0,0]:.2f}  fy={K[1,1]:.2f}  cx={K[0,2]:.2f}  cy={K[1,2]:.2f}")
    d = np.array(datos["coef_distorsion"]).flatten()
    print(f"     dist [k1 k2 p1 p2 k3]: {d.round(5)}")


# ═══════════════════════════════════════════════════════════════
#  PASO ESTÉREO: cv.stereoCalibrate
# ═══════════════════════════════════════════════════════════════

def calibrar_estereo(objpoints, imgpoints_izq, imgpoints_der,
                     mtx_izq, dist_izq, mtx_der, dist_der, shape_gray):
    """
    Calcula la relación geométrica entre los dos lentes.

    Retorna un dict con:
      R  — matriz de rotación del lente derecho respecto al izquierdo
      T  — vector de traslación (‖T‖ = baseline en mm)
      E  — matriz esencial (geometría epipolar)
      F  — matriz fundamental (en píxeles, útil para SLAM/matching)
    """
    img_size = shape_gray[::-1]  # (ancho, alto)

    rms, _, _, _, _, R, T, E, F = cv.stereoCalibrate(
        objpoints,
        imgpoints_izq,
        imgpoints_der,
        mtx_izq, dist_izq,
        mtx_der, dist_der,
        img_size,
        flags=FLAGS_ESTEREO,
        criteria=CRITERIO,
    )

    baseline_mm = float(np.linalg.norm(T))

    # Ángulos de Euler de R (para interpretación humana)
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    if sy > 1e-6:
        roll  = np.degrees(np.arctan2( R[2, 1],  R[2, 2]))
        pitch = np.degrees(np.arctan2(-R[2, 0],  sy))
        yaw   = np.degrees(np.arctan2( R[1, 0],  R[0, 0]))
    else:
        roll  = np.degrees(np.arctan2(-R[1, 2],  R[1, 1]))
        pitch = np.degrees(np.arctan2(-R[2, 0],  sy))
        yaw   = 0.0

    datos = {
        "rms_estereo":    float(rms),
        "baseline_mm":    baseline_mm,
        "baseline_cm":    baseline_mm / 10.0,
        "R":              R.tolist(),
        "T":              T.tolist(),
        "E":              E.tolist(),
        "F":              F.tolist(),
        "euler_grados":   {"roll": roll, "pitch": pitch, "yaw": yaw},
    }
    return datos


def mostrar_resumen_estereo(datos):
    print("\n" + "═" * 50)
    print("  CALIBRACIÓN ESTÉREO")
    print("═" * 50)
    print(f"  Error RMS estéreo   : {datos['rms_estereo']:.4f} px")
    print(f"  Baseline (distancia): {datos['baseline_mm']:.2f} mm  "
          f"({datos['baseline_cm']:.2f} cm)")
    e = datos["euler_grados"]
    print(f"  Rotación entre lentes: roll={e['roll']:.3f}°  "
          f"pitch={e['pitch']:.3f}°  yaw={e['yaw']:.3f}°")
    T = np.array(datos["T"]).flatten()
    print(f"  Vector T (mm)       : Tx={T[0]:.3f}  Ty={T[1]:.3f}  Tz={T[2]:.3f}")
    print("═" * 50)
    print("\n  💡  Interpretación:")
    print(f"      La cámara MMlove tiene sus lentes separados ~{datos['baseline_mm']:.1f} mm.")
    print(f"      Con esta baseline, a 1 m de distancia la disparidad mínima")
    bl = datos["baseline_mm"]
    # Aproximación: profundidad_mm = fx * baseline_mm / disparidad_px
    # asumimos fx ~600 px (típico 1200P)
    fx_aprox = 600
    d_min = fx_aprox * bl / 5000   # 5 m de distancia
    d_max = fx_aprox * bl / 500    # 0.5 m de distancia
    print(f"      útil va de ~{d_min:.1f} px (5 m) a ~{d_max:.1f} px (0.5 m).")
    print("\n  ✅  R, T, E, F guardados en el JSON para uso en SLAM / StereoSGBM.")


def guardar_todo(datos_izq, datos_der, datos_estereo, archivo):
    salida = {
        "lente_izquierdo": datos_izq,
        "lente_derecho":   datos_der,
        "estereo":         datos_estereo,
    }
    with open(archivo, "w") as f:
        json.dump(salida, f, indent=2)
    print(f"\n✅  Calibración completa guardada en '{archivo}'")


def cargar_calibracion_estereo(archivo):
    with open(archivo) as f:
        datos = json.load(f)
    mtx_izq  = np.array(datos["lente_izquierdo"]["matriz_camara"])
    dist_izq = np.array(datos["lente_izquierdo"]["coef_distorsion"])
    mtx_der  = np.array(datos["lente_derecho"]["matriz_camara"])
    dist_der = np.array(datos["lente_derecho"]["coef_distorsion"])
    R  = np.array(datos["estereo"]["R"])
    T  = np.array(datos["estereo"]["T"])
    return mtx_izq, dist_izq, mtx_der, dist_der, R, T, datos


# ═══════════════════════════════════════════════════════════════
#  MODO 1 — CAPTURA EN VIVO (guarda pares izq/der sincronizados)
# ═══════════════════════════════════════════════════════════════

def modo_captura(indice_cam=0, carpeta=CARPETA_CAP):
    cap = cv.VideoCapture(indice_cam)
    if not cap.isOpened():
        print(f"❌  No se pudo abrir la cámara (índice {indice_cam})")
        print("    Prueba con --cam 1 o --cam 2 si el índice 0 es otro dispositivo.")
        return

    # Intentar forzar resolución 2560×720 (dos lentes 1280×720 lado a lado)
    cap.set(cv.CAP_PROP_FRAME_WIDTH,  2560)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT,  720)
    cap.set(cv.CAP_PROP_FPS,           60)

    os.makedirs(carpeta, exist_ok=True)
    contador = len(glob.glob(os.path.join(carpeta, "izq_*.jpg"))) + 1

    objp_base = preparar_objp()
    print("\n═══════════════════════════════════════════════════")
    print("  CAPTURA ESTÉREO EN VIVO")
    print("  [ESPACIO] Guardar par  (solo si AMBOS lentes ven el tablero)")
    print("  [Q]       Salir")
    print(f"  Meta: {MIN_PARES} pares válidos guardados en '{carpeta}/'")
    print("═══════════════════════════════════════════════════\n")

    pares_guardados = contador - 1

    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠  Sin señal de cámara")
            break

        izq, der = dividir_frame_estereo(frame)
        gray_izq = cv.cvtColor(izq, cv.COLOR_BGR2GRAY)
        gray_der = cv.cvtColor(der, cv.COLOR_BGR2GRAY)

        ok_izq, esq_izq = detectar_esquinas(gray_izq)
        ok_der, esq_der = detectar_esquinas(gray_der)

        ambos = ok_izq and ok_der

        # Vista en vivo con marcadores
        vis_izq = izq.copy()
        vis_der = der.copy()
        if ok_izq:
            cv.drawChessboardCorners(vis_izq, (COLS_ESQUINAS, FILAS_ESQUINAS), esq_izq, True)
        if ok_der:
            cv.drawChessboardCorners(vis_der, (COLS_ESQUINAS, FILAS_ESQUINAS), esq_der, True)

        # Etiquetas de estado
        col_izq = (0, 220, 0) if ok_izq else (0, 0, 220)
        col_der = (0, 220, 0) if ok_der else (0, 0, 220)
        cv.putText(vis_izq, "IZQ: OK" if ok_izq else "IZQ: buscando...",
                   (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, col_izq, 2)
        cv.putText(vis_der, "DER: OK" if ok_der else "DER: buscando...",
                   (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, col_der, 2)

        # Banner central
        estado = "★ AMBOS DETECTADOS — presiona ESPACIO" if ambos else "Mueve el tablero..."
        col_banner = (0, 220, 0) if ambos else (180, 180, 180)

        panel = np.hstack([vis_izq, vis_der])
        cv.putText(panel, estado, (panel.shape[1]//2 - 280, panel.shape[0] - 15),
                   cv.FONT_HERSHEY_SIMPLEX, 0.75, col_banner, 2)
        cv.putText(panel, f"Pares guardados: {pares_guardados}/{MIN_PARES}",
                   (10, panel.shape[0] - 15), cv.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

        cv.imshow("Captura Estereo  [ESPACIO=guardar par | Q=salir]", panel)
        tecla = cv.waitKey(1) & 0xFF

        if tecla == ord('q'):
            break
        elif tecla == ord(' ') and ambos:
            nombre_izq = os.path.join(carpeta, f"izq_{contador:03d}.jpg")
            nombre_der = os.path.join(carpeta, f"der_{contador:03d}.jpg")
            cv.imwrite(nombre_izq, izq)
            cv.imwrite(nombre_der, der)
            pares_guardados += 1
            contador += 1
            print(f"  📸  Par {pares_guardados:03d} guardado → {nombre_izq}  {nombre_der}")

    cap.release()
    cv.destroyAllWindows()
    print(f"\nTotal de pares capturados: {pares_guardados}")
    if pares_guardados >= MIN_PARES:
        print(f"✅  Suficientes pares. Ahora ejecuta:")
        print(f"    python calibracion_estereo.py calibrar --carpeta {carpeta}/")
    else:
        print(f"⚠  Necesitas al menos {MIN_PARES} pares. Captura más poses.")


# ═══════════════════════════════════════════════════════════════
#  MODO 2 — CALIBRACIÓN desde pares en disco
# ═══════════════════════════════════════════════════════════════

def _procesar_pares(archivos_izq, archivos_der):
    """
    Lee pares de imágenes, detecta esquinas en ambos y
    devuelve (objpoints, imgpoints_izq, imgpoints_der, shape_gray).
    Solo incluye pares donde AMBOS lentes detectan el tablero.
    """
    objp_base = preparar_objp()
    objpoints, imgpoints_izq, imgpoints_der = [], [], []
    shape_gray = None

    for ruta_izq, ruta_der in zip(archivos_izq, archivos_der):
        img_izq = cv.imread(ruta_izq)
        img_der = cv.imread(ruta_der)
        if img_izq is None or img_der is None:
            print(f"  ⚠  No se pudo leer: {ruta_izq} / {ruta_der}")
            continue

        gray_izq = cv.cvtColor(img_izq, cv.COLOR_BGR2GRAY)
        gray_der = cv.cvtColor(img_der, cv.COLOR_BGR2GRAY)
        shape_gray = gray_izq.shape

        ok_izq, esq_izq = detectar_esquinas(gray_izq)
        ok_der, esq_der = detectar_esquinas(gray_der)

        par = Path(ruta_izq).stem.replace("izq_", "")
        if ok_izq and ok_der:
            objpoints.append(objp_base)
            imgpoints_izq.append(esq_izq)
            imgpoints_der.append(esq_der)
            print(f"  ✔  Par {par}")
        else:
            razon = []
            if not ok_izq: razon.append("IZQ no detectó patrón")
            if not ok_der: razon.append("DER no detectó patrón")
            print(f"  ✘  Par {par}  ({' / '.join(razon)})")

    return objpoints, imgpoints_izq, imgpoints_der, shape_gray


def modo_calibrar(carpeta=CARPETA_CAP, archivo_sal=ARCHIVO_CAL):
    archivos_izq = sorted(glob.glob(os.path.join(carpeta, "izq_*.jpg")))
    archivos_der = sorted(glob.glob(os.path.join(carpeta, "der_*.jpg")))

    if not archivos_izq:
        print(f"❌  No se encontraron imágenes 'izq_*.jpg' en '{carpeta}/'")
        print("    Primero ejecuta el modo captura.")
        return

    if len(archivos_izq) != len(archivos_der):
        print(f"⚠  Número desigual: {len(archivos_izq)} izq vs {len(archivos_der)} der")

    print(f"\nProcesando {len(archivos_izq)} par(es) en '{carpeta}/'...\n")
    objpoints, imgpoints_izq, imgpoints_der, shape_gray = \
        _procesar_pares(archivos_izq, archivos_der)

    pares_validos = len(objpoints)
    print(f"\nPares válidos: {pares_validos}")
    if pares_validos < 3:
        print("❌  Se necesitan al menos 3 pares válidos. Captura más imágenes.")
        return
    if pares_validos < MIN_PARES:
        print(f"⚠  Se recomiendan {MIN_PARES} pares para mejor precisión.")

    # ── Calibración intrínseca izquierda ──
    print("\n[1/3] Calibrando lente IZQUIERDO...")
    mtx_izq, dist_izq, datos_izq = calibrar_mono(
        objpoints, imgpoints_izq, shape_gray, "izquierdo")
    mostrar_resumen_mono(datos_izq)

    # ── Calibración intrínseca derecha ──
    print("\n[2/3] Calibrando lente DERECHO...")
    mtx_der, dist_der, datos_der = calibrar_mono(
        objpoints, imgpoints_der, shape_gray, "derecho")
    mostrar_resumen_mono(datos_der)

    # ── Calibración estéreo ──
    print("\n[3/3] Calculando geometría estéreo (cv.stereoCalibrate)...")
    datos_estereo = calibrar_estereo(
        objpoints, imgpoints_izq, imgpoints_der,
        mtx_izq, dist_izq, mtx_der, dist_der, shape_gray
    )
    mostrar_resumen_estereo(datos_estereo)

    guardar_todo(datos_izq, datos_der, datos_estereo, archivo_sal)


# ═══════════════════════════════════════════════════════════════
#  MODO 3 — RECTIFICACIÓN de un par de imágenes
# ═══════════════════════════════════════════════════════════════

def modo_rectificar(ruta_izq, ruta_der, archivo_cal=ARCHIVO_CAL):
    """
    Aplica stereoRectify + remap para que ambas imágenes queden
    en el mismo plano (líneas epipolares horizontales).
    """
    if not os.path.exists(archivo_cal):
        print(f"❌  No se encontró '{archivo_cal}'. Primero calibra.")
        return

    img_izq = cv.imread(ruta_izq)
    img_der = cv.imread(ruta_der)
    if img_izq is None or img_der is None:
        print("❌  No se pudieron leer las imágenes.")
        return

    mtx_izq, dist_izq, mtx_der, dist_der, R, T, datos = \
        cargar_calibracion_estereo(archivo_cal)

    h, w = img_izq.shape[:2]
    tam = (w, h)

    # Calcular mapas de rectificación
    R1, R2, P1, P2, Q, roi1, roi2 = cv.stereoRectify(
        mtx_izq, dist_izq,
        mtx_der, dist_der,
        tam, R, T,
        alpha=0,   # 0 = sin píxeles negros; 1 = conserva todo
    )

    map1_izq, map2_izq = cv.initUndistortRectifyMap(
        mtx_izq, dist_izq, R1, P1, tam, cv.CV_32FC1)
    map1_der, map2_der = cv.initUndistortRectifyMap(
        mtx_der, dist_der, R2, P2, tam, cv.CV_32FC1)

    rect_izq = cv.remap(img_izq, map1_izq, map2_izq, cv.INTER_LINEAR)
    rect_der = cv.remap(img_der, map1_der, map2_der, cv.INTER_LINEAR)

    # Guardar
    cv.imwrite("rect_izq.jpg", rect_izq)
    cv.imwrite("rect_der.jpg", rect_der)
    print("✅  Imágenes rectificadas guardadas: rect_izq.jpg / rect_der.jpg")

    # Mostrar con líneas epipolares (deben ser horizontales si la calibración es correcta)
    comparacion = np.hstack([rect_izq, rect_der])
    for y_line in range(0, h, 40):
        cv.line(comparacion, (0, y_line), (w*2, y_line), (0, 255, 0), 1)

    escala = min(1.0, 1400 / comparacion.shape[1])
    vis = cv.resize(comparacion, None, fx=escala, fy=escala)
    cv.putText(vis, "IZQ rectificada", (10, 25),
               cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
    cv.putText(vis, "DER rectificada", (int(w*escala)+10, 25),
               cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
    cv.putText(vis, "Lineas epipolares (deben ser horizontales)", (10, vis.shape[0]-10),
               cv.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 100), 1)

    bl = datos["estereo"]["baseline_mm"]
    print(f"\n  Baseline medida: {bl:.2f} mm  ({bl/10:.2f} cm)")
    print("  Si las líneas verdes están alineadas con los puntos correspondientes,")
    print("  la calibración es correcta.")

    cv.imshow("Rectificacion estereo  (cualquier tecla para salir)", vis)
    cv.waitKey(0)
    cv.destroyAllWindows()


# ═══════════════════════════════════════════════════════════════
#  PUNTO DE ENTRADA
# ═══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Calibración estéreo para cámara MMlove USB 1200P",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    sub = parser.add_subparsers(dest="modo", required=True)

    # ── captura ──
    p_cap = sub.add_parser("captura", help="Captura pares izq/der en vivo")
    p_cap.add_argument("--cam",     type=int, default=0,help="Índice de cámara (por defecto 0)")
    p_cap.add_argument("--carpeta", default=CARPETA_CAP, help=f"Carpeta de salida (por defecto '{CARPETA_CAP}')")

    # ── calibrar ──
    p_cal = sub.add_parser("calibrar", help="Calibra desde pares guardados")
    p_cal.add_argument("--carpeta", default=CARPETA_CAP,help=f"Carpeta con pares izq_*/der_* (por defecto '{CARPETA_CAP}')")
    p_cal.add_argument("--sal",     default=ARCHIVO_CAL,help=f"Archivo JSON de salida (por defecto '{ARCHIVO_CAL}')")

    # ── imagenes (alias de calibrar para pares en glob arbitrario) ──
    p_img = sub.add_parser("imagenes",help="Calibra desde pares con glob personalizado")
    p_img.add_argument("--carpeta", default=CARPETA_CAP)
    p_img.add_argument("--sal",     default=ARCHIVO_CAL)

    # ── rectificar ──
    p_rec = sub.add_parser("rectificar", help="Rectifica un par de imágenes")
    p_rec.add_argument("--izq", required=True, help="Imagen del lente izquierdo")
    p_rec.add_argument("--der", required=True, help="Imagen del lente derecho")
    p_rec.add_argument("--cal", default=ARCHIVO_CAL,help=f"Archivo JSON de calibración (por defecto '{ARCHIVO_CAL}')")

    args = parser.parse_args()

    if args.modo == "captura":
        modo_captura(args.cam, args.carpeta)
    elif args.modo in ("calibrar", "imagenes"):
        modo_calibrar(args.carpeta, args.sal)
    elif args.modo == "rectificar":
        modo_rectificar(args.izq, args.der, args.cal)


if __name__ == "__main__":
    main()