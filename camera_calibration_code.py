"""
Calibración de Cámara con OpenCV
Basado en: https://docs.opencv.org/3.4/dc/dbb/tutorial_py_calibration.html

Uso:
  1. Modo imágenes:  python calibracion_camara.py --modo imagenes --patron imagenes_calibracion/*.jpg
  2. Modo cámara:    python calibracion_camara.py --modo camara
  3. Ver resultado:  python calibracion_camara.py --modo undistort --imagen foto.jpg
"""

import cv2 as cv
import numpy as np
import glob
import os
import json
import argparse
from pathlib import Path


# ─────────────────────────────────────────────
#  CONFIGURACIÓN  (ajusta según tu tablero)
# ─────────────────────────────────────────────
COLS_ESQUINAS   = 9      # esquinas internas horizontales (cuadros - 1)
FILAS_ESQUINAS  = 6      # esquinas internas verticales   (cuadros - 1)
TAMANO_CUADRO   = 25.0   # mm reales de cada cuadro (0 = unidades arbitrarias)
MIN_IMAGENES    = 10     # mínimo de capturas válidas para calibrar
ARCHIVO_SALIDA  = "calibracion.json"


# ─────────────────────────────────────────────
#  CRITERIO DE PARADA para cornerSubPix
# ─────────────────────────────────────────────
CRITERIO = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def preparar_puntos_objeto(cols, filas, tamano_cuadro):
    """
    Genera las coordenadas 3-D del patrón en el plano Z=0.
    Si tamano_cuadro > 0 los valores estarán en mm; si es 0, en unidades de cuadro.
    """
    objp = np.zeros((cols * filas, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:filas].T.reshape(-1, 2)
    if tamano_cuadro > 0:
        objp *= tamano_cuadro
    return objp


def detectar_esquinas(img_gray, cols, filas):
    """
    Busca las esquinas del tablero de ajedrez.
    Devuelve (True, esquinas_refinadas) o (False, None).
    """
    encontrado, esquinas = cv.findChessboardCorners(img_gray, (cols, filas), None)
    if encontrado:
        esquinas = cv.cornerSubPix(img_gray, esquinas, (11, 11), (-1, -1), CRITERIO)
    return encontrado, esquinas


def calibrar(objpoints, imgpoints, shape_gris):
    """
    Ejecuta cv.calibrateCamera y devuelve un dict con todos los parámetros.
    """
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
        objpoints, imgpoints, shape_gris[::-1], None, None
    )

    # Error de reproyección medio
    error_total = 0.0
    for i in range(len(objpoints)):
        pts2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error_total += cv.norm(imgpoints[i], pts2, cv.NORM_L2) / len(pts2)
    error_medio = error_total / len(objpoints)

    return {
        "rms":              float(ret),
        "error_reproyeccion": float(error_medio),
        "matriz_camara":    mtx.tolist(),
        "coef_distorsion":  dist.tolist(),
        "num_imagenes":     len(objpoints),
        "resolucion":       list(shape_gris[::-1]),   # [ancho, alto]
    }


def guardar_calibracion(datos, archivo):
    with open(archivo, "w") as f:
        json.dump(datos, f, indent=2)
    print(f"\n✅  Calibración guardada en '{archivo}'")


def cargar_calibracion(archivo):
    with open(archivo) as f:
        datos = json.load(f)
    mtx  = np.array(datos["matriz_camara"])
    dist = np.array(datos["coef_distorsion"])
    return mtx, dist, datos


def mostrar_resumen(datos):
    print("\n" + "=" * 45)
    print("  RESULTADO DE CALIBRACIÓN")
    print("=" * 45)
    print(f"  Imágenes usadas   : {datos['num_imagenes']}")
    print(f"  Error RMS         : {datos['rms']:.4f} px")
    print(f"  Error reproyección: {datos['error_reproyeccion']:.4f} px")
    print(f"  Resolución        : {datos['resolucion']}")
    print("\n  Matriz de cámara (K):")
    for fila in datos["matriz_camara"]:
        print("   ", [f"{v:10.3f}" for v in fila])
    print("\n  Coeficientes de distorsión [k1 k2 p1 p2 k3]:")
    print("   ", [f"{v:.6f}" for v in np.array(datos["coef_distorsion"]).flatten()])
    print("=" * 45)


# ══════════════════════════════════════════════
#  MODO 1: Calibrar desde imágenes en disco
# ══════════════════════════════════════════════
def modo_imagenes(patron_glob):
    archivos = sorted(glob.glob(patron_glob))
    if not archivos:
        print(f"❌  No se encontraron imágenes con el patrón: {patron_glob}")
        return

    objp_base = preparar_puntos_objeto(COLS_ESQUINAS, FILAS_ESQUINAS, TAMANO_CUADRO)
    objpoints, imgpoints = [], []
    shape_gris = None

    print(f"\nProcesando {len(archivos)} imagen(es)...")

    for ruta in archivos:
        img  = cv.imread(ruta)
        if img is None:
            print(f"  ⚠  No se pudo leer: {ruta}")
            continue
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        shape_gris = gray.shape

        encontrado, esquinas = detectar_esquinas(gray, COLS_ESQUINAS, FILAS_ESQUINAS)

        if encontrado:
            objpoints.append(objp_base)
            imgpoints.append(esquinas)
            cv.drawChessboardCorners(img, (COLS_ESQUINAS, FILAS_ESQUINAS), esquinas, encontrado)
            print(f"  ✔  {Path(ruta).name}")
        else:
            print(f"  ✘  {Path(ruta).name}  (patrón no detectado)")

        cv.imshow("Deteccion de esquinas", img)
        cv.waitKey(400)

    cv.destroyAllWindows()

    if len(objpoints) < MIN_IMAGENES:
        print(f"\n⚠  Solo se obtuvieron {len(objpoints)} imagen(es) válida(s). "
              f"Se recomiendan al menos {MIN_IMAGENES}.")
        if len(objpoints) == 0:
            return

    print(f"\nCalibrando con {len(objpoints)} imagen(es)...")
    datos = calibrar(objpoints, imgpoints, shape_gris)
    mostrar_resumen(datos)
    guardar_calibracion(datos, ARCHIVO_SALIDA)


# ══════════════════════════════════════════════
#  MODO 2: Captura en vivo con webcam
# ══════════════════════════════════════════════
def modo_camara(indice_cam=0):
    cap = cv.VideoCapture(indice_cam)
    if not cap.isOpened():
        print(f"❌  No se pudo abrir la cámara (índice {indice_cam})")
        return

    objp_base = preparar_puntos_objeto(COLS_ESQUINAS, FILAS_ESQUINAS, TAMANO_CUADRO)
    objpoints, imgpoints = [], []
    shape_gris = None
    capturando = False

    print("\n═══════════════════════════════════════")
    print("  CALIBRACIÓN EN VIVO")
    print("  [ESPACIO] Capturar frame")
    print("  [C]       Calibrar ahora")
    print("  [Q]       Salir")
    print(f"  Meta: {MIN_IMAGENES} capturas válidas")
    print("═══════════════════════════════════════\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        shape_gris = gray.shape
        encontrado, esquinas = detectar_esquinas(gray, COLS_ESQUINAS, FILAS_ESQUINAS)

        vista = frame.copy()
        if encontrado:
            cv.drawChessboardCorners(vista, (COLS_ESQUINAS, FILAS_ESQUINAS), esquinas, True)

        # HUD
        color_estado = (0, 200, 0) if encontrado else (0, 0, 200)
        estado_txt = "PATRON DETECTADO" if encontrado else "Buscando patron..."
        cv.putText(vista, estado_txt, (10, 30),
                   cv.FONT_HERSHEY_SIMPLEX, 0.8, color_estado, 2)
        cv.putText(vista, f"Capturas: {len(objpoints)}/{MIN_IMAGENES}", (10, 60),
                   cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv.imshow("Calibracion en vivo  [ESPACIO=capturar | C=calibrar | Q=salir]", vista)

        tecla = cv.waitKey(1) & 0xFF

        if tecla == ord('q'):
            break
        elif tecla == ord(' ') and encontrado:
            objpoints.append(objp_base)
            imgpoints.append(esquinas)
            print(f"  📸  Captura {len(objpoints)} registrada")
        elif tecla == ord('c'):
            if len(objpoints) >= 1:
                break
            else:
                print("  ⚠  Necesitas al menos 1 captura válida")

    cap.release()
    cv.destroyAllWindows()

    if not objpoints:
        print("No se realizaron capturas. Saliendo.")
        return

    if len(objpoints) < MIN_IMAGENES:
        print(f"\n⚠  Solo {len(objpoints)} captura(s). "
              f"Se recomiendan {MIN_IMAGENES} para mejor precisión.")

    print(f"\nCalibrando con {len(objpoints)} captura(s)...")
    datos = calibrar(objpoints, imgpoints, shape_gris)
    mostrar_resumen(datos)
    guardar_calibracion(datos, ARCHIVO_SALIDA)


# ══════════════════════════════════════════════
#  MODO 3: Corregir distorsión en una imagen
# ══════════════════════════════════════════════
def modo_undistort(ruta_imagen, archivo_cal=ARCHIVO_SALIDA):
    if not os.path.exists(archivo_cal):
        print(f"❌  No se encontró '{archivo_cal}'. Primero calibra la cámara.")
        return

    mtx, dist, datos = cargar_calibracion(archivo_cal)
    img = cv.imread(ruta_imagen)
    if img is None:
        print(f"❌  No se pudo leer: {ruta_imagen}")
        return

    h, w = img.shape[:2]

    # Matriz de cámara óptima (alpha=1 → conserva todos los píxeles)
    nueva_mtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

    # Método 1: undistort directo
    dst1 = cv.undistort(img, mtx, dist, None, nueva_mtx)

    # Método 2: remapping (equivalente, a veces más rápido en vídeo)
    mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, nueva_mtx, (w, h), cv.CV_32FC1)
    dst2 = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)

    # Recortar región válida
    x, y, rw, rh = roi
    dst_crop = dst1[y:y+rh, x:x+rw]

    # Guardar resultados
    base = Path(ruta_imagen).stem
    cv.imwrite(f"{base}_sin_distorsion.png",   dst1)
    cv.imwrite(f"{base}_recortada.png",        dst_crop)

    print(f"✅  Imágenes guardadas:")
    print(f"   {base}_sin_distorsion.png")
    print(f"   {base}_recortada.png")

    # Mostrar comparación
    comparacion = np.hstack([
        cv.resize(img,   (w // 2, h // 2)),
        cv.resize(dst1,  (w // 2, h // 2)),
    ])
    cv.putText(comparacion, "Original",        (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
    cv.putText(comparacion, "Sin distorsion",  (w//2+10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0,200,0), 2)
    cv.imshow("Comparacion (cualquier tecla para salir)", comparacion)
    cv.waitKey(0)
    cv.destroyAllWindows()


# ──────────────────────────────────────────────
#  PUNTO DE ENTRADA
# ──────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Calibración de cámara con OpenCV",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument(
        "--modo", choices=["imagenes", "camara", "undistort"],
        default="camara",
        help=(
            "imagenes  → calibrar desde archivos en disco\n"
            "camara    → calibrar capturando desde webcam\n"
            "undistort → corregir una imagen con calibración ya guardada"
        ),
    )
    parser.add_argument("--patron",  default="*.jpg",
                        help="Glob de imágenes (modo imagenes). Ej: fotos/*.jpg")
    parser.add_argument("--imagen",  default="",
                        help="Ruta de imagen a corregir (modo undistort)")
    parser.add_argument("--cam",     type=int, default=0,
                        help="Índice de cámara (modo camara, por defecto 0)")
    parser.add_argument("--cal",     default=ARCHIVO_SALIDA,
                        help=f"Archivo JSON de calibración (por defecto: {ARCHIVO_SALIDA})")

    args = parser.parse_args()

    if args.modo == "imagenes":
        modo_imagenes(args.patron)
    elif args.modo == "camara":
        modo_camara(args.cam)
    elif args.modo == "undistort":
        if not args.imagen:
            parser.error("--imagen es obligatorio en modo undistort")
        modo_undistort(args.imagen, args.cal)


if __name__ == "__main__":
    main()