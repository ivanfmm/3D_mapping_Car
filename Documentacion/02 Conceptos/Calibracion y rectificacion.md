---
tags: [nivel/2, nivel/3, tipo/concepto]
aliases: [Calibración, Rectificación, Matriz K, estereo_cal.json]
---
# Calibración y rectificación

> [!info] Nivel 2 → 3 · Concepto base. Lo usan: [[03 Flujo de datos]], [[Camara estereo y disparidad]] · A fondo: [[Calibracion estereo a fondo]]

## Nivel 1 en dos líneas

Cada lente deforma un poco la imagen y no está perfectamente alineado con el otro. **Calibrar** es medir esos defectos con fotos de un tablero de ajedrez. **Rectificar** es "enderezar" las imágenes para que un mismo punto quede en **la misma fila** en ambos lentes.

## Qué guarda `estereo_cal.json`

Se genera una sola vez con `camera_calibration_dualCamera.py`, usando 30 pares de fotos de un tablero de **10×7 esquinas internas** con cuadros de **20 mm**:

| Campo | Qué es | Valor (lente izquierdo, 1280×720) |
|---|---|---|
| `matriz_camara` (K) | Focal y centro óptico en píxeles | $f_x = 871.4$, $f_y = 870.6$, $c_x = 646.6$, $c_y = 351.4$ |
| `coef_distorsion` | Curvatura del lente ($k_1, k_2, p_1, p_2, k_3$) | |
| `rms` | Error promedio de reproyección | 0.38 px (menos de 0.5 es bueno) |
| `estereo.R`, `estereo.T` | Rotación y traslación del lente derecho respecto al izquierdo | $T \approx (-59.4,\ 0,\ -1)$ mm |
| `baseline_mm` | Separación entre lentes | **59.4 mm** |
| `rms_estereo` | Error de la calibración conjunta | 0.48 px |

## La matriz K (nivel 3)

Proyecta un punto 3D de la cámara $(X, Y, Z)$ a un píxel $(u, v)$:

$$
\begin{bmatrix} u \\ v \\ 1 \end{bmatrix} \sim
\underbrace{\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}}_{K}
\begin{bmatrix} X/Z \\ Y/Z \\ 1 \end{bmatrix}
\quad\Rightarrow\quad u = f_x \frac{X}{Z} + c_x,\ \ v = f_y \frac{Y}{Z} + c_y
$$

Esta es la ecuación que **PnP** invierte para recuperar el movimiento. Ver [[PnP RANSAC]].

## Rectificación en el código (`load_rectification`)

```python
R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(K_izq, D_izq, K_der, D_der, size, R, T, alpha=0)
maps_izq = cv2.initUndistortRectifyMap(K_izq, D_izq, R1, P1, size, cv2.CV_32FC1)
# ... luego, en cada frame:
rect_izq = cv2.remap(raw_izq, map1_izq, map2_izq, cv2.INTER_LINEAR)
```

1. `stereoRectify` calcula las rotaciones `R1` y `R2` que hacen paralelas a las dos cámaras virtuales, las nuevas matrices de proyección `P1` y `P2`, y la matriz **Q** (ver [[Camara estereo y disparidad]]).
2. `initUndistortRectifyMap` precalcula **para cada píxel de salida de qué píxel de entrada leer**. Es una tabla de búsqueda: se calcula una vez y es muy rápida.
3. `remap` aplica esa tabla en cada frame.
4. `K_rect = P1[:3, :3]` es la K de la imagen **ya rectificada**. Es la que usan ORB/PnP.

> [!tip] Si la resolución cambia
> `load_rectification` escala la K: si calibraste a 1280×720 y recibes 640×360, multiplica $f_x, c_x$ por $s_x = 0.5$ y $f_y, c_y$ por $s_y = 0.5$. Funciona, pero la distorsión fue medida a otra resolución. Lo ideal es **calibrar a la misma resolución con la que se graba**, como ahora (1280×720).

> [!warning] Hallazgo de la calibración actual
> El modelo de distorsión no tiene solución en las esquinas de la imagen, y eso desplaza el centro de la imagen rectificada (≈ 857, 303) y recorta una franja del lado derecho. Detalle y corrección en [[Calibracion estereo a fondo]].

## Por qué importa la rectificación

Después de rectificar, las **líneas epipolares son horizontales**: el gemelo de un punto de la imagen izquierda está en la **misma fila** de la derecha. RAFT solo busca en una dimensión, y la disparidad se reduce a una resta de coordenadas $u$.
