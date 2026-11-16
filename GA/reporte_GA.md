# Reporte del algoritmo genetico (2026-11-15 20:32)

## Resultado

| Configuracion | Fitness | ATE (m) | RPE trasl. (m) | RPE rot. (grados) | Cobertura |
|---|---|---|---|---|---|
| linea base (DEFAULT_VO_PARAMS) | 12.0275 | 11.9228 | 0.0834 | 0.2126 | 1.000 |
| mejor previo (best_vo_params.json del repo) | 5.2728 | 5.1477 | 0.0974 | 0.2768 | 1.000 |
| mejor de la corrida anterior (resultados_20261115_195049) | 5.0197 | 4.8957 | 0.0967 | 0.2727 | 1.000 |
| **mejor de esta corrida** | **4.7799** | 4.6662 | 0.0876 | 0.2610 | 1.000 |

Todas las filas se evaluaron en esta corrida, con los mismos frames y el mismo codigo, asi que son comparables entre si. Fitness = (ATE + RPE_trasl + 0.1 * RPE_rot) / cobertura^2; menor es mejor.

## Tiempo

- Limite pedido: 30 min. Tiempo real total: **27.3 min** (AG: 27.0 min; el resto es medicion y carga).
- Fin: corte por reloj tras la generacion 39: la siguiente (~99 s) no cabia en el limite.
- Evaluaciones: 411 (2 invalidas por cobertura < 50 % o error).

## Calculo del presupuesto

- Equipo: Intel64 Family 6 Model 154 Stepping 3, GenuineIntel, 20 hilos logicos, RAM libre 33.0 de 39.7 GB.
- Medicion en un solo proceso sobre 120 frames: 35.1 ms/frame con los parametros base y 118.0 ms/frame con el caso pesado (3000 esquinas, 10 niveles, 500 iteraciones de PnP).
- Tiempo planeado por frame en paralelo: 142.4 ms (0.35 base + 0.65 pesado, por 1.6 de perdida al usar todos los nucleos).
- Memoria por worker: 1.90 GB (1.30 MB por frame). Workers: 14 (CPU permite 12, RAM permite 12).
- Plan elegido: 1101 frames, poblacion 14, 8 generaciones estimadas (se siguen corriendo generaciones mientras quepan en el limite), ~157 s por evaluacion, total estimado 23.9 min de 26.4 planeables (88% del limite).
- Real: generacion 0 en 147 s; las demas en promedio 38 s (estimado 157 s).

| Frames | Poblacion | Generaciones que caben | Total estimado (min) |
|---|---|---|---|
| 1101 | 28 | 3 | 21.3 |
| 1101 | 14 | 8 | 23.9 |
| 800 | 28 | 5 | 23.1 |
| 800 | 14 | 12 | 25.0 |
| 550 | 28 | 9 | 26.4 |
| 550 | 14 | 19 | 26.4 |
| 300 | 28 | 17 | 25.9 |
| 300 | 14 | 35 | 25.9 |

## Configuracion del AG

- Cruce SBX (eta 15, prob. 0.75), mutacion polinomial (eta 20, indpb 0.25, prob. 0.35), torneo de 3, elitismo de 2, semilla 42.
- Rangos de busqueda: ampliados (--amplio).
- Poblacion inicial sembrada con: linea base (DEFAULT_VO_PARAMS); mejor previo (best_vo_params.json del repo); mejor de la corrida anterior (resultados_20261115_195049).

## Convergencia

| Gen | Evaluados | Mejor fitness | ATE (m) | Fitness medio | Validos | Tiempo (s) |
|---|---|---|---|---|---|---|
| 0 | 14 | 5.0197 | 4.8957 | 24.1023 | 12/14 | 147 |
| 1 | 11 | 5.0197 | 4.8957 | 20.3250 | 14/14 | 99 |
| 2 | 10 | 5.0197 | 4.8957 | 6.9138 | 14/14 | 69 |
| 3 | 8 | 5.0197 | 4.8957 | 5.5481 | 14/14 | 29 |
| 4 | 10 | 5.0197 | 4.8957 | 5.2860 | 14/14 | 38 |
| 5 | 12 | 5.0194 | 4.8954 | 5.1465 | 14/14 | 41 |
| 6 | 10 | 5.0194 | 4.8954 | 5.3507 | 14/14 | 34 |
| 7 | 8 | 5.0194 | 4.8954 | 5.1680 | 14/14 | 30 |
| 8 | 10 | 5.0194 | 4.8954 | 5.3934 | 14/14 | 36 |
| 9 | 10 | 5.0194 | 4.8954 | 5.4073 | 14/14 | 47 |
| 10 | 8 | 5.0194 | 4.8954 | 5.1492 | 14/14 | 30 |
| 11 | 10 | 5.0194 | 4.8954 | 5.0194 | 14/14 | 33 |
| 12 | 10 | 5.0194 | 4.8954 | 6.8137 | 14/14 | 32 |
| 13 | 8 | 5.0194 | 4.8954 | 8.9316 | 14/14 | 30 |
| 14 | 9 | 5.0194 | 4.8954 | 5.4571 | 14/14 | 32 |
| 15 | 11 | 5.0194 | 4.8954 | 7.2968 | 14/14 | 35 |
| 16 | 9 | 5.0194 | 4.8954 | 5.1665 | 14/14 | 32 |
| 17 | 11 | 5.0194 | 4.8954 | 5.7618 | 14/14 | 37 |
| 18 | 11 | 5.0194 | 4.8954 | 5.6702 | 14/14 | 34 |
| 19 | 9 | 5.0194 | 4.8954 | 5.8604 | 14/14 | 40 |
| 20 | 12 | 5.0194 | 4.8954 | 5.7514 | 14/14 | 38 |
| 21 | 12 | 5.0194 | 4.8954 | 5.7583 | 14/14 | 37 |
| 22 | 12 | 5.0131 | 4.9007 | 5.4854 | 14/14 | 37 |
| 23 | 12 | 4.9885 | 4.8760 | 5.2641 | 14/14 | 37 |
| 24 | 12 | 4.9885 | 4.8760 | 5.5327 | 14/14 | 38 |
| 25 | 11 | 4.9885 | 4.8760 | 5.9459 | 14/14 | 35 |
| 26 | 10 | 4.9885 | 4.8760 | 5.5112 | 14/14 | 33 |
| 27 | 12 | 4.9885 | 4.8760 | 5.5393 | 14/14 | 36 |
| 28 | 9 | 4.9885 | 4.8760 | 5.5341 | 14/14 | 32 |
| 29 | 10 | 4.9885 | 4.8760 | 5.6673 | 14/14 | 34 |
| 30 | 8 | 4.9885 | 4.8760 | 5.4689 | 14/14 | 31 |
| 31 | 8 | 4.9885 | 4.8760 | 5.7479 | 14/14 | 38 |
| 32 | 10 | 4.9885 | 4.8760 | 5.3347 | 14/14 | 34 |
| 33 | 9 | 4.9885 | 4.8760 | 8.1529 | 14/14 | 32 |
| 34 | 10 | 4.9885 | 4.8760 | 5.5743 | 14/14 | 35 |
| 35 | 10 | 4.7799 | 4.6662 | 6.2751 | 14/14 | 34 |
| 36 | 12 | 4.7799 | 4.6662 | 5.3516 | 14/14 | 37 |
| 37 | 10 | 4.7799 | 4.6662 | 5.7855 | 14/14 | 35 |
| 38 | 11 | 4.7799 | 4.6662 | 5.6265 | 14/14 | 40 |
| 39 | 12 | 4.7799 | 4.6662 | 5.0102 | 14/14 | 39 |

## Hiperparametros

| Parametro | Base | Mejor | Rango de busqueda | Nota |
|---|---|---|---|---|
| `orb_nfeatures` | 1500 | **1157** | 300 - 3000 |  |
| `orb_scaleFactor` | 1.2 | **1.457** | 1.1 - 2 |  |
| `orb_nlevels` | 6 | **5** | 2 - 10 |  |
| `orb_patchSize` | 19 | **32** | 15 - 48 |  |
| `orb_fastThreshold` | 10 | **18** | 5 - 40 |  |
| `lowe_ratio` | 0.75 | **0.6149** | 0.4 - 0.9 |  |
| `disp_min` | 2.0 | **9.9935** | 1 - 30 |  |
| `pnp_iterations` | 200 | **382** | 100 - 800 |  |
| `pnp_reproj_err` | 2.0 | **2.8054** | 0.5 - 8 |  |
| `min_pnp_inliers` | 12 | **33** | 8 - 80 |  |
| `floor_band` | 0.35 | **0.3191** | 0.2 - 0.45 |  |
| `floor_ransac_mm` | 40.0 | **22.8674** | 15 - 80 |  |

Acoplados: `orb_edgeThreshold` = `orb_patchSize` y `min_pnp_matches` = `min_pnp_inliers`. Un valor pegado al limite sugiere que el optimo puede estar fuera del rango.

## Los 5 mejores

| # | Fitness | ATE (m) | Parametros |
|---|---|---|---|
| 1 | 4.7799 | 4.6662 | orb_nfeatures=1157, orb_scaleFactor=1.457, orb_nlevels=5, orb_patchSize=32, orb_fastThreshold=18, lowe_ratio=0.6149, disp_min=9.9935, pnp_iterations=382, pnp_reproj_err=2.8054, min_pnp_inliers=33, floor_band=0.3191, floor_ransac_mm=22.8674 |
| 2 | 4.7799 | 4.6662 | orb_nfeatures=1157, orb_scaleFactor=1.457, orb_nlevels=5, orb_patchSize=32, orb_fastThreshold=18, lowe_ratio=0.6149, disp_min=9.9935, pnp_iterations=382, pnp_reproj_err=2.8054, min_pnp_inliers=33, floor_band=0.3191, floor_ransac_mm=22.8674 |
| 3 | 4.7799 | 4.6662 | orb_nfeatures=1157, orb_scaleFactor=1.457, orb_nlevels=5, orb_patchSize=32, orb_fastThreshold=18, lowe_ratio=0.6149, disp_min=9.9935, pnp_iterations=382, pnp_reproj_err=2.8054, min_pnp_inliers=33, floor_band=0.3191, floor_ransac_mm=22.8674 |
| 4 | 4.7799 | 4.6662 | orb_nfeatures=1157, orb_scaleFactor=1.457, orb_nlevels=5, orb_patchSize=32, orb_fastThreshold=18, lowe_ratio=0.6149, disp_min=9.9935, pnp_iterations=382, pnp_reproj_err=2.8054, min_pnp_inliers=33, floor_band=0.3191, floor_ransac_mm=22.8674 |
| 5 | 4.7799 | 4.6662 | orb_nfeatures=1157, orb_scaleFactor=1.457, orb_nlevels=5, orb_patchSize=32, orb_fastThreshold=18, lowe_ratio=0.6149, disp_min=9.9935, pnp_iterations=382, pnp_reproj_err=2.8054, min_pnp_inliers=33, floor_band=0.3191, floor_ransac_mm=22.8674 |

## Archivos

- `best_vo_params.json`: mejor individuo.
- `ga_history.csv`: una fila por generacion.
- `individuos.csv`: todas las evaluaciones, con sus parametros.

## Las dos tandas

| Tanda | Rangos | Limite | Tiempo real | Evaluaciones | Mejor fitness | ATE (m) |
|---|---|---|---|---|---|---|
| 1 (`resultados_20261115/reporte_tanda1.md`) | originales | 24 min | 11.1 min | 108 | 5.0197 | 4.8957 |
| 2 (este reporte) | ampliados | 30 min | 27.3 min | 411 | 4.7799 | 4.6662 |

En la tanda 1 `disp_min` y `pnp_reproj_err` quedaron pegados al tope de su rango. Al ampliar los rangos,
`pnp_reproj_err` bajo a 2.81 y `min_pnp_inliers` a 33 (ya no estan en ningun limite) y `disp_min` se quedo
en 9.99 aun pudiendo llegar a 30, asi que ese valor no era un efecto del rango.

Todo esta optimizado sobre una sola secuencia (KITTI 07): las diferencias chicas entre individuos
(centesimas de fitness) pueden ser ajuste a esa secuencia y no una mejora real.

## Aplicacion al carrito (`visual_odometry.py`)

Los hiperparametros salen de KITTI 07 (auto en la calle, 537.9 mm entre camaras, ~1 m por frame).
El carrito tiene 59.4 mm entre camaras y avanza milimetros por frame, asi que solo se copiaron
los que no dependen de la escala de la escena.

| Parametro del AG | Constante en `visual_odometry.py` | Antes | Ahora | Aplicado |
|---|---|---|---|---|
| `orb_nfeatures` | `ORB_NFEATURES` | 1000 | 1157 | si |
| `orb_scaleFactor` | `ORB_SCALE_FACTOR` | 1.2 | 1.457 | si |
| `orb_nlevels` | `ORB_NLEVELS` | 6 | 5 | si |
| `orb_patchSize` / `orb_edgeThreshold` | `ORB_PATCH_SIZE` | 19 | 32 | si |
| `orb_fastThreshold` | `ORB_FAST_THRESHOLD` | 10 | 18 | si |
| `lowe_ratio` | `LOWE_RATIO` | 0.75 | 0.6149 | si |
| `pnp_iterations` | `PNP_ITERATIONS` | 200 | 382 | si |
| `pnp_reproj_err` | `PNP_REPROJ_ERR` | 2.0 | 2.8054 | si |
| `min_pnp_inliers` / `min_pnp_matches` | `MIN_PNP_INLIERS` / `MIN_PNP_MATCHES` | 12 | 33 | si |
| `disp_min` | `VO_DISP_MIN` (nueva) | 2.0 | 9.9935 | si (~5.4 m en el carrito) |
| `floor_band` | `FLOOR_BAND` | 0.35 | 0.3191 | si |
| `floor_ransac_mm` | `FLOOR_RANSAC_MM` | 10.0 | 10.0 | **no**: depende de la escala (KITTI: camara a 1.65 m del piso) |

Pendiente: validar con una sesion real del carrito (`python reprocesar.py --sparse`) y comparar
frames con pose valida y recorrido contra los valores anteriores. Riesgos a vigilar en interiores
con poca textura: `ORB_FAST_THRESHOLD = 18` detecta menos esquinas y `MIN_PNP_INLIERS = 33` rechaza
mas frames que el 12 anterior.

## Diferencias entre `GA/VO.py` y `visual_odometry.py`

El AG optimizo sobre `GA/VO.py`, que es la odometria anterior a las correcciones de esta rama:
no tiene la profundidad subpixel de `_deproject` ni el tope `PNP_Z_MAX`. Lo unico que se le cambio
fue la SVD de la deteccion de piso (`full_matrices=False`), que reservaba varios GB de RAM por
worker y tumbaba la corrida; el resultado numerico es el mismo.
