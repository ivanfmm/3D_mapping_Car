# Reporte del algoritmo genetico (2026-11-15 20:01)

## Resultado

| Configuracion | Fitness | ATE (m) | RPE trasl. (m) | RPE rot. (grados) | Cobertura |
|---|---|---|---|---|---|
| linea base (DEFAULT_VO_PARAMS) | 12.0275 | 11.9228 | 0.0834 | 0.2126 | 1.000 |
| mejor previo (best_vo_params.json del repo) | 5.2728 | 5.1477 | 0.0974 | 0.2768 | 1.000 |
| **mejor de esta corrida** | **5.0197** | 4.8957 | 0.0967 | 0.2727 | 1.000 |

Todas las filas se evaluaron en esta corrida, con los mismos frames y el mismo codigo, asi que son comparables entre si. Fitness = (ATE + RPE_trasl + 0.1 * RPE_rot) / cobertura^2; menor es mejor.

## Tiempo

- Limite pedido: 24 min. Tiempo real total: **11.1 min** (AG: 10.8 min; el resto es medicion y carga).
- Fin: se completaron las generaciones planeadas.
- Evaluaciones: 108 (0 invalidas por cobertura < 50 % o error).

## Calculo del presupuesto

- Equipo: Intel64 Family 6 Model 154 Stepping 3, GenuineIntel, 20 hilos logicos, RAM libre 33.4 de 39.7 GB.
- Medicion en un solo proceso sobre 120 frames: 28.7 ms/frame con los parametros base y 89.0 ms/frame con el caso pesado (3000 esquinas, 10 niveles, 500 iteraciones de PnP).
- Tiempo planeado por frame en paralelo: 108.7 ms (0.35 base + 0.65 pesado, por 1.6 de perdida al usar todos los nucleos).
- Memoria por worker: 1.90 GB (1.30 MB por frame). Workers: 14 (CPU permite 12, RAM permite 12).
- Plan elegido: 1101 frames, poblacion 14, hasta 9 generaciones, ~120 s por evaluacion, total estimado 20.4 min de 21.1 planeables (88% del limite).
- Real: generacion 0 en 203 s; las demas en promedio 50 s (estimado 120 s).

| Frames | Poblacion | Generaciones que caben | Total estimado (min) |
|---|---|---|---|
| 1101 | 28 | 4 | 20.4 |
| 1101 | 14 | 9 | 20.4 |
| 800 | 28 | 6 | 20.6 |
| 800 | 14 | 13 | 20.6 |
| 550 | 28 | 9 | 20.2 |
| 550 | 14 | 19 | 20.2 |
| 300 | 28 | 18 | 20.9 |
| 300 | 14 | 37 | 20.9 |

## Configuracion del AG

- Cruce SBX (eta 15, prob. 0.75), mutacion polinomial (eta 20, indpb 0.25, prob. 0.35), torneo de 3, elitismo de 2, semilla 42.
- Poblacion inicial sembrada con: linea base (DEFAULT_VO_PARAMS); mejor previo (best_vo_params.json del repo).

## Convergencia

| Gen | Evaluados | Mejor fitness | ATE (m) | Fitness medio | Validos | Tiempo (s) |
|---|---|---|---|---|---|---|
| 0 | 14 | 5.2728 | 5.1477 | 10.7077 | 14/14 | 203 |
| 1 | 11 | 5.2728 | 5.1477 | 8.6467 | 14/14 | 104 |
| 2 | 10 | 5.2728 | 5.1477 | 7.4630 | 14/14 | 100 |
| 3 | 10 | 5.0833 | 4.9588 | 5.5925 | 14/14 | 33 |
| 4 | 10 | 5.0833 | 4.9588 | 5.4107 | 14/14 | 33 |
| 5 | 9 | 5.0298 | 4.9053 | 5.4725 | 14/14 | 32 |
| 6 | 12 | 5.0255 | 4.9015 | 5.2554 | 14/14 | 40 |
| 7 | 8 | 5.0197 | 4.8957 | 5.3806 | 14/14 | 32 |
| 8 | 12 | 5.0197 | 4.8957 | 5.6273 | 14/14 | 36 |
| 9 | 12 | 5.0197 | 4.8957 | 5.1458 | 14/14 | 37 |

## Hiperparametros

| Parametro | Base | Mejor | Rango de busqueda | Nota |
|---|---|---|---|---|
| `orb_nfeatures` | 1500 | **1157** | 500 - 3000 |  |
| `orb_scaleFactor` | 1.2 | **1.4844** | 1.1 - 1.5 |  |
| `orb_nlevels` | 6 | **5** | 4 - 10 |  |
| `orb_patchSize` | 19 | **32** | 15 - 35 |  |
| `orb_fastThreshold` | 10 | **18** | 5 - 25 |  |
| `lowe_ratio` | 0.75 | **0.6149** | 0.6 - 0.9 |  |
| `disp_min` | 2.0 | **9.9935** | 1 - 10 | en el limite superior |
| `pnp_iterations` | 200 | **418** | 100 - 500 |  |
| `pnp_reproj_err` | 2.0 | **3.9744** | 0.5 - 4 | en el limite superior |
| `min_pnp_inliers` | 12 | **38** | 8 - 40 |  |
| `floor_band` | 0.35 | **0.3205** | 0.2 - 0.45 |  |
| `floor_ransac_mm` | 40.0 | **25.784** | 15 - 80 |  |

Acoplados: `orb_edgeThreshold` = `orb_patchSize` y `min_pnp_matches` = `min_pnp_inliers`. Un valor pegado al limite sugiere que el optimo puede estar fuera del rango.

## Los 5 mejores

| # | Fitness | ATE (m) | Parametros |
|---|---|---|---|
| 1 | 5.0197 | 4.8957 | orb_nfeatures=1157, orb_scaleFactor=1.4844, orb_nlevels=5, orb_patchSize=32, orb_fastThreshold=18, lowe_ratio=0.6149, disp_min=9.9935, pnp_iterations=418, pnp_reproj_err=3.9744, min_pnp_inliers=38, floor_band=0.3205, floor_ransac_mm=25.784 |
| 2 | 5.0197 | 4.8957 | orb_nfeatures=1157, orb_scaleFactor=1.4844, orb_nlevels=5, orb_patchSize=32, orb_fastThreshold=18, lowe_ratio=0.6149, disp_min=9.9935, pnp_iterations=418, pnp_reproj_err=3.9744, min_pnp_inliers=38, floor_band=0.3197, floor_ransac_mm=25.784 |
| 3 | 5.0197 | 4.8957 | orb_nfeatures=1157, orb_scaleFactor=1.4844, orb_nlevels=5, orb_patchSize=32, orb_fastThreshold=18, lowe_ratio=0.6149, disp_min=9.9935, pnp_iterations=418, pnp_reproj_err=3.9744, min_pnp_inliers=38, floor_band=0.3203, floor_ransac_mm=25.784 |
| 4 | 5.0255 | 4.9015 | orb_nfeatures=1157, orb_scaleFactor=1.4844, orb_nlevels=5, orb_patchSize=32, orb_fastThreshold=18, lowe_ratio=0.6149, disp_min=9.9935, pnp_iterations=418, pnp_reproj_err=3.9744, min_pnp_inliers=38, floor_band=0.3334, floor_ransac_mm=25.784 |
| 5 | 5.0255 | 4.9015 | orb_nfeatures=1157, orb_scaleFactor=1.4844, orb_nlevels=5, orb_patchSize=32, orb_fastThreshold=18, lowe_ratio=0.6149, disp_min=9.9935, pnp_iterations=418, pnp_reproj_err=3.9744, min_pnp_inliers=38, floor_band=0.3334, floor_ransac_mm=25.784 |

## Archivos

- `best_vo_params.json`: mejor individuo.
- `ga_history.csv`: una fila por generacion.
- `individuos.csv`: todas las evaluaciones, con sus parametros.
