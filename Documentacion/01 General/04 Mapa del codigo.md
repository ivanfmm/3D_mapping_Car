---
tags: [nivel/2, tipo/general]
aliases: [Mapa del código, Archivos del proyecto]
---
# Mapa del código

> [!info] Nivel 2 · Anterior: [[03 Flujo de datos]] · Siguiente: [[VO 1 Intuicion]]

Qué contiene cada archivo, sus funciones principales y en qué nota se explica cada una.

## En la Raspberry Pi

### `server_2.py`: servidor del carro

Detalle en [[Control del carrito]].

| Elemento | Qué hace |
|---|---|
| `COMMANDS` | Diccionario tecla → PWM de las 4 ruedas. Ejemplo: `'a': (-900, -900, 900, 900)` gira sobre su eje |
| `SPEED_FWD = 600`, `SPEED_TURN = 900` | Velocidades (PWM de 0 a 4095) |
| `handle_commands(conn)` | Lee 1 byte a la vez. Implementa **mantener presionado** (`HOLD_S`) y el **watchdog** (`WATCHDOG_S`). Solo escribe al PCA9685 cuando el comando cambia |
| `stream_video(conn)` | Abre la cámara en MJPG 2560×720 a 30 FPS, comprime cada frame a JPEG calidad 80 y lo manda con su tamaño delante |
| `main()` | Abre los puertos 9999 (comandos) y 9998 (video) y lanza un hilo para cada uno |

### `motor.py`: clase `Ordinary_Car`

| Método | Qué hace |
|---|---|
| `set_motor_model(d1, d2, d3, d4)` | Pone el PWM de las 4 ruedas: izquierda-arriba, izquierda-abajo, derecha-arriba, derecha-abajo |
| `duty_range(...)` | Recorta cada valor a ±4095 |
| `left_upper_wheel(duty)`, etc. | Signo positivo = adelante, negativo = atrás. **0 = freno activo**: pone ambos canales del puente H en 4095 |

### `pca9685.py`: driver del chip PCA9685

Habla por **I2C** (dirección `0x40`) con un chip que genera 16 señales PWM de **12 bits** (de ahí el rango 0 a 4095). El carro lo configura a 50 Hz.

## En la PC

### `client_2.py`: cliente, SLAM offline y nube

| Función | Qué hace | Nota |
|---|---|---|
| `recv_exact(sock, n)` | Lee exactamente `n` bytes de un socket TCP | [[02 Arquitectura general]] |
| `read_one_frame(sock)` | Lee `[4 bytes de tamaño][JPEG]` y lo decodifica | [[02 Arquitectura general]] |
| `load_rectification(cal, w, h)` | Lee `estereo_cal.json`, **escala K** si la resolución cambió y calcula los mapas de rectificación, `Q` y `K_rect` | [[Calibracion y rectificacion]] |
| `build_model_args`, `load_raft_model` | Configuran y cargan RAFT-Stereo en la GPU | [[RAFT 2 En el proyecto]] |
| `pad32(img)` | Rellena la imagen a múltiplos de 32 (RAFT lo necesita) | [[RAFT 2 En el proyecto]] |
| `run_raft(model, args, izq, der)` | Devuelve la disparidad (H×W, en px). RAFT da un "flujo" negativo y aquí se le cambia el signo | [[RAFT 2 En el proyecto]] |
| `create_session`, `save_worker`, `_save_frame` | Crean la carpeta de la sesión y escriben JPG + CSV | [[03 Flujo de datos]] |
| `handle_commands(cmd_sock)` | Teclado → Pi, más el latido `'h'` cada 0.25 s | [[02 Arquitectura general]] |
| `video_loop(...)` | Recibe, parte, rectifica y encola | [[03 Flujo de datos]] |
| `slam_offline(...)` | Fase 2: pipeline Lector → RAFT → VO | [[02 Arquitectura general]] |
| `build_point_cloud(sess, vo, Q)` | Fase 3: arma, filtra y alinea la nube | [[Nube 2 En el proyecto]] |
| `main()` | Orquesta todo: conexión, hilos, fases | |

### `visual_odometry.py`: el cerebro de la ubicación

| Elemento | Qué hace | Nota |
|---|---|---|
| `guardar_poses(ruta, poses)` | Escribe `poses.txt` | |
| `mat_to_vec(T)` / `vec_to_mat(v)` | Convierten una matriz 4×4 a 6 números (Rodrigues + traslación) y de regreso | [[Pose y matrices 4x4]] |
| `_LoopDetector` | Índice **FAISS** con las huellas ORB de los keyframes, para reconocer lugares ya visitados | [[LC 3 Deteccion de candidatos]] |
| `_optimize_pose_graph(...)` | PGO con `scipy.optimize.least_squares` | [[LC 6 Optimizacion]] |
| `VisualOdometry.__init__` | Crea el ORB, el matcher y el estado | [[ORB 2 En el proyecto]] |
| `VisualOdometry.update(frame, disp, idx)` | **Función central**: ORB → matching → 3D → PnP → acumular pose → loop closure | [[VO 2 En el proyecto]] |
| `_deproject(pts2d, disp)` | Píxel + disparidad → punto 3D usando `Q` | [[VO 3 Profundidad de los puntos]] |
| `_detect_floor`, `_set_floor`, `_to_planar` | Movimiento plano: detectan el piso y quitan subir/bajar, cabeceo y alabeo | [[VO 5 Movimiento plano]] |
| `_check_loop_closure`, `_verify_loop`, `_run_pgo` | Loop closure | [[LC 2 En el proyecto]] |
| `_icp_refine` | Refinamiento con ICP (**apagado**: `USE_ICP = False`) | |
| `get_frame_points(idx)` | Puntos ORB de un frame en 2D y 3D (para la nube sparse) | [[Nube 7 Nube sparse]] |
| `_fallo(...)` | Maneja cualquier frame fallido: no lo vuelve referencia y lo quita de `all_poses`; reinicia tras `MAX_FALLOS` | [[VO 6 Pose acumulada y fallos]] |
| `_add_loop_keyframe(idx, des)` | Agrega huellas al índice FAISS solo si el carro se movió 50 mm o giró 5° | [[LC 3 Deteccion de candidatos]] |

### Herramientas

| Archivo | Qué hace |
|---|---|
| `camera_calibration_dualCamera.py` | Tres modos: **captura** (fotos del tablero), **calibrar** (`calibrateCamera` por lente + `stereoCalibrate`) y **rectificar** (prueba). Tablero de 10×7 esquinas internas con cuadros de 20 mm. Ver [[Calibracion estereo a fondo]] |
| `reprocesar.py` | Repite las fases 2 y 3 sin el carro: filtro de borrosas (varianza del Laplaciano), VO, nube sparse con puntos ORB y nube densa opcional ([[Nube 2 En el proyecto]]). Tiene banderas para comparar: `--icp`, `--sin-plano`, `--blur`, `--zmax`, `--inclinacion` |
| `ver_nube.py` | Abre uno o varios `.ply` |
| `test_camera.py` | Abre la cámara y la muestra |

## Observaciones de revisión (deuda técnica)

> [!warning] Cosas que encontré al leer el código
> 1. **Parámetros pensados para 640×360, pero el stream es 1280×720 por lente.** Los comentarios de `nlevels=6` y `PNP_REPROJ_ERR = 2.0` dicen "640x360". Con 4 veces más píxeles, conviene volver a medir esos valores. Ver [[ORB 2 En el proyecto]].
> 2. ~~**El índice de FAISS crece sin límite.**~~ **Corregido el 18-oct-2026:** ahora solo entran keyframes. Ver [[ORB 6 Hamming y matching]].
> 2b. ~~**Los frames fallidos perdían su movimiento.**~~ **Corregido el 18-oct-2026** con `_fallo()`. Ver [[VO 6 Pose acumulada y fallos]].
> 2c. **Loop closure apagado** (`USE_LOOP_CLOSURE = False`): nunca se disparaba y, al revivirlo, dio loops falsos que desarman el PGO. Pendiente: verificación más estricta y pérdida robusta. Ver [[ORB 6 Hamming y matching]] y [[LC 2 En el proyecto]].
> 3. **Docstrings desactualizados.** `client_2.py` dice ser `client_3.py`, y el encabezado de `visual_odometry.py` todavía menciona L-BFGS-B.
> 4. **Código muerto en `build_point_cloud`:** `ICP_MIN_FIT`, `skipped_icp` y `prev_pcd` ya no se usan, y quedan bloques de DBSCAN comentados.
> 5. **Mezcla de fines de línea (CRLF/LF)** en varios archivos. Inflan los diffs de git.
> 6. **Grafo de poses partido tras un reinicio por `MAX_FALLOS`:** el frame donde se reinicia la referencia no recibe arista desde el último frame bueno. Ver [[LC 5 Grafo de poses]].
> 7. **La alineación del piso no verifica que el plano sea el piso:** `segment_plane` devuelve el plano más grande, que puede ser una pared. Ver [[Nube 6 Piso y alineacion]].
> 8. **`error_reproyeccion` en `estereo_cal.json` está mal calculado** (sale ≈ 0.045 px, el real es ≈ 0.38 px). Ver [[Calibracion estereo a fondo]].
> 9. **El modelo de distorsión no tiene solución en las esquinas de la imagen:** desplaza el centro de la imagen rectificada y recorta una franja del campo de visión. Ver [[Calibracion estereo a fondo]].
> 10. **RAFT no descarta la franja izquierda sin gemelo** (columnas $u < d$). Ver [[RAFT 2 En el proyecto]].
