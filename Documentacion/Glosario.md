---
tags: [tipo/mapa]
aliases: [Glosario, Términos]
---
# Glosario

> [!info] Volver a [[00 Inicio]]

| Término | Definición corta | Nota |
|---|---|---|
| **Alabeo (*roll*)** | Giro del carro sobre su eje de avance; imposible sobre piso plano | [[VO 5 Movimiento plano]] |
| **Baseline (B)** | Separación entre los dos lentes: 59.4 mm | [[Camara estereo y disparidad]] |
| **BFMatcher** | Comparador "fuerza bruta": todas las huellas contra todas | [[ORB 6 Hamming y matching]] |
| **BRIEF / rBRIEF** | Huella binaria de 256 comparaciones de píxeles; la "r" es la versión girada | [[ORB 5 rBRIEF]] |
| **Cabeceo (*pitch*)** | Giro que inclina la nariz del carro arriba o abajo | [[VO 5 Movimiento plano]] |
| **Centroide de intensidad** | Centro de masa del parche usando el brillo como peso; da la orientación | [[ORB 4 Orientacion]] |
| **Ciclo de trabajo** | Fracción del periodo PWM en que la señal está prendida: duty / 4096 | [[Control del carrito]] |
| **Coordenadas homogéneas** | Agregar un 1 a un punto para rotar y trasladar con una sola matriz | [[Pose y matrices 4x4]] |
| **Correlación (volumen de)** | Tabla de qué tanto se parece cada píxel izquierdo a cada píxel de su fila en la derecha | [[RAFT 4 Correlacion y piramide]] |
| **DBSCAN** | Agrupamiento por densidad; en la nube se queda el grupo más grande | [[Nube 5 Voxel y DBSCAN]] |
| **Descriptor (`des`)** | La huella de una esquina: 32 bytes | [[ORB 2 En el proyecto]] |
| **Disparidad** | Cuántos píxeles se desplaza un punto entre la imagen izquierda y la derecha | [[Camara estereo y disparidad]] |
| **Distorsión radial** | Curvatura del lente; con $k_1 < 0$ la imagen se "encoge" hacia el centro (barril) | [[Calibracion estereo a fondo]] |
| **Drift** | Error que se acumula paso a paso en la odometría | [[VO 6 Pose acumulada y fallos]] |
| **E / F (matrices)** | Esencial y fundamental: la restricción epipolar en coordenadas normalizadas y en píxeles | [[Calibracion estereo a fondo]] |
| **EPnP** | Método directo que usa OpenCV para resolver PnP con pocas parejas dentro de RANSAC | [[VO 4 PnP RANSAC]] |
| **Error de reproyección** | Distancia en píxeles entre donde se vio un punto y donde lo proyecta el modelo | [[VO 4 PnP RANSAC]], [[Calibracion estereo a fondo]] |
| **FAISS** | Librería de búsqueda rápida de vecinos; aquí, con huellas binarias | [[ORB 6 Hamming y matching]] |
| **FAST** | Detector de esquinas con la prueba del círculo de 16 píxeles | [[ORB 3 FAST]] |
| **Framing** | Mandar el tamaño antes de cada mensaje en un flujo TCP | [[02 Arquitectura general]] |
| **GDL** | Grados de libertad: 6 para una pose libre, 3 para el carro sobre el piso | [[VO 5 Movimiento plano]] |
| **Grafo de poses** | Poses (nodos) unidas por mediciones de movimiento relativo (aristas) | [[LC 5 Grafo de poses]] |
| **GRU** | Red recurrente que actualiza un estado en cada iteración; RAFT la usa para refinar la disparidad | [[RAFT 5 Actualizacion con GRU]] |
| **Hamming (distancia)** | Número de bits distintos entre dos huellas | [[ORB 6 Hamming y matching]] |
| **Harris** | Puntaje de "qué tan esquina es"; usa los eigenvalores de la matriz de gradientes | [[ORB 3 FAST]] |
| **Huber (pérdida)** | Costo que crece como $r^2$ para errores chicos y como $|r|$ para errores grandes; resiste a datos falsos | [[LC 6 Optimizacion]] |
| **I2C** | Bus de dos cables con el que la Pi le habla al PCA9685 (dirección `0x40`) | [[Control del carrito]] |
| **ICP** | *Iterative Closest Point*: alinea dos nubes de puntos. Apagado en este proyecto | [[VO 6 Pose acumulada y fallos]] |
| **Inlier / outlier** | Dato que concuerda con el modelo / dato que no (una pareja mala) | [[VO 4 PnP RANSAC]] |
| **Instance norm** | Normalizar cada canal de cada imagen por separado; hace las características robustas a cambios de brillo | [[RAFT 3 Codificadores]] |
| **Jacobiano disperso** | Jacobiano casi todo en ceros; scipy lo aprovecha para estimarlo con pocas evaluaciones | [[LC 6 Optimizacion]] |
| **K (matriz)** | Parámetros internos de la cámara: focal y centro | [[Calibracion y rectificacion]] |
| **KD-tree** | Árbol que parte el espacio para encontrar vecinos sin compararse contra todos los puntos | [[Nube 4 Filtros de ruido]] |
| **Keyframe** | Frame que se conserva porque el carro se movió lo suficiente (50 mm o 5°) | [[Nube 3 Reproyeccion y keyframes]] |
| **Keypoint (`kp`)** | Esquina detectada: posición, tamaño, ángulo, puntaje | [[ORB 2 En el proyecto]] |
| **Laplaciano (varianza del)** | Medida de nitidez: los bordes nítidos dan valores grandes | [[Nube 7 Nube sparse]] |
| **Loop closure** | Reconocer un lugar ya visitado para corregir el drift | [[LC 1 Intuicion]] |
| **Mantener presionado** | El carro se mueve solo mientras sigan llegando repeticiones de la tecla (`HOLD_S = 0.6 s`) | [[Control del carrito]] |
| **ORB** | *Oriented FAST and Rotated BRIEF* | [[ORB 1 Intuicion]] |
| **Outlier estadístico** | Filtro que borra puntos cuya distancia a sus vecinos es mucho mayor que la normal | [[Nube 4 Filtros de ruido]] |
| **PCA9685** | Chip que genera 16 señales PWM de 12 bits | [[Control del carrito]] |
| **PGO** | *Pose Graph Optimization*: ajusta todas las poses para que concuerden con los loops | [[LC 6 Optimizacion]] |
| **Pirámide** | Versiones cada vez más chicas de la imagen, para detectar a varias escalas | [[ORB 3 FAST]] |
| **PnP** | *Perspective-n-Point*: movimiento de la cámara a partir de puntos 3D y su posición 2D | [[VO 4 PnP RANSAC]] |
| **Pose** | Posición + orientación de la cámara (matriz 4×4) | [[Pose y matrices 4x4]] |
| **Puente H** | Circuito que deja pasar corriente al motor en un sentido o en el otro | [[Control del carrito]] |
| **PWM** | Prender y apagar muy rápido para controlar la potencia promedio | [[Control del carrito]] |
| **Q (matriz)** | Convierte (u, v, disparidad) en (X, Y, Z) | [[Camara estereo y disparidad]] |
| **RAFT-Stereo** | Red neuronal que calcula la disparidad de cada píxel en la GPU | [[RAFT 1 Intuicion]] |
| **RANSAC** | Probar muchas veces con muestras pequeñas y quedarse con la hipótesis que más datos apoyan | [[VO 4 PnP RANSAC]] |
| **Ratio de Lowe** | Aceptar una pareja solo si es mucho mejor que la segunda opción | [[ORB 6 Hamming y matching]] |
| **Rectificar** | Enderezar las imágenes para que un punto quede en la misma fila en ambos lentes | [[Calibracion y rectificacion]] |
| **RMS** | Raíz del promedio de los errores al cuadrado | [[Calibracion estereo a fondo]] |
| **Rodrigues (vector)** | Rotación en 3 números: eje × ángulo | [[Pose y matrices 4x4]] |
| **Skid steering** | Girar como tanque: ruedas de un lado hacia adelante y del otro hacia atrás | [[Control del carrito]] |
| **SLAM** | Hacer el mapa y ubicarse en él al mismo tiempo | [[01 Que es el proyecto]] |
| **Subida convexa** | Pasar la disparidad de 1/4 a resolución completa con un promedio ponderado de 3×3 vecinos | [[RAFT 6 Subida convexa]] |
| **Subpíxel** | Precisión menor a un píxel (por ejemplo, una esquina en 812.4) | [[Calibracion estereo a fondo]] |
| **Votos (loop closure)** | Cada huella del frame actual vota por el frame donde está su vecina más cercana | [[LC 3 Deteccion de candidatos]] |
| **Voxel** | Cubito 3D; el *voxel downsample* deja un punto por cubito de 8 mm | [[Nube 5 Voxel y DBSCAN]] |
| **Watchdog** | Frena el carro si deja de recibir señales del cliente | [[Control del carrito]] |
| **Yaw** | Giro del carro sobre el eje vertical: el único giro que conserva el movimiento plano | [[VO 5 Movimiento plano]] |
