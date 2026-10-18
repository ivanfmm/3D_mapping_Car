---
tags: [tipo/mapa]
aliases: [Inicio, Índice, MOC]
---
# 3D Mapping Car: Documentación

> [!abstract] Qué es este vault
> Documentación del proyecto **3D_mapping_Car**: un carrito con cámara estéreo que recorre un lugar y, con la PC, lo reconstruye en 3D.
> Está escrita para leerse **de lo sencillo a lo denso**. Cada nota dice su nivel arriba.
> Basada en el código del repo al **18 de octubre de 2026**, incluidos los cambios todavía sin commit (watchdog, movimiento plano, ICP apagado, PGO con `least_squares`, `_fallo()`, keyframes en el índice de loops, loop closure apagado).

## Cómo leer este vault

Cada tema viene en tres niveles. Puedes quedarte en el nivel que te sirva:

| Nivel | Qué contiene | Ejemplo |
|---|---|---|
| **Nivel 1: Intuición** | Analogías, cero fórmulas | "ORB busca esquinas y les saca una huella digital" |
| **Nivel 2: En el código** | Qué función lo hace, qué parámetros tiene, por qué esos valores | `cv2.ORB_create(nfeatures=1000, ...)` |
| **Nivel 3: Matemáticas** | Fórmulas, derivaciones, ejemplos numéricos | $\theta = \operatorname{atan2}(m_{01}, m_{10})$ |

## Ruta de lectura sugerida

1. **General**
    1. [[01 Que es el proyecto]] (nivel 1): la idea general en 5 minutos
    2. [[02 Arquitectura general]] (nivel 2): las piezas y cómo se hablan
    3. [[03 Flujo de datos]] (nivel 2-3): el viaje de una foto hasta la nube 3D
    4. [[04 Mapa del codigo]] (nivel 2): qué hay en cada archivo `.py` y la deuda técnica
2. **Conceptos base** (léelos cuando una nota los mencione)
    - [[Imagen digital]] (nivel 1)
    - [[Camara estereo y disparidad]] (nivel 1→3)
    - [[Calibracion y rectificacion]] (nivel 2)
    - [[Pose y matrices 4x4]] (nivel 2→3)
    - [[Odometria visual]] (nivel 2): el mapa de `VisualOdometry.update()`
3. **Serie ORB** (de lo intuitivo a lo matemático)
    1. [[ORB 1 Intuicion]]
    2. [[ORB 2 En el proyecto]]
    3. [[ORB 3 FAST]]
    4. [[ORB 4 Orientacion]]
    5. [[ORB 5 rBRIEF]]
    6. [[ORB 6 Hamming y matching]]
4. **Algoritmos del pipeline** (en el orden en que actúan)
    1. [[RAFT-Stereo]]: la red que calcula la disparidad de cada píxel
    2. [[PnP RANSAC]]: de parejas de puntos al movimiento
    3. [[Movimiento plano]]: detección del piso y restricción a 3 grados de libertad
    4. [[Loop closure y PGO]]: reconocer lugares y corregir el camino (apagado por ahora)
    5. [[Nube de puntos]]: filtros, voxel, DBSCAN, alineación del piso y nube sparse
5. **Carrito y calibración**
    - [[Control del carrito]]: teclado → TCP → PCA9685 → motores, mantener presionado y watchdog
    - [[Calibracion estereo a fondo]]: el tablero, `calibrateCamera`, `stereoCalibrate`, distorsión y rectificación
6. [[Glosario]] para cuando una palabra no te suene

## Hallazgos abiertos

La lista completa está en [[04 Mapa del codigo#Observaciones de revisión (deuda técnica)]]. Los más importantes:

| Hallazgo | Nota |
|---|---|
| El modelo de distorsión no tiene solución en las esquinas: desplaza el centro rectificado y recorta campo de visión | [[Calibracion estereo a fondo]] |
| Loop closure apagado: daba loops falsos y el PGO se desarmaba | [[Loop closure y PGO]] |
| El grafo de poses queda partido tras un reinicio por `MAX_FALLOS` | [[Loop closure y PGO]] |
| La alineación del piso puede tomar una pared | [[Nube de puntos]] |
| Parámetros de ORB y PnP pensados para 640×360 | [[ORB 2 En el proyecto]], [[PnP RANSAC]] |

## Ver el grafo con colores

En Obsidian abre **Graph view → Groups** y agrega estos grupos:

| Query | Sugerencia de color |
|---|---|
| `tag:#nivel/1` | verde |
| `tag:#nivel/2` | amarillo |
| `tag:#nivel/3` | rojo |
| `tag:#tipo/orb` | morado |
| `tag:#tipo/algoritmo` | azul |
| `tag:#tipo/hardware` | naranja |
