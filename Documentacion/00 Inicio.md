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
2. **Conceptos base** (léelos cuando una serie los mencione)
    - [[Imagen digital]] (nivel 1)
    - [[Camara estereo y disparidad]] (nivel 1→3)
    - [[Calibracion y rectificacion]] (nivel 2)
    - [[Pose y matrices 4x4]] (nivel 2→3)
3. **Las series**: cada una empieza con la intuición (1), sigue con el código (2) y termina con una nota a fondo por cada pieza (3 en adelante). Están en el orden en que actúan en el pipeline.

| Serie | 1 · Intuición | 2 · En el proyecto | A fondo |
|---|---|---|---|
| **RAFT-Stereo** (disparidad) | [[RAFT 1 Intuicion]] | [[RAFT 2 En el proyecto]] | [[RAFT 3 Codificadores]] · [[RAFT 4 Correlacion y piramide]] · [[RAFT 5 Actualizacion con GRU]] · [[RAFT 6 Subida convexa]] |
| **ORB** (esquinas y huellas) | [[ORB 1 Intuicion]] | [[ORB 2 En el proyecto]] | [[ORB 3 FAST]] · [[ORB 4 Orientacion]] · [[ORB 5 rBRIEF]] · [[ORB 6 Hamming y matching]] |
| **Odometría visual** (dónde está el carro) | [[VO 1 Intuicion]] | [[VO 2 En el proyecto]] | [[VO 3 Profundidad de los puntos]] · [[VO 4 PnP RANSAC]] · [[VO 5 Movimiento plano]] · [[VO 6 Pose acumulada y fallos]] |
| **Loop closure y PGO** (corregir el camino; apagado) | [[LC 1 Intuicion]] | [[LC 2 En el proyecto]] | [[LC 3 Deteccion de candidatos]] · [[LC 4 Verificacion geometrica]] · [[LC 5 Grafo de poses]] · [[LC 6 Optimizacion]] |
| **Nube de puntos** (el mapa) | [[Nube 1 Intuicion]] | [[Nube 2 En el proyecto]] | [[Nube 3 Reproyeccion y keyframes]] · [[Nube 4 Filtros de ruido]] · [[Nube 5 Voxel y DBSCAN]] · [[Nube 6 Piso y alineacion]] · [[Nube 7 Nube sparse]] |

4. **Carrito y calibración**
    - [[Control del carrito]]: teclado → TCP → PCA9685 → motores, mantener presionado y watchdog
    - [[Calibracion estereo a fondo]]: el tablero, `calibrateCamera`, `stereoCalibrate`, distorsión y rectificación
5. [[Glosario]] para cuando una palabra no te suene

## Hallazgos abiertos

La lista completa está en [[04 Mapa del codigo#Observaciones de revisión (deuda técnica)]]. Los más importantes:

| Hallazgo | Nota |
|---|---|
| El modelo de distorsión no tiene solución en las esquinas: desplaza el centro rectificado y recorta campo de visión | [[Calibracion estereo a fondo]] |
| Loop closure apagado: daba loops falsos y el PGO se desarmaba | [[LC 2 En el proyecto]] |
| El grafo de poses queda partido tras un reinicio por `MAX_FALLOS` | [[LC 5 Grafo de poses]] |
| La alineación del piso puede tomar una pared | [[Nube 6 Piso y alineacion]] |
| La franja izquierda sin gemelo de RAFT entra a la nube | [[RAFT 2 En el proyecto]] |
| Parámetros de ORB y PnP pensados para 640×360 | [[ORB 2 En el proyecto]], [[VO 4 PnP RANSAC]] |

## Ver el grafo con colores

En Obsidian abre **Graph view → Groups** y agrega estos grupos:

| Query | Sugerencia de color |
|---|---|
| `tag:#nivel/1` | verde |
| `tag:#nivel/2` | amarillo |
| `tag:#nivel/3` | rojo |
| `tag:#tipo/orb` | morado |
| `tag:#tipo/raft` | azul |
| `tag:#tipo/vo` | cian |
| `tag:#tipo/pgo` | rosa |
| `tag:#tipo/nube` | café |
| `tag:#tipo/hardware` | naranja |
