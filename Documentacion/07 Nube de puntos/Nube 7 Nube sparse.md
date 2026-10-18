---
tags: [nivel/2, nivel/3, tipo/nube]
aliases: [Nube sparse, nube_sparse.ply, trayectoria.ply, reprocesar.py, Nitidez]
---
# Nube 7: La nube sparse (`reprocesar.py`)

> [!info] Nivel 2 → 3 · Serie Nube: [[Nube 1 Intuicion]] · [[Nube 2 En el proyecto]] · [[Nube 3 Reproyeccion y keyframes]] · [[Nube 4 Filtros de ruido]] · [[Nube 5 Voxel y DBSCAN]] · [[Nube 6 Piso y alineacion]] · **7**

`reprocesar.py` repite la odometría y arma una nube **solo con los puntos ORB**. Usa las disparidades `.npy` ya guardadas, así que no necesita ni el carro ni RAFT. Es la herramienta para experimentar con parámetros.

```bash
python reprocesar.py                       # ultima sesion: sparse + densa
python reprocesar.py --sparse              # solo sparse (rapido)
python reprocesar.py captured_frames/session_XXXX --sparse
python reprocesar.py --blur 0.8            # mas estricto con frames borrosos (0 = no filtrar)
python reprocesar.py --max-frames 100      # solo los primeros N frames
python reprocesar.py --zmax 2000           # profundidad maxima de los puntos sparse
python reprocesar.py --todos-los-frames    # sin keyframes, para comparar
python reprocesar.py --icp / --loops / --sin-plano / --inclinacion 12
python ver_nube.py <sesion>/nube_sparse.ply <sesion>/trayectoria.ply
```

## Fase 1: descartar frames borrosos

```python
def nitidez(gray):
    return cv2.Laplacian(gray, cv2.CV_64F).var()

umbral = a.blur * np.median(valores)       # blur = 0.6 por defecto
```

El **Laplaciano** mide cuánto cambia el cambio de intensidad (la segunda derivada). En cada píxel aplica el kernel

$$
\begin{bmatrix} 0 & 1 & 0 \\ 1 & -4 & 1 \\ 0 & 1 & 0 \end{bmatrix}
$$

Un borde nítido da valores grandes (positivos y negativos); uno borroso, valores chicos. La **varianza** de todos esos valores resume qué tan nítida es la imagen.

> [!example] Un borde en una fila
> | Fila | Valores | Segunda diferencia ($a - 2b + c$) |
> |---|---|---|
> | Nítido | 10, 10, 200, 200 | 10 − 20 + 200 = **190**, 10 − 400 + 200 = **−190** |
> | Borroso | 10, 73, 137, 200 | 10 − 146 + 137 = **1**, 73 − 274 + 200 = **−1** |
>
> El mismo cambio de 10 a 200, pero repartido en más píxeles, casi no "suena" en el Laplaciano.

El umbral es **relativo** a la mediana de la sesión, porque la nitidez cambia mucho con la luz y la cámara: un valor fijo como 100 descartaría todo en una sesión oscura o nada en una muy iluminada. Con la mediana en 120 y `--blur 0.6`, el umbral es 72.

Los frames borrosos dañan a ORB: los bits de la huella se vuelven casi aleatorios ([[ORB 5 rBRIEF]]).

## Fase 2: odometría y puntos de cada frame

Para cada frame nítido corre `vo.update(...)` y, si salió bien, toma sus puntos ORB con `vo.get_frame_points(idx)` (los píxeles y los puntos 3D que guardó `_kf_data`, [[VO 3 Profundidad de los puntos]]). Se quedan los que cumplen:

| Filtro | Condición | Por qué |
|---|---|---|
| Profundidad | $100 < Z <$ `--zmax` (3000 mm) | Mismo criterio que la nube densa |
| Borde de profundidad | Rango de disparidad en 5×5 < 2 px | FAST detecta esquinas, y muchas caen justo en la frontera entre un objeto y el fondo |

```python
k = np.ones((5, 5), np.uint8)
rango = cv2.dilate(disp, k) - cv2.erode(disp, k)   # maximo - minimo en 5x5
m = (z > SPARSE_Z_MIN) & (z < a.zmax) & (rango[v, u] < EDGE_MAX_DISP)
```

`dilate` deja en cada píxel el máximo de su ventana y `erode` el mínimo; la resta es cuánto cambia la disparidad alrededor del punto.

> [!example] Una esquina en el borde de una caja
> En la ventana de 5×5 alrededor del punto hay disparidades de 20 (la pared) y de 40 (la caja): rango = 20 px ≥ 2 → **se descarta**. Su disparidad podría ser la de la caja, la de la pared o una mezcla, y el punto quedaría flotando entre las dos.
>
> Una esquina en un cuadro colgado en la pared: todas las disparidades entre 25.6 y 26.3 → rango = 0.7 px → **se queda**.

## Keyframes con las poses finales

La lista de keyframes se arma **al final**, con las poses ya optimizadas (si hubo PGO), con el mismo criterio de 50 mm o 5° ([[Nube 3 Reproyeccion y keyframes]]). Sin esto, con el carro quieto el mismo punto se agregaba cientos de veces con ruido distinto, y se veían "rayos" saliendo de la cámara.

Paso al mundo:

```python
pts_w.append(pts @ T[:3, :3].T + T[:3, 3])   # camara -> mundo
```

Es lo mismo que $R\,p + t$ para cada punto: con los puntos como filas, $(R\,p)^\top = p^\top R^\top$, así que se multiplica por $R^\top$ por la derecha.

## Trayectoria, recorrido y desplazamiento neto

- **`trayectoria.ply`:** un punto rojo por frame, en el centro de la cámara (`T[:3, 3]`).
- **Recorrido:** suma de las distancias entre centros consecutivos.
- **Desplazamiento neto:** distancia del primer centro al último.

En un recorrido que regresa al punto de partida, el desplazamiento neto debería ser casi 0: lo que quede es el **drift**. Es la forma más rápida de comparar dos configuraciones.

## Sugerencia de inclinación

Si no se encontró el piso (odometría en 6 grados de libertad), `reprocesar.py` intenta adivinar la inclinación con la forma de la trayectoria:

```python
c = centros - centros.mean(axis=0)
_, S, Vt = np.linalg.svd(c, full_matrices=False)
if S[1] > 0.2 * S[0]:                      # se movio en dos direcciones (hubo giros)
    n = Vt[2] if Vt[2][1] > 0 else -Vt[2]  # direccion en la que casi no se movio
```

El carro se mueve sobre el piso, así que su trayectoria es casi un plano. La dirección en la que **menos** varían los centros (el último vector de la SVD) es la normal de ese plano, y de ella sale la inclinación: $\arcsin(-n_z)$. Si el carro solo fue en línea recta ($S_1$ pequeño comparado con $S_0$), la trayectoria es casi una línea y cualquier plano que la contenga sirve: no se puede estimar, y el script lo dice.

## Salidas

| Archivo | Contenido |
|---|---|
| `nube_sparse.ply` | Puntos ORB de los keyframes, con color |
| `trayectoria.ply` | Centro de la cámara en cada frame, en rojo |
| `poses.txt` | Las poses finales |
| `mapa_final.ply`, `mapa_paredes.ply` | Solo sin `--sparse` ([[Nube 2 En el proyecto]]) |
