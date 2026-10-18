---
tags: [nivel/2, nivel/3, tipo/vo]
aliases: [_deproject, Profundidad de los keypoints, _ensure_2d]
---
# VO 3: Profundidad de los puntos (`_deproject`)

> [!info] Nivel 2 → 3 · Serie VO: [[VO 1 Intuicion]] · [[VO 2 En el proyecto]] · **3** · [[VO 4 PnP RANSAC]] · [[VO 5 Movimiento plano]] · [[VO 6 Pose acumulada y fallos]] · Requisito: [[Camara estereo y disparidad]]

## Nivel 2: qué hace y por qué

PnP necesita puntos **3D** de un frame y su posición **2D** en otro. La odometría usa:

- **3D:** las esquinas del frame **anterior**, con la disparidad del frame anterior (`self.prev_disp`);
- **2D:** dónde aparecen esas mismas esquinas en el frame actual.

```python
pts_prev_2d = np.float32([self.prev_kp[m.queryIdx].pt for m in matches])
pts_curr_2d = np.float32([kp[m.trainIdx].pt           for m in matches])
pts3d = self._deproject(pts_prev_2d, self.prev_disp)
valid = pts3d[:, 2] > 0          # z = 0 significa "sin profundidad"
```

`_deproject` también se usa al principio de `update()` para guardar los puntos 3D de **todas** las esquinas del frame en `_kf_data`. De ahí los toman la verificación de loops y la nube sparse.

### Paso 0: dejar la disparidad en 2D (`_ensure_2d`)

RAFT puede entregar la disparidad como `(H, W)`, `(1, H, W)` o `(1, 1, H, W)`. `_ensure_2d` aplica `np.squeeze` y, si aún sobran dimensiones, se queda con el primer elemento hasta llegar a `(H, W)`. Si queda en 1D lanza un error claro, porque eso significa que `run_raft` devolvió algo mal.

### Pasos de `_deproject`

```python
uv  = pts2d.astype(int)                      # trunca (812.4 -> 812)
oob = (us < 0) | (us >= W) | (vs < 0) | (vs >= H)
ds  = disp[vs_c, us_c]                       # fila primero, columna despues
ds[oob] = 0.0                                # fuera de la imagen: sin disparidad
pts_h = [u, v, d, 1]                         # (N, 4)
vecs  = (Q @ pts_h.T).T                      # (N, 4): [X', Y', Z', W]
pts3d = vecs[:, :3] / W                      # W cerca de 0 -> se divide entre 1
pts3d[ds < 2.0] = 0.0                        # disparidad < 2 px -> punto invalido
```

1. Convierte las coordenadas a enteros para poder leer la disparidad del píxel.
2. Marca los puntos fuera de la imagen (`oob`) y les pone disparidad 0.
3. Lee `disp[v, u]` (fila, columna: ver [[Imagen digital]]).
4. Multiplica por Q y divide entre la cuarta coordenada.
5. Todo punto con disparidad < 2 px queda en $(0, 0, 0)$: la convención de "sin profundidad".

## Nivel 3: las cuentas

Con la Q de la imagen rectificada ($f \approx 918.9$, $c_x \approx 857.4$, $c_y \approx 303.3$, $B = 59.4$ mm; ver [[Calibracion estereo a fondo]]):

$$
\begin{bmatrix} X' \\ Y' \\ Z' \\ W \end{bmatrix}
= Q \begin{bmatrix} u \\ v \\ d \\ 1 \end{bmatrix}
= \begin{bmatrix} u - c_x \\ v - c_y \\ f \\ d/B \end{bmatrix}
\qquad\Longrightarrow\qquad
X = \frac{(u - c_x)\,B}{d},\quad Y = \frac{(v - c_y)\,B}{d},\quad Z = \frac{f\,B}{d}
$$

> [!example] Ejemplo: un keypoint en (812.4, 233.0) con disparidad 26 px
> - Truncado: $(u, v) = (812, 233)$.
> - $W = 26 / 59.4 = 0.4377$.
> - $X = (812 - 857.4) / 0.4377 \approx -103.6$ mm (un poco a la izquierda del centro óptico)
> - $Y = (233 - 303.3) / 0.4377 \approx -160.5$ mm (arriba, porque Y crece hacia abajo)
> - $Z = 918.9 / 0.4377 \approx 2099$ mm, es decir **≈ 2.1 m**.

### ¿Cuánto error hereda el punto?

Si la disparidad tiene un error $\Delta d$, la profundidad cambia (derivada de $Z = fB/d$):

$$
\Delta Z \approx \frac{Z^2}{fB}\,\Delta d
$$

> [!example] El mismo punto con ±0.4 px de error en la disparidad
> | Disparidad | Z |
> |---|---|
> | 25.6 px | 2132 mm |
> | 26.0 px | 2099 mm |
> | 26.4 px | 2068 mm |
>
> Casi ±3.3 cm por menos de medio píxel. $X$ e $Y$ se escalan igual (también llevan $1/d$), así que el punto se mueve **a lo largo del rayo** de la cámara: el error de profundidad casi no cambia en qué píxel se proyecta el punto, y por eso PnP tolera bastante bien este ruido.

### El truncamiento

`astype(int)` trunca: 812.9 se vuelve 812. En promedio se lee la disparidad medio píxel a la izquierda y arriba del punto real, y $X, Y$ se calculan con ese $u, v$ entero. El sesgo en $X$ es de $Z \cdot 0.5 / f$: a 2 m, ≈ 1 mm. Es pequeño, y `np.round` lo quitaría sin costo. En bordes de profundidad (donde la disparidad salta) leer el píxel vecino sí puede dar un valor muy distinto; la nube sparse filtra esos puntos con el rango de disparidad en 5×5 ([[Nube 7 Nube sparse]]).

## Riesgos

1. **Sin límite de distancia.** Un punto a 5 m tiene un error de profundidad unas 6 veces mayor que uno a 2 m ($5^2 / 2^2$), pero aquí se acepta igual. La nube densa sí limita $Z < 3000$ mm.
2. **La franja izquierda de la imagen** (columnas $u < d$) no tiene gemelo en la derecha y su disparidad es inventada por RAFT ([[RAFT 2 En el proyecto]]). Las esquinas que caen ahí entran a PnP sin filtro.
