---
tags: [nivel/1, nivel/2, nivel/3, tipo/concepto]
aliases: [Cámara estéreo, Disparidad, Estéreo, Profundidad]
---
# Cámara estéreo y disparidad

> [!info] Nivel 1 → 3 · Concepto base. Lo usan: [[03 Flujo de datos]], [[VO 3 Profundidad de los puntos]], [[RAFT 1 Intuicion]], [[Nube 3 Reproyeccion y keyframes]]

## Nivel 1: el truco del dedo

Pon un dedo frente a tu nariz y cierra un ojo, luego el otro: **el dedo "brinca" mucho**. Haz lo mismo con algo lejano: casi no se mueve.

- Cosa **cerca** → brinca **mucho** → disparidad **grande**
- Cosa **lejos** → brinca **poco** → disparidad **pequeña**

La **disparidad** es cuántos píxeles se movió un punto entre la imagen izquierda y la derecha. La cámara del carro tiene dos lentes separados unos **6 cm** (59.4 mm), como dos ojos.

## Nivel 2: en el proyecto

1. **RAFT-Stereo** ([[RAFT 1 Intuicion]], una red neuronal en la GPU) calcula la disparidad de **cada píxel** de la imagen izquierda: `run_raft()` devuelve una matriz de 720×1280 con valores en píxeles, incluso con decimales.
2. Esa disparidad se convierte en distancia de dos formas:
    - Para **todos los píxeles** (nube densa): `cv2.reprojectImageTo3D(disp, Q)`
    - Solo para los **puntos ORB** (odometría): `VisualOdometry._deproject(pts2d, disp)`
3. Ambas usan la matriz **Q**, que sale de la calibración (`cv2.stereoRectify`). Ver [[Calibracion y rectificacion]].

## Nivel 3: la fórmula y de dónde sale

### Triángulos semejantes

Con las imágenes rectificadas, un punto $P = (X, Y, Z)$ se proyecta en cada cámara así (modelo *pinhole*):

$$
u_L = f\frac{X}{Z} + c_x \qquad u_R = f\frac{X - B}{Z} + c_x
$$

donde $f$ es la distancia focal en píxeles y $B$ es la separación entre lentes (*baseline*). Restando:

$$
d = u_L - u_R = \frac{f\,B}{Z} \quad\Longrightarrow\quad \boxed{Z = \frac{f\,B}{d}}
$$

Con los números del proyecto ($f \approx 871$ px, $B = 59.4$ mm), $fB \approx 51{,}737$ px·mm:

| Distancia Z | Disparidad d | Error en Z si d falla 1 px |
|---|---|---|
| 0.5 m | 103 px | 5 mm |
| 1 m | 52 px | 19 mm |
| 2 m | 26 px | 77 mm |
| **3 m** | **17 px** | **174 mm** |
| 5 m | 10 px | 483 mm |
| 8 m | 6.5 px | 1.24 m |

> [!note] Qué $f$ usar
> Esta tabla usa la $f$ de la K original (≈ 871 px). La que realmente usa Q es la de la imagen rectificada, que con `alpha = 0` sale ≈ 919 px: los números cambian alrededor de 5 %. Ver [[Calibracion estereo a fondo]].

### ¿Por qué el error crece tan rápido?

Derivamos $Z = fB/d$ respecto a $d$:

$$
\frac{\partial Z}{\partial d} = -\frac{fB}{d^2} = -\frac{Z^2}{fB}
\quad\Longrightarrow\quad
|\Delta Z| \approx \frac{Z^2}{fB}\,|\Delta d|
$$

El error crece con el **cuadrado** de la distancia: al doble de distancia, cuatro veces el error. De aquí sale `Z_MAX = 3000 mm` en `build_point_cloud`: a 3 m, un píxel de error ya son **17 cm**.

> [!tip] Cómo se reduce el error
> Hay tres caminos: más **baseline** (lentes más separados), más **resolución** (una $f$ mayor en píxeles) o una disparidad **subpíxel** más precisa. Por eso pasar de 640 a 1280 de ancho ayudó: la $f$ en píxeles se duplica y el error se reduce a la mitad.

### La matriz Q (lo que hace `_deproject`)

OpenCV empaqueta la fórmula en una matriz de 4×4. Para un píxel $(u, v)$ con disparidad $d$:

$$
\begin{bmatrix} X' \\ Y' \\ Z' \\ W \end{bmatrix}
= Q \begin{bmatrix} u \\ v \\ d \\ 1 \end{bmatrix}
= \begin{bmatrix} 1 & 0 & 0 & -c_x \\ 0 & 1 & 0 & -c_y \\ 0 & 0 & 0 & f \\ 0 & 0 & 1/B & 0 \end{bmatrix}
\begin{bmatrix} u \\ v \\ d \\ 1 \end{bmatrix}
$$

y el punto 3D es $(X'/W,\ Y'/W,\ Z'/W)$. Comprobación: $Z = f / (d/B) = fB/d$. Es la misma fórmula.

Esto es lo que hace el código:

```python
pts_h = np.stack([us, vs, ds, ones], axis=1)   # (N, 4): [u, v, d, 1]
vecs  = (self.Q @ pts_h.T).T                   # (N, 4): [X', Y', Z', W]
pts3d = vecs[:, :3] / w                        # divide entre W
pts3d[ds < ICP_DISP_MIN] = 0.0                 # disparidad < 2 px -> punto invalido (z = 0)
```

Un punto con `z = 0` significa **"sin profundidad"**, y el resto del código lo descarta (`valid = pts3d[:, 2] > 0`).
