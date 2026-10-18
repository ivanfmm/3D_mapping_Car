---
tags: [nivel/2, nivel/3, tipo/nube]
aliases: [segment_plane, Alineación del piso, mapa_paredes.ply]
---
# Nube 6: El piso y la alineación

> [!info] Nivel 2 → 3 · Serie Nube: [[Nube 1 Intuicion]] · [[Nube 2 En el proyecto]] · [[Nube 3 Reproyeccion y keyframes]] · [[Nube 4 Filtros de ruido]] · [[Nube 5 Voxel y DBSCAN]] · **6** · [[Nube 7 Nube sparse]]

## El problema

El sistema del mundo es el de la **cámara del primer frame**. Si la cámara estaba inclinada (por ejemplo, 12° hacia arriba), todo el mapa sale inclinado 12°. Hay que encontrar el piso y girar la nube para que quede horizontal.

## Nivel 2: los pasos

```python
plane_model, inliers = pcd_final.segment_plane(
    distance_threshold=30.0, ransac_n=3, num_iterations=1000)
a, b, c, d = plane_model                 # plano: a x + b y + c z + d = 0
normal = np.array([a, b, c]) / norm
if normal[1] < 0:
    normal = -normal                     # que apunte hacia +Y (abajo)
# R_align lleva la normal al eje (0, 1, 0)
pcd_final.rotate(R_align, center=center)
pcd_walls.rotate(R_align, center=center)
```

1. **RANSAC de plano** sobre la nube final: 3 puntos al azar por intento, 1000 intentos, inliers a menos de 30 mm. Devuelve el plano **con más puntos**.
2. Voltea la normal si hace falta para que tenga $n_y > 0$.
3. Calcula la rotación que lleva la normal a $(0, 1, 0)$.
4. **`mapa_paredes`** = todo menos los inliers del piso.
5. Guarda las versiones `_raw` (antes de rotar) y luego rota las dos nubes alrededor del **mismo centro** (el de `pcd_final`), para que sigan alineadas entre sí.

## Nivel 3: ¿bastan 1000 intentos?

Si el piso es el 20 % de los puntos, una muestra de 3 puntos cae toda en el piso con probabilidad $0.2^3 = 0.008$. En 1000 intentos:

$$
P = 1 - (1 - 0.008)^{1000} \approx 0.9997
$$

Sobra. (La fórmula general está en [[VO 4 PnP RANSAC]].)

## Nivel 3: la rotación (fórmula de Rodrigues)

Para llevar un vector unitario $\mathbf{n}$ a otro $\mathbf{y} = (0, 1, 0)$ con la rotación más pequeña:

$$
\mathbf{v} = \mathbf{n} \times \mathbf{y}, \qquad s = \lVert \mathbf{v} \rVert = \sin\theta, \qquad c = \mathbf{n}\cdot\mathbf{y} = \cos\theta
$$

$$
R = I + [\mathbf{v}]_\times + [\mathbf{v}]_\times^2\, \frac{1 - c}{s^2},
\qquad
[\mathbf{v}]_\times = \begin{bmatrix} 0 & -v_z & v_y \\ v_z & 0 & -v_x \\ -v_y & v_x & 0 \end{bmatrix}
$$

- $\mathbf{v}$ es el **eje** de giro (perpendicular a los dos vectores) y su longitud es el seno del ángulo.
- Es la misma fórmula de Rodrigues de [[Pose y matrices 4x4]], escrita con $\mathbf{v}$ sin normalizar.
- Si $s \approx 0$ la normal ya está alineada y el código deja $R = I$.

> [!example] Cámara inclinada 12° hacia arriba
> El piso, visto desde la cámara, tiene normal $\mathbf{n} = (0,\ 0.978,\ -0.208)$.
> - $\mathbf{v} = \mathbf{n} \times \mathbf{y} = (n_y \cdot 0 - n_z \cdot 1,\ n_z \cdot 0 - n_x \cdot 0,\ n_x \cdot 1 - n_y \cdot 0) = (0.208,\ 0,\ 0)$
> - $s = 0.208$, $c = 0.978$ → ángulo $= \arcsin(0.208) = 12°$, alrededor del eje X.
>
> La nube gira 12° alrededor de X, justo lo que hace falta para deshacer la inclinación.

## Hallazgos

### 1. No se verifica que el plano sea el piso

`segment_plane` devuelve el plano **más grande**. En un pasillo, o con la cámara cerca de una pared, puede ser una pared, y entonces el mapa queda con esa pared "acostada". La odometría sí lo revisa ($n_y \ge 0.7$, [[VO 5 Movimiento plano]]); aquí no.

Propuesta simple: si `vo.floor_info` existe, usar `vo.A[1]` (la normal del piso que ya encontró la odometría) en lugar de buscar el plano. Con el movimiento plano todas las cámaras tienen la misma inclinación, así que esa normal vale también en el sistema del mundo (el de la cámara del frame 0). Si no existe, al menos rechazar planos con $|n_y| < 0.7$.

### 2. El eje Y apunta hacia abajo

Es la convención de la cámara ([[Pose y matrices 4x4]]), y la alineación la conserva: la normal del piso va a $+Y$. Si un visor muestra el mapa de cabeza, es por eso.

### 3. `mapa_paredes` también pierde lo que está pegado al piso

Todo lo que queda a menos de 30 mm del plano (cables, un control, la base de un mueble) sale como "piso" y se quita.
