---
tags: [nivel/2, nivel/3, tipo/nube]
aliases: [reprojectImageTo3D, Keyframes de la nube, Máscara de profundidad]
---
# Nube 3: Keyframes, reproyección y paso al mundo

> [!info] Nivel 2 → 3 · Serie Nube: [[Nube 1 Intuicion]] · [[Nube 2 En el proyecto]] · **3** · [[Nube 4 Filtros de ruido]] · [[Nube 5 Voxel y DBSCAN]] · [[Nube 6 Piso y alineacion]] · [[Nube 7 Nube sparse]]

## 1. ¿Entra este frame? (keyframes)

```python
rel = np.linalg.inv(prev_T) @ cur_T            # movimiento desde el ultimo aceptado
mov = np.linalg.norm(rel[:3, 3])               # mm
ang = np.degrees(np.linalg.norm(cv2.Rodrigues(rel[:3, :3])[0]))
if mov < MIN_POSE_MOV and ang < MIN_POSE_ROT:  # 50 mm y 5 grados
    continue
```

Un frame entra solo si, desde el **último frame aceptado**, el carro avanzó ≥ 50 mm **o** giró ≥ 5°. El movimiento relativo $P_{prev}^{-1} P_{actual}$ es la misma idea que las aristas del grafo ([[LC 5 Grafo de poses]]), y el ángulo es la longitud del vector de Rodrigues ([[Pose y matrices 4x4]]).

> [!example] El carro quieto al inicio
> Si el carro estuvo quieto los primeros 100 frames, de esos entra **solo el primero**: ninguno de los otros se movió 50 mm ni giró 5° respecto a él. Sin este filtro, la misma pared se agregaría 100 veces, con 100 versiones distintas de su ruido.

Los frames que fallaron en la odometría no llegan aquí: `_fallo()` los quita de `all_poses` ([[VO 6 Pose acumulada y fallos]]).

## 2. Píxeles → 3D (`reprojectImageTo3D`)

```python
pts3d = cv2.reprojectImageTo3D(disp, Q)    # (720, 1280, 3), en mm
```

Para cada píxel $(u, v)$ con disparidad $d$, hace la misma cuenta que `_deproject` de la odometría ([[VO 3 Profundidad de los puntos]]), pero para todos los píxeles a la vez:

$$
X = \frac{(u - c_x)\,B}{d}, \qquad Y = \frac{(v - c_y)\,B}{d}, \qquad Z = \frac{f\,B}{d}
$$

El resultado ocupa $1280 \times 720 \times 3 \times 4$ bytes ≈ 11 MB por frame (`float32`).

## 3. La máscara: qué píxeles se quedan

| Filtro | Condición | Por qué |
|---|---|---|
| m1 | `disp > 2.0` | Con menos de 2 px de disparidad, el punto está tan lejos (≥ 27 m con $fB \approx 54{,}584$) que no hay información útil |
| m2 | `isfinite(z)` | Disparidad 0 da $Z = \infty$ |
| m3 | `z > 100` mm | Más cerca de 10 cm es basura (la cámara no enfoca y la disparidad sería enorme) |
| m4 | `z < 3000` mm | A 3 m un píxel de error ya son ≈ 17 cm ([[Camara estereo y disparidad]]) |

Cada 10 frames se imprime cuántos píxeles quitó cada filtro. Si m1 quita mucho, hay mucha zona lejana o sin textura; si m4 quita mucho, el cuarto es grande para la cámara.

## 4. Color

```python
cols = img[mask][:, ::-1].astype(np.float64) / 255.0
```

- `img[mask]` deja una fila por píxel válido: `(N, 3)` en orden BGR.
- `[:, ::-1]` invierte las columnas: BGR → RGB.
- `/ 255.0` lo deja entre 0 y 1, como lo pide Open3D.

## 5. Cámara → mundo

```python
pts_h = np.hstack([pts, np.ones((len(pts), 1))])   # (N, 4): agregar el 1
pts_w = (T_pose @ pts_h.T).T[:, :3]
```

Coordenadas homogéneas ([[Pose y matrices 4x4]]): $p_{mundo} = R\,p_{cam} + t$.

> [!example] Un punto, de píxel a mundo
> El píxel (812, 233) con disparidad 26 px da, frente a la cámara, $p_{cam} = (-103.6,\ -160.5,\ 2099)$ mm ([[VO 3 Profundidad de los puntos]]).
>
> La pose de ese frame: el carro avanzó 500 mm y giró 90° (*yaw*, sobre Y). La rotación $R_y(90°)$ manda $(x, y, z) \to (z,\ y,\ -x)$:
>
> $$R\,p_{cam} = (2099,\ -160.5,\ 103.6), \qquad p_{mundo} = (2099,\ -160.5,\ 103.6 + 500) = (2099,\ -160.5,\ 603.6)$$
>
> Visto desde la primera cámara: a 2.1 m a la derecha, 16 cm arriba y 60 cm adelante.

Después se quitan los puntos con algún valor no finito y el frame pasa al outlier estadístico ([[Nube 4 Filtros de ruido]]). Si le quedan menos de 50 puntos, se descarta.
