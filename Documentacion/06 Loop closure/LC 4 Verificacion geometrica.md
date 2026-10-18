---
tags: [nivel/2, nivel/3, tipo/pgo]
aliases: [_verify_loop, Verificación de loops]
---
# LC 4: Verificación geométrica (`_verify_loop`)

> [!info] Nivel 2 → 3 · Serie LC: [[LC 1 Intuicion]] · [[LC 2 En el proyecto]] · [[LC 3 Deteccion de candidatos]] · **4** · [[LC 5 Grafo de poses]] · [[LC 6 Optimizacion]] · Requisito: [[VO 4 PnP RANSAC]]

Pregunta de este paso: **¿de verdad es el mismo lugar, y dónde estoy respecto a él?** Los votos solo dicen que las huellas se parecen; aquí se exige que la **geometría** cuadre.

## Nivel 2: los pasos

```python
raw = self.matcher.knnMatch(des_a, des_b, k=2)              # a = candidato, b = actual
matches = [p[0] for p in raw if len(p) == 2 and p[0].distance < LOWE_RATIO * p[1].distance]
pts3d    = pts3d_a[[m.queryIdx for m in matches]]           # 3D del candidato (de _kf_data)
pts_b_2d = np.float32([kp_b[m.trainIdx].pt for m in matches])
...
success, rvec, tvec, inliers = cv2.solvePnPRansac(
    pts3d, pts_b_2d, self.K, None, iterationsCount=150, reprojectionError=2.0)
...
M = np.linalg.inv(T)               # PnP da a->b; el grafo usa inv(P_a) @ P_b
if USE_PLANAR and self.A is not None:
    M = self._to_planar(M)
```

1. **Matching** entre el candidato y el frame actual, con el mismo ratio de Lowe (0.75) que la odometría.
2. Los puntos 3D del candidato ya están calculados en `_kf_data` (por eso se guardan las huellas, los puntos 3D y los píxeles de cada frame).
3. Se quitan los que no tienen profundidad.
4. **PnP RANSAC**: 150 intentos, 2 px de tolerancia. Necesita al menos `MIN_LOOP_INLIERS = 6` inliers.
5. Se **invierte** el resultado para seguir la convención del grafo: $T_{ab} = P_a^{-1} P_b$ ("dónde está $b$ visto desde $a$"; ver [[LC 5 Grafo de poses]]).
6. Se aplica el **movimiento plano**, igual que a la odometría ([[VO 5 Movimiento plano]]).

Si cualquier paso falla, no hay loop.

## Nivel 3: por qué 6 inliers no bastan

RANSAC arma cada hipótesis con **5 parejas** ([[VO 4 PnP RANSAC]]). Esas 5 parejas casi siempre son "inliers" de la hipótesis que ellas mismas generaron, sean buenas o malas. Pedir 6 inliers significa pedir que **solo una pareja más** esté de acuerdo.

Entre cientos de parejas que pasaron el ratio de Lowe, encontrar una sexta que por casualidad caiga a menos de 2 px de la predicción no es raro, sobre todo con texturas repetidas. En la odometría se piden 12, y ahí además el detector de saltos protege; en el loop no hay ninguna otra revisión.

> [!example] Cómo se vería un umbral más estricto
> | `MIN_LOOP_INLIERS` | Parejas que confirman más allá de la muestra |
> |---|---|
> | 6 (actual) | 1 |
> | 12 | 7 |
> | 30 | 25 |

## Nivel 3: la revisión de sentido común que falta

El loop 85→180 que rompió el mapa decía que entre esos frames la cámara giró **142°**. La cámara ve unos 70° de ancho. Si dos vistas apuntan a direcciones separadas por más que el campo de visión, **no pueden estar viendo lo mismo**: el loop era falso, aunque PnP lo haya "verificado".

La odometría también da una estimación de ese giro: $P_a^{-1} P_b$. Con el movimiento plano el giro casi no deriva, así que una comparación sencilla atraparía estos casos. El ángulo entre dos rotaciones es:

$$
\theta = \arccos\!\left(\frac{\operatorname{tr}\!\left(R_{odo}^\top R_{loop}\right) - 1}{2}\right)
$$

> [!example] Ejemplo
> La odometría dice que entre los frames 85 y 180 el carro giró 10°; el loop dice 142°. La diferencia es $\theta = 132°$. Con un umbral de, por ejemplo, 30°, el loop se rechaza.

Esta comparación es una **propuesta** (no está en el código). La traslación es menos confiable para comparar, porque sí acumula drift, que es justo lo que el loop quiere corregir.
