---
tags: [nivel/1, nivel/2, nivel/3, tipo/vo]
aliases: [PnP, PnP RANSAC, solvePnPRansac, RANSAC, Perspective-n-Point]
---
# VO 4: PnP RANSAC (de parejas de puntos al movimiento)

> [!info] Nivel 1 → 3 · Serie VO: [[VO 1 Intuicion]] · [[VO 2 En el proyecto]] · [[VO 3 Profundidad de los puntos]] · **4** · [[VO 5 Movimiento plano]] · [[VO 6 Pose acumulada y fallos]]

## Nivel 1: la idea

Tienes una foto vieja de un cuarto en la que sabes a qué distancia estaba cada esquina. Te dan una foto nueva tomada desde otro lugar y te dicen cuáles esquinas son las mismas. ¿Desde dónde se tomó la nueva?

Eso es **PnP** (*Perspective-n-Point*): con $n$ puntos 3D y su posición 2D en la foto nueva, encontrar la **rotación y la traslación** de la cámara. Algunas parejas están mal (ORB se equivocó), y **RANSAC** lo resuelve probando muchas veces con unos pocos puntos al azar y quedándose con la respuesta que **más puntos apoyan**. Las parejas malas casi nunca se ponen de acuerdo entre sí, así que pierden la votación.

## Nivel 2: en el código

```python
success, rvec, tvec, inliers = cv2.solvePnPRansac(
    pts3d, pts_curr_2d, self.K, None,   # None: imagen rectificada, sin distorsion
    iterationsCount=PNP_ITERATIONS,     # 200
    reprojectionError=PNP_REPROJ_ERR,   # 2.0 px
)
```

| Entrada | Forma | Qué es |
|---|---|---|
| `pts3d` | (N, 3), mm | Puntos ORB del frame anterior, en coordenadas de la cámara anterior ([[VO 3 Profundidad de los puntos]]) |
| `pts_curr_2d` | (N, 2), px | Dónde aparecen esos mismos puntos en el frame actual |
| `self.K` | 3 × 3 | `K_rect` (`P1[:3, :3]`) |
| distorsión | `None` | Las imágenes ya están rectificadas |

| Salida | Qué es |
|---|---|
| `success` | Si encontró una solución |
| `rvec` | Rotación como vector de Rodrigues ([[Pose y matrices 4x4]]) |
| `tvec` | Traslación en **mm** |
| `inliers` | Índices de las parejas que apoyan la solución |

**Dirección:** el resultado cumple $X_{actual} = R\,X_{anterior} + t$: $T$ lleva puntos de la cámara anterior a la actual. Por eso después se acumula con `pose = pose @ inv(T)` ([[VO 6 Pose acumulada y fallos]]).

### Qué hace OpenCV por dentro

Con los valores por defecto (`SOLVEPNP_ITERATIVE`):

1. Toma **5 parejas al azar** y resuelve con **EPnP**, un método directo y rápido.
2. Cuenta los **inliers**: parejas cuyo error de reproyección es menor a 2 px.
3. Repite. Según la confianza por defecto (0.99) y la fracción de inliers que va viendo, ajusta cuántas vueltas le faltan, así que puede terminar antes de las 200.
4. Con todos los inliers del mejor intento, **refina** minimizando el error de reproyección (Levenberg-Marquardt).

Después, si hay menos de `MIN_PNP_INLIERS = 12` inliers, el frame falla.

## Nivel 3: las matemáticas

### Modelo de proyección

Un punto $X$ del frame anterior, visto desde la cámara actual:

$$
X' = R\,X + t, \qquad
u = f_x \frac{X'_x}{X'_z} + c_x, \quad v = f_y \frac{X'_y}{X'_z} + c_y
$$

### Error de reproyección

Para cada pareja $i$, con $\pi$ la proyección de arriba y $u_i$ el píxel que ORB observó:

$$
e_i = \big\lVert \pi\big(K(R X_i + t)\big) - u_i \big\rVert, \qquad \text{inlier si } e_i < 2 \text{ px}
$$

El refinamiento final busca $R, t$ que minimicen $\sum_i e_i^2$ sobre los inliers.

> [!example] Ejemplo: el carro avanza 100 mm en línea recta
> Con $f \approx 871$ y $c_x \approx 646.6$:
> - Punto en el frame anterior: $X = (100, 0, 1000)$ mm → $u = 871.4 \cdot 100/1000 + 646.6 \approx 733.7$.
> - El carro avanza 100 mm: $R = I$, $t = (0, 0, -100)$ → $X' = (100, 0, 900)$ → $u' = 871.4 \cdot 100/900 + 646.6 \approx 743.4$.
> - El punto se movió unos 9.7 px a la derecha: al acercarte, las cosas "se abren" hacia los lados.
> - Si ORB lo observó en $u = 744.6$, el error es 1.2 px → **inlier**. Si lo observó en $u = 760$, el error es 16.6 px → **outlier**.

> [!note] El umbral de 2 px se eligió para 640×360
> A 1280×720, 2 px equivalen a 1 px de la resolución anterior: el umbral quedó el doble de estricto en proporción.

### ¿Cuántas iteraciones bastan?

Sea $w$ la fracción de parejas buenas y $s = 5$ el tamaño de la muestra. Una muestra sale **toda buena** con probabilidad $w^s$, y después de $k$ intentos:

$$
P = 1 - (1 - w^s)^k
\qquad\Longrightarrow\qquad
k = \frac{\log(1 - P)}{\log(1 - w^s)}
$$

| Parejas buenas ($w$) | $k$ necesario para $P = 0.99$ | $P$ con 200 intentos |
|---|---|---|
| 80 % | 12 | ≈ 100 % |
| 60 % | 57 | ≈ 100 % |
| 50 % | 146 | 99.8 % |
| 40 % | 448 | 87 % |
| 30 % | 1893 | 39 % |

Con menos del 40 % de parejas buenas, 200 intentos ya no alcanzan. Por eso el ratio de Lowe, que limpia **antes** de PnP, es tan importante. Si `failed not success or no inliers` sale seguido, subir `PNP_ITERATIONS` es un experimento barato.

### ¿Por qué 3D↔2D y no 3D↔3D?

Alinear 3D con 3D usaría la profundidad de los **dos** frames, y el error de profundidad crece con $Z^2$. En PnP el frame actual solo aporta píxeles, que ORB mide con precisión subpíxel, y la profundidad ruidosa entra una sola vez. Además, como esa profundidad está en mm reales, la traslación sale en mm reales; con una sola cámara saldría sin escala.

## Riesgos

1. **Puntos lejanos con el mismo peso.** Experimento sugerido: filtrar `pts3d[:, 2] < 3000` antes de PnP y comparar la cobertura.
2. **Mínimo de inliers bajo.** 12 de unas 1000 esquinas sirve para no perder frames, pero deja pasar soluciones débiles. El detector de saltos atrapa las peores ([[VO 6 Pose acumulada y fallos]]).
