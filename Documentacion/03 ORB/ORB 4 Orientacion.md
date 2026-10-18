---
tags: [nivel/3, tipo/orb]
aliases: [Orientación ORB, Centroide de intensidad, Momentos de imagen]
---
# ORB 4: Orientación por centroide de intensidad

> [!info] Nivel 3 · Serie ORB: [[ORB 1 Intuicion]] · [[ORB 2 En el proyecto]] · [[ORB 3 FAST]] · **4** · [[ORB 5 rBRIEF]] · [[ORB 6 Hamming y matching]]

## Nivel 1: ¿hacia dónde apunta una esquina?

Imagina el parche alrededor de la esquina como una lámina de metal donde los **píxeles claros pesan más**. Si buscas el punto donde se equilibra (su **centro de masa**), normalmente no queda en el centro, sino "jalado" hacia la parte clara. La **flecha del centro al punto de equilibrio** es la orientación de la esquina.

Si el carro gira, la imagen gira **y la flecha gira igual**. Medir la huella **relativa a la flecha** la hace inmune al giro (ver [[ORB 5 rBRIEF]]).

## Nivel 3: momentos de la imagen

Con coordenadas $(x, y)$ relativas a la esquina, dentro de un **círculo** de radio $r$ = `patchSize`/2 = **9** px:

$$
m_{pq} = \sum_{x^2 + y^2 \le r^2} x^p\, y^q\, I(x, y)
$$

| Momento | Fórmula | Significado físico |
|---|---|---|
| $m_{00}$ | $\sum I$ | "Masa" total (brillo total) |
| $m_{10}$ | $\sum x\, I$ | Cuánto pesa hacia la derecha |
| $m_{01}$ | $\sum y\, I$ | Cuánto pesa hacia abajo (y crece hacia abajo) |

Centroide y ángulo:

$$
C = \left(\frac{m_{10}}{m_{00}},\ \frac{m_{01}}{m_{00}}\right), \qquad
\boxed{\theta = \operatorname{atan2}(m_{01},\ m_{10})}
$$

$m_{00}$ no hace falta para el ángulo: se cancela dentro de `atan2`. OpenCV lo guarda en `kp.angle` en grados, de 0 a 360.

> [!example] Ejemplo numérico
> Solo dos píxeles brillan (los demás valen 0): $(x, y) = (2, 0)$ con $I = 100$ y $(0, -2)$ con $I = 100$.
> - $m_{10} = 2\cdot100 + 0\cdot100 = 200$
> - $m_{01} = 0\cdot100 + (-2)\cdot100 = -200$
> - $\theta = \operatorname{atan2}(-200, 200) = -45° \equiv 315°$
>
> Como **y crece hacia abajo**, el vector $(1, -1)$ apunta a la **derecha y hacia arriba**: justo entre los dos píxeles brillantes.

## ¿Por qué funciona contra la rotación?

Escribe cada posición como un número complejo $z = x + iy$. Entonces:

$$
m_{10} + i\,m_{01} = \sum z\, I(z)
\quad\text{y}\quad
\theta = \arg\Big(\sum z\, I(z)\Big)
$$

Si el parche gira un ángulo $\alpha$, cada posición se vuelve $z' = e^{i\alpha} z$ con el mismo brillo:

$$
\sum z'\, I = e^{i\alpha} \sum z\, I \quad\Longrightarrow\quad \theta' = \theta + \alpha
$$

La orientación **gira exactamente lo mismo que la imagen**. Esa es la propiedad que necesitamos.

> [!tip] ¿Por qué un círculo y no un cuadrado?
> Al girar un cuadrado, sus esquinas "barren" píxeles que antes estaban fuera, y los momentos cambian sin que la imagen lo haya hecho. Un círculo es igual en todas las direcciones. OpenCV precalcula, para cada fila, hasta qué columna llega el círculo (`umax`).

## Relación con FAST

FAST exige un **arco de 9 o más píxeles** más claros o más oscuros que el centro ([[ORB 3 FAST]]). Ese arco desbalancea el parche, así que el centroide queda lejos del centro y $\theta$ es **estable**. Por eso orientación y FAST combinan bien.

> [!warning] Cuándo falla
> En un parche casi uniforme, $m_{10} \approx m_{01} \approx 0$, y $\theta$ queda dominado por el ruido. Con `fastThreshold = 10` (bajo) entran esquinas de poco contraste, cuya orientación es menos confiable. Es un *trade-off* entre cantidad y calidad.
