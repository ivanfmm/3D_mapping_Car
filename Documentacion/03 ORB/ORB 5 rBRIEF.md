---
tags: [nivel/3, tipo/orb]
aliases: [BRIEF, rBRIEF, Descriptor ORB, Huella binaria]
---
# ORB 5: rBRIEF (la huella de 256 bits)

> [!info] Nivel 3 · Serie ORB: [[ORB 1 Intuicion]] · [[ORB 2 En el proyecto]] · [[ORB 3 FAST]] · [[ORB 4 Orientacion]] · **5** · [[ORB 6 Hamming y matching]]

## Nivel 1: 256 preguntas de sí/no

Alrededor de la esquina se eligen **256 parejas de píxeles** (siempre las mismas posiciones relativas). Para cada pareja se pregunta: **"¿el primero es más oscuro que el segundo?"**. Las 256 respuestas (1 = sí, 0 = no) son la huella. Dos fotos de la misma esquina deberían dar casi las mismas respuestas.

## Nivel 3: la definición

### 1. Suavizar primero

Si un solo píxel tiene ruido, puede voltear la respuesta de una pregunta. Por eso OpenCV **desenfoca** cada piso de la pirámide con un Gaussiano de 7×7 ($\sigma = 2$) antes de comparar. Es un filtro paso-bajas: quita las variaciones finas (el ruido) y deja la forma general.

### 2. La prueba binaria

Para una pareja de posiciones $(\mathbf{a}, \mathbf{b})$ dentro del parche suavizado $\tilde I$:

$$
\tau(\mathbf{a}, \mathbf{b}) =
\begin{cases} 1 & \text{si } \tilde I(\mathbf{a}) < \tilde I(\mathbf{b}) \\ 0 & \text{en otro caso} \end{cases}
$$

Con 256 parejas sale un número de 256 bits, que se guarda en **32 bytes**: una fila de `des` (forma `(N, 32)`, `uint8`). Dentro de cada byte, la prueba 0 es el bit **menos** significativo:

$$
\text{byte}_k = \sum_{j=0}^{7} 2^j\, \tau_{8k + j}
$$

> [!example] Ejemplo: un byte de huella
> Las primeras 8 pruebas dan `τ0..τ7 = 1, 0, 1, 1, 0, 0, 1, 0`:
>
> $$\text{byte}_0 = 1\cdot2^0 + 1\cdot2^2 + 1\cdot2^3 + 1\cdot2^6 = 1 + 4 + 8 + 64 = 77$$
>
> Entonces `des[i, 0] == 77`.

### 3. "Rotated": girar las preguntas

Para que la huella no cambie cuando el carro gira, las posiciones de cada pareja se giran con el ángulo $\theta$ de la esquina ([[ORB 4 Orientacion]]) antes de leer los píxeles:

$$
\mathbf{a}' = R_\theta\,\mathbf{a}, \quad \mathbf{b}' = R_\theta\,\mathbf{b}, \qquad
R_\theta = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}
$$

Si la imagen giró $\alpha$, entonces $\theta$ también giró $\alpha$ y las preguntas se hacen sobre **los mismos puntos físicos**. El resultado es la misma huella.

### 4. ¿Qué parejas? Aprendidas o aleatorias

| Variante | Cómo se eligen las 256 parejas | Problema |
|---|---|---|
| BRIEF original | Al azar (Gaussiana alrededor del centro) | No es invariante a rotación |
| *Steered* BRIEF | Al azar y luego giradas con θ | Al girar, los bits se **correlacionan** y la huella distingue menos |
| **rBRIEF** (paper de ORB) | **Aprendidas**: de todas las pruebas posibles en un parche de 31×31, se eligen de forma voraz las de **varianza alta** (media ≈ 0.5) y **baja correlación** entre sí | Solo existe para un parche de 31×31 |

> [!warning] Detalle del proyecto: con `patchSize = 19` no se usa el patrón aprendido
> En el código fuente de OpenCV (`orb.cpp`), el patrón aprendido `bit_pattern_31_` solo se usa si `patchSize == 31`. Con cualquier otro tamaño, OpenCV genera un **patrón aleatorio fijo** (semilla constante), así que el proyecto usa algo más cercano a *steered BRIEF* que a rBRIEF.
> Experimento para comparar con `reprocesar.py --sparse`: `patchSize = 31` y `edgeThreshold = 31`, contra los valores actuales. Mide la cobertura de VO y el número de loop closures. Ojo: un parche más grande pierde esquinas cerca del borde y cuesta un poco más.

## ¿Contra qué es robusta la huella?

| Cambio entre fotos | ¿La huella aguanta? | Por qué |
|---|---|---|
| Traslación | Sí | Se mide relativa a la esquina |
| Giro en el plano de la imagen | Sí | Las parejas se giran con θ |
| Acercarse o alejarse | Más o menos | La pirámide cubre saltos de ×1.2 por piso |
| Más o menos luz (uniforme) | Sí | Comparar $a < b$ no cambia si todo sube +20 |
| Cambio de punto de vista 3D grande | No | La forma del parche se deforma (perspectiva) |
| Desenfoque por movimiento | Poco | Los bits se vuelven aleatorios. Por eso `reprocesar.py` filtra los frames borrosos |
