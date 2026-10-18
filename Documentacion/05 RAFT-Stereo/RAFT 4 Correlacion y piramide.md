---
tags: [nivel/3, tipo/raft]
aliases: [Volumen de correlación, Pirámide de correlación, corr_implementation]
---
# RAFT 4: Volumen de correlación y pirámide

> [!info] Nivel 3 · Serie RAFT: [[RAFT 1 Intuicion]] · [[RAFT 2 En el proyecto]] · [[RAFT 3 Codificadores]] · **4** · [[RAFT 5 Actualizacion con GRU]] · [[RAFT 6 Subida convexa]]

## La fórmula

Para cada fila $y$, compara cada posición izquierda $x$ contra cada posición derecha $x'$ de la misma fila con un **producto punto** de sus vectores de características ($D = 256$):

$$
C(y, x, x') = \frac{1}{\sqrt{D}} \sum_{c=1}^{D} f_{izq}(c, y, x)\; f_{der}(c, y, x')
$$

- Un producto punto grande significa que los dos vectores "apuntan" hacia el mismo lado: las zonas se parecen.
- Dividir entre $\sqrt{D} = 16$ mantiene los valores en un rango manejable aunque haya 256 sumandos.
- Es **1D** porque, con las imágenes rectificadas, el gemelo solo puede estar en la misma fila. En flujo óptico general (dos fotos cualesquiera) habría que comparar contra toda la imagen.

> [!example] Ejemplo con vectores de 3 números (sin el factor $1/\sqrt{D}$)
> El bloque $x = 10$ de la izquierda tiene $f_{izq} = (1, 0, 2)$. En su fila de la derecha:
>
> | $x'$ | $f_{der}$ | Producto punto |
> |---|---|---|
> | 6 | (0, 1, 0) | 0 |
> | 7 | (1, 0, 2) | **5** |
> | 8 | (1, 1, 1) | 3 |
> | 9 | (0, 0, 1) | 2 |
>
> El mejor parecido está en $x' = 7$: disparidad de $10 - 7 = 3$ bloques. La red no se queda con el máximo sin más: lee los valores alrededor de su estimación y decide la corrección ([[RAFT 5 Actualizacion con GRU]]). Así puede usar el contexto donde no hay un pico claro, como en una pared lisa.

## ¿Cuánto mide el volumen?

A 1/4 de resolución hay $184 \times 320 = 58{,}880$ bloques en la imagen izquierda, y cada uno se compara con los 320 de su fila:

$$
184 \times 320 \times 320 \approx 18.8 \text{ millones de valores} \approx 75 \text{ MB en float32}
$$

| `corr_implementation` | Qué hace | Ventaja | Costo |
|---|---|---|---|
| `reg` | Calcula todo el volumen una vez y lo guarda | Pocas cuentas: ≈ 4.8 mil millones de multiplicaciones, una sola vez | Guarda el volumen y su pirámide en la GPU |
| **`alt`** (el del proyecto) | En cada iteración calcula solo los 36 valores que se van a leer por bloque | No guarda el volumen | $58{,}880 \times 36 \times 256 \approx 543$ millones de multiplicaciones **por iteración**: ≈ 10.9 mil millones en 20 iteraciones |
| `reg_cuda`, `alt_cuda` | Versiones más rápidas de las anteriores | Velocidad | Requieren compilar la extensión de CUDA de `RAFT-Stereo/sampler` |

`alt` hace unas dos veces más cuentas que `reg`, a cambio de no ocupar memoria con el volumen.

## La pirámide

El lado derecho se promedia de 2 en 2 **a lo ancho** para formar 4 pisos. Cada piso es la misma comparación, pero contra una versión cada vez más "borrosa" y más corta de la fila.

| Piso | Ancho de la fila derecha | Un paso equivale a (resolución completa) | Alcance con ±4 pasos |
|---|---|---|---|
| 0 | 320 | 4 px | ±16 px |
| 1 | 160 | 8 px | ±32 px |
| 2 | 80 | 16 px | ±64 px |
| 3 | 40 | 32 px | ±128 px |

## La búsqueda local

En cada iteración, la red tiene una estimación de dónde está el gemelo: $x' = x - d$. En cada piso $i$ lee 9 valores ($r = 4$ a cada lado) alrededor de $x'/2^i$:

$$
C_i\!\left(y,\ x,\ \frac{x'}{2^i} + k\right), \qquad k = -4, \dots, 4
$$

Son $4 \times 9 = 36$ números por bloque: lo que la red "ve" en cada iteración. El piso 3 ve lejos (±128 px, suficiente para un objeto a medio metro, que tiene ≈ 110 px de disparidad) y el piso 0 afina.

### Posiciones con decimales

La estimación casi nunca es un número entero. El valor en una posición intermedia se **interpola** entre sus dos vecinos:

$$
C(7.3) \approx 0.7\,C(7) + 0.3\,C(8)
$$

> [!example] Ejemplo
> Si $C(7) = 5$ y $C(8) = 3$, entonces $C(7.3) \approx 0.7 \cdot 5 + 0.3 \cdot 3 = 4.4$. Esto le permite a la red mover su estimación en fracciones de píxel y obtener disparidad **subpíxel**.

En el código (`PytorchAlternateCorrBlock1D`), la interpolación la hace `F.grid_sample` sobre el mapa de características derecho, y luego se calcula el producto punto con el izquierdo.
