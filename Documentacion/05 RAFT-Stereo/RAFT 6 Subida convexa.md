---
tags: [nivel/3, tipo/raft]
aliases: [Subida convexa, Convex upsampling, upsample_flow]
---
# RAFT 6: Subida convexa y salida

> [!info] Nivel 3 · Serie RAFT: [[RAFT 1 Intuicion]] · [[RAFT 2 En el proyecto]] · [[RAFT 3 Codificadores]] · [[RAFT 4 Correlacion y piramide]] · [[RAFT 5 Actualizacion con GRU]] · **6**

## El problema

Todo el trabajo se hizo a 1/4 de resolución (184 × 320). Hay que regresar a 736 × 1280 sin perder los bordes de los objetos.

## La fórmula

Cada píxel final es una combinación de los **9 vecinos** (3×3) del bloque al que pertenece, en la malla de 1/4. Los pesos los predice la red para cada píxel y pasan por un *softmax*, así que son positivos y suman 1:

$$
d_{full}(p) = 4 \sum_{k=1}^{9} w_k(p)\; d_{1/4}(n_k), \qquad w_k \ge 0,\quad \sum_k w_k = 1
$$

- **"Convexa"** porque el resultado siempre queda entre el mínimo y el máximo de los 9 vecinos: nunca inventa un valor fuera de ese rango.
- **El factor 4:** un bloque a 1/4 equivale a 4 píxeles a resolución completa. Una disparidad de 20 a 1/4 es de 80 al final.

## En el código

```python
# en BasicMultiUpdateBlock: (2**n_downsample)**2 * 9 = 16 * 9 = 144 canales
mask = .25 * self.mask(net[0])

# en RAFTStereo.upsample_flow
mask = mask.view(N, 1, 9, factor, factor, H, W)
mask = torch.softmax(mask, dim=2)               # pesos de los 9 vecinos
up_flow = F.unfold(factor * flow, [3, 3], padding=1)
up_flow = torch.sum(mask * up_flow, dim=2)      # promedio ponderado
```

- Cada bloque de 1/4 se convierte en $4 \times 4 = 16$ píxeles, y cada uno necesita 9 pesos: 144 canales por bloque.
- `F.unfold` junta, para cada bloque, los valores de sus 9 vecinos.
- El `0.25` solo equilibra el entrenamiento.

## ¿Por qué no interpolar normal?

Una interpolación normal promedia a los vecinos con pesos fijos. En un **borde de profundidad**, eso inventa valores intermedios que no existen.

> [!example] Ejemplo: el borde de una caja frente a la pared
> A 1/4, un vecino tiene disparidad 20 (la pared) y el otro 40 (la caja).
>
> | Método | Disparidad a 1/4 | A resolución completa (×4) | Profundidad ($Z = 54{,}584 / d$) |
> |---|---|---|---|
> | Pared | 20 | 80 px | 682 mm |
> | Caja | 40 | 160 px | 341 mm |
> | Interpolación con pesos fijos 50/50 | 30 | 120 px | **455 mm**: un punto flotando entre la caja y la pared |
> | Subida convexa (pesos 0.05 / 0.95) | 39 | 156 px | 350 mm: pegado a la caja |
>
> La red aprende a darle peso solo a los vecinos **del mismo lado del borde**. Los puntos flotantes que aun así se cuelan los quitan los filtros de la nube ([[Nube 4 Filtros de ruido]]).

## La salida final

```python
flow_up = self.upsample_flow(coords1 - coords0, up_mask)
flow_up = flow_up[:, :1]            # solo la componente horizontal
...
if test_mode:
    return coords1 - coords0, flow_up   # (1/4, resolucion completa)
```

1. La subida se hace **una sola vez**, en la última iteración (`test_mode`): las anteriores no la necesitan.
2. Se queda con el canal horizontal; el vertical siempre es 0.
3. `run_raft` toma la versión completa, le cambia el signo, recorta el relleno y pone en 0 los valores negativos ([[RAFT 2 En el proyecto]]).

El resultado es un arreglo de 720 × 1280 en `float32`, con la disparidad en píxeles y con decimales.
