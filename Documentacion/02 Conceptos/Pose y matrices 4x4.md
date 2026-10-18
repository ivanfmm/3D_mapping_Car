---
tags: [nivel/2, nivel/3, tipo/concepto]
aliases: [Pose, Matriz de transformación, SE(3), Rodrigues]
---
# Pose y matrices 4×4

> [!info] Nivel 2 → 3 · Concepto base. Lo usan: [[VO 6 Pose acumulada y fallos]], [[LC 5 Grafo de poses]], [[Nube 3 Reproyeccion y keyframes]]

## Nivel 1: qué es una pose

La **pose** responde dos preguntas: **dónde estoy** (posición) y **hacia dónde miro** (orientación). En el proyecto, cada frame tiene la pose de la cámara en ese instante.

## Ejes de la cámara (convención de OpenCV)

```
        Z (hacia adelante, a donde mira la cámara)
       ↗
      /
     o────► X (derecha)
     │
     ▼
     Y (hacia ABAJO)
```

Todas las distancias están en **milímetros**, porque la calibración usó cuadros de 20 mm.

## Nivel 2: la matriz 4×4

Una pose se guarda como una matriz $T$ de 4×4:

$$
T = \begin{bmatrix} R & \mathbf{t} \\ \mathbf{0}^\top & 1 \end{bmatrix}
= \begin{bmatrix} r_{11} & r_{12} & r_{13} & t_x \\ r_{21} & r_{22} & r_{23} & t_y \\ r_{31} & r_{32} & r_{33} & t_z \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

- $R$ (3×3) = **rotación**
- $\mathbf{t}$ (3×1) = **traslación**, es decir, la posición

Para transformar un punto se le agrega un 1 (**coordenadas homogéneas**):

$$
\begin{bmatrix} \mathbf{p}' \\ 1 \end{bmatrix} = T \begin{bmatrix} \mathbf{p} \\ 1 \end{bmatrix}
\quad\Leftrightarrow\quad \mathbf{p}' = R\,\mathbf{p} + \mathbf{t}
$$

El 1 extra permite meter la rotación **y** la traslación en una sola multiplicación.

| Operación | En código | Significado |
|---|---|---|
| Componer | `A @ B` | Aplicar primero B y luego A |
| Invertir | `np.linalg.inv(T)` | Deshacer: $R^\top$ y $-R^\top \mathbf{t}$ |
| Posición | `T[:3, 3]` | Dónde está la cámara |
| Rotación | `T[:3, :3]` | Hacia dónde mira |

## Cómo se acumula la pose en `update()`

PnP entrega una $T$ que **lleva puntos de la cámara anterior a la cámara actual**. Pero la pose que guardamos es **cámara → mundo**. Por eso se invierte:

```python
self.pose = self.pose @ np.linalg.inv(T)
```

> [!example] Ejemplo numérico
> El carro avanza 100 mm hacia adelante. Una pared que estaba a Z = 1000 mm ahora se ve a Z = 900 mm. PnP entrega $\mathbf{t} = (0, 0, -100)$, porque "los puntos se acercaron". Al invertir queda $(0, 0, +100)$: **la cámara avanzó 100 mm**. Empezando en la identidad, la nueva pose tiene `T[:3, 3] = [0, 0, 100]`.

## Nivel 3: vector de Rodrigues (la rotación en 3 números)

Una matriz de rotación tiene 9 números, pero solo **3 grados de libertad**. El **vector de Rodrigues** $\mathbf{r} = \theta\,\hat{\mathbf{k}}$ los guarda así:

- **Dirección** $\hat{\mathbf{k}}$ = el eje de giro
- **Longitud** $\theta = \lVert \mathbf{r} \rVert$ = el ángulo de giro, en radianes

$$
R = I + \sin\theta\,[\hat{\mathbf{k}}]_\times + (1-\cos\theta)\,[\hat{\mathbf{k}}]_\times^2,
\qquad
[\hat{\mathbf{k}}]_\times = \begin{bmatrix} 0 & -k_z & k_y \\ k_z & 0 & -k_x \\ -k_y & k_x & 0 \end{bmatrix}
$$

En el código:

- `cv2.Rodrigues(rvec)` convierte de vector a matriz, y de matriz a vector.
- `mat_to_vec(T)` produce 6 números `[rx, ry, rz, tx, ty, tz]`. Son las variables del PGO.
- `angle_deg = np.degrees(np.linalg.norm(rvec))` es el ángulo total de un paso. Se usa para detectar saltos (`JUMP_MAX_ROT = 30°`).
