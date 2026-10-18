---
tags: [nivel/2, nivel/3, tipo/vo]
aliases: [Pose acumulada, Drift, _fallo, Detector de saltos, MAX_FALLOS]
---
# VO 6: Pose acumulada, saltos y fallos

> [!info] Nivel 2 → 3 · Serie VO: [[VO 1 Intuicion]] · [[VO 2 En el proyecto]] · [[VO 3 Profundidad de los puntos]] · [[VO 4 PnP RANSAC]] · [[VO 5 Movimiento plano]] · **6**

## Nivel 2: acumular la pose

PnP entrega $T$, que lleva puntos de la cámara anterior a la actual. La pose que guardamos es **cámara → mundo**, así que el paso se invierte antes de componerlo:

```python
self.pose = self.pose @ np.linalg.inv(T)
self.all_poses[frame_idx] = self.pose.copy()

# arista secuencial: lo que midio la odometria entre la referencia y este frame
T_rel = np.linalg.inv(self.all_poses[self._prev_idx]) @ self.pose
self.seq_edges.append((self._prev_idx, frame_idx, T_rel))
```

> [!example] El carro avanza 100 mm
> Una pared que estaba a $Z = 1000$ mm ahora se ve a 900 mm. PnP entrega $t = (0, 0, -100)$ ("los puntos se acercaron"). Al invertir queda $(0, 0, +100)$: **la cámara avanzó 100 mm**. Empezando en la identidad, la nueva pose tiene `T[:3, 3] = [0, 0, 100]`. (Más detalle en [[Pose y matrices 4x4]].)

El sistema del mundo es el de la cámara del **primer frame**: X a la derecha, Y hacia abajo, Z hacia adelante, en mm.

## Nivel 2: el detector de saltos

```python
if tvec_mag > JUMP_MAX_TRANS or angle_deg > JUMP_MAX_ROT:   # 300 mm o 30 grados
    self._fallo(...)
```

| Límite | Por frame | A 30 FPS equivale a |
|---|---|---|
| `JUMP_MAX_TRANS` | 300 mm | 9 m/s |
| `JUMP_MAX_ROT` | 30° | 900°/s |

Ningún carrito se mueve así: si PnP lo dice, son parejas malas que se pusieron de acuerdo.

## Nivel 2: qué pasa cuando un frame falla (`_fallo`)

Cualquier salida temprana de `update()` (pocas esquinas, pocas parejas, PnP sin inliers o un salto) llama a `_fallo()`:

1. El frame fallido **no** se vuelve la referencia: el siguiente frame se compara contra el **último frame bueno**, así que su movimiento sí se mide.
2. El frame fallido **se quita de `all_poses`**, para que no entre a la nube ni al grafo con una pose vieja.
3. Si se juntan `MAX_FALLOS = 5` fallos seguidos, se reinicia la referencia en ese frame. Solo entonces se pierde el movimiento del hueco: el último frame bueno ya quedó tan lejos que casi no comparten esquinas.

| Frame | Resultado | Antes del 18-oct: se comparaba con | Ahora: se compara con |
|---|---|---|---|
| 10 | OK | 9 | 9 |
| 11 | Falla | 10 | 10 |
| 12 | OK | **11** (pierde 10→11) | **10** (mide 10→12 completo) |

> [!success] Resultado de la prueba (18-oct-2026)
> En `session_20261018_114813` (487 frames) se inyectaron 12 frames negros en 4 huecos. El error final contra la corrida limpia bajó de **212 mm** (antes) a **9 mm** (ahora). La corrida limpia quedó idéntica (recorrido de 4171 mm) y tardó 21 s en lugar de 100 s.

> [!warning] Efecto secundario en el grafo de poses
> El frame del reinicio conserva la última pose conocida, pero ninguna arista lo une con el último frame bueno: el grafo queda partido en dos. Mientras el loop closure esté apagado no importa. Ver [[LC 5 Grafo de poses]].

## Nivel 2: por qué el ICP está apagado

`_icp_refine` refinaba el resultado de PnP alineando las nubes densas de dos frames seguidos (ICP punto a punto con Open3D). Se apagó (`USE_ICP = False`) por dos razones medidas:

- Con el carro **quieto** (`session_20261018_092006`, frames 0-39), la odometría con ICP marcó **0.79 m** de recorrido falso; sin ICP, 0.04 m. El piso domina la nube y el ICP "patina" sobre el plano.
- Era el 80 % del tiempo de la odometría: 220 ms por frame con ICP, 45 ms sin él.

`reprocesar.py --icp` lo vuelve a prender para comparar.

## Nivel 3: cómo crece el *drift*

Cada paso tiene un error, y la pose es la suma de todos los pasos. Hay dos tipos de error, y crecen distinto:

**Error aleatorio** (a veces de más, a veces de menos, independiente en cada paso). Si cada paso tiene desviación $\sigma$, después de $N$ pasos

$$
\sigma_N = \sigma \sqrt{N}
$$

Con $\sigma = 1$ mm y 487 frames: $\sqrt{487} \approx 22$ mm. Crece lento.

**Error sistemático** (siempre hacia el mismo lado; por ejemplo, una escala mal calibrada). Crece en línea recta con $N$: 1 mm por paso son 487 mm. Por eso la restricción plana importa tanto: quita en cada paso el error sistemático en Y, cabeceo y alabeo antes de que se acumule ([[VO 5 Movimiento plano]]).

**Error de rumbo.** Un error en el giro es el más caro, porque desvía **todo el camino que sigue**. Si el rumbo queda mal por un ángulo $\theta$, después de recorrer $L$ el desfase lateral es aproximadamente

$$
L \sin\theta
$$

Con $\theta = 5°$ y $L = 4$ m: $4000 \cdot \sin 5° \approx 350$ mm.

> [!note] La escala depende de la calibración
> Todo se mide en mm gracias a la Q. Un error de 1 % en el *baseline* o en la $f$ (ver [[Calibracion estereo a fondo]]) es un error sistemático de 1 % en **todo** el recorrido: en 4171 mm, unos 42 mm.
