---
tags: [nivel/2, nivel/3, tipo/pgo]
aliases: [_LoopDetector, Votos FAISS, Detección de lugares, _add_loop_keyframe]
---
# LC 3: Detección de candidatos (FAISS y votos)

> [!info] Nivel 2 → 3 · Serie LC: [[LC 1 Intuicion]] · [[LC 2 En el proyecto]] · **3** · [[LC 4 Verificacion geometrica]] · [[LC 5 Grafo de poses]] · [[LC 6 Optimizacion]] · Requisito: [[ORB 6 Hamming y matching]]

Pregunta de este paso: **¿este frame se parece a alguno que ya vi hace rato?** Es una búsqueda rápida y tolerante; la verificación estricta viene después ([[LC 4 Verificacion geometrica]]).

## Nivel 2: qué entra al índice (`_add_loop_keyframe`)

```python
if self._last_lc_pose is not None:
    rel = np.linalg.inv(self._last_lc_pose) @ self.pose
    mov = np.linalg.norm(rel[:3, 3])
    ang = np.degrees(np.linalg.norm(cv2.Rodrigues(rel[:3, :3])[0]))
    if mov < LC_KF_MOV_MM and ang < LC_KF_ROT_DEG:   # 50 mm y 5 grados
        return
self._loop_detector.add(frame_idx, des)
self._last_lc_pose = self.pose.copy()
```

Solo entran **keyframes**: frames donde el carro avanzó ≥ 50 mm o giró ≥ 5° desde el último que entró. Los frames fallidos tampoco entran, porque no tienen pose confiable. Antes del 18-oct-2026 entraban las ~1000 huellas de **todos** los frames.

## Nivel 2: el índice (`_LoopDetector`)

```python
self.index     = faiss.IndexBinaryFlat(256)   # busqueda exacta por Hamming, 256 bits
self._frame_of = []                           # posicion en el indice -> frame
```

- **`add(frame_idx, des)`:** agrega las huellas (32 bytes cada una) y anota a qué frame pertenece cada una.
- **`query(des, current_idx)`:** para **cada** huella del frame actual busca su vecino más cercano en todo el índice y lo convierte en un voto.

```python
D, I = self.index.search(des, 1)                  # k = 1: solo el mas cercano
for (dist,), (nn_id,) in zip(D, I):
    if int(dist) > LC_HAMMING_MAX:                # 80 bits: pareja de mala calidad
        continue
    fid = self._frame_of[nn_id]
    if current_idx - fid < MIN_LOOP_SEP:          # menos de 80 frames: es reciente
        continue
    votes[fid] = votes.get(fid, 0) + 1
best = max(votes, key=votes.get)
```

El frame con más votos es el candidato, si tiene al menos `MIN_LOOP_VOTES = 8`.

> [!example] Ejemplo de votación
> El frame 400 tiene 1000 huellas. Sus vecinos más cercanos en el índice:
>
> | Caso | Huellas | Qué pasa |
> |---|---|---|
> | Vecino a más de 80 bits | 520 | No votan |
> | Vecino en un frame reciente (321 a 399) | 410 | No votan |
> | Vecino en el frame 85 | 41 | 41 votos para el 85 |
> | Vecino en el frame 90 | 17 | 17 votos para el 90 |
> | Vecinos repartidos en otros frames | 12 | 1 o 2 votos cada uno |
>
> Gana el 85 con 41 votos (≥ 8): es el candidato.

## Nivel 3: por qué estaba muerto

Con `k = 1`, cada huella solo ve a **su** vecino más cercano. Cuando el índice tenía todos los frames, ese vecino casi siempre estaba en un frame de hace unos instantes: la misma esquina vista desde casi el mismo lugar se parece más que la misma esquina vista 300 frames después.

> [!example] Una huella del frame 400
> | Candidato | Distancia de Hamming |
> |---|---|
> | La misma esquina en el frame 399 | 18 bits |
> | La misma esquina en el frame 85 | 35 bits |
>
> FAISS devuelve solo el 399. Como $400 - 399 < 80$, se descarta. El 85, que era el loop real, **nunca aparece**. En `session_20261018_114813` hubo 476 mil huellas en el índice y 0 loops.

Al meter solo keyframes, los frames recientes casi idénticos desaparecen del índice y los loops empezaron a aparecer, pero el problema de fondo sigue: los keyframes recientes (menos de 80 frames atrás) todavía pueden "robarse" el vecino. Una alternativa simple sería pedir varios vecinos (`k` = 5 o 10) y votar por el primero que no sea reciente.

### Cuánto cuesta buscar

`IndexBinaryFlat` es búsqueda **exacta**: compara cada huella del frame actual contra **todas** las del índice. Si el índice tiene $M$ huellas, una consulta cuesta $1000 \cdot M$ comparaciones (cada una es un XOR y un conteo de bits). Como $M$ crece con los keyframes, el costo total de una sesión crece con el **cuadrado** del número de keyframes. Con solo keyframes, $M$ crece mucho más lento que antes y estar quieto ya no cuesta nada. El cálculo completo está en [[ORB 6 Hamming y matching]].

### ¿Por qué 80 bits?

Dos huellas al azar difieren en promedio 128 bits, con desviación 8. 80 bits está 6 desviaciones por debajo: es muy improbable que sea casualidad **si las huellas fueran al azar**. Con texturas repetidas (losetas, cortinas) eso no se cumple, y por eso los votos solo producen candidatos, no loops.
