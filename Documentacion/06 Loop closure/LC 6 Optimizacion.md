---
tags: [nivel/2, nivel/3, tipo/pgo]
aliases: [PGO, Pose Graph Optimization, least_squares, _optimize_pose_graph, Huber]
---
# LC 6: La optimización (`least_squares`)

> [!info] Nivel 2 → 3 · Serie LC: [[LC 1 Intuicion]] · [[LC 2 En el proyecto]] · [[LC 3 Deteccion de candidatos]] · [[LC 4 Verificacion geometrica]] · [[LC 5 Grafo de poses]] · **6**

## Nivel 2: `_optimize_pose_graph` paso a paso

```python
fixed_id = frame_ids[0]                 # la primera pose no se mueve
var_ids  = frame_ids[1:]
x0 = np.concatenate([mat_to_vec(all_poses[fid]) for fid in var_ids])

res = least_squares(
    residuals, x0,
    jac_sparsity=sparsity,
    method='trf',
    x_scale='jac',       # escala cada variable segun su efecto (rad vs mm)
    max_nfev=PGO_MAX_ITER,
)
```

1. **Fija la primera pose.** Si todas fueran variables, el mapa completo podría moverse o girar junto sin cambiar el costo: habría infinitas soluciones. Fijar una "ancla" deja una sola.
2. **Variables:** 6 números por cada otra pose (`mat_to_vec`: vector de Rodrigues + traslación; ver [[Pose y matrices 4x4]]). Con 487 frames son $486 \times 6 = 2916$ incógnitas.
3. **Residuos:** 6 por arista ([[LC 5 Grafo de poses]]).
4. **Resuelve** con `least_squares`.
5. **Regresa** las poses optimizadas. `_run_pgo` reemplaza `all_poses` y pone `self.pose` igual a la del último frame, para que la odometría siga desde la pose corregida.

### Cuándo corre

- Solo después de aceptar un loop.
- Como máximo una vez cada `LOOP_PGO_COOLDOWN = 30` frames.
- Un mismo candidato no vuelve a crear un loop si ya hay uno con él a menos de 30 frames.

### Los argumentos de `least_squares`

| Argumento | Valor | Qué significa |
|---|---|---|
| `method` | `'trf'` | *Trust Region Reflective*: en cada paso confía en una aproximación lineal solo dentro de una "región de confianza" que se agranda o achica según funcione |
| `x_scale` | `'jac'` | Ajusta la escala de cada variable según cuánto mueve a los residuos, porque mezcla radianes con milímetros |
| `max_nfev` | 50 | Evaluaciones máximas de los residuos |
| `jac_sparsity` | matriz de 0 y 1 | Qué residuos dependen de qué variables (abajo) |
| `loss` | (por defecto, lineal) | Mínimos cuadrados normales. Ver "Pérdida robusta" |

## Nivel 3: el jacobiano disperso

`least_squares` necesita el **jacobiano**: cuánto cambia cada residuo al mover un poco cada variable. Como no se le da una fórmula, scipy lo estima moviendo variables y viendo qué pasa (diferencias finitas).

- Con 2916 residuos y 2916 variables, el jacobiano tiene unos **8.5 millones** de entradas.
- Pero cada arista depende solo de **2 poses**: sus 6 filas solo tienen 12 columnas distintas de cero.

```python
sparsity = lil_matrix((6 * len(edges), 6 * len(var_ids)), dtype=np.int8)
for e, (i, j, _, _) in enumerate(edges):
    for fid in (i, j):
        if fid != fixed_id:
            c = col_of[fid]
            sparsity[6*e : 6*e+6, c : c+6] = 1
```

Sin esa información, scipy tendría que mover **una variable a la vez**: 2916 evaluaciones de todos los residuos para un solo jacobiano. Con ella, junta las columnas que no comparten filas (por ejemplo, la pose 10 y la pose 200, que nunca aparecen en la misma arista) y las mueve **al mismo tiempo**. Así bastan unas **12 a 18 evaluaciones**.

> [!example] Por qué se pueden mover juntas
> Si muevo un poco la pose 10, solo cambian los residuos de las aristas (9, 10) y (10, 11). Si al mismo tiempo muevo la pose 200, solo cambian los de (199, 200) y (200, 201). Ningún residuo cambia por las dos, así que una sola evaluación sirve para medir el efecto de ambas.

## Nivel 3: pérdida robusta (pendiente)

Con mínimos cuadrados normales, el costo de un residuo es $r^2$. Un loop **falso** con error de 100 sigmas aporta $100^2 = 10{,}000$ (por 5 de peso, 50,000) y arrastra a todo el mapa. Eso fue lo que pasó al revivir el loop closure: el recorrido pasó de 4 m a 90 m.

Con la pérdida de Huber (`loss='huber'`, `f_scale=1`), scipy le aplica a cada residuo $s = r^2$:

$$
\rho(s) = \begin{cases} s & s \le 1 \\ 2\sqrt{s} - 1 & s > 1 \end{cases}
$$

| Residuo $r$ (en sigmas) | Mínimos cuadrados ($r^2$) | Huber |
|---|---|---|
| 0.5 | 0.25 | 0.25 |
| 1 | 1 | 1 |
| 3 | 9 | 5 |
| 10 | 100 | 19 |
| 100 | 10,000 | 199 |

Los errores normales se tratan igual que antes y los enormes pierden fuerza: crecen como $|r|$ y no como $r^2$. Huber **no** sustituye a una buena verificación ([[LC 4 Verificacion geometrica]]): solo limita el daño de los loops falsos que se cuelen.

## Detalle menor

Después del PGO, `_last_lc_pose` (la referencia para decidir qué frames entran al índice de loops, [[LC 3 Deteccion de candidatos]]) no se actualiza con las poses optimizadas.
