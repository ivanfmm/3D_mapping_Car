---
tags: [nivel/2, nivel/3, tipo/nube]
aliases: [Voxel, Voxel downsample, DBSCAN, DBSCAN_MAX]
---
# Nube 5: Voxel downsample y DBSCAN

> [!info] Nivel 2 → 3 · Serie Nube: [[Nube 1 Intuicion]] · [[Nube 2 En el proyecto]] · [[Nube 3 Reproyeccion y keyframes]] · [[Nube 4 Filtros de ruido]] · **5** · [[Nube 6 Piso y alineacion]] · [[Nube 7 Nube sparse]]

## Voxel downsample: un punto por cubito

```python
pcd_combined = pcd_combined.voxel_down_sample(VOXEL_SIZE)   # 8 mm
```

El espacio se divide en cubitos (**voxels**) de 8 mm de lado. Cada punto cae en el cubito con llave

$$
\big(\lfloor x/8 \rfloor,\ \lfloor y/8 \rfloor,\ \lfloor z/8 \rfloor\big)
$$

y de cada cubito queda **un solo punto**: el promedio de posiciones y de colores de los que cayeron ahí.

> [!example] Ejemplo
> | Punto (mm) | Llave |
> |---|---|
> | (3, 5, 1) | (0, 0, 0) |
> | (6, 2, 7) | (0, 0, 0) |
> | (9, 1, 1) | (1, 0, 0) |
>
> Los dos primeros se vuelven uno solo en (4.5, 3.5, 4); el tercero queda igual.

### Qué gana el mapa

1. **Sin repetidos:** la misma pared vista por 30 keyframes deja un punto por cubito, no 30.
2. **Menos ruido:** promediar $n$ mediciones independientes con ruido $\sigma$ deja un ruido de $\sigma / \sqrt{n}$. Si 4 frames vieron el mismo cubito, el ruido baja a la mitad.
3. **Tamaño manejable** para los pasos siguientes.

### Por qué una sola vez y al final

Antes se aplicaba por frame con cubitos de 20 mm, y eso tiraba puntos antes de tiempo. Ahora se aplica una sola vez, sobre todo lo acumulado: así cada cubito promedia lo que vieron **todos** los frames.

### El costo: 8 mm de resolución

Nada más chico que unos 8 mm se distingue en el mapa: dos cables separados 5 mm quedan en el mismo cubito. Bajar `VOXEL_SIZE` da más detalle, pero más puntos (a la mitad del tamaño, hasta 4 veces más puntos en superficies y 8 veces en volumen) y más riesgo de pasar `DBSCAN_MAX`.

## DBSCAN: quedarse con el bloque principal

```python
labels = np.array(pcd_tmp.cluster_dbscan(eps=60.0, min_points=20))
counts   = np.bincount(labels[labels >= 0])
main_lbl = int(counts.argmax())
pcd_final = pcd_tmp.select_by_index(np.where(labels == main_lbl)[0])
```

**DBSCAN** (*Density-Based Spatial Clustering of Applications with Noise*) agrupa puntos por **densidad**:

1. Un punto es **núcleo** si tiene al menos 20 vecinos a menos de 60 mm.
2. Dos núcleos a menos de 60 mm quedan en el mismo grupo, y los grupos crecen encadenando núcleos.
3. Un punto que no es núcleo pero está a menos de 60 mm de uno se une a su grupo (punto **borde**).
4. Lo que no alcanza ningún grupo es **ruido** (etiqueta −1).

> [!example] Ejemplo
> Un cuarto (paredes, piso y muebles conectados entre sí) forma un solo grupo grande. Un reflejo en la ventana que quedó flotando a 50 cm de la pared forma un grupo chico separado. Unos cuantos puntos sueltos son ruido. El código se queda solo con el grupo grande.

Ventajas frente a otros agrupamientos: no hay que decir cuántos grupos hay, y los grupos pueden tener cualquier forma (una pared es larga y delgada).

El código imprime cuántos grupos encontró, el tamaño del principal, cuánto ruido quitó y los 5 grupos más grandes. Si el segundo grupo es casi tan grande como el primero, probablemente el mapa se partió (por ejemplo, dos cuartos unidos por una puerta angosta) y se está tirando la mitad.

### Por qué existe `DBSCAN_MAX`

> [!warning] Memoria
> Open3D guarda cada punto en `float64`: 24 bytes de posición + 24 de color = 48 bytes, así que 2 millones de puntos son ≈ 96 MB. El problema es DBSCAN, que además guarda los vecinos de cada punto: su memoria crece con (puntos × vecinos), y con nubes grandes se acabó la memoria y se cayó el proceso. Arriba de 2 millones de puntos el paso se salta, y se usa la nube sin agrupar.
