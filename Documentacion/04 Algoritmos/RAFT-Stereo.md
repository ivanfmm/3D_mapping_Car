---
tags: [nivel/1, nivel/2, nivel/3, tipo/algoritmo]
aliases: [RAFT, Red de disparidad, run_raft]
---
# RAFT-Stereo: la red que calcula la disparidad

> [!info] Nivel 1 → 3 · Requisitos: [[Camara estereo y disparidad]], [[Calibracion y rectificacion]] · Lo usa: [[03 Flujo de datos]] (paso 6)

## Nivel 1: la idea

Para cada píxel de la imagen izquierda hay que encontrar su gemelo en la derecha. Gracias a la rectificación, el gemelo está **en la misma fila**, un poco más a la izquierda. Esa distancia en píxeles es la disparidad.

RAFT-Stereo lo resuelve como quien endereza un cuadro torcido: empieza con una suposición (disparidad 0 en toda la imagen), revisa qué tan bien coincide, corrige un poco y repite. En el proyecto repite **20 veces**.

| Paso | Qué hace | Analogía |
|---|---|---|
| Extraer características | Convierte cada zona de la imagen en un vector de 256 números que describe "cómo se ve" | Como la huella de ORB, pero aprendida y para **todos** los píxeles |
| Volumen de correlación | Compara cada píxel izquierdo contra todos los de su fila en la derecha | Una tabla de "qué tanto se parecen" |
| Actualización iterativa | Una red recurrente (GRU) mira la tabla alrededor de la estimación actual y propone una corrección | Enderezar el cuadro poco a poco |
| Subir resolución | La red trabaja a 1/4 del tamaño; al final regresa a la resolución completa | |

> [!note] ¿Por qué no usar ORB para esto?
> ORB da unas 1000 esquinas por imagen. La nube densa necesita profundidad en **cada** píxel, incluidas las paredes lisas, donde no hay esquinas. Una red entrenada puede "rellenar" esas zonas usando el contexto que las rodea.

## Nivel 2: en el código

### Configuración (`build_model_args`)

| Argumento | Valor | Qué significa |
|---|---|---|
| `corr_implementation` | `'alt'` | Calcula la correlación al vuelo en lugar de guardarla completa: usa menos memoria de la GPU |
| `corr_levels` | 4 | Pisos de la pirámide de correlación |
| `corr_radius` | 4 | Posiciones que mira a cada lado de la estimación actual, por piso (9 en total) |
| `n_downsample` | 2 | La red trabaja a $1/2^2 = 1/4$ de la resolución |
| `n_gru_layers` | 3 | GRUs a 1/4, 1/8 y 1/16 de la resolución |
| `hidden_dims` | `[128, 128, 128]` | Tamaño del estado de cada GRU |
| `mixed_precision` | `True` | Usa `float16` donde se puede: más rápido y con menos memoria |
| `valid_iters` | `INFER_ITERS = 20` | Iteraciones de refinamiento |
| `context_norm` | `'batch'` | Normalización de la red de contexto |
| `shared_backbone`, `slow_fast_gru` | `False` | Opciones del modelo *realtime*; aquí apagadas |
| `dropout` | 0.0 | Solo sirve al entrenar |

> [!note] De dónde salen estos valores
> Son los del demo de Middlebury del README de RAFT-Stereo (`--corr_implementation alt --mixed_precision`) y tienen que corresponder a los pesos que se cargan. Si cambias de pesos, cambia también la configuración.

### Pesos disponibles (`RAFT-Stereo/models/`)

| Archivo | Uso |
|---|---|
| `raftstereo-middlebury.pth` | **El que se usa** (`RAFT_MODEL`). Ajustado con Middlebury: interiores en alta resolución |
| `raftstereo-eth3d.pth` | Ajustado con ETH3D |
| `raftstereo-sceneflow.pth` | Entrenado solo con datos sintéticos (SceneFlow) |
| `raftstereo-realtime.pth` | Versión rápida. Pide otra configuración: `--shared_backbone --n_downsample 3 --n_gru_layers 2 --slow_fast_gru --valid_iters 7` |
| `iraftstereo_rvc.pth` | Versión del *Robust Vision Challenge*; usa `--context_norm instance` |

Las rutas se pueden cambiar sin tocar el código con las variables de entorno `RAFT_DIR` y `RAFT_MODEL`.

### Carga (`load_raft_model`)

```python
model = RAFTStereo(args)
state = torch.load(str(RAFT_MODEL), map_location='cpu')
clean_state = {k.replace('module.', ''): v for k, v in state.items()}
model.load_state_dict(clean_state)
model = model.cuda().eval()
```

- Los pesos se guardaron con `nn.DataParallel`, que le antepone `module.` a cada nombre. Por eso se quita antes de cargarlos.
- `.cuda()` sube el modelo a la GPU y `.eval()` lo pone en modo inferencia.

### Una inferencia (`run_raft`)

1. **`pad32`:** rellena abajo y a la derecha (con `BORDER_REFLECT`) hasta un múltiplo de 32. El alto pasa de 720 a 736; el ancho 1280 ya es múltiplo. La red reduce la imagen a 1/4, 1/8 y 1/16, y la pirámide de correlación la divide todavía más: con múltiplos de 32 todas las divisiones son exactas.
2. **`to_tensor`:** BGR → RGB (la red se entrenó con RGB) y `(H, W, 3)` → `(1, 3, H, W)` en la GPU. Dentro de la red cada valor se lleva a $[-1, 1]$ con $2\,(I/255) - 1$.
3. **`model(..., iters=20, test_mode=True)`** devuelve dos cosas: el flujo a 1/4 y el flujo a resolución completa. Solo se usa el segundo.
4. **Signo:** la red entrega el desplazamiento horizontal $x_{der} - x_{izq}$, que es negativo. La disparidad es $d = -\text{flujo}$.
5. **`clamp(min=0)`:** una disparidad negativa es imposible (el punto quedaría detrás de la cámara). Se vuelve 0 y luego la descartan los filtros (`disp > 2`).
6. **Recorte:** `[:orig_h, :orig_w]` quita el relleno.

El decorador `@torch.no_grad()` evita que PyTorch guarde información para entrenar, lo que ahorra memoria.

> [!example] Tamaños de los datos con una imagen del proyecto
> | Etapa | Tamaño |
> |---|---|
> | Entrada | 720 × 1280 |
> | Con relleno | 736 × 1280 |
> | Características (1/4) | 184 × 320, 256 canales |
> | Estados de las GRU | 184 × 320, 92 × 160 y 46 × 80 |
> | Salida | 736 × 1280 → recorte a 720 × 1280, `float32` |

La disparidad se guarda en `disparity_raw/disp_XXXXXX.npy` para que `reprocesar.py` no tenga que volver a correr RAFT. El tiempo total se imprime al final de la fase 2 como `[TIEMPO] RAFT`.

## Nivel 3: cómo funciona por dentro

### 1. Dos redes codificadoras

- **`fnet` (características):** procesa las dos imágenes y produce un vector de $D = 256$ números por píxel, a 1/4 de resolución.
- **`cnet` (contexto):** procesa solo la izquierda y produce el estado inicial y el contexto de cada GRU.

### 2. Volumen de correlación 1D

Para cada fila $y$, compara cada posición izquierda $x$ contra cada posición derecha $x'$ de la misma fila con un producto punto:

$$
C(y, x, x') = \frac{1}{\sqrt{D}} \sum_{c=1}^{D} f_{izq}(c, y, x)\; f_{der}(c, y, x')
$$

Es **1D** porque, con las imágenes rectificadas, el gemelo solo puede estar en la misma fila.

> [!example] Ejemplo con vectores de 3 números (sin el factor $1/\sqrt{D}$)
> El píxel $x = 10$ de la izquierda tiene $f_{izq} = (1, 0, 2)$. En su fila de la derecha:
>
> | $x'$ | $f_{der}$ | Producto punto |
> |---|---|---|
> | 6 | (0, 1, 0) | 0 |
> | 7 | (1, 0, 2) | **5** |
> | 8 | (1, 1, 1) | 3 |
> | 9 | (0, 0, 1) | 2 |
>
> La mejor coincidencia está en $x' = 7$, así que la disparidad es $10 - 7 = 3$. La red no se queda con el máximo sin más: lee los valores alrededor de su estimación y decide la corrección. Eso le permite usar el contexto donde no hay un pico claro, como en una pared lisa.

Con las medidas del proyecto, el primer piso tiene $184 \times 320 \times 320 \approx 18.8$ millones de valores (≈ 75 MB en `float32`). La implementación `reg` lo guarda completo; `alt` calcula solo los valores que se piden en cada iteración.

### 3. Pirámide y búsqueda local

El lado derecho se promedia de 2 en 2 a lo ancho para formar 4 pisos. En cada iteración, alrededor de la estimación actual se leen $2r + 1 = 9$ valores por piso: **36 números por píxel**.

| Piso | Un paso equivale a (resolución completa) | Alcance con ±4 pasos |
|---|---|---|
| 0 | 4 px | ±16 px |
| 1 | 8 px | ±32 px |
| 2 | 16 px | ±64 px |
| 3 | 32 px | ±128 px |

El piso 3 "ve de lejos" y el piso 0 afina. ±128 px alcanza para un objeto a medio metro, que tiene unos 110 px de disparidad.

### 4. Actualización con GRU

En cada iteración $t$:

$$
d_{t+1} = d_t + \Delta d_t
$$

Un codificador de movimiento junta las 36 correlaciones con la disparidad actual, las GRU actualizan su estado y una capa final propone $\Delta d_t$. El componente vertical de la corrección se fuerza a cero (`delta_flow[:, 1] = 0.0`), otra vez por la rectificación. Las tres GRU intercambian información: la de 1/16 ve mucho contexto (útil en paredes) y la de 1/4 cuida el detalle. El tiempo crece en línea recta con el número de iteraciones.

### 5. Subida convexa (de 1/4 a resolución completa)

Cada píxel final es una combinación de sus 9 vecinos (3×3) en la malla de 1/4. Los pesos los predice la red y pasan por un *softmax*, así que son positivos y suman 1:

$$
d_{full}(p) = 4 \sum_{k=1}^{9} w_k(p)\, d_{1/4}(n_k), \qquad \sum_k w_k = 1
$$

El factor 4 aparece porque un píxel a 1/4 equivale a 4 píxeles a resolución completa: una disparidad de 20 a 1/4 es de 80 al final. La ventaja frente a interpolar con promedios fijos: en un borde, la red puede darle peso solo a los vecinos del mismo lado, así que el borde no se "embarra".

## Limitaciones y riesgos en el proyecto

1. **Franja izquierda sin gemelo.** Un punto en la columna $u$ de la imagen izquierda aparece en $u - d$ de la derecha. Si $u < d$, el gemelo cae fuera de la imagen derecha y RAFT da un valor inventado. Con una pared a 1 m ($d \approx 55$ px), las primeras ~55 columnas no son confiables, y el código no las descarta. Propuesta simple: en `build_point_cloud`, quitar los píxeles con $u < d$.
2. **Sin chequeo izquierda-derecha.** No se calcula la disparidad en sentido contrario para comparar. Las zonas que un lente ve y el otro no (oclusiones) quedan con valores dudosos.
3. **Reflejos y superficies sin textura.** En un espejo, la disparidad que se mide es la del reflejo, no la de la superficie; el piso de madera brillante también refleja. La red rellena con contexto, y el resultado puede ser razonable o no.
4. **Iteraciones.** Menos de 20 es más rápido pero menos preciso. Cualquier cambio se puede medir con `[TIEMPO] RAFT` y comparando la nube.
5. **Requiere GPU NVIDIA** (`.cuda()`). Sin GPU la fase 2 no corre; `reprocesar.py` sí, porque usa las `.npy` guardadas.
