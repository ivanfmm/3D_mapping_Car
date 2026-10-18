---
tags: [nivel/1, tipo/concepto]
aliases: [Imagen digital, Píxel, Pixeles]
---
# Imagen digital

> [!info] Nivel 1 · Concepto base. Lo usan: [[ORB 1 Intuicion]], [[Camara estereo y disparidad]]

## Una imagen es una tabla de números

Una foto es una **cuadrícula de píxeles**. En escala de grises, cada píxel es un número entre **0 (negro) y 255 (blanco)**. En color, cada píxel tiene tres números. OpenCV los guarda en orden **B, G, R** (azul, verde, rojo), no RGB.

En Python con NumPy, una imagen izquierda del proyecto es:

```python
izq.shape   # (720, 1280, 3) -> alto, ancho, canales
izq.dtype   # uint8 -> enteros de 0 a 255
```

> [!warning] Ojo con el orden
> NumPy pide primero la **fila** y luego la **columna**: `img[v, u]`. Pero las coordenadas de un punto se escriben al revés, `(u, v)` = (columna, fila). Este cruce es una fuente clásica de bugs. En `_deproject` se lee `disp[vs, us]` justamente por eso.

## Coordenadas (u, v)

```
(0,0) ───────────► u  (columnas, hacia la derecha)
  │
  │      · (u=640, v=360)  ≈ centro de una imagen de 1280x720
  │
  ▼
  v  (filas, hacia ABAJO)
```

El origen está en la **esquina superior izquierda**, y **v crece hacia abajo** (al revés que en tus clases de matemáticas). Por eso, en la cámara, el eje **Y apunta hacia abajo**. Ver [[Pose y matrices 4x4]].

## Gradiente: dónde cambia la intensidad

Un **borde** es un lugar donde los números cambian bruscamente, por ejemplo de 30 a 200. Una **esquina** es un lugar donde cambian en **dos direcciones** a la vez. ORB busca justamente esas esquinas. Ver [[ORB 1 Intuicion]].

| Zona | Cómo cambian los valores | ¿Sirve para reconocerla? |
|---|---|---|
| Pared lisa | Casi no cambian | No: todos los puntos se ven iguales |
| Borde (orilla de una mesa) | Cambian en una dirección | Más o menos: puedes deslizarte a lo largo del borde y se ve igual |
| Esquina | Cambian en dos direcciones | Sí: es única |

## Escala de grises

La odometría trabaja en gris (`cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)`), porque ORB solo compara intensidades. El color se usa al final, para pintar la nube.
