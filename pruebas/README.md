# Pruebas del carrito

Una subcarpeta por prueba, nombrada con fecha y número: `AAAA-MM-DD_NN/`
(por ejemplo `2026-10-17_01/`).

## Qué va en cada prueba

- `notas.md`: qué se probó, condiciones (luz, recorrido, si cerró vuelta) y qué se observó.
- `consola.txt`: salida completa de `client_2.py` (y de `server_2.py` si hubo errores).
- `muestras/`: unos pocos frames representativos, no la captura completa.
- `.ply` resultantes, solo si pesan poco.

## Plantilla de `notas.md`

```
Fecha:
Commit / rama:
Resolución recibida por lente:
Iluminación:
Recorrido (duración, ¿cerró vuelta?):
Frames totales / fallidos:
Loop closures detectados:
Observaciones:
```

> GitHub rechaza archivos de más de 100 MB. `captured_frames/` completa y las
> nubes grandes no se suben aquí.
