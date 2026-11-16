# 3D_mapping_Car

Carrito con cámara estéreo que recorre un cuarto y genera su nube de puntos 3D.
La explicación completa está en [Documentacion/00 Inicio.md](Documentacion/00%20Inicio.md).

## Archivos

| Archivo | Dónde corre | Qué hace |
|---|---|---|
| `server_2.py`, `motor.py`, `pca9685.py` | Raspberry Pi | Manda el video estéreo y mueve los motores |
| `client_2.py` | PC con GPU | Maneja el carrito, graba los frames y al cerrar corre RAFT-Stereo + odometría + nube |
| `visual_odometry.py` | PC | Odometría visual (ORB + PnP), movimiento plano, loop closure opcional |
| `rectificacion.py` | PC | Carga la calibración y calcula los mapas de rectificación y `Q` |
| `nube.py` | PC | Nube densa: filtros, piso y alineación |
| `reprocesar.py` | PC (sin GPU) | Rehace poses y nube de una sesión ya capturada, sin carrito y sin RAFT |
| `ver_nube.py` | PC | Abre uno o varios `.ply` |
| `camera_calibration_dualCamera.py` | PC | Captura del tablero, calibración y prueba de rectificación |

## Instalación (PC)

```bash
pip install -r requirements.txt
git clone https://github.com/princeton-vl/RAFT-Stereo
```

Descarga `raftstereo-middlebury.pth` en `RAFT-Stereo/models/`. Si RAFT-Stereo está en
otra carpeta, define la variable de entorno `RAFT_DIR`. `reprocesar.py` y `ver_nube.py`
no necesitan torch ni RAFT-Stereo.

## Uso

En la Raspberry Pi (conectada al hotspot de la PC):

```bash
python3 server_2.py
```

En la PC:

```bash
PI_IP=10.42.0.146 python client_2.py
```

`W/A/S/D` mantener presionado para mover, `Q` frenar, `X` salir y procesar. Las teclas
funcionan con el foco en la terminal o en las ventanas de vista previa.

Para rehacer la nube de una sesión sin el carrito:

```bash
python reprocesar.py --sparse
python reprocesar.py captured_frames/session_XXXX
```

## Salidas de una sesión (`captured_frames/session_*/`)

| Archivo | Sistema de coordenadas |
|---|---|
| `poses.txt`, `trayectoria.ply`, `nube_sparse.ply`, `mapa_final_raw.ply`, `mapa_paredes_raw.ply` | Primera cámara |
| `mapa_final.ply`, `mapa_paredes.ply`, `trayectoria_alineada.ply` | Alineado al piso (Y = normal del piso); la rotación está en `alineacion.json` |

Solo se pueden abrir juntos archivos del mismo sistema. Cada sesión guarda su propia
copia de `estereo_cal.json`, así se puede reprocesar aunque después se recalibre.
