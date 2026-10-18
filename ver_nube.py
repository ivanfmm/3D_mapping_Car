"""
ver_nube.py - Abre una o varias nubes de puntos .ply con Open3D.

Uso:
  python ver_nube.py                          -> mapa_final.ply de la ultima sesion
  python ver_nube.py archivo.ply              -> ese archivo
  python ver_nube.py nube_sparse.ply trayectoria.ply   -> varias juntas
"""
import sys
from pathlib import Path

import open3d as o3d

if len(sys.argv) > 1:
    rutas = [Path(r) for r in sys.argv[1:]]
else:
    # buscar la sesion mas reciente en captured_frames
    sesiones = sorted((Path(__file__).parent / 'captured_frames').glob('session_*'))
    if not sesiones:
        sys.exit("No hay sesiones en captured_frames/")
    rutas = [sesiones[-1] / 'mapa_final.ply']

geometrias = []
for ruta in rutas:
    if not ruta.exists():
        sys.exit(f"No existe {ruta}")
    nube = o3d.io.read_point_cloud(str(ruta))
    print(f"{ruta.name}: {len(nube.points):,} puntos")
    geometrias.append(nube)

# ejes de referencia: rojo = X, verde = Y, azul = Z (10 cm, unidades en mm)
geometrias.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=100))
o3d.visualization.draw_geometries(geometrias, window_name=rutas[0].name)
