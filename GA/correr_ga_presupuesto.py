"""
correr_ga_presupuesto.py - Corre el AG de optimize_ga.py dentro de un limite de tiempo.

1. Mide el equipo (nucleos, RAM libre) y cuanto tarda una evaluacion real.
2. Con eso calcula workers, frames, poblacion y generaciones para no pasar del limite.
3. Corre el AG con un corte por reloj (no empieza una generacion que no alcanza a terminar).
4. Escribe resultados_<fecha>/: reporte_GA.md, best_vo_params.json, ga_history.csv,
   individuos.csv (todas las evaluaciones).

No sobrescribe best_vo_params.json ni ga_history.csv del repo.

Uso:
  python correr_ga_presupuesto.py                 -> 40 minutos
  python correr_ga_presupuesto.py --minutos 20
  python correr_ga_presupuesto.py --base-dir OTRA_CARPETA   (con 07/, disparidad/, 07.txt)
"""
import argparse
import csv
import ctypes
import json
import math
import multiprocessing as mp
import os
import platform
import random
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from deap import tools

import optimize_ga as og
from VO import DEFAULT_VO_PARAMS
from evaluator import KITTIDatasetCache, evaluate_vo_params

AQUI = Path(__file__).resolve().parent

# Fraccion del limite que se planea usar (el resto es colchon)
MARGEN = 0.88
# Un proceso solo es mas rapido que 12 a la vez (nucleos E, temperatura, cache):
# factor medido->esperado en paralelo. Se corrige con el tiempo real de la gen 0.
FACTOR_PARALELO = 1.6
FRAMES_BENCH = 120
MIN_GENERACIONES = 10


# Rangos ampliados (--amplio): la primera corrida dejo varios parametros pegados
# al limite del rango original de optimize_ga.GENE_SPECS.
GENE_SPECS_AMPLIO = [
    ("orb_nfeatures",     300.0,  3000.0, True),
    ("orb_scaleFactor",   1.10,   2.00,   False),
    ("orb_nlevels",       2.0,    10.0,   True),
    ("orb_patchSize",     15.0,   48.0,   True),
    ("orb_fastThreshold", 5.0,    40.0,   True),
    ("lowe_ratio",        0.40,   0.90,   False),
    ("disp_min",          1.0,    30.0,   False),
    ("pnp_iterations",    100.0,  800.0,  True),
    ("pnp_reproj_err",    0.50,   8.00,   False),
    ("min_pnp_inliers",   8.0,    80.0,   True),
    ("floor_band",        0.20,   0.45,   False),
    ("floor_ransac_mm",   15.0,   80.0,   False),
]
MAX_GENERACIONES = 60


def _evaluar_params(params):
    """Worker: recibe el diccionario ya decodificado (el rango lo decide el proceso padre)."""
    try:
        return evaluate_vo_params(params, og._WORKER_CACHE)
    except Exception as e:
        return 1e6, {"ate_rmse": None, "rpe_trans_rmse": None, "rpe_rot_rmse": None,
                     "coverage": 0.0, "valid": False, "error": repr(e)}


def ram_libre_gb():
    try:
        if os.name == 'nt':
            class MS(ctypes.Structure):
                _fields_ = [('dwLength', ctypes.c_ulong), ('dwMemoryLoad', ctypes.c_ulong)] + \
                           [(n, ctypes.c_ulonglong) for n in
                            ('ullTotalPhys', 'ullAvailPhys', 'ullTotalPageFile', 'ullAvailPageFile',
                             'ullTotalVirtual', 'ullAvailVirtual', 'ullAvailExtendedVirtual')]
            m = MS()
            m.dwLength = ctypes.sizeof(MS)
            ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(m))
            return m.ullAvailPhys / 2**30, m.ullTotalPhys / 2**30
        pag = os.sysconf('SC_PAGE_SIZE')
        return os.sysconf('SC_AVPHYS_PAGES') * pag / 2**30, os.sysconf('SC_PHYS_PAGES') * pag / 2**30
    except Exception:
        return 8.0, 16.0


def contar_frames(seq_dir, disp_dir, gt_file):
    img_dir = seq_dir / 'image_2'
    if not img_dir.exists():
        img_dir = seq_dir / 'image_0'
    n_gt = sum(1 for l in open(gt_file) if l.strip())
    n = 0
    while n < n_gt and (img_dir / f'{n:06d}.png').exists() and (
            (disp_dir / f'{n:06d}_disp.npy').exists() or (disp_dir / f'{n:06d}.npy').exists()):
        n += 1
    return n, img_dir


def encode_params(params):
    genes = []
    for name, low, high, _ in og.GENE_SPECS:
        g = (float(params[name]) - low) / (high - low)
        genes.append(max(0.0, min(1.0, g)))
    return genes


def fmt(x, nd=4):
    return '-' if x is None else f'{x:.{nd}f}'


def planear(t_frame_par, t_carga_frame, n_disp, workers, presupuesto_s, min_gens=MIN_GENERACIONES):
    """Devuelve (frames, pop, gens, detalle) que caben en presupuesto_s."""
    opciones_frames = sorted({n_disp, min(n_disp, 800), min(n_disp, 550), min(n_disp, 300)}, reverse=True)
    planes = []
    for frames in opciones_frames:
        t_eval  = frames * t_frame_par
        t_carga = frames * t_carga_frame * 1.5 + 10        # los workers cargan a la vez
        for pop in (2 * workers, workers):   # multiplos de workers: lotes completos
            lotes0   = math.ceil(pop / workers)             # gen 0 evalua a todos
            lotes_g  = math.ceil(0.85 * (pop - 2) / workers)  # ~85% de los hijos cambian
            t_gen0   = lotes0 * t_eval
            t_gen    = lotes_g * t_eval
            gens     = int((presupuesto_s - t_carga - t_gen0) // t_gen)
            planes.append(dict(frames=frames, pop=pop, gens=gens, t_eval=t_eval, t_carga=t_carga,
                               t_gen0=t_gen0, t_gen=t_gen,
                               total=t_carga + t_gen0 + max(gens, 0) * t_gen))
    # Preferencia: todos los frames; luego poblacion grande; siempre >= MIN_GENERACIONES
    for p in planes:
        if p['gens'] >= min_gens:
            p['gens'] = min(p['gens'], 40)
            return p, planes
    mejor = max(planes, key=lambda p: p['gens'])
    mejor['gens'] = max(mejor['gens'], 1)
    return mejor, planes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--minutos', type=float, default=40.0)
    ap.add_argument('--base-dir', default=str(AQUI))
    ap.add_argument('--workers', type=int, default=0, help='0 = calcular')
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--min-gens', type=int, default=MIN_GENERACIONES,
                    help='generaciones minimas antes de recortar frames')
    ap.add_argument('--amplio', action='store_true', help='usar GENE_SPECS_AMPLIO')
    a = ap.parse_args()
    if a.amplio:
        og.GENE_SPECS[:] = GENE_SPECS_AMPLIO

    t_inicio = time.perf_counter()
    limite_s = a.minutos * 60.0
    presupuesto_s = limite_s * MARGEN

    base = Path(a.base_dir)
    seq_dir, disp_dir, gt_file = base / '07', base / 'disparidad', base / '07.txt'
    for p in (seq_dir / 'calib.txt', gt_file, disp_dir):
        if not p.exists():
            raise SystemExit(f'Falta {p}')
    n_disp, img_dir = contar_frames(seq_dir, disp_dir, gt_file)
    if n_disp < 50:
        raise SystemExit(f'Solo hay {n_disp} frames consecutivos con imagen y disparidad '
                         f'en {img_dir} y {disp_dir}. Descarga los datos del README.')

    # ---------------- 1. Equipo ----------------
    logicos = os.cpu_count() or 4
    libre_gb, total_gb = ram_libre_gb()
    print(f'Equipo: {platform.processor()} | {logicos} hilos | RAM {libre_gb:.1f}/{total_gb:.1f} GB libres')
    print(f'Datos: {n_disp} frames | limite {a.minutos:g} min (se planean {presupuesto_s/60:.1f})')

    # ---------------- 2. Medir una evaluacion ----------------
    cv2.setNumThreads(1)
    n_bench = min(FRAMES_BENCH, n_disp)
    t0 = time.perf_counter()
    cache = KITTIDatasetCache(seq_dir, disp_dir, gt_file, 0, n_bench)
    t_carga_frame = (time.perf_counter() - t0) / n_bench
    mb_frame = (cache.frames_gray[0].nbytes + cache.disparities[0].nbytes) / 2**20

    pesado = DEFAULT_VO_PARAMS.copy()
    pesado.update(orb_nfeatures=3000, orb_nlevels=10, pnp_iterations=500)
    tiempos = {}
    for nombre, prm in (('base', DEFAULT_VO_PARAMS), ('pesado', pesado)):
        t0 = time.perf_counter()
        evaluate_vo_params(prm, cache)
        tiempos[nombre] = (time.perf_counter() - t0) / n_bench
    del cache
    # Un lote dura lo que su individuo mas lento: se planea cerca del caso pesado
    t_frame_1 = 0.35 * tiempos['base'] + 0.65 * tiempos['pesado']
    t_frame_par = t_frame_1 * FACTOR_PARALELO
    print(f'Medido: {1000*tiempos["base"]:.1f} ms/frame (base), {1000*tiempos["pesado"]:.1f} ms/frame (pesado), '
          f'carga {1000*t_carga_frame:.1f} ms/frame, {mb_frame:.2f} MB/frame en RAM')

    # ---------------- 3. Presupuesto ----------------
    gb_worker = n_disp * mb_frame / 1024 + 0.5          # cache + interprete y librerias
    w_cpu = max(2, int(logicos * 0.6))                   # deja libres los hilos de hyperthreading
    w_ram = max(1, int(libre_gb * 0.7 // gb_worker))
    workers = a.workers or min(w_cpu, w_ram)
    plan, planes = planear(t_frame_par, t_carga_frame, n_disp, workers, presupuesto_s, a.min_gens)
    print(f'Workers: {workers} (CPU permite {w_cpu}, RAM permite {w_ram} a {gb_worker:.2f} GB c/u)')
    print(f'Plan: {plan["frames"]} frames | poblacion {plan["pop"]} | hasta {plan["gens"]} generaciones | '
          f'~{plan["t_eval"]:.0f} s por evaluacion | estimado {plan["total"]/60:.1f} min')

    # ---------------- 4. AG ----------------
    random.seed(a.seed)
    np.random.seed(a.seed)
    out = AQUI / f'resultados_{datetime.now():%Y%m%d_%H%M%S}'
    out.mkdir()

    toolbox = og.build_toolbox()
    pop = toolbox.population(n=plan['pop'])
    sembrados = [('linea base (DEFAULT_VO_PARAMS)', og.encode_default_params())]
    previo = AQUI / 'best_vo_params.json'
    if previo.exists():
        sembrados.append(('mejor previo (best_vo_params.json del repo)',
                          encode_params(json.load(open(previo))['best_params'])))
    anteriores = sorted(AQUI.glob('resultados_*/best_vo_params.json'))
    if anteriores:
        sembrados.append((f'mejor de la corrida anterior ({anteriores[-1].parent.name})',
                          encode_params(json.load(open(anteriores[-1]))['best_params'])))
    for k, (_, genes) in enumerate(sembrados):
        pop[k][:] = genes

    hof = tools.HallOfFame(maxsize=5)
    historia, filas_ind, ref = [], [], {}
    cx_prob, mut_prob, elite = 0.75, 0.35, 2
    motivo_fin = f'se llego al maximo de {MAX_GENERACIONES} generaciones'
    t_gens = []

    with mp.Pool(workers, initializer=og._init_worker,
                 initargs=(str(seq_dir), str(disp_dir), str(gt_file), 0, plan['frames'])) as pool:
        t_ag = time.perf_counter()
        for gen in range(MAX_GENERACIONES + 1):
            t_gen = time.perf_counter()
            pend = [ind for ind in pop if not ind.fitness.valid]
            if pend:
                for ind, (fit, info) in zip(pend, pool.map(_evaluar_params, [og.decode_individual(i) for i in pend], chunksize=1)):
                    ind.fitness.values = (fit,)
                    ind.metrics = info
                    filas_ind.append([gen, fit, info.get('ate_rmse'), info.get('rpe_trans_rmse'),
                                      info.get('rpe_rot_rmse'), info.get('coverage'),
                                      json.dumps(og.decode_individual(ind))])
            if gen == 0:
                for k, (nombre, _) in enumerate(sembrados):
                    ref[nombre] = (pop[k].fitness.values[0], dict(pop[k].metrics))
            hof.update(pop)
            mejor = hof[0]
            validos = [i.fitness.values[0] for i in pop if i.fitness.values[0] < 1e5]
            dt = time.perf_counter() - t_gen
            t_gens.append(dt)
            historia.append(dict(gen=gen, evaluados=len(pend), best=mejor.fitness.values[0],
                                 media=float(np.mean(validos)) if validos else None,
                                 validos=len(validos), t=dt, **{k: mejor.metrics.get(k) for k in
                                 ('ate_rmse', 'rpe_trans_rmse', 'rpe_rot_rmse', 'coverage')}))
            transcurrido = time.perf_counter() - t_inicio
            print(f'[Gen {gen:02d}] evaluados {len(pend):02d} en {dt:6.1f}s | mejor {mejor.fitness.values[0]:9.4f} | '
                  f'ATE {fmt(mejor.metrics.get("ate_rmse"))} m | validos {len(validos)}/{len(pop)} | '
                  f'reloj {transcurrido/60:5.1f} min', flush=True)
            json.dump({'generation': gen, 'best_fitness': mejor.fitness.values[0], 'metrics': mejor.metrics,
                       'best_params': og.decode_individual(mejor)},
                      open(out / 'best_vo_params.json', 'w', encoding='utf-8'), indent=4)

            if gen == MAX_GENERACIONES:
                break
            # Corte por reloj: la siguiente generacion debe caber (con la mas lenta vista hasta ahora)
            t_sig = max(t_gens[1:]) if len(t_gens) > 1 else t_gens[0]
            if transcurrido + 1.1 * t_sig > limite_s * 0.95:
                motivo_fin = (f'corte por reloj tras la generacion {gen}: la siguiente '
                              f'(~{t_sig:.0f} s) no cabia en el limite')
                break

            elites = [toolbox.clone(i) for i in tools.selBest(pop, elite)]
            hijos = [toolbox.clone(i) for i in toolbox.select(pop, len(pop) - elite)]
            for h1, h2 in zip(hijos[::2], hijos[1::2]):
                if random.random() < cx_prob:
                    toolbox.mate(h1, h2)
                    del h1.fitness.values, h2.fitness.values
            for h in hijos:
                if random.random() < mut_prob:
                    toolbox.mutate(h)
                    del h.fitness.values
            pop[:] = elites + hijos
        t_ag = time.perf_counter() - t_ag

    t_total = time.perf_counter() - t_inicio
    mejor = hof[0]
    best_params = og.decode_individual(mejor)

    # ---------------- 5. Archivos ----------------
    with open(out / 'ga_history.csv', 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['gen', 'evaluados', 'best_fitness', 'best_ate_m', 'best_rpe_trans_m',
                    'best_rpe_rot_deg', 'best_coverage', 'mean_fitness', 'validos', 'gen_time_s'])
        for h in historia:
            w.writerow([h['gen'], h['evaluados'], h['best'], h['ate_rmse'], h['rpe_trans_rmse'],
                        h['rpe_rot_rmse'], h['coverage'], h['media'], h['validos'], round(h['t'], 2)])
    with open(out / 'individuos.csv', 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['gen', 'fitness', 'ate_m', 'rpe_trans_m', 'rpe_rot_deg', 'coverage', 'params_json'])
        w.writerows(filas_ind)

    n_eval = len(filas_ind)
    n_fallidos = sum(1 for r in filas_ind if r[1] >= 1e5)
    L = []
    L += [f'# Reporte del algoritmo genetico ({datetime.now():%Y-%m-%d %H:%M})', '']
    L += ['## Resultado', '']
    L += ['| Configuracion | Fitness | ATE (m) | RPE trasl. (m) | RPE rot. (grados) | Cobertura |',
          '|---|---|---|---|---|---|']
    for nombre, (fit, m) in ref.items():
        L.append(f'| {nombre} | {fmt(fit)} | {fmt(m.get("ate_rmse"))} | {fmt(m.get("rpe_trans_rmse"))} | '
                 f'{fmt(m.get("rpe_rot_rmse"))} | {fmt(m.get("coverage"), 3)} |')
    m = mejor.metrics
    L.append(f'| **mejor de esta corrida** | **{fmt(mejor.fitness.values[0])}** | {fmt(m.get("ate_rmse"))} | '
             f'{fmt(m.get("rpe_trans_rmse"))} | {fmt(m.get("rpe_rot_rmse"))} | {fmt(m.get("coverage"), 3)} |')
    L += ['', 'Todas las filas se evaluaron en esta corrida, con los mismos frames y el mismo codigo, '
              'asi que son comparables entre si. Fitness = (ATE + RPE_trasl + 0.1 * RPE_rot) / cobertura^2; menor es mejor.', '']
    L += ['## Tiempo', '',
          f'- Limite pedido: {a.minutos:g} min. Tiempo real total: **{t_total/60:.1f} min** '
          f'(AG: {t_ag/60:.1f} min; el resto es medicion y carga).',
          f'- Fin: {motivo_fin}.',
          f'- Evaluaciones: {n_eval} ({n_fallidos} invalidas por cobertura < 50 % o error).', '']
    L += ['## Calculo del presupuesto', '',
          f'- Equipo: {platform.processor()}, {logicos} hilos logicos, RAM libre {libre_gb:.1f} de {total_gb:.1f} GB.',
          f'- Medicion en un solo proceso sobre {n_bench} frames: {1000*tiempos["base"]:.1f} ms/frame con los '
          f'parametros base y {1000*tiempos["pesado"]:.1f} ms/frame con el caso pesado '
          f'(3000 esquinas, 10 niveles, 500 iteraciones de PnP).',
          f'- Tiempo planeado por frame en paralelo: {1000*t_frame_par:.1f} ms '
          f'(0.35 base + 0.65 pesado, por {FACTOR_PARALELO} de perdida al usar todos los nucleos).',
          f'- Memoria por worker: {gb_worker:.2f} GB ({mb_frame:.2f} MB por frame). '
          f'Workers: {workers} (CPU permite {w_cpu}, RAM permite {w_ram}).',
          f'- Plan elegido: {plan["frames"]} frames, poblacion {plan["pop"]}, {plan["gens"]} generaciones estimadas '
          f'(se siguen corriendo generaciones mientras quepan en el limite), '
          f'~{plan["t_eval"]:.0f} s por evaluacion, total estimado {plan["total"]/60:.1f} min '
          f'de {presupuesto_s/60:.1f} planeables ({MARGEN:.0%} del limite).',
          f'- Real: generacion 0 en {t_gens[0]:.0f} s; las demas en promedio '
          f'{(np.mean(t_gens[1:]) if len(t_gens) > 1 else 0):.0f} s (estimado {plan["t_gen"]:.0f} s).', '',
          '| Frames | Poblacion | Generaciones que caben | Total estimado (min) |', '|---|---|---|---|']
    for p in planes:
        L.append(f'| {p["frames"]} | {p["pop"]} | {p["gens"]} | {p["total"]/60:.1f} |')
    L += ['', '## Configuracion del AG', '',
          f'- Cruce SBX (eta 15, prob. {cx_prob}), mutacion polinomial (eta 20, indpb 0.25, prob. {mut_prob}), '
          f'torneo de 3, elitismo de {elite}, semilla {a.seed}.',
          f'- Rangos de busqueda: {"ampliados (--amplio)" if a.amplio else "originales de optimize_ga.py"}.',
          f'- Poblacion inicial sembrada con: {"; ".join(n for n, _ in sembrados)}.', '']
    L += ['## Convergencia', '',
          '| Gen | Evaluados | Mejor fitness | ATE (m) | Fitness medio | Validos | Tiempo (s) |', '|---|---|---|---|---|---|---|']
    for h in historia:
        L.append(f'| {h["gen"]} | {h["evaluados"]} | {fmt(h["best"])} | {fmt(h["ate_rmse"])} | '
                 f'{fmt(h["media"])} | {h["validos"]}/{plan["pop"]} | {h["t"]:.0f} |')
    L += ['', '## Hiperparametros', '',
          '| Parametro | Base | Mejor | Rango de busqueda | Nota |', '|---|---|---|---|---|']
    for name, low, high, is_int in og.GENE_SPECS:
        v = best_params[name]
        g = (float(v) - low) / (high - low)
        nota = 'en el limite inferior' if g < 0.03 else ('en el limite superior' if g > 0.97 else '')
        L.append(f'| `{name}` | {DEFAULT_VO_PARAMS[name]} | **{v}** | {low:g} - {high:g} | {nota} |')
    L += ['', 'Acoplados: `orb_edgeThreshold` = `orb_patchSize` y `min_pnp_matches` = `min_pnp_inliers`. '
              'Un valor pegado al limite sugiere que el optimo puede estar fuera del rango.', '',
          '## Los 5 mejores', '', '| # | Fitness | ATE (m) | Parametros |', '|---|---|---|---|']
    for k, ind in enumerate(hof):
        p = og.decode_individual(ind)
        L.append(f'| {k+1} | {fmt(ind.fitness.values[0])} | {fmt(ind.metrics.get("ate_rmse"))} | '
                 + ', '.join(f'{n}={p[n]}' for n, *_ in og.GENE_SPECS) + ' |')
    L += ['', '## Archivos', '', '- `best_vo_params.json`: mejor individuo.',
          '- `ga_history.csv`: una fila por generacion.',
          '- `individuos.csv`: todas las evaluaciones, con sus parametros.', '']
    (out / 'reporte_GA.md').write_text('\n'.join(L), encoding='utf-8')
    print(f'\nListo en {t_total/60:.1f} min. Reporte: {out / "reporte_GA.md"}')


if __name__ == '__main__':
    mp.freeze_support()
    main()
