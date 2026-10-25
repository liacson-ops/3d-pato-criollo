#!/usr/bin/env python3
"""Verifica que ninguna pieza se cruce con otra en las poses de uso: reposo, parado,
cuclillas, un ciclo completo de marcha y los extremos de cada articulación.

Mueve las mallas de cada eslabón con la cinemática de cad/pato.py (sin MuJoCo) y mide
el volumen de intersección entre piezas de eslabones distintos. Las piezas que van
pegadas o atornilladas entre sí se tocan sin superponerse (volumen 0).

  python3 cad/choques.py            # resumen por pose
  python3 cad/choques.py --rapido   # solo reposo, parado, cuclillas y 8 fases de marcha
"""
import argparse
import itertools
import math
import sys
from pathlib import Path

import numpy as np

RAIZ = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(RAIZ / 'cad'), str(RAIZ / 'sim')]
import pato  # noqa: E402

UMBRAL = 0.5   # mm³


def rotacion(ancla, eje, ang):
    """Matriz 4x4 de rotación de `ang` alrededor del eje `eje` que pasa por `ancla`."""
    e = np.asarray(eje, float) / np.linalg.norm(eje)
    k = np.array([[0, -e[2], e[1]], [e[2], 0, -e[0]], [-e[1], e[0], 0]])
    R = np.eye(3) + math.sin(ang) * k + (1 - math.cos(ang)) * k @ k
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = ancla - R @ ancla
    return T


def transformadas(r, q):
    """Matriz 4x4 de cada eslabón para la pose q (nombre de articulación -> ángulo)."""
    T = {'tronco': np.eye(4)}
    pendientes = list(r.articulaciones)
    while pendientes:
        for a in list(pendientes):
            if a.padre in T:
                T[a.hijo] = T[a.padre] @ rotacion(a.ancla, a.eje, q.get(a.nombre, 0.0))
                pendientes.remove(a)
    return T


def choques(r, q):
    T = transformadas(r, q)
    mallas = [(p, p.malla.transform(T[p.eslabon][:3, :])) for p in r.piezas]
    cajas = [m.bounding_box() for _, m in mallas]
    malos = []
    for (pa, ma), ca in zip(mallas, cajas):
        pass
    for i, j in itertools.combinations(range(len(mallas)), 2):
        pa, ma = mallas[i]
        pb, mb = mallas[j]
        if pa.eslabon == pb.eslabon:
            continue
        ca, cb = cajas[i], cajas[j]
        if any(ca[k + 3] < cb[k] - 0.01 or cb[k + 3] < ca[k] - 0.01 for k in range(3)):
            continue
        v = (ma ^ mb).volume()
        if v > UMBRAL:
            malos.append((round(v, 1), pa.nombre, pb.nombre))
    return sorted(malos, reverse=True)


def poses(r, rapido=False):
    from marcha import objetivos, PARADO
    base = {a.nombre: 0.0 for a in r.articulaciones}
    parado = {a.nombre: a.parado for a in r.articulaciones}
    cuclillas = dict(base)
    for lado in ('izq', 'der'):
        cuclillas[f'cadera_pitch_{lado}'] = -0.65
        cuclillas[f'rodilla_{lado}'] = 1.3
        cuclillas[f'tobillo_{lado}'] = -0.65
    out = [('reposo', base), ('parado', parado), ('cuclillas', cuclillas)]
    p = dict(frecuencia=1.0, balanceo=0.14, levante=0.40, zancada=0.26, giro=0.15, rampa=0.001)
    n = 8 if rapido else 24
    for k in range(n):
        out.append((f'marcha {k + 1}/{n}', dict(base, **objetivos(k / n, p))))
    if rapido:
        return out
    for a in r.articulaciones:
        for v in a.rango:
            q = dict(parado)
            q[a.nombre] = v
            out.append((f'{a.nombre} = {v:+.2f}', q))
    sentado = dict(base)
    for lado in ('izq', 'der'):
        sentado[f'cadera_pitch_{lado}'] = -1.15
        sentado[f'rodilla_{lado}'] = 1.15
        sentado[f'tobillo_{lado}'] = -0.6
    out.append(('cadera y rodilla al tope', sentado))
    # Cabeza: vértices de la envolvente boca/pitch (pato.BOCA_MAX) con el cuello girado o no.
    for yaw in (-1.5, 0.0, 1.5):
        for pitch, boca in [(-0.5, 0.3)] + pato.BOCA_MAX:
            q = dict(parado, cuello_yaw=yaw, cuello_pitch=pitch, boca=boca)
            out.append((f'cabeza: yaw {yaw:+.1f}, pitch {pitch:+.2f}, boca {boca:.2f}', q))
    rr = next(a.rango for a in r.articulaciones if a.nombre == 'cadera_roll_izq')
    ry = next(a.rango for a in r.articulaciones if a.nombre == 'cadera_yaw_izq')
    # Las dos patas al tope a la vez. Cerradas hacia adentro Y con las puntas giradas (talones o
    # puntas juntos) se tocan: es una auto-colisión que el controlador tiene que evitar (el modelo
    # de entrenamiento la simula); no se puede impedir con topes por eje. No se prueban esas dos.
    combos = [(rr[0], 0.0), (0.0, ry[0]), (rr[1], ry[0]), (rr[1], ry[1])]
    for roll, yaw in combos:
        q = dict(parado)
        for lado in ('izq', 'der'):
            q[f'cadera_roll_{lado}'] = roll
            q[f'cadera_yaw_{lado}'] = yaw
        out.append((f'roll {roll:+.2f} y yaw {yaw:+.2f} en las dos patas', q))
    # Un pie bien adentro y el otro bien afuera (balanceo máximo) con giro en el mismo sentido del mundo
    for signo in (1, -1):
        q = dict(parado)
        q['cadera_roll_izq'], q['cadera_roll_der'] = signo * rr[0], -signo * rr[0]
        q['cadera_yaw_izq'], q['cadera_yaw_der'] = signo * ry[0], -signo * ry[0]
        out.append((f'balanceo máximo {signo:+d}', q))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--rapido', action='store_true')
    args = ap.parse_args()
    r = pato.construir()
    total = 0
    for nombre, q in poses(r, args.rapido):
        malos = choques(r, q)
        total += len(malos)
        if malos:
            print(f'✗ {nombre}: {len(malos)} cruces')
            for v, a, b in malos[:6]:
                print(f'     {v:8.1f} mm³  {a}  ×  {b}')
        else:
            print(f'✓ {nombre}')
    print(f'\n{"SIN CRUCES" if not total else f"{total} cruces en total"}')
    sys.exit(1 if total else 0)


if __name__ == '__main__':
    main()
