#!/usr/bin/env python3
"""Paso programado a mano (osciladores) sobre el modelo de choques, en MuJoCo.

Es el mismo generador de marcha que usa el simulador web (visor/sim.js), para
probar parámetros rápido desde Python:

  python3 sim/marcha.py                 # prueba los parámetros por defecto, 8 s
  python3 sim/marcha.py --barrido       # barre amplitudes y reporta cuáles caminan

Convenciones de signo (ver cad/pato.py): hip pitch negativo = muslo hacia adelante,
rodilla positiva = flexión, roll positivo en las dos caderas = el cuerpo se va a la
derecha (los ejes de la pata derecha están espejados).
"""
import argparse
import itertools
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

RAIZ = Path(__file__).resolve().parent.parent
XML = RAIZ / 'salida' / 'mjcf' / 'pato_sim.xml'

PARADO = dict(cadera_pitch=-0.35, rodilla=0.70, tobillo=-0.35)
DEFECTO = dict(frecuencia=2.0, balanceo=0.05, levante=0.15, zancada=0.24, giro=0.0, rampa=1.0)


def objetivos(t, p, vel=1.0):
    """Ángulo objetivo de cada articulación en el instante t, para la marcha."""
    f = p['frecuencia']
    fase = 2 * math.pi * f * t
    g = min(1.0, t / p['rampa']) * vel          # arranque suave
    sen = math.sin(fase)
    q = {}
    for lado, s in (('izq', 1), ('der', -1)):
        # Mismo giro en el mundo para las dos caderas (ejes espejados). El signo sale de medir los
        # apoyos en MuJoCo: como no hay roll de tobillo, el pie gira con la pata y apoya por un
        # canto; eso corre el punto de apoyo más que el centro de masa. Con sen > 0 el pie izquierdo
        # queda apoyado por el canto de afuera y descargado: es el que se levanta.
        roll = -p['balanceo'] * sen * g * s
        swing = max(0.0, s * sen)                 # 0..1 mientras esta pata está en el aire
        lev = p['levante'] * swing * g
        zanc = p['zancada'] * g * (s * math.cos(fase))   # atrás→adelante en el aire, al revés apoyada
        q[f'cadera_yaw_{lado}'] = p['giro'] * g * s
        q[f'cadera_roll_{lado}'] = roll
        q[f'cadera_pitch_{lado}'] = PARADO['cadera_pitch'] - lev / 2 + zanc
        q[f'rodilla_{lado}'] = PARADO['rodilla'] + lev
        q[f'tobillo_{lado}'] = PARADO['tobillo'] - lev / 2 - zanc
    q.update(cuello_yaw=0.0, cuello_pitch=0.0, boca=0.0)
    return q


def correr(p, segundos=8.0, vel=1.0, semilla=0):
    m = mujoco.MjModel.from_xml_path(str(XML))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    zmin = min(d.geom_xpos[g][2] - m.geom_size[g][2] for g in range(m.ngeom)
               if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX)
    d.qpos[2] -= zmin - 0.003
    ids = {m.actuator(i).name: i for i in range(m.nu)}
    for _ in range(500):                          # medio segundo parado
        mujoco.mj_step(m, d)
    x0 = d.xpos[1].copy()
    pasos = int(segundos / m.opt.timestep)
    tmax = np.zeros(m.nu)
    for k in range(pasos):
        t = k * m.opt.timestep
        for nombre, v in objetivos(t, p, vel).items():
            d.ctrl[ids[nombre]] = v
        mujoco.mj_step(m, d)
        tmax = np.maximum(tmax, np.abs(d.actuator_force))
        arriba = d.xmat[1].reshape(3, 3)[2, 2]    # componente z del eje z del tronco
        if arriba < 0.5:
            return dict(cayo=True, t=t, avance=float(d.xpos[1][0] - x0[0]), tmax=tmax)
    return dict(cayo=False, t=segundos, avance=float(d.xpos[1][0] - x0[0]),
                desvio=float(d.xpos[1][1] - x0[1]), tmax=tmax)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--barrido', action='store_true')
    ap.add_argument('--segundos', type=float, default=8.0)
    args = ap.parse_args()
    m = mujoco.MjModel.from_xml_path(str(XML))
    nombres = [m.actuator(i).name for i in range(m.nu)]
    stall = np.abs(m.actuator_forcerange[:, 1])
    if not args.barrido:
        r = correr(DEFECTO, args.segundos)
        print(f"{'SE CAYÓ a los %.1f s' % r['t'] if r['cayo'] else 'camina'}: avance {r['avance'] * 100:.1f} cm")
        uso = r['tmax'] / stall * 100
        for n, u in sorted(zip(nombres, uso), key=lambda x: -x[1])[:6]:
            print(f'  {n:18s} pico {u:5.1f} % del torque máximo')
        return
    print('frec  balan  levan  zanc  ->  resultado')
    for f, b, l, z in itertools.product((0.8, 1.0, 1.2, 1.6), (0.05, 0.07, 0.09), (0.15, 0.25, 0.35), (0.12, 0.18, 0.24)):
        p = dict(DEFECTO, frecuencia=f, balanceo=b, levante=l, zancada=z)
        r = correr(p, args.segundos)
        print(f'{f:4.1f}  {b:5.2f}  {l:5.2f}  {z:5.2f}  ->  '
              f"{'cae %.1fs' % r['t'] if r['cayo'] else 'ok    '}  avance {r['avance'] * 100:6.1f} cm  "
              f"pico {r['tmax'].max() / stall.max() * 100:4.0f} %")


if __name__ == '__main__':
    main()
