#!/usr/bin/env python3
"""Torques estáticos del Pato Criollo en MuJoCo, contra el torque máximo de cada servo.

Poses:
  parado        dos pies, rodillas a 40°
  cuclillas     dos pies, rodillas a 90°
  una_pata      parado sobre el pie derecho, el izquierdo levantado

Escribe salida/torques.md y salida/torques.json.

  python3 sim/torques.py
"""
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

RAIZ = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(RAIZ / 'cad'))
from componentes import MG90S, MG996R  # noqa: E402

XML = RAIZ / 'salida' / 'mjcf' / 'pato.xml'
STALL = {'mg996r': MG996R.stall_kgcm_6v, 'mg90s': MG90S.stall_kgcm_6v}


def pose(m, rodilla, pie_izq_arriba=False):
    q = {}
    for lado in ('izq', 'der'):
        q[f'cadera_pitch_{lado}'] = -rodilla / 2
        q[f'rodilla_{lado}'] = rodilla
        q[f'tobillo_{lado}'] = -rodilla / 2
    if pie_izq_arriba:
        q['cadera_pitch_izq'] -= 0.35
        q['rodilla_izq'] += 0.7
    return q


def preparar(m, d, q):
    mujoco.mj_resetData(m, d)
    for nombre, v in q.items():
        d.qpos[m.jnt_qposadr[m.joint(nombre).id]] = v
    d.qpos[3:7] = [1, 0, 0, 0]
    mujoco.mj_forward(m, d)
    # Bajar el robot hasta que la suela más baja toque el piso.
    zmin = min(d.geom_xpos[g][2] - m.geom_rbound[g] for g in range(m.ngeom)
               if m.geom_contype[g] and m.geom_bodyid[g] != 0)
    piso = np.inf
    for g in range(m.ngeom):
        if m.geom_contype[g] and m.geom_bodyid[g] != 0 and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            mid = m.geom_dataid[g]
            v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
            mundo = v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
            piso = min(piso, mundo[:, 2].min())
    d.qpos[2] -= piso if np.isfinite(piso) else zmin
    for i in range(m.nu):
        d.ctrl[i] = d.qpos[m.jnt_qposadr[m.actuator_trnid[i, 0]]]
    mujoco.mj_forward(m, d)


def medir(nombre_pose, q, apoyo_der=False, segundos=2.0):
    spec = mujoco.MjSpec.from_file(str(XML))
    m = spec.compile()
    d = mujoco.MjData(m)
    preparar(m, d, q)
    qpos = d.qpos.copy()
    if apoyo_der:
        # Fija el pie derecho al mundo exactamente donde quedó en la pose.
        pie = m.body('pie_der').id
        rot = d.xmat[pie].reshape(3, 3)
        eq = spec.add_equality()
        eq.type = mujoco.mjtEq.mjEQ_WELD
        eq.objtype = mujoco.mjtObj.mjOBJ_BODY
        eq.name1 = 'pie_der'
        quat_inv = np.array(d.xquat[pie]) * [1, -1, -1, -1]
        eq.data[:11] = [0, 0, 0, *(-rot.T @ d.xpos[pie]), *quat_inv, 1.0]
        m = spec.compile()
        d = mujoco.MjData(m)
    # Servos muy rígidos y sin límite: medimos el torque que hace falta, no lo que el servo logra.
    for i in range(m.nu):
        m.actuator_gainprm[i, 0] *= 30
        m.actuator_biasprm[i, 1] *= 30
        m.actuator_forcerange[i] = [-50, 50]
    d.qpos[:] = qpos
    for i in range(m.nu):
        d.ctrl[i] = d.qpos[m.jnt_qposadr[m.actuator_trnid[i, 0]]]
    mujoco.mj_forward(m, d)
    for _ in range(int(segundos / m.opt.timestep)):
        mujoco.mj_step(m, d)
    filas = []
    for i in range(m.nu):
        nombre = m.actuator(i).name
        clase = 'mg996r' if abs(m.actuator_forcerange[i, 1]) > 0.5 else 'mg90s'   # por el torque máximo del XML
        t = float(d.actuator_force[i]) / 9.81 * 100  # N·m -> kg·cm
        filas.append(dict(articulacion=nombre, servo=clase.upper(), kgcm=round(abs(t), 2),
                          uso=round(abs(t) / STALL[clase] * 100, 1)))
    altura = float(d.subtree_com[1][2])
    return dict(pose=nombre_pose, altura_cm_masa_m=round(altura, 3), filas=filas)


def main():
    m = mujoco.MjModel.from_xml_path(str(XML))
    masa = float(sum(m.body_mass))
    resultados = [
        medir('parado (rodillas 40°)', pose(m, 0.70)),
        medir('cuclillas (rodillas 90°)', pose(m, 1.57)),
        medir('en una pata (apoya la derecha)', pose(m, 0.70, pie_izq_arriba=True), apoyo_der=True),
    ]
    salida = dict(masa_kg=round(masa, 3), stall_kgcm=STALL, poses=resultados)
    (RAIZ / 'salida' / 'torques.json').write_text(json.dumps(salida, ensure_ascii=False, indent=1))
    md = ['# Torques estáticos (MuJoCo)', '',
          f'Masa del modelo: **{masa:.2f} kg**. Torque máximo a 6 V: MG996R {STALL["mg996r"]} kg·cm, '
          f'MG90S {STALL["mg90s"]} kg·cm. Caminando, los picos suelen ser 2 a 3 veces el valor estático: '
          'conviene que el estático no pase del 30–40 % del máximo.', '']
    claves = [f['articulacion'] for f in resultados[0]['filas']]
    md.append('| Articulación | Servo | ' + ' | '.join(r['pose'] for r in resultados) + ' |')
    md.append('| --- | --- |' + ' ---: |' * len(resultados))
    for i, k in enumerate(claves):
        celdas = [f"{r['filas'][i]['kgcm']:.2f} kg·cm ({r['filas'][i]['uso']:.0f} %)" for r in resultados]
        md.append(f"| {k} | {resultados[0]['filas'][i]['servo']} | " + ' | '.join(celdas) + ' |')
    (RAIZ / 'salida' / 'torques.md').write_text('\n'.join(md) + '\n')
    print('\n'.join(md))


if __name__ == '__main__':
    main()
