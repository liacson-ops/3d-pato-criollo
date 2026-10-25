"""Entorno Gymnasium del Pato Criollo para aprender a caminar (MuJoCo, CPU).

Observación (39 valores, parecida a la del XGO-Duck para poder reusar su runtime):
  giro del tronco (3), gravedad en el tronco (3), posición de las 10 articulaciones
  de las patas respecto de la pose de parado (10), velocidad (10), acción anterior (10),
  comando [vx, vy, giro] (3).
Acción (10): desplazamiento objetivo de cada articulación de las patas, en radianes,
sobre la pose de parado. El cuello y la boca se mantienen en 0.

Incluye lo que hace falta para que lo aprendido sirva en el robot real:
  - choques consigo mismo (pato_rl.xml: envolventes convexas de cada pieza) y con el piso;
  - variaciones aleatorias de masa, fricción, ganancia y torque de los servos;
  - retardo de los servos (cola de acciones) y empujones aleatorios;
  - control a 50 Hz con física a 500 Hz, como en el robot.
"""
from __future__ import annotations

import collections
from pathlib import Path

import gymnasium as gym
import mujoco
import numpy as np

RAIZ = Path(__file__).resolve().parent.parent
XML = RAIZ / 'salida' / 'mjcf' / 'pato_rl.xml'

PATAS = ['cadera_yaw', 'cadera_roll', 'cadera_pitch', 'rodilla', 'tobillo']
ARTICULACIONES = [f'{a}_{lado}' for lado in ('izq', 'der') for a in PATAS]
PARADO = {'cadera_pitch': -0.35, 'rodilla': 0.70, 'tobillo': -0.35}
CONTROL_HZ = 50
OBS_DIM = 39
ACT_DIM = 10


class PatoEnv(gym.Env):
    metadata = {'render_modes': ['rgb_array'], 'render_fps': CONTROL_HZ}

    def __init__(self, xml=XML, max_segundos=15.0, aleatorizar=True, empujones=True,
                 comando_fijo=None, render_mode=None, semilla=None):
        super().__init__()
        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.data = mujoco.MjData(self.model)
        self.render_mode = render_mode
        self.aleatorizar, self.empujones, self.comando_fijo = aleatorizar, empujones, comando_fijo
        self.sub = int(round(1.0 / (CONTROL_HZ * self.model.opt.timestep)))
        self.max_pasos = int(max_segundos * CONTROL_HZ)
        self.rng = np.random.default_rng(semilla)
        self.act_id = np.array([self.model.actuator(n).id for n in ARTICULACIONES])
        self.jnt_qpos = np.array([self.model.jnt_qposadr[self.model.joint(n).id] for n in ARTICULACIONES])
        self.jnt_qvel = np.array([self.model.jnt_dofadr[self.model.joint(n).id] for n in ARTICULACIONES])
        self.rango = np.array([self.model.jnt_range[self.model.joint(n).id] for n in ARTICULACIONES])
        self.parado = np.array([PARADO.get(n.rsplit('_', 1)[0], 0.0) for n in ARTICULACIONES])
        self.tronco = self.model.body('tronco').id
        self.pies = [self.model.body(f'pie_{l}').id for l in ('izq', 'der')]
        self.suelo = self.model.geom('suelo').id
        self.nominal = dict(mass=self.model.body_mass.copy(), gain=self.model.actuator_gainprm.copy(),
                            bias=self.model.actuator_biasprm.copy(), fr=self.model.actuator_forcerange.copy(),
                            fric=self.model.geom_friction.copy())
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (OBS_DIM,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (ACT_DIM,), np.float32)
        self.escala_accion = 0.5          # rad por unidad de acción
        self.renderer = None
        self._reset_estado()

    # ------------------------------------------------------------------ utilidades
    def _reset_estado(self):
        self.paso = 0
        self.accion_prev = np.zeros(ACT_DIM)
        self.cola = collections.deque(maxlen=4)
        self.retardo = 0
        self.comando = np.zeros(3)
        self.tiempo_aire = np.zeros(2)

    def _aleatorizar(self):
        m, n, r = self.model, self.nominal, self.rng
        if not self.aleatorizar:
            m.body_mass[:] = n['mass']; m.actuator_gainprm[:] = n['gain']; m.actuator_biasprm[:] = n['bias']
            m.actuator_forcerange[:] = n['fr']; m.geom_friction[:] = n['fric']
            self.retardo = 1
            return
        m.body_mass[:] = n['mass'] * r.uniform(0.85, 1.15, m.nbody)
        k = r.uniform(0.7, 1.3, m.nu)
        m.actuator_gainprm[:, 0] = n['gain'][:, 0] * k
        m.actuator_biasprm[:, 1] = n['bias'][:, 1] * k
        m.actuator_forcerange[:] = n['fr'] * r.uniform(0.7, 1.0, (m.nu, 1))
        m.geom_friction[:, 0] = n['fric'][:, 0] * r.uniform(0.6, 1.4)
        self.retardo = int(r.integers(0, 3))          # 0–2 ciclos de control (0–40 ms)

    def _obs(self):
        d = self.data
        R = d.xmat[self.tronco].reshape(3, 3)
        giro = R.T @ d.cvel[self.tronco][:3]          # velocidad angular en el marco del tronco
        grav = R.T @ np.array([0, 0, -1.0])
        q = d.qpos[self.jnt_qpos] - self.parado
        qd = d.qvel[self.jnt_qvel]
        obs = np.concatenate([giro, grav, q, qd * 0.1, self.accion_prev, self.comando])
        return obs.astype(np.float32)

    def _contactos_propios(self):
        n = 0
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            if c.geom1 != self.suelo and c.geom2 != self.suelo:
                n += 1
        return n

    def _pie_apoyado(self):
        apoyo = [False, False]
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            for g in (c.geom1, c.geom2):
                b = self.model.geom_bodyid[g]
                if b in self.pies:
                    apoyo[self.pies.index(b)] = True
        return apoyo

    # ------------------------------------------------------------------ gym
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self._reset_estado()
        self._aleatorizar()
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        self.data.qpos[self.jnt_qpos] = self.parado + self.rng.uniform(-0.05, 0.05, ACT_DIM)
        self.data.ctrl[self.act_id] = self.data.qpos[self.jnt_qpos]
        mujoco.mj_forward(self.model, self.data)
        zmin = min(self.data.geom_xpos[g][2] - self.model.geom_rbound[g] for g in range(self.model.ngeom)
                   if self.model.geom_contype[g] and g != self.suelo)
        self.data.qpos[2] -= zmin - 0.002
        mujoco.mj_forward(self.model, self.data)
        if self.comando_fijo is not None:
            self.comando = np.array(self.comando_fijo, float)
        else:
            self.comando = np.array([self.rng.uniform(-0.05, 0.25), self.rng.uniform(-0.1, 0.1),
                                     self.rng.uniform(-0.6, 0.6)])
            if self.rng.random() < 0.15:
                self.comando[:] = 0.0                    # también tiene que saber quedarse quieto
        return self._obs(), {}

    def step(self, accion):
        accion = np.clip(np.asarray(accion, float), -1, 1)
        self.cola.append(accion)
        aplicada = self.cola[max(0, len(self.cola) - 1 - self.retardo)]
        objetivo = np.clip(self.parado + self.escala_accion * aplicada, self.rango[:, 0], self.rango[:, 1])
        self.data.ctrl[self.act_id] = objetivo
        if self.empujones and self.rng.random() < 0.01:
            self.data.xfrc_applied[self.tronco, :2] = self.rng.uniform(-6, 6, 2)
        for _ in range(self.sub):
            mujoco.mj_step(self.model, self.data)
        self.data.xfrc_applied[self.tronco] = 0
        self.paso += 1

        d = self.data
        R = d.xmat[self.tronco].reshape(3, 3)
        v_local = R.T @ d.cvel[self.tronco][3:]
        w_local = R.T @ d.cvel[self.tronco][:3]
        arriba = R[2, 2]
        z = d.xpos[self.tronco][2]
        cayo = arriba < 0.6 or z < 0.12
        propios = self._contactos_propios()
        apoyo = self._pie_apoyado()
        self.tiempo_aire = np.where(apoyo, 0.0, self.tiempo_aire + 1.0 / CONTROL_HZ)
        torque = d.actuator_force[self.act_id] / np.abs(self.model.actuator_forcerange[self.act_id, 1])

        # Recompensa: seguir el comando, mantenerse derecho y gastar poco.
        r = {
            'vel_x': 2.0 * np.exp(-((v_local[0] - self.comando[0]) ** 2) / 0.02),
            'vel_y': 0.5 * np.exp(-((v_local[1] - self.comando[1]) ** 2) / 0.02),
            'giro': 1.0 * np.exp(-((w_local[2] - self.comando[2]) ** 2) / 0.2),
            'vivo': 0.5,
            'derecho': 0.5 * arriba,
            'altura': -5.0 * max(0.0, 0.175 - z),
            'accion': -0.05 * float(np.sum((accion - self.accion_prev) ** 2)),
            'torque': -0.02 * float(np.sum(torque ** 2)),
            'balanceo': -0.05 * float(w_local[0] ** 2 + w_local[1] ** 2),
            'choque_propio': -0.5 * propios,
            'pies_aire': 0.3 * float(np.sum(np.clip(self.tiempo_aire, 0, 0.3)) if np.linalg.norm(self.comando) > 0.05 else 0.0),
        }
        recompensa = float(sum(r.values()))
        if cayo:
            recompensa -= 10.0
        self.accion_prev = accion
        obs = self._obs()
        truncado = self.paso >= self.max_pasos
        info = {'recompensas': r, 'cayo': cayo, 'v_x': float(v_local[0]), 'choques_propios': propios}
        return obs, recompensa, bool(cayo), bool(truncado), info

    def render(self):
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, 480, 640)
        self.renderer.update_scene(self.data, camera=-1)
        return self.renderer.render()


def hacer_env(**kw):
    def _f():
        return PatoEnv(**kw)
    return _f
