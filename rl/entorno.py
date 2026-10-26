"""Entorno Gymnasium del Pato Criollo (MuJoCo, CPU). Dos modos:

  caminar      seguir un comando de velocidad [vx, vy, giro] sin caerse, con paso de pato (política principal);
  levantarse   arrancar caído (de espaldas, de frente o de costado) y volver a quedar de pie
               (con la mecánica actual no lo logra; queda para cuando el diseño lo permita).

Observación (41 valores, parecida a la del XGO-Duck para poder reusar su runtime):
  giro del tronco (3), gravedad en el tronco (3), posición de las 10 articulaciones
  de las patas respecto de la pose de parado (10), velocidad (10), acción anterior (10),
  comando [vx, vy, giro] (3), reloj de la marcha [sen, cos] (2).
Acción (10): desplazamiento objetivo de cada articulación de las patas, en radianes, sobre la
pose de parado: objetivo = parado + escala × acción (escala 0,5 rad caminando, 0,8 levantándose).

Es un juguete para una familia, así que la marcha tiene que ser de pato y no de robot apurado:
  - un reloj fijo de 1,6 pasos por segundo marca qué pie apoya y cuál va en el aire (el runtime de
    la Pi lo genera igual); seguirlo se premia, y con el comando en cero el reloj se detiene con
    los dos pies apoyados;
  - los movimientos bruscos y las velocidades de servo altas se penalizan más que antes;
  - la velocidad pedida es baja la mayor parte del tiempo (5 a 12 cm/s) y rápida solo a veces.

Incluye lo que hace falta para que lo aprendido sirva en el robot real y en una casa:
  - choques consigo mismo (pato_rl.xml: envolventes convexas de cada pieza) y con el piso;
  - piso de casa: en parte de los episodios, umbrales y alfombras (escalones de 5 a 15 mm) o
    rampas suaves, con un campo de alturas que se rellena al azar;
  - variaciones aleatorias de masa, fricción, ganancia y torque de los servos;
  - retardo de los servos (cola de acciones) y empujones aleatorios;
  - control a 50 Hz con física a 500 Hz, como en el robot.
La cabeza (cuello yaw, cuello pitch, boca) no la maneja la política: es un gesto programado
(rl/cabeza.py). Al entrenar la marcha se mueve al azar en parte de los episodios.
"""
from __future__ import annotations

import collections
import json
import math
from pathlib import Path

import gymnasium as gym
import mujoco
import numpy as np

RAIZ = Path(__file__).resolve().parent.parent
XML = RAIZ / 'salida' / 'mjcf' / 'pato_rl.xml'

PATAS = ['cadera_yaw', 'cadera_roll', 'cadera_pitch', 'rodilla', 'tobillo']
ARTICULACIONES = [f'{a}_{lado}' for lado in ('izq', 'der') for a in PATAS]
CABEZA = ['cuello_yaw', 'cuello_pitch', 'boca']
PARADO = {'cadera_pitch': -0.35, 'rodilla': 0.70, 'tobillo': -0.35}
CONTROL_HZ = 50
OBS_DIM = 41
ACT_DIM = 10
MODOS = ('caminar', 'levantarse')
ESCALA_ACCION = {'caminar': 0.5, 'levantarse': 0.8}
Z_TRONCO_PARADO = 0.185     # altura del origen del tronco con las rodillas a 40°
PASOS_POR_SEGUNDO = 1.6     # ritmo del paso de pato (reloj de la marcha)
VX_LENTO = 0.10             # velocidad "normal" del juguete, m/s


def _boca_max_tabla():
    try:
        datos = json.loads((RAIZ / 'salida' / 'robot.json').read_text())
        return [tuple(x) for x in datos['envolventes']['boca_max_por_cuello_pitch']]
    except (OSError, KeyError, ValueError):
        return [(0.0, 0.3), (0.45, 0.15), (0.6, 0.0)]


BOCA_MAX = _boca_max_tabla()


def boca_max(cuello_pitch):
    """Cuánto puede abrir la boca según cuánto mira abajo (misma regla que cad/pato.py)."""
    for pitch, boca in BOCA_MAX:
        if cuello_pitch <= pitch + 1e-9:
            return boca
    return 0.0


class PatoEnv(gym.Env):
    metadata = {'render_modes': ['rgb_array'], 'render_fps': CONTROL_HZ}

    def __init__(self, xml=XML, modo='caminar', max_segundos=None, aleatorizar=True, empujones=True,
                 comando_fijo=None, cabeza_aleatoria=True, terreno=None, render_mode=None, semilla=None):
        """terreno: None = al azar si aleatorizar, si no plano; 'plano', 'umbrales' o 'rampas' lo fija."""
        super().__init__()
        assert modo in MODOS, modo
        self.modo = modo
        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.data = mujoco.MjData(self.model)
        self.render_mode = render_mode
        self.aleatorizar, self.empujones, self.comando_fijo = aleatorizar, empujones, comando_fijo
        self.cabeza_aleatoria = cabeza_aleatoria and modo == 'caminar'
        self.terreno_fijo = terreno
        self.sub = int(round(1.0 / (CONTROL_HZ * self.model.opt.timestep)))
        if max_segundos is None:
            max_segundos = 15.0 if modo == 'caminar' else 6.0
        self.max_pasos = int(max_segundos * CONTROL_HZ)
        self.rng = np.random.default_rng(semilla)
        self.act_id = np.array([self.model.actuator(n).id for n in ARTICULACIONES])
        self.jnt_qpos = np.array([self.model.jnt_qposadr[self.model.joint(n).id] for n in ARTICULACIONES])
        self.jnt_qvel = np.array([self.model.jnt_dofadr[self.model.joint(n).id] for n in ARTICULACIONES])
        self.rango = np.array([self.model.jnt_range[self.model.joint(n).id] for n in ARTICULACIONES])
        self.cab_act = np.array([self.model.actuator(n).id for n in CABEZA])
        self.cab_qpos = np.array([self.model.jnt_qposadr[self.model.joint(n).id] for n in CABEZA])
        self.cab_rango = np.array([self.model.jnt_range[self.model.joint(n).id] for n in CABEZA])
        self.parado = np.array([PARADO.get(n.rsplit('_', 1)[0], 0.0) for n in ARTICULACIONES])
        self.tronco = self.model.body('tronco').id
        self.pies = [self.model.body(f'pie_{l}').id for l in ('izq', 'der')]
        self.suelo = self.model.geom('suelo').id
        self.piso_ids = {self.suelo}
        self.hfield = None
        if self.model.nhfield:
            self.hfield = self.model.hfield('terreno').id
            self.piso_ids.add(self.model.geom('terreno').id)
        self.nominal = dict(mass=self.model.body_mass.copy(), gain=self.model.actuator_gainprm.copy(),
                            bias=self.model.actuator_biasprm.copy(), fr=self.model.actuator_forcerange.copy(),
                            fric=self.model.geom_friction.copy())
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (OBS_DIM,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (ACT_DIM,), np.float32)
        self.escala_accion = ESCALA_ACCION[modo]
        self.renderer = None
        self.caida_inicial = None
        self.terreno = 'plano'
        self._reset_estado()

    # ------------------------------------------------------------------ utilidades
    def _reset_estado(self):
        self.paso = 0
        self.accion_prev = np.zeros(ACT_DIM)
        self.cola = collections.deque(maxlen=4)
        self.retardo = 0
        self.comando = np.zeros(3)
        self.tiempo_aire = np.zeros(2)
        self.pasos_sin_pies = 0
        self.pasos_de_pie = 0
        self.fase = 0.0                  # reloj de la marcha (rad)
        self.z_pie_parado = 0.026
        self.cabeza_plan = None          # (yaw, pitch, boca, frecuencia) o None: cabeza quieta
        self.cabeza_objetivo = np.zeros(3)

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

    # ------------------------------------------------------------------ terreno
    def fijar_terreno(self, tipo):
        """Rellena el campo de alturas: 'plano', 'umbrales' (baldosas y alfombras con escalones de
        5 a 15 mm) o 'rampas' (ondulaciones suaves de hasta 1 cm). El centro queda plano para arrancar."""
        self.terreno = tipo
        if self.hfield is None:
            return
        m, r = self.model, self.rng
        n = m.hfield_nrow[self.hfield]
        lado = m.hfield_size[self.hfield][0]            # semi-ancho en metros
        zmax = m.hfield_size[self.hfield][2]
        ys, xs = np.meshgrid(np.linspace(-lado, lado, n), np.linspace(-lado, lado, n), indexing='ij')
        if tipo == 'umbrales':
            baldosa = r.uniform(0.3, 0.6)
            fila, col = np.floor(ys / baldosa + 1e3), np.floor(xs / baldosa + 1e3)
            semilla = (fila * 7919 + col * 104729) % 1000 / 1000.0
            alturas = np.select([semilla < 0.45, semilla < 0.75, semilla < 0.93], [0.0, 0.005, 0.010], 0.015)
            z = alturas / zmax
        elif tipo == 'rampas':
            z = np.zeros_like(xs)
            for _ in range(3):
                kx, ky, f = r.uniform(-1, 1, 2) * 2.5, r.uniform(0, 2 * math.pi), r.uniform(0.3, 1.0)
                z += f * (0.5 + 0.5 * np.sin(kx[0] * xs + kx[1] * ys + ky))
            z = z / z.max() * 0.010 / zmax
        else:
            z = np.zeros_like(xs)
        z[xs ** 2 + ys ** 2 < 0.3 ** 2] = 0.0           # arranca en plano
        adr = m.hfield_adr[self.hfield]
        m.hfield_data[adr:adr + n * n] = np.clip(z, 0, 1).ravel()

    def altura_piso(self, x, y):
        """Altura del terreno en (x, y) del mundo, en metros."""
        if self.hfield is None:
            return 0.0
        m = self.model
        n = m.hfield_nrow[self.hfield]; lado = m.hfield_size[self.hfield][0]
        c = int(np.clip((x + lado) / (2 * lado) * (n - 1), 0, n - 1))
        f = int(np.clip((y + lado) / (2 * lado) * (n - 1), 0, n - 1))
        return float(m.hfield_data[m.hfield_adr[self.hfield] + f * n + c] * m.hfield_size[self.hfield][2])

    # ------------------------------------------------------------------ cabeza
    def fijar_cabeza(self, yaw, pitch, boca):
        """Objetivo de los servos de la cabeza, dentro de los topes y de la envolvente boca/pitch."""
        yaw = float(np.clip(yaw, *self.cab_rango[0]))
        pitch = float(np.clip(pitch, *self.cab_rango[1]))
        boca = float(np.clip(boca, 0.0, min(self.cab_rango[2][1], boca_max(pitch))))
        self.cabeza_objetivo[:] = (yaw, pitch, boca)
        self.data.ctrl[self.cab_act] = self.cabeza_objetivo

    def cabeza_actual(self):
        return self.data.qpos[self.cab_qpos].copy()

    def _plan_cabeza(self):
        """Al entrenar la marcha: la mitad de los episodios con la cabeza quieta, un cuarto en una pose
        fija al azar y un cuarto moviéndose despacio (mirar alrededor, abrir la boca)."""
        self.cabeza_plan = None
        if not self.cabeza_aleatoria:
            self.fijar_cabeza(0, 0, 0)
            return
        u = self.rng.random()
        r = self.rng
        if u < 0.5:
            self.fijar_cabeza(0, 0, 0)
        elif u < 0.75:
            self.fijar_cabeza(r.uniform(-1.5, 1.5), r.uniform(-0.5, 0.6), r.uniform(0, 0.3))
        else:
            self.cabeza_plan = (r.uniform(0.3, 1.5), r.uniform(0.2, 0.55), r.uniform(0, 0.3), r.uniform(0.2, 0.7))

    def _mover_cabeza(self, t):
        if self.cabeza_plan is None:
            return
        ay, ap, ab, f = self.cabeza_plan
        fase = 2 * math.pi * f * t
        self.fijar_cabeza(ay * math.sin(fase), ap * math.sin(0.7 * fase), ab * (0.5 + 0.5 * math.sin(1.3 * fase)))

    # ------------------------------------------------------------------ contactos
    def _contactos_propios(self):
        n = 0
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            if c.geom1 not in self.piso_ids and c.geom2 not in self.piso_ids:
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

    def pies_apoyados(self):
        return self._pie_apoyado()

    def _apoyar_en_el_piso(self, margen=0.002):
        mujoco.mj_forward(self.model, self.data)
        zmin = min(self.data.geom_xpos[g][2] - self.model.geom_rbound[g] for g in range(self.model.ngeom)
                   if self.model.geom_contype[g] and g not in self.piso_ids)
        self.data.qpos[2] -= zmin - margen
        mujoco.mj_forward(self.model, self.data)

    def _reset_parado(self):
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        self.data.qpos[self.jnt_qpos] = self.parado + self.rng.uniform(-0.05, 0.05, ACT_DIM)
        self.data.ctrl[self.act_id] = self.data.qpos[self.jnt_qpos]
        self._apoyar_en_el_piso()
        self.z_pie_parado = float(np.mean([self.data.xpos[p][2] for p in self.pies]))

    def _reset_caido(self):
        """Lo tira al piso: de espaldas, de frente o de costado, con las patas en cualquier ángulo,
        y deja que se asiente medio segundo con los servos sosteniendo esa pose."""
        m, d, r = self.model, self.data, self.rng
        mujoco.mj_resetDataKeyframe(m, d, 0)
        q = r.uniform(self.rango[:, 0], self.rango[:, 1])
        d.qpos[self.jnt_qpos] = q
        d.ctrl[self.act_id] = q
        self.fijar_cabeza(0, r.uniform(-0.3, 0.3), 0)
        lado = r.choice(['espaldas', 'frente', 'costado_izq', 'costado_der'])
        eje, ang = {'espaldas': ((1, 0, 0), math.pi), 'frente': ((0, 1, 0), math.pi / 2),
                    'costado_izq': ((1, 0, 0), math.pi / 2), 'costado_der': ((1, 0, 0), -math.pi / 2)}[lado]
        ang += r.uniform(-0.4, 0.4)
        quat = np.zeros(4)
        mujoco.mju_axisAngle2Quat(quat, np.array(eje, float), ang)
        giro = np.zeros(4)
        mujoco.mju_axisAngle2Quat(giro, np.array([0, 0, 1.0]), r.uniform(-math.pi, math.pi))
        qf = np.zeros(4)
        mujoco.mju_mulQuat(qf, giro, quat)
        d.qpos[3:7] = qf
        self._apoyar_en_el_piso(margen=0.03)
        d.qvel[:] = 0
        for _ in range(int(0.5 * CONTROL_HZ) * self.sub):
            mujoco.mj_step(m, d)
        self.caida_inicial = lado

    def _elegir_comando(self):
        r = self.rng
        u = r.random()
        if u < 0.15:
            return np.zeros(3)                                        # quieto
        if u < 0.70:
            vx = r.uniform(0.05, 0.12)                                # lo normal: paso tranquilo
        elif u < 0.90:
            vx = r.uniform(0.12, 0.25)                                # a veces rápido
        else:
            vx = r.uniform(-0.05, 0.0)                                # marcha atrás corta
        giro = r.uniform(-0.6, 0.6) if r.random() < 0.6 else 0.0
        vy = r.uniform(-0.08, 0.08) if r.random() < 0.3 else 0.0
        return np.array([vx, vy, giro])

    # ------------------------------------------------------------------ gym
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self._reset_estado()
        self._aleatorizar()
        tipo = self.terreno_fijo
        if tipo is None:
            tipo = self.rng.choice(['plano', 'umbrales', 'rampas'], p=[0.4, 0.3, 0.3]) if self.aleatorizar else 'plano'
        self.fijar_terreno(tipo)
        caido = (options or {}).get('caido', self.modo == 'levantarse')
        if caido:
            self._reset_caido()
        else:
            self._reset_parado()
            self._plan_cabeza()
        if self.modo == 'levantarse':
            self.comando[:] = 0.0
        elif self.comando_fijo is not None:
            self.comando = np.array(self.comando_fijo, float)
        else:
            self.comando = self._elegir_comando()
        return self._obs(), {}

    def _obs(self):
        d = self.data
        R = d.xmat[self.tronco].reshape(3, 3)
        giro = R.T @ d.cvel[self.tronco][:3]          # velocidad angular en el marco del tronco
        grav = R.T @ np.array([0, 0, -1.0])
        q = d.qpos[self.jnt_qpos] - self.parado
        qd = d.qvel[self.jnt_qvel]
        reloj = [math.sin(self.fase), math.cos(self.fase)]
        obs = np.concatenate([giro, grav, q, qd * 0.1, self.accion_prev, self.comando, reloj])
        return obs.astype(np.float32)

    # ------------------------------------------------------------------ navegación y cámara
    def posicion(self):
        """(x, y) del tronco en el mundo, en metros. En el robot real sale de la odometría o de un marcador."""
        return self.data.xpos[self.tronco][:2].copy()

    def rumbo(self):
        """Ángulo del frente del tronco en el mundo (rad)."""
        R = self.data.xmat[self.tronco].reshape(3, 3)
        return float(math.atan2(R[1, 0], R[0, 0]))

    def pelota(self, pos=None):
        """Posición del objeto a seguir (cuerpo mocap 'pelota'). Con pos la mueve; devuelve la actual."""
        i = self.model.body('pelota').mocapid[0]
        if pos is not None:
            self.data.mocap_pos[i] = pos
        return self.data.mocap_pos[i].copy()

    def camara_pixel(self, punto, ancho=640, alto=480):
        """Proyecta un punto del mundo en la imagen de la cámara de la cabeza: (u, v, distancia) o None si
        queda fuera del encuadre. Es lo que en el robot real da la detección por color en OpenCV."""
        cam = self.model.camera('camara').id
        R = self.data.cam_xmat[cam].reshape(3, 3)
        p = R.T @ (np.asarray(punto, float) - self.data.cam_xpos[cam])
        prof = -p[2]
        if prof <= 0.02:
            return None
        f = (alto / 2) / math.tan(math.radians(self.model.cam_fovy[cam]) / 2)
        u, v = ancho / 2 + f * p[0] / prof, alto / 2 - f * p[1] / prof
        if not (0 <= u < ancho and 0 <= v < alto):
            return None
        return float(u), float(v), float(prof)

    def estado_cuerpo(self):
        """(componente vertical del eje z del tronco, altura del tronco sobre el piso local)."""
        R = self.data.xmat[self.tronco].reshape(3, 3)
        x, y, z = self.data.xpos[self.tronco]
        return float(R[2, 2]), float(z - self.altura_piso(x, y))

    def alzado(self):
        """True si lleva medio segundo sin ningún pie apoyado (lo levantaron o lo tienen en brazos)."""
        return self.pasos_sin_pies >= CONTROL_HZ // 2

    def step(self, accion):
        accion = np.clip(np.asarray(accion, float), -1, 1)
        self.cola.append(accion)
        aplicada = self.cola[max(0, len(self.cola) - 1 - self.retardo)]
        objetivo = np.clip(self.parado + self.escala_accion * aplicada, self.rango[:, 0], self.rango[:, 1])
        self.data.ctrl[self.act_id] = objetivo
        self._mover_cabeza(self.paso / CONTROL_HZ)
        if self.empujones and self.modo == 'caminar' and self.rng.random() < 0.01:
            self.data.xfrc_applied[self.tronco, :2] = self.rng.uniform(-6, 6, 2)
        for _ in range(self.sub):
            mujoco.mj_step(self.model, self.data)
        self.data.xfrc_applied[self.tronco] = 0
        self.paso += 1
        # Reloj de la marcha: avanza solo con comando; quieto, se detiene con los dos pies apoyados.
        if np.linalg.norm(self.comando) > 0.03:
            self.fase = (self.fase + 2 * math.pi * PASOS_POR_SEGUNDO / CONTROL_HZ) % (2 * math.pi)
        else:
            self.fase = 0.0

        d = self.data
        R = d.xmat[self.tronco].reshape(3, 3)
        v_local = R.T @ d.cvel[self.tronco][3:]
        w_local = R.T @ d.cvel[self.tronco][:3]
        arriba, z = self.estado_cuerpo()
        cayo = arriba < 0.6 or z < 0.12
        propios = self._contactos_propios()
        apoyo = self._pie_apoyado()
        self.tiempo_aire = np.where(apoyo, 0.0, self.tiempo_aire + 1.0 / CONTROL_HZ)
        self.pasos_sin_pies = 0 if any(apoyo) else self.pasos_sin_pies + 1
        torque = d.actuator_force[self.act_id] / np.abs(self.model.actuator_forcerange[self.act_id, 1])
        qd = d.qvel[self.jnt_qvel]
        q_rel = d.qpos[self.jnt_qpos] - self.parado
        de_pie = arriba > 0.9 and z > 0.16
        self.pasos_de_pie = self.pasos_de_pie + 1 if de_pie else 0

        if self.modo == 'caminar':
            r, terminado = self._recompensa_caminar(v_local, w_local, arriba, z, accion, torque, qd, propios, apoyo)
            if cayo:
                r['caida'] = -10.0
                terminado = True
        else:
            r, terminado = self._recompensa_levantarse(arriba, z, accion, torque, propios, q_rel, apoyo)
        recompensa = float(sum(r.values()))
        self.accion_prev = accion
        obs = self._obs()
        truncado = self.paso >= self.max_pasos
        info = {'recompensas': r, 'cayo': bool(cayo), 'de_pie': bool(de_pie), 'v_x': float(v_local[0]),
                'choques_propios': propios, 'alzado': self.alzado()}
        return obs, recompensa, bool(terminado), bool(truncado), info

    def _recompensa_caminar(self, v_local, w_local, arriba, z, accion, torque, qd, propios, apoyo):
        # Lecciones: (1) con un bono por estar vivo alto y un premio al tiempo en el aire que valía con
        # los dos pies, aprendió a saltar en el lugar; (2) con el premio al giro blando, giraba sobre su
        # eje; (3) premiando solo la velocidad, salía apurado y se caía. Ahora: avanzar paga lineal, el
        # giro se sigue de cerca, y el paso se premia por seguir el reloj (un pie apoyado y el otro en el
        # aire, alternando a 1,6 por segundo), con penalizaciones fuertes a lo brusco.
        con_comando = np.linalg.norm(self.comando) > 0.03
        vz = float(self.data.cvel[self.tronco][5])
        cx = self.comando[0]
        if abs(cx) > 0.03:
            avance = 2.0 * float(np.clip(v_local[0] / abs(cx), -1, 1)) * np.sign(cx)
        else:
            avance = -1.0 * abs(float(v_local[0]))
        # Reloj: sen > 0 -> pie izquierdo apoyado y derecho en el aire; sen < 0 al revés;
        # cerca del cruce (|sen| < 0,3) los dos apoyados (doble apoyo).
        s = math.sin(self.fase)
        if not con_comando:
            deseado = (True, True)
        else:
            deseado = (s > -0.3, s < 0.3)
        # Acertar el pie en el aire vale más que acertar el apoyado: si no, quedarse parado cobraba.
        reloj = 0.0
        for apoyado, quiere_apoyo in zip(apoyo, deseado):
            if apoyado != quiere_apoyo:
                reloj -= 0.3
            else:
                reloj += 0.1 if quiere_apoyo else 0.4
        # Pie en el aire: que levante de verdad (hasta 8 mm), para no arrastrarlo por la alfombra.
        levante = 0.0
        if con_comando:
            for i, (apoyado, quiere_apoyo) in enumerate(zip(apoyo, deseado)):
                if not quiere_apoyo:
                    zp = self.data.xpos[self.pies[i]][2] - self.z_pie_parado
                    levante += 0.2 * float(np.clip(zp / 0.008, 0, 1))
        r = {
            'vel_x': 1.5 * np.exp(-((v_local[0] - cx) ** 2) / (0.004 + (0.4 * cx) ** 2)),   # ancho según la velocidad pedida
            'avance': avance,
            'vel_y': 0.3 * np.exp(-((v_local[1] - self.comando[1]) ** 2) / 0.05),
            'giro': 1.0 * np.exp(-((w_local[2] - self.comando[2]) ** 2) / 0.1),
            'reloj': reloj,
            'levante': levante,
            'vivo': 0.2,
            'derecho': 0.5 * arriba,
            'altura': -5.0 * max(0.0, 0.175 - z),
            'rebote': -2.0 * vz ** 2,
            'vuelo': -0.5 if not any(apoyo) else 0.0,
            'accion': -0.15 * float(np.sum((accion - self.accion_prev) ** 2)),
            'vel_art': -0.0005 * float(np.sum(qd ** 2)),
            'torque': -0.02 * float(np.sum(torque ** 2)),
            'balanceo': -0.02 * float(w_local[0] ** 2 + w_local[1] ** 2),
            'choque_propio': -0.5 * propios,
        }
        return r, False

    def _recompensa_levantarse(self, arriba, z, accion, torque, propios, q_rel, apoyo):
        # Enderezarse y subir pagan siempre; la postura de parado solo cuando ya está casi derecho.
        # Medio segundo de pie termina el episodio con un bono grande: cuanto antes, mejor.
        logrado = self.pasos_de_pie >= CONTROL_HZ // 2
        r = {
            'derecho': 1.0 * arriba,
            'altura': 2.0 * float(np.clip(z / Z_TRONCO_PARADO, 0, 1)),
            'postura': 1.0 * float(np.exp(-np.sum(q_rel ** 2) / 2.0)) if arriba > 0.8 else 0.0,
            'pies': 0.3 * sum(apoyo) if arriba > 0.8 else 0.0,
            'tiempo': -0.1,
            'accion': -0.02 * float(np.sum((accion - self.accion_prev) ** 2)),
            'torque': -0.02 * float(np.sum(torque ** 2)),
            'choque_propio': -0.2 * propios,
            'logrado': 20.0 if logrado else 0.0,
        }
        return r, logrado

    def render(self):
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, 480, 640)
        self.renderer.update_scene(self.data, camera=-1)
        return self.renderer.render()


def hacer_env(**kw):
    def _f():
        return PatoEnv(**kw)
    return _f
