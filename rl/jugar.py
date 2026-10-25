#!/usr/bin/env python3
"""Corre las políticas entrenadas en el simulador, con el supervisor que usa el robot:
camina con la política de marcha, hace gestos con la cabeza y, si se cae, pasa a la política de
levantarse hasta quedar de pie y vuelve a caminar.

  mjpython rl/jugar.py                       # ventana de MuJoCo (en Mac mjpython; en Linux python3)
  python3 rl/jugar.py --sin-ventana --segundos 20
  python3 rl/jugar.py --demo                 # cuac mirando arriba, camina, lo tiran, se levanta, sigue
  python3 rl/jugar.py --caido                # arranca en el piso: prueba solo levantarse
  python3 rl/jugar.py --gesto mirar_arriba_y_abrir --comando 0 0 0
  python3 rl/jugar.py --ir-a 1 0 1 1 0 1          # va de punto en punto (x y x y ...) y se queda
  python3 rl/jugar.py --seguir                     # sigue la pelota con la cabeza y el cuerpo
  python3 rl/jugar.py --onnx rl/corridas/ultimo_caminar/pato_caminar.onnx

Por defecto usa rl/corridas/ultimo_caminar y rl/corridas/ultimo_levantarse (si existe).
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from entorno import PatoEnv, CONTROL_HZ, ESCALA_ACCION  # noqa: E402
from cabeza import Cabeza, GESTOS  # noqa: E402
from navegar import Navegador, Seguidor  # noqa: E402

RAIZ = Path(__file__).resolve().parent.parent


def cargar_politica(corrida=None, onnx=None, modo_esperado=None):
    """Devuelve (politica(obs) -> accion, escala_accion) o None si no hay nada que cargar."""
    if onnx:
        import onnxruntime as ort
        s = ort.InferenceSession(str(onnx), providers=['CPUExecutionProvider'])
        config = Path(onnx).parent / 'config.json'
        escala = json.loads(config.read_text()).get('escala_accion', 0.5) if config.exists() else 0.5
        return (lambda obs: s.run(None, {'obs': obs[None].astype(np.float32)})[0][0]), escala
    corrida = Path(corrida)
    if not (corrida / 'modelo.zip').exists():
        return None
    config = corrida / 'config.json'
    if modo_esperado and config.exists() and json.loads(config.read_text()).get('modo', modo_esperado) != modo_esperado:
        return None                                   # p. ej. "ultimo" apunta a una corrida de otro modo
    print(f'política de {modo_esperado or "?"}: {corrida.resolve().name}')
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
    modelo = PPO.load(str(corrida / 'modelo.zip'), device='cpu')
    venv = VecNormalize.load(str(corrida / 'normalizacion.pkl'), DummyVecEnv([lambda: PatoEnv(aleatorizar=False)]))
    venv.training = False
    config = corrida / 'config.json'
    escala = json.loads(config.read_text()).get('escala_accion', 0.5) if config.exists() else 0.5
    return (lambda obs: modelo.predict(venv.normalize_obs(obs), deterministic=True)[0]), escala


class Supervisor:
    """Elige qué política manda en cada ciclo: caminar, o levantarse si se cayó.
    Es la misma lógica que va a correr en la Pi."""

    def __init__(self, caminar, levantarse=None):
        self.caminar, self.levantarse = caminar, levantarse
        self.modo = 'caminar'
        self.de_pie_desde = 0
        self.caidas = self.levantadas = 0

    def accion(self, env, obs):
        arriba, z = env.estado_cuerpo()
        if self.modo == 'caminar' and (arriba < 0.6 or z < 0.12):
            self.caidas += 1
            self.modo = 'levantarse'
            self.de_pie_desde = 0
        if self.modo == 'levantarse':
            if arriba > 0.9 and z > 0.16:
                self.de_pie_desde += 1
                if self.de_pie_desde >= CONTROL_HZ // 2:      # medio segundo de pie: vuelve a caminar
                    self.levantadas += 1
                    self.modo = 'caminar'
            else:
                self.de_pie_desde = 0
        if self.modo == 'levantarse':
            if self.levantarse is None:
                env.escala_accion = ESCALA_ACCION['caminar']
                return np.zeros(10)                      # sin política de levantarse: se queda quieto
            politica, escala = self.levantarse
        else:
            politica, escala = self.caminar
        env.escala_accion = escala
        return politica(obs)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('corrida', nargs='?', default=None, help='corrida de caminar (por defecto ultimo_caminar)')
    ap.add_argument('--levantarse', default=None, help='corrida de levantarse (por defecto ultimo_levantarse)')
    ap.add_argument('--onnx', default=None, help='ONNX de caminar en vez de la corrida')
    ap.add_argument('--onnx-levantarse', default=None)
    ap.add_argument('--segundos', type=float, default=15.0)
    ap.add_argument('--comando', type=float, nargs=3, default=[0.2, 0.0, 0.0], metavar=('VX', 'VY', 'GIRO'))
    ap.add_argument('--gesto', choices=sorted(GESTOS), default=None, help='gesto de la cabeza al arrancar')
    ap.add_argument('--demo', action='store_true', help='cuac mirando arriba, camina, empujón, se levanta, sigue')
    ap.add_argument('--caido', action='store_true', help='arranca tirado en el piso')
    ap.add_argument('--ir-a', type=float, nargs='+', default=None, metavar='XY', help='puntos x y x y ... en metros')
    ap.add_argument('--seguir', action='store_true', help='sigue la pelota (se mueve sola) con la cámara')
    ap.add_argument('--sin-ventana', action='store_true')
    ap.add_argument('--semilla', type=int, default=0, help='cambia la pose inicial')
    args = ap.parse_args()

    corridas = RAIZ / 'rl' / 'corridas'
    caminar = cargar_politica(args.corrida or corridas / 'ultimo_caminar', args.onnx, 'caminar')
    if caminar is None and not args.onnx:
        caminar = cargar_politica(corridas / 'ultimo', None, 'caminar')
    if caminar is None:
        sys.exit('no hay política de caminar: entrená con python3 rl/entrenar.py')
    levantarse = cargar_politica(args.levantarse or corridas / 'ultimo_levantarse', args.onnx_levantarse, 'levantarse')
    if levantarse is None:
        print('sin política de levantarse (python3 rl/entrenar.py --modo levantarse): si se cae, se queda')
    sup = Supervisor(caminar, levantarse)
    cabeza = Cabeza()
    if args.gesto:
        cabeza.hacer(args.gesto)
    if args.demo:
        cabeza.hacer('mirar_arriba_y_abrir', 'mirar_alrededor')
        args.segundos = max(args.segundos, 25.0)
    navegador = seguidor = None
    if args.ir_a:
        if len(args.ir_a) % 2:
            sys.exit('--ir-a necesita pares x y')
        navegador = Navegador(list(zip(args.ir_a[::2], args.ir_a[1::2])))
        args.segundos = max(args.segundos, 10.0 * len(navegador.puntos))
    if args.seguir:
        seguidor = Seguidor()
        args.segundos = max(args.segundos, 30.0)

    env = PatoEnv(aleatorizar=False, empujones=False, comando_fijo=args.comando, max_segundos=args.segundos,
                  cabeza_aleatoria=False)
    obs, _ = env.reset(seed=args.semilla, options={'caido': args.caido})
    if args.caido:
        sup.modo, sup.caidas = 'levantarse', 1
    x0 = env.data.xpos[env.tronco][0]
    visor = None
    if not args.sin_ventana:
        import mujoco.viewer
        try:
            visor = mujoco.viewer.launch_passive(env.model, env.data)
        except RuntimeError as e:   # en macOS la ventana solo abre bajo mjpython
            sys.exit(f'{e}\nEn macOS: mjpython rl/jugar.py ... (o usá --sin-ventana)')
    choques = 0
    centrado = visto = 0
    t0 = time.time()
    while True:
        t = env.paso / CONTROL_HZ
        comando = list(args.comando)
        if navegador is not None:
            comando = navegador.comando(env.posicion(), env.rumbo())
        if seguidor is not None:
            # La pelota da una vuelta lenta alrededor del origen; el robot tiene que seguirla.
            ang = 0.25 * t
            env.pelota([0.9 * math.cos(ang), 0.9 * math.sin(ang), 0.03])
            det = env.camara_pixel(env.pelota())
            if det is not None:
                u, v, prof = det
                det = (u, v, seguidor.f * 0.03 / prof)
                visto += 1
                centrado += abs(u - 320) < 64 and abs(v - 240) < 48
            yaw_a, pitch_a, _ = env.cabeza_actual()
            (yaw, pitch, boca), comando = seguidor.paso(det, yaw_a, pitch_a)
            cabeza.ultimo = (yaw, pitch, boca)
        if args.demo:
            if t < 6.0 or sup.modo == 'levantarse':
                comando = [0.0, 0.0, 0.0]           # quieto mientras hace el gesto y mientras se levanta
            if 12.0 <= t < 12.2:
                env.data.xfrc_applied[env.tronco, :3] = [25.0, 0.0, 0.0]   # lo tiran
            elif t >= 12.2:
                env.data.xfrc_applied[env.tronco] = 0
        env.comando[:] = comando
        if sup.modo == 'caminar' and not args.caido:
            env.fijar_cabeza(*cabeza.objetivo(t))
        else:
            env.fijar_cabeza(0.0, 0.0, 0.0)
        obs, r, fin, trunc, info = env.step(sup.accion(env, obs))
        choques += info['choques_propios']
        if visor is not None:
            visor.sync()
            time.sleep(max(0, env.paso / CONTROL_HZ - (time.time() - t0)))
        if trunc or (fin and env.modo == 'caminar' and sup.levantarse is None):
            break
    avance = env.data.xpos[env.tronco][0] - x0
    arriba, z = env.estado_cuerpo()
    estado = 'de pie' if arriba > 0.6 and z > 0.12 else 'EN EL PISO'
    print(f'terminó {estado} a los {env.paso / CONTROL_HZ:.1f} s; avanzó {avance * 100:.0f} cm '
          f'({avance / (env.paso / CONTROL_HZ) * 100:.1f} cm/s); caídas: {sup.caidas}, se levantó: {sup.levantadas}; '
          f'contactos consigo mismo: {choques}')
    if navegador is not None:
        print(f'llegó a {navegador.i} de {len(navegador.puntos)} puntos; quedó a '
              f'{np.linalg.norm(navegador.puntos[-1] - env.posicion()) * 100:.0f} cm del último')
    if seguidor is not None:
        n = env.paso
        print(f'pelota: vista el {100 * visto / n:.0f} % del tiempo, centrada el {100 * centrado / n:.0f} %; '
              f'distancia final {np.linalg.norm(env.pelota()[:2] - env.posicion()) * 100:.0f} cm')
    if visor is not None:
        visor.close()


if __name__ == '__main__':
    main()
