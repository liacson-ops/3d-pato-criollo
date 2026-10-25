#!/usr/bin/env python3
"""Corre una política entrenada en el simulador y muestra qué tan bien camina.

  python3 rl/jugar.py                           # última corrida, ventana de MuJoCo
  python3 rl/jugar.py rl/corridas/<fecha> --sin-ventana --segundos 20
  python3 rl/jugar.py --onnx rl/corridas/ultimo/pato_marcha.onnx   # prueba el ONNX exportado
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from entorno import PatoEnv  # noqa: E402


def cargar(args):
    if args.onnx:
        import onnxruntime as ort
        s = ort.InferenceSession(args.onnx, providers=['CPUExecutionProvider'])
        return lambda obs: s.run(None, {'obs': obs[None].astype(np.float32)})[0][0]
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
    corrida = Path(args.corrida).resolve()
    modelo = PPO.load(str(corrida / 'modelo.zip'), device='cpu')
    venv = VecNormalize.load(str(corrida / 'normalizacion.pkl'), DummyVecEnv([lambda: PatoEnv(aleatorizar=False)]))
    venv.training = False
    return lambda obs: modelo.predict(venv.normalize_obs(obs), deterministic=True)[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('corrida', nargs='?', default='rl/corridas/ultimo')
    ap.add_argument('--onnx', default=None)
    ap.add_argument('--segundos', type=float, default=15.0)
    ap.add_argument('--comando', type=float, nargs=3, default=[0.2, 0.0, 0.0], metavar=('VX', 'VY', 'GIRO'))
    ap.add_argument('--sin-ventana', action='store_true')
    ap.add_argument('--semilla', type=int, default=0, help='cambia la pose inicial (±0,05 rad en cada servo)')
    args = ap.parse_args()
    politica = cargar(args)
    env = PatoEnv(aleatorizar=False, empujones=False, comando_fijo=args.comando, max_segundos=args.segundos)
    obs, _ = env.reset(seed=args.semilla)
    x0 = env.data.xpos[env.tronco][0]
    visor = None
    if not args.sin_ventana:
        import mujoco.viewer
        try:
            visor = mujoco.viewer.launch_passive(env.model, env.data)
        except RuntimeError as e:   # en macOS la ventana solo abre bajo mjpython
            sys.exit(f'{e}\nEn macOS: mjpython rl/jugar.py ... (o usá --sin-ventana)')
    choques = 0
    t0 = time.time()
    while True:
        obs, r, fin, trunc, info = env.step(politica(obs))
        choques += info['choques_propios']
        if visor is not None:
            visor.sync()
            time.sleep(max(0, env.paso / 50 - (time.time() - t0)))
        if fin or trunc:
            break
    avance = env.data.xpos[env.tronco][0] - x0
    print(f"{'SE CAYÓ' if info['cayo'] else 'terminó de pie'} a los {env.paso / 50:.1f} s; "
          f'avanzó {avance * 100:.0f} cm ({avance / (env.paso / 50) * 100:.1f} cm/s); '
          f'contactos consigo mismo: {choques}')
    if visor is not None:
        visor.close()


if __name__ == '__main__':
    main()
