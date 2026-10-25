#!/usr/bin/env python3
"""Entrena la política de marcha con PPO (Stable-Baselines3) en CPU, apto para MacBook M1.

  python3 rl/entrenar.py                       # 8 entornos, 10 M pasos (una noche en una M1)
  python3 rl/entrenar.py --pasos 2e6 --envs 4  # prueba corta
  python3 rl/entrenar.py --continuar rl/corridas/ultimo   # sigue una corrida

Guarda en rl/corridas/<fecha>/: modelo.zip, normalizacion.pkl y logs para TensorBoard
(tensorboard --logdir rl/corridas). Después: python3 rl/exportar_onnx.py rl/corridas/<fecha>
"""
import argparse
import datetime
import os
import sys
from pathlib import Path

RAIZ = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(RAIZ / 'rl'))


def _hay_tensorboard():
    try:
        import tensorboard  # noqa: F401
        return True
    except ImportError:
        print('tensorboard no está instalado: no se guardan curvas (pip install tensorboard)')
        return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pasos', type=float, default=10e6)
    ap.add_argument('--envs', type=int, default=max(2, min(8, (os.cpu_count() or 4) - 1)))
    ap.add_argument('--continuar', type=str, default=None)
    ap.add_argument('--semilla', type=int, default=0)
    args = ap.parse_args()

    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CheckpointCallback
    from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize, VecMonitor
    from entorno import hacer_env

    torch.set_num_threads(1)   # la física ya usa un proceso por entorno
    salida = RAIZ / 'rl' / 'corridas' / (datetime.datetime.now().strftime('%Y-%m-%d_%H%M') if not args.continuar
                                        else Path(args.continuar).name)
    salida.mkdir(parents=True, exist_ok=True)
    venv = VecMonitor(SubprocVecEnv([hacer_env(semilla=args.semilla * 100 + i) for i in range(args.envs)]))
    if args.continuar:
        venv = VecNormalize.load(str(Path(args.continuar) / 'normalizacion.pkl'), venv)
        modelo = PPO.load(str(Path(args.continuar) / 'modelo.zip'), env=venv, device='cpu')
    else:
        venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0)
        modelo = PPO('MlpPolicy', venv, device='cpu', verbose=1, seed=args.semilla,
                     n_steps=1024, batch_size=4096, n_epochs=5, learning_rate=3e-4, gamma=0.99, gae_lambda=0.95,
                     ent_coef=0.005, clip_range=0.2, policy_kwargs=dict(net_arch=[256, 128, 64]),
                     tensorboard_log=str(salida) if _hay_tensorboard() else None)
    guardar = CheckpointCallback(save_freq=max(1, 500_000 // args.envs), save_path=str(salida / 'checkpoints'),
                                 name_prefix='ppo', save_vecnormalize=True)
    try:
        modelo.learn(total_timesteps=int(args.pasos), callback=guardar, reset_num_timesteps=not args.continuar)
    finally:
        modelo.save(str(salida / 'modelo.zip'))
        venv.save(str(salida / 'normalizacion.pkl'))
        enlace = RAIZ / 'rl' / 'corridas' / 'ultimo'
        if enlace.is_symlink() or enlace.exists():
            enlace.unlink()
        enlace.symlink_to(salida.name)
        print(f'guardado en {salida}')


if __name__ == '__main__':
    main()
