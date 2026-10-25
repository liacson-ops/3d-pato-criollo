#!/usr/bin/env python3
"""Entrena una política con PPO (Stable-Baselines3) en CPU, apto para MacBook M1.

  python3 rl/entrenar.py                            # caminar: 8 entornos, 10 M pasos (1–2 h en una M1)
  python3 rl/entrenar.py --modo levantarse          # levantarse del piso: 5 M pasos alcanzan
  python3 rl/entrenar.py --pasos 2e6 --envs 4       # prueba corta
  python3 rl/entrenar.py --continuar rl/corridas/ultimo_caminar   # sigue una corrida

Guarda en rl/corridas/<fecha>_<modo>/: modelo.zip y normalizacion.pkl (al día cada 100 k pasos),
config.json, checkpoints cada 1 M y logs para TensorBoard (tensorboard --logdir rl/corridas).
rl/corridas/ultimo_<modo> apunta a la última corrida de cada modo.
Después: python3 rl/exportar_onnx.py rl/corridas/ultimo_<modo>
"""
import argparse
import datetime
import json
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


def _enlazar(nombre, destino):
    enlace = RAIZ / 'rl' / 'corridas' / nombre
    if enlace.is_symlink() or enlace.exists():
        enlace.unlink()
    enlace.symlink_to(destino)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--modo', choices=('caminar', 'levantarse'), default='caminar')
    ap.add_argument('--pasos', type=float, default=None, help='caminar: 10 M; levantarse: 5 M')
    ap.add_argument('--envs', type=int, default=max(2, min(8, (os.cpu_count() or 4) - 1)))
    ap.add_argument('--continuar', type=str, default=None)
    ap.add_argument('--semilla', type=int, default=0)
    ap.add_argument('--cabeza-quieta', action='store_true', help='caminar sin mover la cabeza al azar')
    args = ap.parse_args()

    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
    from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize, VecMonitor
    from entorno import hacer_env, OBS_DIM, ACT_DIM, ESCALA_ACCION

    torch.set_num_threads(1)   # la física ya usa un proceso por entorno
    if args.continuar:
        salida = Path(args.continuar).resolve()
        modo = json.loads((salida / 'config.json').read_text()).get('modo', args.modo)
    else:
        modo = args.modo
        salida = RAIZ / 'rl' / 'corridas' / f"{datetime.datetime.now().strftime('%Y-%m-%d_%H%M')}_{modo}"
    pasos = int(args.pasos) if args.pasos else (10_000_000 if modo == 'caminar' else 5_000_000)
    salida.mkdir(parents=True, exist_ok=True)
    (salida / 'config.json').write_text(json.dumps(dict(
        modo=modo, obs_dim=OBS_DIM, act_dim=ACT_DIM, escala_accion=ESCALA_ACCION[modo],
        cabeza_aleatoria=not args.cabeza_quieta), indent=1))
    venv = VecMonitor(SubprocVecEnv([hacer_env(modo=modo, semilla=args.semilla * 100 + i,
                                               cabeza_aleatoria=not args.cabeza_quieta)
                                     for i in range(args.envs)]))
    if args.continuar:
        venv = VecNormalize.load(str(salida / 'normalizacion.pkl'), venv)
        modelo = PPO.load(str(salida / 'modelo.zip'), env=venv, device='cpu')
    else:
        venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0)
        modelo = PPO('MlpPolicy', venv, device='cpu', verbose=1, seed=args.semilla,
                     n_steps=1024, batch_size=4096, n_epochs=5, learning_rate=3e-4, gamma=0.99, gae_lambda=0.95,
                     ent_coef=0.005, clip_range=0.2, policy_kwargs=dict(net_arch=[256, 128, 64]),
                     tensorboard_log=str(salida) if _hay_tensorboard() else None)

    def guardar():
        modelo.save(str(salida / 'modelo.zip'))
        venv.save(str(salida / 'normalizacion.pkl'))

    class GuardarCadaTanto(BaseCallback):
        # Cada 100 k pasos deja modelo.zip y normalizacion.pkl al día: mientras entrena se puede
        # mirar cómo va con `mjpython rl/jugar.py` desde otra terminal.
        def __init__(self, cada):
            super().__init__()
            self.cada, self.proximo = cada, cada

        def _on_step(self):
            if self.num_timesteps >= self.proximo:
                guardar()
                self.proximo += self.cada
            return True

    _enlazar('ultimo', salida.name)
    _enlazar(f'ultimo_{modo}', salida.name)
    callbacks = [GuardarCadaTanto(100_000),
                 CheckpointCallback(save_freq=max(1, 1_000_000 // args.envs), save_path=str(salida / 'checkpoints'),
                                    name_prefix='ppo', save_vecnormalize=True)]
    print(f'modo {modo}: {pasos:,} pasos con {args.envs} entornos -> {salida}')
    try:
        modelo.learn(total_timesteps=pasos, callback=callbacks, reset_num_timesteps=not args.continuar)
    finally:
        guardar()
        print(f'guardado en {salida}')


if __name__ == '__main__':
    main()
