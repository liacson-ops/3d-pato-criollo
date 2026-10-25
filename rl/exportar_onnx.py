#!/usr/bin/env python3
"""Exporta la política entrenada a ONNX, con la normalización de observaciones adentro.

  python3 rl/exportar_onnx.py rl/corridas/ultimo        # escribe rl/corridas/<corrida>/pato_marcha.onnx

Entrada:  obs [1, 39] (sin normalizar; ver rl/entorno.py)
Salida:   accion [1, 10] en [-1, 1]; objetivo = parado + 0,5 rad × accion
El archivo corre en la Raspberry Pi Zero 2 W con onnxruntime, como las políticas del XGO-Duck.
"""
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from entorno import OBS_DIM, ACT_DIM  # noqa: E402


class PoliticaDeterminista(torch.nn.Module):
    def __init__(self, politica, media, var, clip):
        super().__init__()
        self.politica = politica
        self.media = torch.tensor(media, dtype=torch.float32)
        self.desvio = torch.tensor(np.sqrt(var + 1e-8), dtype=torch.float32)
        self.clip = clip

    def forward(self, obs):
        x = torch.clamp((obs - self.media) / self.desvio, -self.clip, self.clip)
        latente = self.politica.mlp_extractor.forward_actor(x)
        return torch.tanh(self.politica.action_net(latente)) if False else torch.clamp(self.politica.action_net(latente), -1, 1)


def main():
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import VecNormalize
    import gymnasium as gym

    corrida = Path(sys.argv[1] if len(sys.argv) > 1 else 'rl/corridas/ultimo').resolve()
    modelo = PPO.load(str(corrida / 'modelo.zip'), device='cpu')
    # VecNormalize necesita un env; uno vacío alcanza para leer las estadísticas.
    from stable_baselines3.common.vec_env import DummyVecEnv
    from entorno import PatoEnv
    venv = VecNormalize.load(str(corrida / 'normalizacion.pkl'), DummyVecEnv([lambda: PatoEnv(aleatorizar=False)]))
    red = PoliticaDeterminista(modelo.policy, venv.obs_rms.mean, venv.obs_rms.var, venv.clip_obs).eval()
    ejemplo = torch.zeros(1, OBS_DIM)
    salida = corrida / 'pato_marcha.onnx'
    comun = dict(input_names=['obs'], output_names=['accion'], opset_version=17,
                 dynamic_axes={'obs': {0: 'lote'}, 'accion': {0: 'lote'}})
    try:
        # Exportador clásico (TorchScript): no necesita onnxscript. En torch < 2.5 no existe `dynamo`.
        torch.onnx.export(red, ejemplo, str(salida), dynamo=False, **comun)
    except TypeError:
        torch.onnx.export(red, ejemplo, str(salida), **comun)
    import onnxruntime as ort
    sesion = ort.InferenceSession(str(salida), providers=['CPUExecutionProvider'])
    a = sesion.run(None, {'obs': np.zeros((1, OBS_DIM), np.float32)})[0]
    assert a.shape == (1, ACT_DIM)
    print(f'{salida} ({salida.stat().st_size / 1024:.0f} kB); salida de prueba {np.round(a[0], 3)}')


if __name__ == '__main__':
    main()
