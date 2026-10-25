# Entrenar al Pato Criollo a caminar (MacBook con Apple Silicon)

Aprendizaje por refuerzo (PPO) sobre MuJoCo, todo en CPU. No hace falta GPU. Lo que sale es un archivo ONNX que corre en la Raspberry Pi Zero 2 W igual que las políticas del XGO-Duck.

## Instalación (una vez)

```bash
# Python 3.11 o 3.12 (con Homebrew: brew install python@3.12)
cd 3d-pato-criollo
python3 -m venv .venv && source .venv/bin/activate
pip install -r rl/requirements.txt
pip install manifold3d trimesh            # solo si vas a regenerar el modelo (cad/pato.py)
```

El modelo de simulación que usa el entrenamiento es `salida/mjcf/pato_rl.xml`: viene en el repo, con la envolvente convexa de cada pieza y choques habilitados entre eslabones no vecinos (patas entre sí, patas con el tronco, cabeza con el tronco). Si cambiás el diseño, regeneralo con `python3 cad/pato.py`.

Prueba rápida de que todo está instalado (menos de un minuto):

```bash
python3 rl/entrenar.py --pasos 16384 --envs 2      # entrena 16 k pasos y guarda en rl/corridas/<fecha>
python3 rl/exportar_onnx.py rl/corridas/ultimo     # exporta y verifica el ONNX
python3 rl/jugar.py --sin-ventana --segundos 5     # lo corre sin ventana
```

Si `tensorboard` no está instalado el entrenamiento avisa y sigue sin guardar curvas.

## Entrenar

```bash
python3 rl/entrenar.py --pasos 2e6 --envs 4      # media hora: tiene que aprender a no caerse
python3 rl/entrenar.py                           # 10 M pasos, 8 entornos: unas 2 horas en una M1
tensorboard --logdir rl/corridas                 # curvas de recompensa (rollout/ep_rew_mean)
```

Cada corrida queda en `rl/corridas/<fecha>/` y `rl/corridas/ultimo` apunta a la última. El entrenamiento no muestra nada: imprime una tabla por tanda (`ep_rew_mean`, `ep_len_mean`, `fps`) y guarda `modelo.zip` al día cada 100 k pasos, así que **mientras entrena podés abrir otra terminal y mirar cómo va con `mjpython rl/jugar.py`**. Se puede cortar con Ctrl+C: guarda igual. Para seguir una corrida: `python3 rl/entrenar.py --continuar rl/corridas/ultimo --pasos 5e6`.

Velocidad de referencia: en la máquina de prueba (4 núcleos, sin GPU) da 1.300 pasos de control por segundo con 4 entornos (cada paso de control son 10 pasos de física con choques entre mallas). En una M1 con 8 entornos esperá 2.000–4.000: 10 M pasos son de 1 a 2 horas. Lo que más cuesta es la física, no la red, así que la GPU no cambia nada acá.

## Ver el resultado

```bash
mjpython rl/jugar.py                             # abre la ventana de MuJoCo con la última política (en Mac, mjpython; en Linux, python3)
python3 rl/jugar.py --sin-ventana --segundos 20  # solo imprime cuánto avanzó y si se cayó
mjpython rl/jugar.py --comando 0.0 0.0 0.5       # girar en el lugar
```

`mjpython` viene con el paquete `mujoco` y queda en el mismo `bin/` del entorno virtual. En macOS la ventana no funciona con `python3` a secas.

## Exportar a la Pi

```bash
python3 rl/exportar_onnx.py rl/corridas/ultimo   # -> rl/corridas/<fecha>/pato_marcha.onnx
python3 rl/jugar.py --onnx rl/corridas/ultimo/pato_marcha.onnx --sin-ventana
```

El ONNX recibe la observación de 39 valores sin normalizar (la normalización va adentro) y devuelve 10 acciones en [-1, 1]; el objetivo de cada servo es `parado + 0,5 rad × acción`. Ese contrato es el que va a implementar el runtime de la Pi.

## Qué simula

| Qué | Cómo |
| --- | --- |
| Choques consigo mismo | envolvente convexa de cada pieza impresa y de cada servo; MuJoCo filtra solo los pares padre-hijo |
| Choques con el piso | plano con fricción aleatoria (0,6–1,4×) |
| Servos | actuadores de posición con el torque máximo de cada modelo; ganancia y torque aleatorios (0,7–1,3× y 0,7–1,0×) |
| Retardo | 0 a 40 ms entre la orden y el servo |
| Masas | ±15 % en cada pieza |
| Empujones | fuerza horizontal al azar sobre el tronco, 1 % de los ciclos |
| Comando | velocidad adelante (−0,05 a 0,25 m/s), lateral (±0,1) y giro (±0,6 rad/s); 15 % del tiempo quieto |

Lo que todavía no está: terreno con escalones y obstáculos (hay que agregar un campo de alturas en `pato_rl.xml`) y el modelo del servo medido en el banco de la fase 0 (reemplaza los actuadores de posición por el modelo de BAM).

## Si no aprende

- Con 2 M pasos ya tiene que mantenerse de pie casi siempre (`rollout/ep_len_mean` cerca del máximo, 750). Si se cae siempre, revisá que el modelo se pare solo y camine con la marcha a mano: `python3 sim/marcha.py` (en el modelo de choques, `pato_sim.xml`) y `python3 rl/jugar.py --sin-ventana` con una política recién inicializada.
- Si camina raro o "patina", subí la penalización `accion` o `torque` en `rl/entorno.py` y reentrená.
- Si el ONNX se comporta distinto que `jugar.py` sin ONNX, es la normalización: `exportar_onnx.py` la verifica con una observación de prueba.
