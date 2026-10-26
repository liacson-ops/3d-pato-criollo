# Entrenar al Pato Criollo (MacBook con Apple Silicon)

Aprendizaje por refuerzo (PPO) sobre MuJoCo, todo en CPU. No hace falta GPU. Lo que sale son archivos ONNX que corren en la Raspberry Pi Zero 2 W igual que las políticas del XGO-Duck.

Tres cosas, y solo dos se entrenan:

| Qué | Cómo | Dónde |
| --- | --- | --- |
| **Caminar** | política entrenada (`--modo caminar`), sigue un comando de velocidad y giro | `rl/entorno.py` |
| **Levantarse si se cae** | segunda política (`--modo levantarse`), arranca tirado de espaldas, de frente o de costado. **Con la mecánica actual no lo logra** (ver abajo) | `rl/entorno.py` |
| **Que lo alcen** | no se entrena: el supervisor detecta medio segundo sin pies apoyados, recoge las patas y mira arriba | `rl/jugar.py` |
| **Mirar arriba y abrir la boca** (y otros gestos) | programado, no se entrena: trayectorias suaves de los tres MG90S de la cabeza, dentro de la envolvente boca/pitch | `rl/cabeza.py` |

Arriba de la política de marcha van dos capas que no se entrenan (`rl/navegar.py`):

| Qué | Cómo |
| --- | --- |
| **Ir de un punto A a un punto B** | `Navegador`: lista de puntos (x, y); apunta al siguiente con el comando de giro, avanza y frena cerca. En la simulación usa la posición real; en el robot va a salir de la odometría o de un marcador visto por la cámara |
| **Seguir un objeto con la cámara** | `Seguidor`: con la posición del objeto en la imagen (en el robot, detección por color con OpenCV; acá, la proyección exacta de la cámara de la cabeza) centra el objeto moviendo el cuello; si queda muy de costado gira el cuerpo, y si está lejos camina hacia él hasta quedar a 45 cm. Si lo pierde, barre con la cabeza |

Las junta un **supervisor** (`rl/jugar.py`, clase `Supervisor`, la misma lógica que va a correr en la Pi): camina con la política de marcha; si el tronco se inclina más de 53° o baja de 12 cm, pasa a la política de levantarse; cuando lleva medio segundo de pie, vuelve a caminar. **Si lo alzan** (medio segundo sin ningún pie apoyado) recoge las patas, se queda quieto y mira hacia arriba, a quien lo tiene; cuando lo apoyan vuelve a caminar. Los gestos de la cabeza corren en paralelo mientras camina. La marcha se entrena con la cabeza moviéndose al azar en la mitad de los episodios, para que seguir caminando con la cabeza girada o el pico abierto no la sorprenda.

## Un pato natural, no un robot apurado

Es un juguete para una familia, así que la marcha se entrena para que se parezca a un pato y sea tranquila:

- **Reloj de paso:** un ritmo fijo de 1,6 pasos por segundo dice qué pie apoya y cuál va en el aire; seguirlo se premia (más por levantar el pie que por apoyarlo, para que quedarse parado no cobre) y se premia que el pie en el aire suba hasta 8 mm, para no arrastrarlo por la alfombra. Con el comando en cero el reloj se detiene con los dos pies apoyados. El reloj va en la observación (seno y coseno), y el runtime de la Pi lo genera igual.
- **Suavidad:** las penalizaciones por cambios bruscos de acción y por velocidad de los servos son tres veces más fuertes que antes.
- **Velocidad tranquila:** el 55 % de los episodios pide entre 5 y 12 cm/s, el 20 % hasta 25 cm/s, el 15 % quieto y el 10 % marcha atrás corta. El premio por velocidad se estrecha con la velocidad pedida, así que a 10 cm/s no sirve ir a 15.
- **Piso de casa:** en el 60 % de los episodios el piso no es plano: baldosas y alfombras con escalones de 5 a 15 mm (`umbrales`) o rampas suaves de hasta 1 cm (`rampas`). Se prueba con `mjpython rl/jugar.py --terreno umbrales`.
- **Lo alzan:** `mjpython rl/jugar.py --alzar` lo levanta 20 cm a los 4 s con una "mano" y lo vuelve a apoyar, para ver que recoge las patas y después sigue.

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
python3 rl/entrenar.py --pasos 2e6 --envs 4      # caminar, prueba de 10 minutos en una M1: tiene que dejar de caerse
python3 rl/entrenar.py                           # caminar, 10 M pasos con 8 entornos: menos de una hora
python3 rl/entrenar.py --pasos 60e6              # toda la noche (unas 5 horas): la marcha definitiva
python3 rl/entrenar.py --modo levantarse         # levantarse, 5 M pasos (hoy no lo logra, ver abajo)
tensorboard --logdir rl/corridas                 # curvas de recompensa (rollout/ep_rew_mean)
```

La tasa de aprendizaje baja linealmente a lo largo de la corrida (de 3e-4 a 3e-5), así que una corrida larga termina más fina que una corta: para la noche conviene una sola corrida de 60 M y no varias cortas encadenadas.

En zsh no pegues el texto que sigue al `#`: la terminal se lo pasa al script.

Cada corrida queda en `rl/corridas/<fecha>_<modo>/` y `rl/corridas/ultimo_caminar` y `ultimo_levantarse` apuntan a la última de cada modo. El entrenamiento no muestra nada: imprime una tabla por tanda (`ep_rew_mean`, `ep_len_mean`, `fps`) y guarda `modelo.zip` al día cada 100 k pasos, así que **mientras entrena podés abrir otra terminal y mirar cómo va con `mjpython rl/jugar.py`**. Se puede cortar con Ctrl+C: guarda igual. Para seguir una corrida: `python3 rl/entrenar.py --continuar rl/corridas/ultimo_caminar --pasos 5e6`.

Velocidad de referencia: en la máquina de prueba (4 núcleos, sin GPU) da 1.300 pasos de control por segundo con 4 entornos (cada paso de control son 10 pasos de física con choques entre mallas). En una M1 con 8 entornos esperá 2.000–4.000: 10 M pasos son de 1 a 2 horas. Lo que más cuesta es la física, no la red, así que la GPU no cambia nada acá.

## Ver el resultado

```bash
mjpython rl/jugar.py                             # camina con la última política (en Mac, mjpython; en Linux, python3)
mjpython rl/jugar.py --demo                      # cuac mirando arriba, mira alrededor, camina, lo tiran, se levanta, sigue
mjpython rl/jugar.py --caido                     # arranca tirado en el piso: prueba solo levantarse
mjpython rl/jugar.py --gesto mirar_arriba_y_abrir --comando 0 0 0
mjpython rl/jugar.py --comando 0.0 0.0 0.5       # girar en el lugar
mjpython rl/jugar.py --ir-a 1 0 1 1 0 1          # va a (1,0), después a (1,1), después a (0,1) y se queda
mjpython rl/jugar.py --seguir                    # sigue la pelota roja, que da vueltas sola, con la cabeza y el cuerpo
python3 rl/jugar.py --sin-ventana --segundos 20  # solo imprime cuánto avanzó, cuántas veces se cayó y se levantó
```

Gestos disponibles (`rl/cabeza.py`): `mirar_arriba_y_abrir`, `mirar_alrededor`, `mirar_abajo`, `cuac`. Agregar uno es escribir una función `gesto(t) -> (yaw, pitch, boca)` con su `duracion`.

`mjpython` viene con el paquete `mujoco` y queda en el mismo `bin/` del entorno virtual. En macOS la ventana no funciona con `python3` a secas.

## Exportar a la Pi

```bash
python3 rl/exportar_onnx.py rl/corridas/ultimo_caminar      # -> pato_caminar.onnx
python3 rl/exportar_onnx.py rl/corridas/ultimo_levantarse   # -> pato_levantarse.onnx
python3 rl/jugar.py --sin-ventana --onnx rl/corridas/ultimo_caminar/pato_caminar.onnx --onnx-levantarse rl/corridas/ultimo_levantarse/pato_levantarse.onnx
```

Cada ONNX recibe la observación de 41 valores sin normalizar (la normalización va adentro): giro del tronco (3), gravedad en el tronco (3), posición de las 10 articulaciones respecto de la pose de parado, velocidad (×0,1), acción anterior, comando [vx, vy, giro] y el reloj de paso [sen, cos] a 1,6 Hz (detenido en 0 cuando el comando es cero). Devuelve 10 acciones en [-1, 1]; el objetivo de cada servo es `parado + escala × acción`, con la escala de `config.json` (0,5 rad caminando, 0,8 levantándose). Ese contrato, más el supervisor, la navegación, el seguimiento y los gestos, es lo que va a implementar el runtime de la Pi.

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

## Levantarse: lo que dio y por qué

Entrenada 1 M pasos, la política de levantarse no se paró ni una vez; la recompensa se estancó a los 400 k. Para descartar un problema de aprendizaje se probaron 400 trayectorias al azar desde cada pose caída: ninguna lo para, y lo más derecho que llega es a 56° de la vertical. Es un límite del diseño:

- de costado no hay fuerza lateral: la cadera roll gira 0,25 rad hacia afuera y 0,09 hacia adentro, así que no puede rodar hasta los pies;
- de frente o de espaldas, las patas (6,4 cm) levantan la cadera 6 cm y el cuerpo con la cabeza mide más de 20: no alcanza para pivotar.

Alternativas: soltar el torque al caer y pedir ayuda con un "cuac" (barato, es lo que hacen los robots de juguete), o agregar dos alas con MG90S que empujen contra el piso para rodar hasta los pies (cambio de diseño, después reentrenar). Hasta decidirlo, el supervisor se queda quieto si no hay política de levantarse, y el modo `levantarse` queda para cuando el diseño lo permita.

## Si no aprende

- **Gira sobre su eje y se cae de golpe.** Pasó con la segunda recompensa: el premio por seguir el giro pedido era blando (0,5, sigma² 0,2) y la política giraba sin castigo. Ahora pesa 1,0 con sigma² 0,1. Los tumbos de golpe son falta de entrenamiento: a 2,4 M pasos todavía se caía una vez por episodio; la curva seguía subiendo.
- **Salta en el lugar en vez de caminar.** Pasó en la primera corrida: la recompensa por estar vivo, por "velocidad lateral cero" y por "giro cero" sumaban más quieto que caminando, y el premio al tiempo en el aire valía con los dos pies a la vez. La recompensa actual (`rl/entorno.py`) paga avanzar de forma lineal, solo premia el tiempo en el aire con un pie apoyado y penaliza volar y rebotar. Si vuelve a pasar, subí `avance` o bajá `vivo`.

- Con 2 M pasos ya tiene que mantenerse de pie casi siempre (`rollout/ep_len_mean` cerca del máximo, 750). Si se cae siempre, revisá que el modelo se pare solo y camine con la marcha a mano: `python3 sim/marcha.py` (en el modelo de choques, `pato_sim.xml`) y `python3 rl/jugar.py --sin-ventana` con una política recién inicializada.
- Si camina raro o "patina", subí la penalización `accion` o `torque` en `rl/entorno.py` y reentrená.
- Si el ONNX se comporta distinto que `jugar.py` sin ONNX, es la normalización: `exportar_onnx.py` la verifica con una observación de prueba.
