# Pato Criollo 🦆

Robot pato bípedo **hecho con piezas que se consiguen en MercadoLibre Argentina**, inspirado en el [XGO-Duck](https://github.com/LuwuDynamics/xgoduck_hardware) y el [Microduck de Pollen Robotics](https://github.com/pollen-robotics/microduck). El objetivo es que sea lo más barato posible.

Este repo tiene el **modelo 3D v0**: paramétrico, con las medidas reales de cada componente, masas calculadas, un modelo de simulación para MuJoCo y un **simulador que corre en el navegador**. Sirve para ver cómo queda, validar proporciones y torques, probar que se para y camina, y planificar el armado. **Las piezas definitivas para imprimir salen después de la fase 0**, cuando midamos los servos reales (ver [PLAN.md](PLAN.md)).

El aspecto sigue al pato original para que se reconozca enseguida: cabeza en forma de pan con un ojo redondo amarillo a cada lado, pico ancho y chato que abre como una tapa desde atrás, tronco cajita blanco y los servos del cuello y de las patas a la vista.

![Pato Criollo, vista ¾](docs/img/pato_tres_cuartos.png)

| Rayos X | Explotado | Saludo |
| --- | --- | --- |
| ![Rayos X](docs/img/pato_rayos_x.png) | ![Explotado](docs/img/pato_explotado.png) | ![Saludo](docs/img/pato_saludo.png) |

## Probarlo: simulador en el navegador

![Simulador](docs/img/simulador.png)

Descargá **[`salida/simulador.html`](salida/simulador.html)** (botón *Download raw file*, son 16 MB porque lleva el motor de física adentro) y abrilo con doble clic. Corre **MuJoCo** compilado a WebAssembly, con las masas e inercias del modelo y los servos como actuadores de posición con el torque máximo de cada uno. Funciona sin internet.

- **Parado / Caminar / Manual / Soltar torque.** Caminar usa una marcha programada a mano (osciladores); con *Manual* movés cada articulación con un deslizador.
- **Empujón** (barra espaciadora) para ver si se recupera. **Acostar** lo deja con las patas rectas para ver cómo se levanta... o no.
- Los deslizadores de marcha cambian frecuencia, balanceo, levante, zancada y giro en vivo. Teclas: ↑ caminar, ↓ parar, ← → girar.
- A la derecha, el **torque de cada servo en % de su máximo**, en tiempo real. Rojo = saturado.
- *Ver cajas de choque* muestra las formas que usa la física (una caja por eslabón).

Qué da hoy: se queda parado, aguanta empujones de 6 N y camina derecho, aunque despacio (unos 2 cm/s con la marcha a mano). En el banco de pruebas con Python ([`sim/marcha.py`](sim/marcha.py)) ninguna de las 81 combinaciones de marcha probadas lo hizo caer.

## Ver el modelo en 3D

Descargá **[`salida/pato.html`](salida/pato.html)** y abrilo con doble clic en cualquier navegador. Es un único archivo de unos 2 MB que funciona sin internet.

Desde el visor podés:
- girar y hacer zoom;
- ver el robot armado, en rayos X (carcasa transparente), explotado o **paso a paso**;
- cambiar la pose: patas rectas, parado, cuclillas, saludo;
- pasar el puntero sobre cualquier pieza para ver qué es, cuánto pesa y en qué paso va.

## Qué tiene adentro

| | |
| --- | --- |
| Altura | **31,5 cm** parado (el original mide unos 25 cm) |
| Masa estimada | **1,26 kg** (31 piezas impresas; la cabeza completa pesa 146 g) |
| Servos | **8× MG996R** (cadera roll, cadera pitch, rodilla y tobillo de cada pata) y **5× MG90S** (cadera yaw ×2, cuello yaw, cuello pitch, boca) |
| Patas | 5 articulaciones cada una. Los servos de cadera pitch, rodilla y tobillo van **acostados** (eje lateral, cuerpo hacia atrás) en una cuna impresa igual para los tres: así muslo y canilla miden 32 mm y la pata queda compacta |
| Control | Raspberry Pi Zero 2 W a bordo, PCA9685 (PWM), 2× MCP3008 (lectura de los potenciómetros), MPU-6050 (IMU) |
| Cámara | Cámara de la Pi en la frente, con el lente justo arriba del pico |
| Energía | LiPo 2S de 1500 mAh, 2× XL4016 a 6,0 V (uno por pata), Mini560 de 5 V para la Pi |

### Torques (simulación MuJoCo, [`sim/torques.py`](sim/torques.py))

| Pose | Articulación más exigida | Uso del torque máximo |
| --- | --- | ---: |
| Parado (rodillas a 40°) | tobillo, 2,2 kg·cm | 20 % |
| Cuclillas (rodillas a 90°) | tobillo, 1,6 kg·cm | 14 % |
| En una pata, sin desplazar el cuerpo (peor caso) | cadera roll, 5,2 kg·cm | **47 %** |
| Cabeza (cualquier pose) | cuello pitch, 0,65 kg·cm | 30 % (MG90S) |

Al caminar, los picos suelen ser 2 a 3 veces el valor estático. **La cadera roll es el punto a vigilar.** Un paso normal desplaza el cuerpo sobre el pie de apoyo, así que exige menos que el peor caso. Tabla completa: [`salida/torques.md`](salida/torques.md).

Caminando en la simulación aparece otro problema: **los MG90S de la cadera yaw saturan** (100 % del torque) porque reciben toda la torsión de la pata apoyada. Opciones para la v1: trabar el yaw (patas de 4 articulaciones, como muchos bípedos chicos), pasar esos dos servos a MG996R, o un tope mecánico que descargue al servo. Se decide con lo que midamos en la fase 0.

## Cómo se arma

Las imágenes salen del mismo visor, en modo *Paso a paso*: en cada paso lo nuevo aparece en color y lo anterior, transparente.

**Antes de empezar, hay que modificar los servos:**
- Se abren y se suelda un cable al cursor (pin del medio) del potenciómetro interno, para que la Pi pueda leer la posición.
- Los MG996R de las patas van **con sus orejas**: la cuna impresa los sujeta por ahí (apoyos a ambos lados del cuerpo).
- A los MG90S del **cuello pitch** se les corta la oreja del lado opuesto al eje. Los del **cuello yaw** y la **boca** al revés: se corta la oreja del lado del eje.

| | |
| --- | --- |
| ![Paso 1](docs/img/armado_paso_01.png) **1. Pies:** suela de TPU, paleta y orejas del tobillo. | ![Paso 2](docs/img/armado_paso_02.png) **2. Canilla:** cuna con el servo del tobillo acostado. Se une al pie. |
| ![Paso 3](docs/img/armado_paso_03.png) **3. Muslo:** cuna con el servo de la rodilla. Se une a la canilla. | ![Paso 4](docs/img/armado_paso_04.png) **4. Soporte U** de cadera: cuna con el servo pitch. Se une al muslo. |
| ![Paso 5](docs/img/armado_paso_05.png) **5. Servo roll** de cadera, placa yaw y eje. | ![Paso 6](docs/img/armado_paso_06.png) **6. Piso del tronco:** rulemanes 608 y servos yaw. Se cuelgan las patas. |
| ![Paso 7](docs/img/armado_paso_07.png) **7. Batería, IMU y estante A:** reguladores XL4016 y placa de los MCP3008. | ![Paso 8](docs/img/armado_paso_08.png) **8. Estante B:** Pi Zero 2 W, PCA9685 y regulador de 5 V. |
| ![Paso 9](docs/img/armado_paso_09.png) **9. Cuello:** servo yaw y servo pitch. | ![Paso 10](docs/img/armado_paso_10.png) **10. Cabeza:** base, servo de la boca, mandíbula con el pico inferior, cámara, ojos, pico superior y cúpula. |
| ![Paso 11](docs/img/armado_paso_11.png) **11. Carcasa** del cuerpo (dos mitades). | ![Frente](docs/img/pato_frente.png) **Listo.** |

## Archivos

```
cad/componentes.py        medidas y masas reales: MG996R, MG90S, Pi Zero 2 W, LiPo, PCA9685, XL4016…
cad/pato.py               el robot completo: piezas, eslabones, articulaciones, pasos de armado
sim/torques.py            torques estáticos en MuJoCo contra el torque máximo de cada servo
sim/marcha.py             marcha programada a mano y barrido de parámetros (MuJoCo, Python)
visor/                    visor 3D y simulador (three.js + MuJoCo WASM), capturas y pruebas automáticas
salida/simulador.html     simulador físico autocontenido (16 MB, se abre con doble clic)
salida/pato.html          visor autocontenido (se abre con doble clic)
salida/pato.glb           modelo 3D con colores (sirve en Blender o en https://gltf-viewer.donmccurdy.com)
salida/piezas_impresas/   STL de las 31 piezas impresas (v0, todavía no para imprimir)
salida/mjcf/pato.xml      modelo MuJoCo con masas e inercias (mallas)
salida/mjcf/pato_sim.xml  el mismo con cajas de choque, es el que usa el simulador
salida/resumen.md         masa de cada pieza
docs/img/                 imágenes de este README
PLAN.md                   plan completo, fases y presupuesto
COMPRAS-FASE0.md          lista de compras de la fase 0 (banco de pruebas de servos)
```

## Regenerar todo

```bash
pip install manifold3d trimesh numpy mujoco        # probado con 3.5.4 / 5.2.0 / 2.5.3 / 3.17.0
python3 cad/pato.py                                # STL, GLB, MJCF, resumen
python3 sim/torques.py                             # tabla de torques
python3 sim/marcha.py                              # ¿camina? ¿con qué picos de torque?
cd visor && npm install && npm run construir       # salida/pato.html y salida/simulador.html
node capturas.mjs && node probar_sim.mjs           # docs/img y prueba automática del simulador (Playwright)
```

Todo sale de `cad/pato.py`. Si cambiás una medida, como el largo del muslo o el servo, se regeneran juntos el 3D, la simulación y las imágenes.

## Estado y próximos pasos

- [x] Plan y presupuesto ([PLAN.md](PLAN.md))
- [x] Modelo 3D v0 con componentes reales, sin choques entre piezas (verificado por intersección de mallas)
- [x] Simulación de torques estáticos
- [x] Simulador en el navegador: se para, camina con marcha a mano, aguanta empujones
- [ ] **Fase 0:** banco de pruebas de servos (soporte imprimible y scripts de medición para la Pi 4), [lista de compras](COMPRAS-FASE0.md)
- [ ] v1: piezas para imprimir con las medidas reales de los servos: agujeros, tornillería M3, tolerancias
- [ ] Caminar programado a mano, después política RL entrenada con el modelo identificado del servo

## Licencia

- Código (`cad/`, `sim/`, `visor/`): Apache-2.0.
- Diseño (modelos y STL): CC BY-SA 4.0.

Diseño propio: no usa archivos del XGO-Duck ni del Microduck; solo toma la idea y las proporciones como referencia.
