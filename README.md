# Pato Criollo 🦆

Robot pato bípedo **hecho con piezas que se consiguen en MercadoLibre Argentina**, inspirado en el [XGO-Duck](https://github.com/LuwuDynamics/xgoduck_hardware) y el [Microduck de Pollen Robotics](https://github.com/pollen-robotics/microduck). El objetivo es que sea lo más barato posible.

Este repo tiene el **modelo 3D v0**: paramétrico, con las medidas reales de cada componente, masas calculadas y un modelo de simulación para MuJoCo. Sirve para ver cómo queda, validar proporciones y torques, y planificar el armado. **Las piezas definitivas para imprimir salen después de la fase 0**, cuando midamos los servos reales (ver [PLAN.md](PLAN.md)).

![Pato Criollo, vista ¾](docs/img/pato_tres_cuartos.png)

| Rayos X | Explotado | Saludo |
| --- | --- | --- |
| ![Rayos X](docs/img/pato_rayos_x.png) | ![Explotado](docs/img/pato_explotado.png) | ![Saludo](docs/img/pato_saludo.png) |

## Ver el modelo en 3D

Descargá **[`salida/pato.html`](salida/pato.html)** (botón *Download raw file*) y abrilo con doble clic en cualquier navegador. Es un único archivo de unos 2 MB que funciona sin internet.

Desde el visor podés:
- girar y hacer zoom;
- ver el robot armado, en rayos X (carcasa transparente), explotado o **paso a paso**;
- cambiar la pose: patas rectas, parado, cuclillas, saludo;
- pasar el puntero sobre cualquier pieza para ver qué es, cuánto pesa y en qué paso va.

## Qué tiene adentro

| | |
| --- | --- |
| Altura | ~38 cm (cabeza incluida) |
| Masa estimada | **1,18 kg** (29 piezas impresas que suman 358 g) |
| Servos | **8× MG996R** (cadera roll, cadera pitch, rodilla y tobillo de cada pata) y **5× MG90S** (cadera yaw ×2, cuello yaw, cuello pitch, boca) |
| Patas | 5 articulaciones cada una. Muslo y canilla de 45 mm (el original tiene 42 mm) |
| Control | Raspberry Pi Zero 2 W a bordo, PCA9685 (PWM), 2× MCP3008 (lectura de los potenciómetros), MPU-6050 (IMU) |
| Cámara | Cámara de la Pi en la frente, entre los ojos |
| Energía | LiPo 2S de 1500 mAh, 2× XL4016 a 6,0 V (uno por pata), Mini560 de 5 V para la Pi |

### Torques (simulación MuJoCo, [`sim/torques.py`](sim/torques.py))

| Pose | Articulación más exigida | Uso del torque máximo |
| --- | --- | ---: |
| Parado (rodillas a 40°) | tobillo, 1,95 kg·cm | 18 % |
| Cuclillas (rodillas a 90°) | tobillo, 1,92 kg·cm | 18 % |
| En una pata, sin desplazar el cuerpo (peor caso) | cadera roll, 4,41 kg·cm | **40 %** |
| Cabeza (cualquier pose) | cuello pitch, 0,46 kg·cm | 21 % (MG90S) |

Al caminar, los picos suelen ser 2 a 3 veces el valor estático. **La cadera roll es el punto a vigilar.** Un paso normal desplaza el cuerpo sobre el pie de apoyo, así que exige menos que el peor caso. Tabla completa: [`salida/torques.md`](salida/torques.md).

## Cómo se arma

Las imágenes salen del mismo visor, en modo *Paso a paso*: en cada paso lo nuevo aparece en color y lo anterior, transparente.

**Antes de empezar, hay que modificar los servos:**
- Se abren y se suelda un cable al cursor (pin del medio) del potenciómetro interno, para que la Pi pueda leer la posición.
- A los MG996R de **rodilla** se les cortan **las dos orejas**.
- A los de **tobillo** y **cadera roll**, **la oreja del lado opuesto al eje**.
- A los MG90S del **cuello pitch** también la oreja del lado opuesto al eje. Los del **cuello yaw** y la **boca** al revés: se corta la oreja del lado del eje.

Así no chocan entre sí, y la carcasa impresa los sujeta.

| | |
| --- | --- |
| ![Paso 1](docs/img/armado_paso_01.png) **1. Pies:** suela de TPU, paleta y orejas del tobillo. | ![Paso 2](docs/img/armado_paso_02.png) **2. Canilla** con el servo del tobillo. Se une al pie. |
| ![Paso 3](docs/img/armado_paso_03.png) **3. Muslo** con el servo de la rodilla. Se une a la canilla. | ![Paso 4](docs/img/armado_paso_04.png) **4. Soporte U** de cadera con el servo pitch. Se une al muslo. |
| ![Paso 5](docs/img/armado_paso_05.png) **5. Servo roll** de cadera, placa yaw y eje. | ![Paso 6](docs/img/armado_paso_06.png) **6. Piso del tronco:** rulemanes 608 y servos yaw. Se cuelgan las patas. |
| ![Paso 7](docs/img/armado_paso_07.png) **7. Batería, IMU y estante A:** reguladores XL4016 y placa de los MCP3008. | ![Paso 8](docs/img/armado_paso_08.png) **8. Estante B:** Pi Zero 2 W, PCA9685 y regulador de 5 V. |
| ![Paso 9](docs/img/armado_paso_09.png) **9. Cuello:** servo yaw y servo pitch. | ![Paso 10](docs/img/armado_paso_10.png) **10. Cabeza:** servo de la boca, cámara, ojos y pico. |
| ![Paso 11](docs/img/armado_paso_11.png) **11. Carcasa** del cuerpo (dos mitades) y cola. | ![Frente](docs/img/pato_frente.png) **Listo.** |

## Archivos

```
cad/componentes.py        medidas y masas reales: MG996R, MG90S, Pi Zero 2 W, LiPo, PCA9685, XL4016…
cad/pato.py               el robot completo: piezas, eslabones, articulaciones, pasos de armado
sim/torques.py            torques estáticos en MuJoCo contra el torque máximo de cada servo
visor/                    visor 3D (three.js) y capturas automáticas (Playwright)
salida/pato.html          visor autocontenido (se abre con doble clic)
salida/pato.glb           modelo 3D con colores (sirve en Blender o en https://gltf-viewer.donmccurdy.com)
salida/piezas_impresas/   STL de las 29 piezas impresas (v0, todavía no para imprimir)
salida/mjcf/pato.xml      modelo MuJoCo con masas e inercias
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
cd visor && npm install && node construir.mjs      # salida/pato.html
node capturas.mjs                                  # docs/img (necesita Playwright con Chromium)
```

Todo sale de `cad/pato.py`. Si cambiás una medida, como el largo del muslo o el servo, se regeneran juntos el 3D, la simulación y las imágenes.

## Estado y próximos pasos

- [x] Plan y presupuesto ([PLAN.md](PLAN.md))
- [x] Modelo 3D v0 con componentes reales, sin choques entre piezas (verificado por intersección de mallas)
- [x] Simulación de torques estáticos
- [ ] **Fase 0:** banco de pruebas de servos (soporte imprimible y scripts de medición para la Pi 4), [lista de compras](COMPRAS-FASE0.md)
- [ ] v1: piezas para imprimir con las medidas reales de los servos: agujeros, tornillería M3, tolerancias
- [ ] Caminar programado a mano, después política RL entrenada con el modelo identificado del servo

## Licencia

- Código (`cad/`, `sim/`, `visor/`): Apache-2.0.
- Diseño (modelos y STL): CC BY-SA 4.0.

Diseño propio: no usa archivos del XGO-Duck ni del Microduck; solo toma la idea y las proporciones como referencia.
