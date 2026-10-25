# Plan: Pato Criollo

Una versión del XGO-Duck (nombre provisorio) **hecha con piezas de MercadoLibre Argentina, lo más barata posible**, y mejor donde se pueda. Usa la Raspberry Pi Zero 2 W y la Pi 4 que ya tenés, con sus cámaras.

> Los precios son referencias tomadas de búsquedas en MercadoLibre (octubre 2026). Cambian todas las semanas: verificalos al comprar.

## Decisiones tomadas y avance (25/10/2026)

- **GPU NVIDIA:** hay una, para el entrenamiento de la fase 4. Todavía no hace falta conectarla.
- **Modificar los servos:** sí. Se abren, se suelda el cable del potenciómetro y se cortan orejas donde hace falta.
- **Fase 0:** arranca. Lista de compras en [COMPRAS-FASE0.md](COMPRAS-FASE0.md).
- **Modelo 3D v0 terminado** (ver [README](README.md)):
  - muslo y canilla de **45 mm**, con un servo por tramo (el original tiene 42 mm);
  - todos los servos alimentados a **6,0 V**, porque los MG90S no aguantan más;
  - orejas de servo cortadas donde chocan;
  - cuello de 2 articulaciones;
  - cámara en la frente;
  - masa estimada **1,22 kg** (cabeza al estilo del original: cúpula, ojo amarillo y pico ancho que abre como tapa).
- **Torques del modelo nuevo (MuJoCo):** parado, el tobillo usa el 18 % del máximo del MG996R. En el peor caso (una pata, sin desplazar el cuerpo), la **cadera roll llega al 40 %**. Es la articulación a vigilar en la fase 0 y en el entrenamiento.

---

## 0. Resumen

- **El cambio de fondo son los servos.** Los servos del original (Feetech HD-1910 de bus serie, con encoder) no se consiguen en Argentina y son caros. Se reemplazan por **servos PWM comunes**: MG996R en las patas y MG90S en la cabeza.
- Para que la política de aprendizaje por refuerzo (RL) sepa dónde está cada articulación, se **agrega lectura de posición abriendo el servo**: se suelda un cable al potenciómetro interno y se lee con un conversor analógico-digital (ADC). Es una modificación conocida y casi gratis.
- **Los MG996R son más grandes que los HD-1910, así que hay que rediseñar patas y tronco.** De lo que imprimiste se puede reaprovechar poco; probablemente la cabeza.
- **Lo que no cambia:** la idea del pato, los 10 grados de libertad de las patas, el control por política RL a 50 Hz, la interfaz web y la arquitectura de software que ya armamos (`hal/`).
- **Costo estimado en electrónica, servos, batería y filamento: ARS 210.000–290.000.** Ver §4.
- **El riesgo principal es el torque.** Por eso el plan arranca con una prueba de banco barata, antes de comprar todo (§5, fase 0).

## 1. Por qué cambian los servos, con números

Simulé el modelo del XGO-Duck (MuJoCo, `xgoduck_rl`, 0,80 kg) y medí el torque estático que exige cada articulación:

| Articulación | Parado en 2 patas | Apoyado en 1 pata |
| --- | ---: | ---: |
| Cadera lateral (hip roll) | 0,3 kg·cm | **2,1 kg·cm** |
| Rodilla | 0,6 kg·cm | **1,5 kg·cm** |
| Cadera adelante-atrás (hip pitch) | 0,3 kg·cm | 0,3 kg·cm |
| Tobillo | 0,0 kg·cm | 0,3 kg·cm |
| Cuello (pitch) | 0,35 kg·cm | 0,35 kg·cm |

Al caminar, los picos son entre 2 y 3 veces esos valores, es decir **4–6 kg·cm**. El entrenamiento original limita el torque a **10,6 kg·cm**.

| Servo | Precio aprox. en ML | Torque máximo (stall) | Peso | Lectura de posición | Veredicto |
| --- | ---: | ---: | ---: | --- | --- |
| HD-1910 (original) | importado, ~31 € | 15 kg·cm a 7,4 V | 21 g | encoder | caro, no se consigue |
| **MG996R** | ~ARS 12.000 | 9,4–11 kg·cm (los clones, menos) | 55 g | hay que agregarla | **patas** |
| **MG90S** (engranajes de metal) | ~ARS 4.500–7.000 | 1,8–2,2 kg·cm | 13 g | hay que agregarla | **cabeza, boca y cadera yaw** |
| DS3218 | ~ARS 70.000 | 20 kg·cm | 60 g | hay que agregarla | caro |

- **Con MG90S en las patas no camina:** su torque máximo queda por debajo de los picos.
- **Con MG996R hay margen, pero menos que en el original.** El robot va a pesar unos 1,1 kg contra 0,8 kg, así que hay que entrenar la política **con los límites reales del MG996R** para que aprenda un paso que el servo pueda dar. Eso es posible porque el RL se reentrena igual (§6).

## 2. Diseño propuesto

### Servos (13 en lugar de 15)

| Zona | Articulaciones | Servo | Cant. |
| --- | --- | --- | ---: |
| Patas | cadera lateral, cadera adelante-atrás, rodilla, tobillo (×2 patas) | MG996R | 8 |
| Patas | cadera yaw, para girar (×2) | MG90S | 2 |
| Cuello y cabeza | yaw y pitch (antes eran 4 articulaciones) | MG90S | 2 |
| Pico | boca | MG90S | 1 |

Pasar el cuello de 4 a 2 articulaciones **saca peso donde más complica**: en el original, la cabeza pesa casi un tercio del robot. Además ahorra 2 servos.

### Electrónica

```
LiPo 2S ── fusible ── llave ─┬─ XL4016 (6,0 V) ── pata izquierda (4× MG996R + 1× MG90S)
                             ├─ XL4016 (6,0 V) ── pata derecha + cabeza
                             └─ Mini560 (5 V) ── Pi Zero 2 W ── cámara (en la cabeza)
Pi Zero 2 W ── I²C ── PCA9685 (13 señales PWM) + MPU-6050 (IMU)
            └─ SPI ── 2× MCP3008 (13 potenciómetros + tensión de batería + tensión de servos)
```

- **PCA9685:** genera los pulsos PWM con su propio reloj, sin cargar a la Pi. Lo configuramos a unos 150 Hz para tener más resolución y respuesta; en la fase 0 se verifica que los servos lo toleren.
- **MCP3008:** 16 canales de 10 bits a cientos de miles de muestras por segundo, así que leer los 13 potenciómetros a 100 Hz sobra.
  - Cada canal lleva un divisor resistivo y un capacitor, porque el potenciómetro llega a la tensión de los servos (6,0 V).
  - Un canal mide la tensión de los servos para normalizar la lectura de posición.
- **Dos reguladores XL4016, uno por pata:** un MG996R trabado consume más de 2 A. Dos reguladores de 8 A pico son más baratos y confiables que uno grande.
- **Batería LiPo 2S de 1500–2200 mAh y 20C o más:** más liviana y con más corriente que un pack de 18650. Requiere cargador balanceador y bolsa ignífuga.

### Qué hace cada Raspberry Pi

| Placa | Rol |
| --- | --- |
| **Pi Zero 2 W (a bordo)** | Control a 100 Hz, política a 50 Hz, interfaz web y transmisión de la cámara de la cabeza en baja resolución. |
| **Pi 4 (afuera del robot)** | 1) Banco de pruebas de servos en la fase 0. 2) "Cerebro externo" para visión pesada: recibe el video del pato, detecta caras, pelotas o personas y le manda órdenes de movimiento a la API `/api/command` que ya existe. Para órdenes de alto nivel, la latencia por Wi-Fi no es problema. |

La Pi 4 también se podría llevar a bordo, pero pesa más, consume entre 3 y 6 W y calienta. Conviene dejarla afuera.

## 3. Mejoras respecto al original

1. **Diseño en código, paramétrico.** Patas y tronco se generan con un script, igual que la placa adaptadora. El mismo script genera las STL **y el modelo de simulación** (masas e inercias reales). Si cambiás de servo, regenerás robot y simulación juntos.
2. **Un solo tipo de tornillo (M3) y rulemanes comunes** (608ZZ de skate, 6700ZZ). Se acaban los 204 tornillos M2 autorroscantes y los rulemanes de medida rara.
3. **Medición de batería y de la tensión de los servos:** aviso de batería baja y apagado seguro. El original no lo tiene.
4. **Cámara en la cabeza:** seguimiento de caras o de una pelota con el cuello, video en la interfaz web, y "seguime" usando la Pi 4.
5. **Paso a paso sin RL primero:** un caminar programado a mano (osciladores) para tener el pato andando antes de entrenar.
6. **Todo con licencia abierta propia:** código Apache-2.0 y diseño CERN-OHL o CC BY-SA, sin depender de los archivos del original.

## 4. Presupuesto aproximado (MercadoLibre)

| Cant. | Pieza | ARS aprox. |
| ---: | --- | ---: |
| 8 | Servo MG996R (los packs salen más baratos por unidad) | 90.000–100.000 |
| 5 | Servo MG90S de metal | 23.000–30.000 |
| 1 | PCA9685 (16 canales PWM, I²C) | 4.000–8.000 |
| 2 | MCP3008 (ADC de 8 canales, SPI) | 6.000–12.000 |
| 1 | GY-521 (MPU-6050) | 3.000–5.000 |
| 2 | Regulador XL4016 de 8 A | 12.000–18.000 |
| 1 | Mini560 de 5 V | 3.000–5.000 |
| 1 | LiPo 2S de 1500–2200 mAh, 20C o más | 25.000–35.000 |
| 1 | Cargador balanceador (tipo B3 o B6) | 10.000–25.000 |
| — | Rulemanes 608ZZ y 6700ZZ, tornillos M3, cables, resistencias, capacitores, llave, fusible, conectores | 15.000–25.000 |
| ~1 kg | Filamento PLA o PETG | 20.000–25.000 |
| | **Total** | **≈ 210.000–290.000** |

La Pi Zero 2 W, la Pi 4 y las cámaras ya las tenés. Para comparar: solo los 15 servos originales importados cuestan unos 470 €, y el kit oficial USD 352 más la importación.

## 5. Fases, con controles antes de seguir

| Fase | Qué se hace | Quién | Para pasar a la siguiente |
| --- | --- | --- | --- |
| **0. Banco de servos** (≈ ARS 55.000–60.000) | Comprar 2 MG996R, 2 MG90S, PCA9685, MCP3008, MPU-6050 y un XL4016. Abrir un servo de cada tipo y soldar el cable del potenciómetro. En la Pi 4: medir torque con un brazo y peso conocido, zona muerta, juego, velocidad, ruido de la lectura y temperatura a 150 Hz. Con eso se identifica el modelo del servo para la simulación (BAM). | Vos armás, yo escribo los scripts de medición. | MG996R: aguanta **6 kg·cm durante 60 s** a 6,0 V sin recalentar. Lectura del potenciómetro con ruido **menor a 0,5°**. Si no lo cumple, se evalúa otro servo antes de gastar más. |
| **1. Diseño en código y simulación** | Patas y tronco para MG996R, cabeza de 2 articulaciones, modelo MuJoCo generado automáticamente. Simular parado, en una pata y el paso programado, con los límites medidos. | Yo | En simulación, torque pico por debajo del **60 %** del torque máximo medido. |
| **2. Una pata en el banco** | Imprimir una pata, cargarla con medio robot de peso (bolsa de arena) y hacer sentadillas y pasos. | Vos imprimís y armás, yo hago el software. | Sin juego excesivo, sin calentamiento y con una lectura de posición que coincida con la simulación. |
| **3. Pato completo, sin RL** | Armado completo, nueva capa de hardware en `hal/` (PCA9685 + MCP3008 + MPU-6050), calibración, pose y paso programado. | Los dos | Se para solo y da pasos programados sin caerse. |
| **4. RL** | Entrenar caminar y levantarse con el modelo identificado y variaciones aleatorias (fork de `xgoduck_rl`, que es Apache-2.0). Exportar a ONNX y correrlo en la Pi Zero. | Yo preparo todo; el entrenamiento necesita **GPU NVIDIA**. | Camina con la política RL. |
| **5. Cámara y extras** | Seguimiento con la cabeza, transmisión de video, "seguime" con la Pi 4, aviso de batería. | Yo, con tus pruebas | — |

## 6. Software

- **Se reutiliza lo que ya hicimos:**
  - el proceso de hardware separado con prioridad de tiempo real;
  - las reglas de seguridad (orden vencida = sin torque, apagado seguro);
  - la interfaz web y la API;
  - el simulador y los tests.
- **Cambia el backend de hardware:** sale el bus Feetech y entran PCA9685 + MCP3008 + MPU-6050.
- **El runtime de Luwu no publica licencia,** así que no podemos distribuir una versión modificada. Para la nueva cantidad de articulaciones escribimos un runtime propio y liviano. La parte de política y filtros es chica.
- **Entrenamiento:** `mjlab` necesita una **GPU NVIDIA con CUDA**. Si no tenés, se alquila por hora: hay servicios a unos USD 0,3–0,6 por hora y cada política lleva algunas horas.

## 7. Qué se reutiliza de lo ya impreso

- **Probablemente:** la cabeza (`head_shell`, `jaw`, ojos, pico de TPU), sobre un cuello nuevo de 2 articulaciones. Conviene reimprimirla más liviana: 2 paredes y relleno bajo.
- **No sirve:** patas, caderas y soportes de servos, porque son para el HD-1910.
- **La placa adaptadora de la Pi:** se rediseña dentro del tronco nuevo.

## 8. Riesgos

| Riesgo | Mitigación |
| --- | --- |
| MG996R truchos, con menos torque | Medir en la fase 0. Comprar a vendedores con buenas opiniones y reemplazar los que salgan malos. |
| La zona muerta y el juego de los PWM dificultan pasar de simulación a realidad | Modelar el servo con datos reales (BAM), agregar variaciones aleatorias en el entrenamiento, y usar 150 Hz para ganar resolución. |
| El potenciómetro es ruidoso | Capacitor en cada canal, filtrado, y la velocidad se estima filtrando, no derivando en crudo. |
| Corriente en picos, la Pi se reinicia | Reguladores separados para la Pi y para cada pata. Capacitores. Medir la tensión. |
| LiPo | Cargador balanceador, bolsa ignífuga y corte por tensión baja medido por el ADC. |
| Peso de la cabeza | Cuello de 2 articulaciones e impresión liviana. |

## 9. Decisiones

Respondidas el 25/10/2026: hay GPU NVIDIA, se modifican los servos y arranca la fase 0. Ver "Decisiones tomadas y avance", al principio de este documento.
