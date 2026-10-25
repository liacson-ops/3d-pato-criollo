# Compras de la fase 0: banco de pruebas de servos

Con esto se miden 2 MG996R y 2 MG90S antes de comprar el resto: torque, zona muerta, juego, velocidad, ruido de la lectura del potenciómetro y temperatura. Todo se conecta a la **Raspberry Pi 4**, que ya tenés.

Los links son **búsquedas** de MercadoLibre Argentina: elegí un vendedor con buenas opiniones. Precios aproximados de octubre de 2026.

| Cant. | Pieza | Búsqueda | ARS aprox. |
| ---: | --- | --- | ---: |
| 2 | Servo **MG996R** (que diga "engranajes metálicos") | [mg996r](https://listado.mercadolibre.com.ar/servo-motor-mg996r) | 24.000 |
| 2 | Servo **MG90S** de metal | [mg90s](https://listado.mercadolibre.com.ar/mg90s-servos) | 11.000 |
| 1 | Módulo **PCA9685** (16 canales PWM, I²C) | [pca9685](https://listado.mercadolibre.com.ar/pca9685) | 5.000 |
| 1 | **MCP3008** (ADC de 8 canales, SPI, DIP-16) | [mcp3008](https://listado.mercadolibre.com.ar/mcp-3008) | 5.000 |
| 1 | **GY-521** (MPU-6050). En el banco va en el brazo y mide el ángulo real | [gy-521 mpu6050](https://listado.mercadolibre.com.ar/mpu-6050) | 4.000 |
| 1 | Regulador **XL4016** de 8 A, regulable | [xl4016](https://listado.mercadolibre.com.ar/modulo-step-down-xl4016-8a) | 7.000 |
| 1 | Fuente de **12 V y 5 A** (tipo tira LED), si no tenés una | [fuente 12v 5a](https://listado.mercadolibre.com.ar/fuente-12v-5a) | 10.000 |
| 1 | Protoboard de 830 puntos + cables Dupont M-M y M-H | [protoboard 830 kit cables](https://listado.mercadolibre.com.ar/protoboard-830-cables-dupont) | 6.000 |
| 1 | Kit surtido de resistencias (hacen falta 47 kΩ y 33 kΩ) | [kit resistencias](https://listado.mercadolibre.com.ar/kit-resistencias-surtidas) | 4.000 |
| 10 | Capacitores cerámicos de 10 nF (103) y 100 nF (104) | [capacitor ceramico 103](https://listado.mercadolibre.com.ar/capacitor-ceramico-103) | 1.500 |
| 1 | Cable fino para soldar en los potenciómetros (wire-wrap 30 AWG o cable plano) | [cable wire wrap 30awg](https://listado.mercadolibre.com.ar/cable-wire-wrap-30awg) | 3.000 |
| 1 | Termocontraíble fino | [termocontraible 2mm](https://listado.mercadolibre.com.ar/termocontraible-2mm) | 1.500 |
| 1 | *Opcional:* sensor de temperatura **DS18B20** + resistencia de 4,7 kΩ | [ds18b20](https://listado.mercadolibre.com.ar/ds18b20) | 2.500 |
| — | Tornillos M3 × 12 con tuercas, tornillo M6 × 30 con tuercas (son las pesas del brazo) | [tornillos m3 kit](https://listado.mercadolibre.com.ar/kit-tornillos-m3) | 4.000 |
| | **Total** | | **≈ 88.000** (≈ 78.000 sin la fuente) |

Esto queda más caro que lo que calculé en el plan, porque se suman la fuente, el protoboard y el kit de resistencias. Todo eso se reutiliza en el robot.

**Herramientas:** soldador fino, multímetro, balanza de cocina (para pesar el brazo y las pesas) y un sargento chico para fijar el banco al borde de la mesa.

**Lo que viene en el repo para esta fase**, en preparación:
- soporte imprimible para cada servo y brazo péndulo con lugar para el GY-521;
- scripts de medición para la Pi 4. Guardan los datos en el formato de BAM (Rhoban), así sirven directo para identificar el modelo del servo que usa el entrenamiento.
