# Torques estáticos (MuJoCo)

Masa del modelo: **1.18 kg**. Torque máximo a 6 V: MG996R 11.0 kg·cm, MG90S 2.2 kg·cm. Caminando, los picos suelen ser 2 a 3 veces el valor estático: conviene que el estático no pase del 30–40 % del máximo.

| Articulación | Servo | parado (rodillas 40°) | cuclillas (rodillas 90°) | en una pata (apoya la derecha) |
| --- | --- | ---: | ---: | ---: |
| cadera_yaw_izq | MG90S | 0.03 kg·cm (1 %) | 0.03 kg·cm (1 %) | 0.00 kg·cm (0 %) |
| cadera_roll_izq | MG996R | 0.08 kg·cm (1 %) | 0.13 kg·cm (1 %) | 0.12 kg·cm (1 %) |
| cadera_pitch_izq | MG996R | 1.35 kg·cm (12 %) | 1.03 kg·cm (9 %) | 0.21 kg·cm (2 %) |
| rodilla_izq | MG996R | 0.80 kg·cm (7 %) | 1.54 kg·cm (14 %) | 0.21 kg·cm (2 %) |
| tobillo_izq | MG996R | 1.95 kg·cm (18 %) | 1.92 kg·cm (18 %) | 0.01 kg·cm (0 %) |
| cadera_yaw_der | MG90S | 0.03 kg·cm (1 %) | 0.03 kg·cm (1 %) | 0.01 kg·cm (0 %) |
| cadera_roll_der | MG996R | 0.08 kg·cm (1 %) | 0.14 kg·cm (1 %) | 4.41 kg·cm (40 %) |
| cadera_pitch_der | MG996R | 1.35 kg·cm (12 %) | 1.03 kg·cm (9 %) | 0.38 kg·cm (3 %) |
| rodilla_der | MG996R | 0.80 kg·cm (7 %) | 1.53 kg·cm (14 %) | 1.21 kg·cm (11 %) |
| tobillo_der | MG996R | 1.95 kg·cm (18 %) | 1.92 kg·cm (17 %) | 0.49 kg·cm (4 %) |
| cuello_yaw | MG90S | 0.00 kg·cm (0 %) | 0.00 kg·cm (0 %) | 0.00 kg·cm (0 %) |
| cuello_pitch | MG90S | 0.45 kg·cm (20 %) | 0.46 kg·cm (21 %) | 0.17 kg·cm (8 %) |
| boca | MG90S | 0.02 kg·cm (1 %) | 0.03 kg·cm (1 %) | 0.01 kg·cm (0 %) |
