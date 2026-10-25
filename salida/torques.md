# Torques estáticos (MuJoCo)

Masa del modelo: **1.22 kg**. Torque máximo a 6 V: MG996R 11.0 kg·cm, MG90S 2.2 kg·cm. Caminando, los picos suelen ser 2 a 3 veces el valor estático: conviene que el estático no pase del 30–40 % del máximo.

| Articulación | Servo | parado (rodillas 40°) | cuclillas (rodillas 90°) | en una pata (apoya la derecha) |
| --- | --- | ---: | ---: | ---: |
| cadera_yaw_izq | MG90S | 0.06 kg·cm (3 %) | 0.08 kg·cm (3 %) | 0.00 kg·cm (0 %) |
| cadera_roll_izq | MG996R | 0.06 kg·cm (0 %) | 0.04 kg·cm (0 %) | 0.13 kg·cm (1 %) |
| cadera_pitch_izq | MG996R | 1.33 kg·cm (12 %) | 1.24 kg·cm (11 %) | 0.20 kg·cm (2 %) |
| rodilla_izq | MG996R | 0.95 kg·cm (9 %) | 0.57 kg·cm (5 %) | 0.22 kg·cm (2 %) |
| tobillo_izq | MG996R | 1.27 kg·cm (12 %) | 1.11 kg·cm (10 %) | 0.01 kg·cm (0 %) |
| cadera_yaw_der | MG90S | 0.06 kg·cm (3 %) | 0.08 kg·cm (4 %) | 0.02 kg·cm (1 %) |
| cadera_roll_der | MG996R | 0.06 kg·cm (0 %) | 0.04 kg·cm (0 %) | 4.61 kg·cm (42 %) |
| cadera_pitch_der | MG996R | 1.33 kg·cm (12 %) | 1.24 kg·cm (11 %) | 1.14 kg·cm (10 %) |
| rodilla_der | MG996R | 0.95 kg·cm (9 %) | 0.57 kg·cm (5 %) | 0.48 kg·cm (4 %) |
| tobillo_der | MG996R | 1.27 kg·cm (12 %) | 1.11 kg·cm (10 %) | 1.33 kg·cm (12 %) |
| cuello_yaw | MG90S | 0.00 kg·cm (0 %) | 0.00 kg·cm (0 %) | 0.00 kg·cm (0 %) |
| cuello_pitch | MG90S | 0.48 kg·cm (22 %) | 0.48 kg·cm (22 %) | 0.20 kg·cm (9 %) |
| boca | MG90S | 0.15 kg·cm (7 %) | 0.12 kg·cm (6 %) | 0.22 kg·cm (10 %) |
