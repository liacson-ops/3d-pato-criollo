# Torques estáticos (MuJoCo)

Masa del modelo: **1.37 kg**. Torque máximo a 6 V: MG996R 11.0 kg·cm, MG90S 2.2 kg·cm. Caminando, los picos suelen ser 2 a 3 veces el valor estático: conviene que el estático no pase del 30–40 % del máximo.

| Articulación | Servo | parado (rodillas 40°) | cuclillas (rodillas 90°) | en una pata (apoya la derecha) |
| --- | --- | ---: | ---: | ---: |
| cadera_yaw_izq | MG996R | 0.02 kg·cm (0 %) | 0.07 kg·cm (1 %) | 0.00 kg·cm (0 %) |
| cadera_roll_izq | MG996R | 0.17 kg·cm (2 %) | 0.19 kg·cm (2 %) | 0.16 kg·cm (1 %) |
| cadera_pitch_izq | MG996R | 1.85 kg·cm (17 %) | 1.22 kg·cm (11 %) | 0.06 kg·cm (0 %) |
| rodilla_izq | MG996R | 0.86 kg·cm (8 %) | 1.11 kg·cm (10 %) | 0.24 kg·cm (2 %) |
| tobillo_izq | MG996R | 2.23 kg·cm (20 %) | 1.64 kg·cm (15 %) | 0.01 kg·cm (0 %) |
| cadera_yaw_der | MG996R | 0.02 kg·cm (0 %) | 0.07 kg·cm (1 %) | 0.01 kg·cm (0 %) |
| cadera_roll_der | MG996R | 0.17 kg·cm (2 %) | 0.19 kg·cm (2 %) | 5.65 kg·cm (51 %) |
| cadera_pitch_der | MG996R | 1.85 kg·cm (17 %) | 1.22 kg·cm (11 %) | 1.03 kg·cm (9 %) |
| rodilla_der | MG996R | 0.86 kg·cm (8 %) | 1.12 kg·cm (10 %) | 0.33 kg·cm (3 %) |
| tobillo_der | MG996R | 2.23 kg·cm (20 %) | 1.64 kg·cm (15 %) | 1.01 kg·cm (9 %) |
| cuello_yaw | MG996R | 0.00 kg·cm (0 %) | 0.00 kg·cm (0 %) | 0.00 kg·cm (0 %) |
| cuello_pitch | MG996R | 0.65 kg·cm (6 %) | 0.43 kg·cm (4 %) | 0.22 kg·cm (2 %) |
| boca | MG996R | 0.53 kg·cm (5 %) | 0.54 kg·cm (5 %) | 0.24 kg·cm (2 %) |
