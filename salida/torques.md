# Torques estáticos (MuJoCo)

Masa del modelo: **1.26 kg**. Torque máximo a 6 V: MG996R 11.0 kg·cm, MG90S 2.2 kg·cm. Caminando, los picos suelen ser 2 a 3 veces el valor estático: conviene que el estático no pase del 30–40 % del máximo.

| Articulación | Servo | parado (rodillas 40°) | cuclillas (rodillas 90°) | en una pata (apoya la derecha) |
| --- | --- | ---: | ---: | ---: |
| cadera_yaw_izq | MG90S | 0.02 kg·cm (1 %) | 0.07 kg·cm (3 %) | 0.00 kg·cm (0 %) |
| cadera_roll_izq | MG996R | 0.16 kg·cm (1 %) | 0.19 kg·cm (2 %) | 0.16 kg·cm (2 %) |
| cadera_pitch_izq | MG996R | 1.78 kg·cm (16 %) | 1.14 kg·cm (10 %) | 0.06 kg·cm (0 %) |
| rodilla_izq | MG996R | 0.89 kg·cm (8 %) | 1.05 kg·cm (10 %) | 0.24 kg·cm (2 %) |
| tobillo_izq | MG996R | 2.18 kg·cm (20 %) | 1.58 kg·cm (14 %) | 0.01 kg·cm (0 %) |
| cadera_yaw_der | MG90S | 0.02 kg·cm (1 %) | 0.07 kg·cm (3 %) | 0.01 kg·cm (1 %) |
| cadera_roll_der | MG996R | 0.16 kg·cm (1 %) | 0.19 kg·cm (2 %) | 5.22 kg·cm (47 %) |
| cadera_pitch_der | MG996R | 1.79 kg·cm (16 %) | 1.14 kg·cm (10 %) | 0.87 kg·cm (8 %) |
| rodilla_der | MG996R | 0.89 kg·cm (8 %) | 1.06 kg·cm (10 %) | 0.38 kg·cm (4 %) |
| tobillo_der | MG996R | 2.19 kg·cm (20 %) | 1.59 kg·cm (14 %) | 0.85 kg·cm (8 %) |
| cuello_yaw | MG90S | 0.00 kg·cm (0 %) | 0.00 kg·cm (0 %) | 0.00 kg·cm (0 %) |
| cuello_pitch | MG90S | 0.65 kg·cm (30 %) | 0.39 kg·cm (18 %) | 0.20 kg·cm (9 %) |
| boca | MG90S | 0.40 kg·cm (18 %) | 0.42 kg·cm (19 %) | 0.22 kg·cm (10 %) |
