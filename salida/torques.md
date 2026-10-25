# Torques estáticos (MuJoCo)

Masa del modelo: **1.36 kg**. Torque máximo a 6 V: MG996R 11.0 kg·cm, MG90S 2.2 kg·cm. Caminando, los picos suelen ser 2 a 3 veces el valor estático: conviene que el estático no pase del 30–40 % del máximo.

| Articulación | Servo | parado (rodillas 40°) | cuclillas (rodillas 63°) | en una pata (apoya la derecha) |
| --- | --- | ---: | ---: | ---: |
| cadera_yaw_izq | MG996R | 0.00 kg·cm (0 %) | 0.00 kg·cm (0 %) | 0.00 kg·cm (0 %) |
| cadera_roll_izq | MG996R | 0.15 kg·cm (1 %) | 0.16 kg·cm (2 %) | 0.15 kg·cm (1 %) |
| cadera_pitch_izq | MG996R | 0.54 kg·cm (5 %) | 0.55 kg·cm (5 %) | 0.39 kg·cm (4 %) |
| rodilla_izq | MG996R | 0.08 kg·cm (1 %) | 0.36 kg·cm (3 %) | 0.03 kg·cm (0 %) |
| tobillo_izq | MG996R | 0.63 kg·cm (6 %) | 0.68 kg·cm (6 %) | 0.06 kg·cm (0 %) |
| cadera_yaw_der | MG996R | 0.00 kg·cm (0 %) | 0.00 kg·cm (0 %) | 0.01 kg·cm (0 %) |
| cadera_roll_der | MG996R | 0.07 kg·cm (1 %) | 0.06 kg·cm (1 %) | 5.63 kg·cm (51 %) |
| cadera_pitch_der | MG996R | 0.57 kg·cm (5 %) | 0.57 kg·cm (5 %) | 1.50 kg·cm (14 %) |
| rodilla_der | MG996R | 0.08 kg·cm (1 %) | 0.37 kg·cm (3 %) | 0.12 kg·cm (1 %) |
| tobillo_der | MG996R | 0.66 kg·cm (6 %) | 0.71 kg·cm (6 %) | 1.59 kg·cm (14 %) |
| cuello_yaw | MG90S | 0.00 kg·cm (0 %) | 0.00 kg·cm (0 %) | 0.00 kg·cm (0 %) |
| cuello_pitch | MG90S | 0.21 kg·cm (10 %) | 0.21 kg·cm (10 %) | 0.21 kg·cm (10 %) |
| boca | MG90S | 0.23 kg·cm (10 %) | 0.23 kg·cm (10 %) | 0.23 kg·cm (10 %) |
