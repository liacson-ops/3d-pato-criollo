"""Componentes comprados, con sus medidas y masas reales (mm, g).

Las medidas son las de las hojas de datos típicas. Los clones varían ±0,5 mm:
la fase 0 las mide con calibre antes de imprimir las piezas definitivas.

Marco local de un servo: el eje de salida es +Z y pasa por el origen, la cara
del lado del eje (tapa) está en z = 0 y la caja queda en z < 0. El largo de la
caja va sobre +X, con el eje a `eje_desde_punta` del extremo -X.
"""
from dataclasses import dataclass

import numpy as np
from manifold3d import Manifold

SEG = 40


def caja(x0, x1, y0, y1, z0, z1):
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cilindro_z(r, z0, z1, x=0.0, y=0.0, seg=SEG):
    return Manifold.cylinder(z1 - z0, r, r, seg).translate([x, y, z0])


@dataclass(frozen=True)
class Servo:
    nombre: str
    largo: float          # L
    ancho: float          # W
    alto: float           # altura de la caja, sin el eje
    eje_desde_punta: float
    largo_orejas: float
    orejas_desde_tapa: float
    espesor_orejas: float
    r_buje: float
    alto_buje: float
    r_estria: float
    alto_estria: float
    r_horn: float
    espesor_horn: float
    masa: float
    stall_kgcm_6v: float
    vel_s_60: float       # segundos por 60° a 6 V

    @property
    def alto_horn(self):
        """Del plano de la tapa a la cara exterior del horn."""
        return self.alto_buje + self.alto_estria + self.espesor_horn

    def caja(self, orejas='ambas'):
        """orejas: 'ambas', 'eje' (se corta la del extremo lejano al eje), 'lejos' o 'ninguna'."""
        x0 = -self.eje_desde_punta
        x1 = x0 + self.largo
        cuerpo = caja(x0, x1, -self.ancho / 2, self.ancho / 2, -self.alto, 0)
        sobra = (self.largo_orejas - self.largo) / 2
        oz = -self.orejas_desde_tapa
        tramos = {'eje': [(x0 - sobra, x0)], 'lejos': [(x1, x1 + sobra)], 'ninguna': [],
                  'ambas': [(x0 - sobra, x0), (x1, x1 + sobra)]}[orejas]
        for a, b in tramos:
            cuerpo += caja(a, b, -self.ancho / 2, self.ancho / 2, oz - self.espesor_orejas, oz)
        return cuerpo + cilindro_z(self.r_buje, 0, self.alto_buje)

    def horn(self):
        z0 = self.alto_buje
        estria = cilindro_z(self.r_estria, z0, z0 + self.alto_estria)
        disco = cilindro_z(self.r_horn, z0 + self.alto_estria, self.alto_horn)
        return estria + disco


MG996R = Servo('MG996R', largo=40.7, ancho=19.7, alto=36.8, eje_desde_punta=10.2,
               largo_orejas=54.5, orejas_desde_tapa=7.6, espesor_orejas=2.5,
               r_buje=6.5, alto_buje=4.0, r_estria=2.9, alto_estria=2.0,
               r_horn=12.0, espesor_horn=2.5, masa=55.0, stall_kgcm_6v=11.0, vel_s_60=0.14)

MG90S = Servo('MG90S', largo=22.8, ancho=12.2, alto=22.7, eje_desde_punta=5.9,
              largo_orejas=32.5, orejas_desde_tapa=6.8, espesor_orejas=2.5,
              r_buje=5.9, alto_buje=4.0, r_estria=2.4, alto_estria=1.8,
              r_horn=7.5, espesor_horn=2.0, masa=13.4, stall_kgcm_6v=2.2, vel_s_60=0.10)


def marco(origen, z, x):
    """Matriz 3x4 que lleva el marco local (x, y, z) a origen con ejes x, z dados."""
    z = np.asarray(z, float) / np.linalg.norm(z)
    x = np.asarray(x, float) / np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z, np.asarray(origen, float)])


# Placas y módulos: (largo X, ancho Y, alto Z, masa g, color)
PI_ZERO_2W = (65.0, 30.0, 5.0, 10.0)
CAMARA_PI = (25.0, 24.0, 1.0, 3.0)        # placa; el lente es aparte
LIPO_2S_1500 = (86.0, 34.0, 15.0, 85.0)
PCA9685 = (62.5, 25.4, 12.0, 9.0)        # con los pines de los servos
XL4016 = (62.0, 42.0, 22.0, 35.0)        # con disipadores
MINI560 = (23.0, 18.0, 5.0, 2.0)
GY521 = (21.0, 16.0, 3.0, 2.0)
PLACA_MCP3008 = (40.0, 28.0, 10.0, 10.0)  # 2× MCP3008 + divisores en placa perforada
RULEMAN_608 = (22.0, 8.0, 7.0, 12.0)      # diámetro exterior, interior, ancho, masa
