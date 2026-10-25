"""Capas de arriba de la política de marcha: ir de un punto a otro y seguir un objeto con la cámara.

La política solo entiende un comando [vx, vy, giro]. Estas dos clases lo generan:

  Navegador   lista de puntos (x, y) en el mundo -> comando que apunta al siguiente y avanza.
              En la simulación la posición y el rumbo son los reales; en el robot van a salir de
              la odometría (pasos contados) o de un marcador visto por la cámara.
  Seguidor    posición del objeto en la imagen (u, v, tamaño) -> ángulos de la cabeza que lo
              centran, y comando del cuerpo para girar hacia él y acercarse hasta una distancia.
              En el robot la detección la hace OpenCV por color; acá la da env.camara_pixel().
"""
from __future__ import annotations

import math

import numpy as np

from entorno import boca_max


def _acotar(x, a, b):
    return max(a, min(b, x))


def _envolver(ang):
    return (ang + math.pi) % (2 * math.pi) - math.pi


class Navegador:
    def __init__(self, puntos, tolerancia=0.10, vx_max=0.2, giro_max=0.6):
        self.puntos = [np.asarray(p, float) for p in puntos]
        self.i = 0
        self.tolerancia, self.vx_max, self.giro_max = tolerancia, vx_max, giro_max

    @property
    def terminado(self):
        return self.i >= len(self.puntos)

    def comando(self, posicion, rumbo):
        """Comando [vx, vy, giro] para ir al punto actual; [0, 0, 0] cuando no queda ninguno."""
        while not self.terminado and np.linalg.norm(self.puntos[self.i] - posicion) < self.tolerancia:
            self.i += 1
        if self.terminado:
            return [0.0, 0.0, 0.0]
        d = self.puntos[self.i] - posicion
        error = _envolver(math.atan2(d[1], d[0]) - rumbo)
        giro = _acotar(2.0 * error, -self.giro_max, self.giro_max)
        # Primero apunta, después avanza; cerca del punto va más despacio para no pasarse.
        vx = self.vx_max * _acotar(1.0 - abs(error) / 0.8, 0.0, 1.0) * _acotar(np.linalg.norm(d) / 0.2, 0.3, 1.0)
        return [float(vx), 0.0, float(giro)]


class Seguidor:
    """Centra un objeto en la cámara moviendo la cabeza, y si queda muy de costado o lejos, el cuerpo.

    radio_objeto: radio real del objeto (m), para estimar la distancia por su tamaño en la imagen.
    """

    def __init__(self, ancho=640, alto=480, fovy_grados=49.0, radio_objeto=0.03, distancia=0.45,
                 ganancia=0.6, yaw_max=1.5, pitch_min=-0.5, pitch_max=0.6):
        self.ancho, self.alto = ancho, alto
        self.f = (alto / 2) / math.tan(math.radians(fovy_grados) / 2)
        self.radio_objeto, self.distancia, self.ganancia = radio_objeto, distancia, ganancia
        self.yaw_max, self.pitch_min, self.pitch_max = yaw_max, pitch_min, pitch_max
        self.yaw = self.pitch = 0.0
        self.perdido = 0

    def paso(self, deteccion, yaw_actual, pitch_actual):
        """deteccion = (u, v, radio_px) del objeto en la imagen, o None si no se ve.
        Devuelve ((yaw, pitch, boca) para la cabeza, comando [vx, vy, giro] para el cuerpo)."""
        if deteccion is None:
            self.perdido += 1
            if self.perdido > 25:                       # medio segundo sin verlo: barre con la cabeza
                self.yaw = _acotar(self.yaw + (0.6 if (self.perdido // 75) % 2 == 0 else -0.6) / 50, -1.2, 1.2)
                self.pitch = 0.1
            return (self.yaw, self.pitch, 0.0), [0.0, 0.0, 0.0]
        self.perdido = 0
        u, v, radio_px = deteccion
        # Error angular del objeto respecto del centro de la imagen (modelo de agujero de alfiler).
        ex = math.atan((u - self.ancho / 2) / self.f)    # positivo = a la derecha de la imagen
        ey = math.atan((v - self.alto / 2) / self.f)     # positivo = abajo
        self.yaw = _acotar(yaw_actual - self.ganancia * ex, -self.yaw_max, self.yaw_max)   # yaw positivo = izquierda
        self.pitch = _acotar(pitch_actual + self.ganancia * ey, self.pitch_min, self.pitch_max)   # pitch positivo = abajo
        dist = self.f * self.radio_objeto / max(radio_px, 1.0)
        # Cuerpo: gira hacia donde mira la cabeza si quedó muy de costado; avanza si está lejos.
        giro = _acotar(1.5 * self.yaw, -0.6, 0.6) if abs(self.yaw) > 0.35 else 0.0
        vx = _acotar(0.5 * (dist - self.distancia), -0.05, 0.2) if abs(self.yaw) < 0.9 else 0.0
        if abs(vx) < 0.03:
            vx = 0.0
        return (self.yaw, self.pitch, min(0.3, boca_max(self.pitch)) if dist < self.distancia + 0.05 else 0.0), \
            [float(vx), 0.0, float(giro)]
