"""Gestos programados de la cabeza (cuello yaw, cuello pitch, boca). No se entrenan: son los tres
MG90S y alcanza con trayectorias suaves. Respetan los topes y la envolvente boca/pitch del modelo.

Cada gesto es una función gesto(t) -> (yaw, pitch, boca) en radianes, con t en segundos desde que
empezó, y tiene un atributo `duracion`. `Cabeza` los encadena y devuelve el objetivo actual.

  from cabeza import Cabeza, GESTOS
  cabeza = Cabeza(); cabeza.hacer('mirar_arriba_y_abrir')
  yaw, pitch, boca = cabeza.objetivo(t)
"""
from __future__ import annotations

import math

from entorno import boca_max


def _rampa(t, t0, t1, a, b):
    """Interpola de a a b entre t0 y t1 con arranque y frenado suaves (coseno)."""
    if t <= t0:
        return a
    if t >= t1:
        return b
    s = (t - t0) / (t1 - t0)
    return a + (b - a) * (0.5 - 0.5 * math.cos(math.pi * s))


def mirar_arriba_y_abrir(t):
    """Levanta la cabeza, abre el pico entero, lo cierra y vuelve. Es el gesto de "cuac"."""
    pitch = _rampa(t, 0.0, 0.6, 0.0, -0.5) if t < 2.4 else _rampa(t, 2.4, 3.0, -0.5, 0.0)
    boca = _rampa(t, 0.5, 0.9, 0.0, 0.3) if t < 1.9 else _rampa(t, 1.9, 2.3, 0.3, 0.0)
    return 0.0, pitch, boca
mirar_arriba_y_abrir.duracion = 3.0


def mirar_alrededor(t):
    """Gira la cabeza a un lado y al otro, mirando un poco hacia abajo."""
    yaw = 1.0 * math.sin(2 * math.pi * t / 3.0)
    pitch = _rampa(t, 0.0, 0.5, 0.0, 0.25) if t < 2.5 else _rampa(t, 2.5, 3.0, 0.25, 0.0)
    return yaw, pitch, 0.0
mirar_alrededor.duracion = 3.0


def cuac(t):
    """Abre y cierra el pico dos veces, rápido, con la cabeza derecha."""
    boca = 0.3 * max(0.0, math.sin(2 * math.pi * t / 0.6))
    return 0.0, 0.0, boca if t < 1.2 else 0.0
cuac.duracion = 1.2


def mirar_abajo(t):
    """Mira al piso (la cámara ve justo delante de los pies). Con la cabeza baja la boca va cerrada."""
    pitch = _rampa(t, 0.0, 0.6, 0.0, 0.6) if t < 2.4 else _rampa(t, 2.4, 3.0, 0.6, 0.0)
    return 0.0, pitch, 0.0
mirar_abajo.duracion = 3.0


GESTOS = {f.__name__: f for f in (mirar_arriba_y_abrir, mirar_alrededor, cuac, mirar_abajo)}


class Cabeza:
    def __init__(self):
        self.cola = []
        self.actual = None
        self.t0 = 0.0
        self.ultimo = (0.0, 0.0, 0.0)

    def hacer(self, *nombres):
        self.cola += [GESTOS[n] for n in nombres]

    def ocupada(self):
        return self.actual is not None or bool(self.cola)

    def objetivo(self, t):
        """Objetivo (yaw, pitch, boca) en el instante t, con la boca acotada por la envolvente."""
        if self.actual is None and self.cola:
            self.actual, self.t0 = self.cola.pop(0), t
        if self.actual is not None:
            yaw, pitch, boca = self.actual(t - self.t0)
            if t - self.t0 >= self.actual.duracion:
                self.actual = None
            self.ultimo = (yaw, pitch, min(boca, boca_max(pitch)))
        return self.ultimo
