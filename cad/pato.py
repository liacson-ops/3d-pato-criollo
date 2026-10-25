#!/usr/bin/env python3
"""Pato Criollo v0: modelo paramétrico completo.

Genera, a partir de una sola descripción:
  salida/piezas_impresas/*.stl   piezas para imprimir (v0: para validar proporciones)
  salida/pato.glb                 modelo con colores y jerarquía de eslabones (visor)
  salida/robot.json               eslabones, articulaciones, piezas, pasos de armado
  salida/mjcf/pato.xml            modelo MuJoCo con masas e inercias calculadas
  salida/resumen.md               masas por pieza y total

Coordenadas en mm: X hacia adelante (pico), Y a la izquierda, Z arriba, suelo en
z = 0 con las patas estiradas (pose cero). Las piezas se modelan en esa pose y en
coordenadas de mundo; cada eslabón guarda el punto de su articulación (ancla).

  python3 cad/pato.py
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import trimesh
from manifold3d import Manifold

from componentes import (MG90S, MG996R, GY521, LIPO_2S_1500, MINI560, PCA9685, PI_ZERO_2W,
                         PLACA_MCP3008, RULEMAN_608, XL4016, Servo, caja, marco)

SALIDA = Path(__file__).resolve().parent.parent / 'salida'
SEG = 40
PLA = 1.24e-3   # g/mm³
TPU = 1.21e-3

# ---------------------------------------------------------------------------
# Medidas principales (mm)
# ---------------------------------------------------------------------------
Y_PATA = 38.0          # eje de cada pata respecto del centro
Z_TOBILLO = 34.0
Z_RODILLA = Z_TOBILLO + 45.0     # canilla de 45 mm (original: 42)
Z_CADERA = Z_RODILLA + 45.0      # muslo de 45 mm (original: 42)
Z_ROLL = Z_CADERA + 30.0
Z_PISO = (178.0, 186.0)
# Tronco: cajita redondeada, como el original (la electrónica va en estantes adentro).
TRONCO_X = (-50.0, 66.0)
TRONCO_Y = (-46.0, 46.0)
TRONCO_Z = (187.0, 284.0)
TRONCO_R = 12.0
Z_CORTE_TRONCO = 232.0
# Cabeza: cúpula tipo "pan" con el piso plano y pico ancho que abre como una tapa.
CAB_X = (-15.0, 115.0)
CAB_Y = (-52.0, 52.0)
Z_CAB = (305.0, 365.0)
R_CAB_ARRIBA, R_CAB_ABAJO = 26.0, 10.0
X_CUELLO = 40.0
Z_PITCH = 295.0
BISAGRA_BOCA = (-5.0, 312.0)       # (x, z) del eje del pico inferior, atrás de la cabeza
PARED = 1.6

# Servos de eje lateral (pitch, rodilla, tobillo): tapa hacia afuera en y = 50.
Y_TAPA = 50.0
Y_FONDO = Y_TAPA - MG996R.alto                   # 13.2
Y_HORN = Y_TAPA + MG996R.alto_horn               # 58.5
PLACA_EXT = (Y_HORN, Y_HORN + 3.0)
PLACA_INT = (Y_FONDO - 4.0, Y_FONDO - 1.0)

COLORES = {
    'pla': '#e9e6df', 'carcasa': '#fbfaf7', 'naranja': '#f3c118', 'amarillo_claro': '#f9dd7a', 'tpu': '#d9822b',
    'mg996r': '#26282c', 'mg90s': '#2c5aa0', 'horn': '#f2f2f2', 'pcb_verde': '#1f7a3a',
    'pcb_azul': '#1f4fa0', 'pcb_violeta': '#5b3a8a', 'lipo': '#2b6cb0', 'aluminio': '#a9b0b7',
    'negro': '#111111', 'blanco': '#ffffff', 'acero': '#8c939a',
}


# ---------------------------------------------------------------------------
# Geometría
# ---------------------------------------------------------------------------
def cil(p0, p1, r, seg=SEG):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    v = p1 - p0
    h = float(np.linalg.norm(v))
    z = v / h
    a = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
    x = np.cross(a, z)
    return Manifold.cylinder(h, r, r, seg).transform(marco(p0, z, x))


def elipsoide(c, r, seg=SEG):
    return Manifold.sphere(1.0, seg).scale(list(r)).translate(list(c))


def cascara(c, r, pared):
    return elipsoide(c, r, 64) - elipsoide(c, np.asarray(r) - pared, 64)


def caja_redondeada(x, y, z, r, r_abajo=None, encoger=0.0):
    """Envolvente de esferas en las esquinas: caja con todas las aristas redondeadas.
    r_abajo permite otro radio en las 4 esquinas de abajo (cúpula). encoger reduce
    los radios para obtener la cara interior de una cáscara."""
    r_abajo = r if r_abajo is None else r_abajo
    esferas = []
    for zc, rc in ((z[1] - r, r), (z[0] + r_abajo, r_abajo)):
        for xc in (x[0] + rc, x[1] - rc):
            for yc in (y[0] + rc, y[1] - rc):
                esferas.append(Manifold.sphere(rc - encoger, 48).translate([xc, yc, zc]))
    return Manifold.batch_hull(esferas)


def placa_redondeada(x, y, z, r):
    """Placa plana con esquinas redondeadas (eje Z)."""
    return Manifold.batch_hull([cil((xc, yc, z[0]), (xc, yc, z[1]), r)
                                for xc in (x[0] + r, x[1] - r) for yc in (y[0] + r, y[1] - r)])


def placa_y(y0, y1, discos, rects=()):
    """Placa entre y0 e y1 con forma de cápsula: envolvente de discos (x, z, r)
    y rectángulos (x0, x1, z0, z1)."""
    partes = [cil((x, y0, z), (x, y1, z), r) for x, z, r in discos]
    partes += [caja(x0, x1, y0, y1, z0, z1) for x0, x1, z0, z1 in rects]
    return Manifold.batch_hull(partes)


def placa_x(x0, x1, discos, rects=()):
    """Igual que placa_y, en un plano YZ: discos (y, z, r), rects (y0, y1, z0, z1)."""
    partes = [cil((x0, y, z), (x1, y, z), r) for y, z, r in discos]
    partes += [caja(x0, x1, ya, yb, z0, z1) for ya, yb, z0, z1 in rects]
    return Manifold.batch_hull(partes)


def elipse_z(c, rx, ry, z0, z1):
    return Manifold.cylinder(z1 - z0, 1.0, 1.0, 64).scale([rx, ry, 1]).translate([c[0], c[1], z0])


def servo_en(servo: Servo, origen, eje, largo, orejas='ambas'):
    m = marco(origen, eje, largo)
    return servo.caja(orejas).transform(m), servo.horn().transform(m)


# ---------------------------------------------------------------------------
# Modelo
# ---------------------------------------------------------------------------
@dataclass
class Pieza:
    nombre: str
    eslabon: str
    malla: Manifold
    color: str
    tipo: str                 # 'impresa', 'tpu', 'comprada'
    paso: int
    masa: float | None = None  # g; None = volumen × densidad × relleno
    relleno: float = 0.6
    descripcion: str = ''

    def masa_g(self):
        if self.masa is not None:
            return self.masa
        dens = TPU if self.tipo == 'tpu' else PLA
        return self.malla.volume() * dens * self.relleno


@dataclass
class Articulacion:
    nombre: str
    padre: str
    hijo: str
    ancla: np.ndarray
    eje: np.ndarray
    rango: tuple
    servo: Servo
    parado: float = 0.0


@dataclass
class Robot:
    piezas: list = field(default_factory=list)
    articulaciones: list = field(default_factory=list)
    anclas: dict = field(default_factory=dict)
    padres: dict = field(default_factory=dict)
    masas_extra: dict = field(default_factory=dict)  # eslabón -> (g, punto) sin malla

    def agregar(self, *piezas):
        self.piezas.extend(piezas)


PASOS = {
    1: 'Pies: suela de TPU, placa y orejas del tobillo',
    2: 'Canilla con el servo del tobillo; se une al pie',
    3: 'Muslo con el servo de la rodilla; se une a la canilla',
    4: 'Soporte U de cadera con el servo pitch; se une al muslo',
    5: 'Servo roll de cadera, placa yaw y eje',
    6: 'Piso del tronco: rulemanes 608 y servos yaw de cadera; se cuelgan las patas',
    7: 'Batería, IMU y estante A (reguladores XL4016 y placa de los MCP3008)',
    8: 'Estante B: Pi Zero 2 W, PCA9685 y regulador de 5 V',
    9: 'Cuello: servo yaw y servo pitch',
    10: 'Cabeza: cúpula, base, servo de la boca, cámara, ojos y pico',
    11: 'Carcasa del cuerpo (dos mitades)',
}


def pata(r: Robot, lado: str):
    """Construye la pata izquierda y, para la derecha, la espeja en Y."""
    s = 1 if lado == 'izq' else -1
    espejo = (lambda m: m) if s == 1 else (lambda m: m.mirror([0, 1, 0]))
    y = Y_PATA
    nombres = {k: f'{k}_{lado}' for k in ('cadera_yaw', 'cadera_roll', 'muslo', 'canilla', 'pie')}

    def add(nombre, eslabon, malla, color, tipo, paso, **kw):
        r.agregar(Pieza(f'{nombre}_{lado}', nombres[eslabon], espejo(malla), color, tipo, paso, **kw))

    # Pie (paso 1)
    talon, dedos = (-22, y, 15), [(48, y - 22, 8), (55, y, 8), (48, y + 22, 8)]
    contorno = Manifold.batch_hull([cil((px, py, 0), (px, py, 1), pr) for px, py, pr in [talon, *dedos]])
    muescas = [cil((62, y + d, -1), (62, y + d, 20), 6) for d in (-11, 11)]
    paleta = contorno - muescas[0] - muescas[1]
    add('suela', 'pie', paleta.scale([1, 1, 2]), COLORES['tpu'], 'tpu', 1, relleno=0.5,
        descripcion='Suela de TPU')
    add('paleta_pie', 'pie', paleta.scale([1, 1, 5]).translate([0, 0, 2]), COLORES['naranja'], 'impresa', 1,
        relleno=0.5, descripcion='Pie de pato (paleta)')
    oreja_ext = placa_y(*PLACA_EXT, [(0, Z_TOBILLO, 13)], [(-14, 14, 7, 9)])
    oreja_int = placa_y(*PLACA_INT, [(0, Z_TOBILLO, 10)], [(-14, 14, 7, 9)])
    pivote = cil((0, PLACA_INT[1], Z_TOBILLO), (0, Y_FONDO, Z_TOBILLO), 3.5)
    add('orejas_tobillo', 'pie', oreja_ext + oreja_int + pivote, COLORES['pla'], 'impresa', 1,
        descripcion='Orejas del tobillo (horn afuera, pivote adentro)')

    # Canilla con servo de tobillo (paso 2)
    caja_t, horn_t = servo_en(MG996R, (0, Y_TAPA, Z_TOBILLO), (0, 1, 0), (0, 0, 1), 'eje')
    add('servo_tobillo', 'canilla', caja_t, COLORES['mg996r'], 'comprada', 2, masa=MG996R.masa,
        descripcion='MG996R tobillo')
    add('horn_tobillo', 'pie', horn_t, COLORES['horn'], 'comprada', 2, masa=2.0)
    mx = MG996R.ancho / 2 + 0.3
    paredes = (caja(mx, mx + 2.5, Y_FONDO, Y_TAPA, 21, 67.5) + caja(-mx - 2.5, -mx, Y_FONDO, Y_TAPA, 21, 67.5))
    puente_ext = caja(-mx - 2.5, mx + 2.5, Y_TAPA, PLACA_EXT[1], 65, 67.5)
    puente_int = caja(-mx - 2.5, mx + 2.5, PLACA_INT[0], Y_FONDO, 65, 67.5)
    placa_ext = placa_y(*PLACA_EXT, [(0, Z_RODILLA, 13)], [(-mx - 2.5, mx + 2.5, 65, 67.5)])
    placa_int = placa_y(*PLACA_INT, [(0, Z_RODILLA, 10)], [(-mx - 2.5, mx + 2.5, 65, 67.5)])
    piv = cil((0, PLACA_INT[1], Z_RODILLA), (0, Y_FONDO, Z_RODILLA), 3.5)
    add('soporte_canilla', 'canilla', paredes + puente_ext + puente_int + placa_ext + placa_int + piv,
        COLORES['pla'], 'impresa', 2, descripcion='Canilla: aloja el servo del tobillo')

    # Muslo con servo de rodilla (paso 3)
    caja_r, horn_r = servo_en(MG996R, (0, Y_TAPA, Z_RODILLA), (0, 1, 0), (0, 0, 1), 'ninguna')
    add('servo_rodilla', 'muslo', caja_r, COLORES['mg996r'], 'comprada', 3, masa=MG996R.masa,
        descripcion='MG996R rodilla')
    add('horn_rodilla', 'canilla', horn_r, COLORES['horn'], 'comprada', 3, masa=2.0)
    z0, z1 = 68.5, 112.5
    paredes = caja(mx, mx + 2.5, Y_FONDO, Y_TAPA, z0, z1) + caja(-mx - 2.5, -mx, Y_FONDO, Y_TAPA, z0, z1)
    puente_ext = caja(-mx - 2.5, mx + 2.5, Y_TAPA, PLACA_EXT[1], 110, z1)
    puente_int = caja(-mx - 2.5, mx + 2.5, PLACA_INT[0], Y_FONDO, 110, z1)
    placa_ext = placa_y(*PLACA_EXT, [(0, Z_CADERA, 13)], [(-mx - 2.5, mx + 2.5, 110, z1)])
    placa_int = placa_y(*PLACA_INT, [(0, Z_CADERA, 10)], [(-mx - 2.5, mx + 2.5, 110, z1)])
    piv = cil((0, PLACA_INT[1], Z_CADERA), (0, Y_FONDO, Z_CADERA), 3.5)
    add('soporte_muslo', 'muslo', paredes + puente_ext + puente_int + placa_ext + placa_int + piv,
        COLORES['pla'], 'impresa', 3, descripcion='Muslo: aloja el servo de la rodilla')

    # Soporte U con servo pitch (paso 4)
    caja_p, horn_p = servo_en(MG996R, (0, Y_TAPA, Z_CADERA), (0, 1, 0), (-1, 0, 0))
    add('servo_cadera_pitch', 'cadera_roll', caja_p, COLORES['mg996r'], 'comprada', 4, masa=MG996R.masa,
        descripcion='MG996R cadera pitch')
    add('horn_cadera_pitch', 'muslo', horn_p, COLORES['horn'], 'comprada', 4, masa=2.0)
    roll_tapa = 12.0
    x_front = roll_tapa + MG996R.alto_horn       # 20.5
    rects = [(13.5, 52, 112, 136)]
    u_front = placa_x(x_front, x_front + 3, [(y, Z_ROLL, 13)], rects)
    u_back = placa_x(-41.0, -38.0, [(y, Z_ROLL, 10)], rects)
    sobra = (MG996R.largo_orejas - MG996R.largo) / 2
    separ_f = caja(MG996R.eje_desde_punta + sobra, x_front, 16, 48, 116, 132)
    separ_b = caja(-38.0, -30.5 - sobra, 16, 48, 116, 132)
    piv = cil((-38.0, y, Z_ROLL), (roll_tapa - MG996R.alto, y, Z_ROLL), 3.5)
    add('soporte_u', 'cadera_roll', u_front + u_back + separ_f + separ_b + piv, COLORES['pla'],
        'impresa', 4, descripcion='Soporte U de cadera (roll → pitch)')

    # Servo roll, placa yaw y eje (paso 5)
    caja_ro, horn_ro = servo_en(MG996R, (roll_tapa, y, Z_ROLL), (1, 0, 0), (0, -1, 0), 'eje')
    add('servo_cadera_roll', 'cadera_yaw', caja_ro, COLORES['mg996r'], 'comprada', 5, masa=MG996R.masa,
        descripcion='MG996R cadera roll')
    add('horn_cadera_roll', 'cadera_roll', horn_ro, COLORES['horn'], 'comprada', 5, masa=2.0)
    xr0 = roll_tapa - MG996R.alto
    tope_roll = Z_ROLL + MG996R.ancho / 2
    placa = caja(-26, 14, 8, 52, 165, 171)
    oreja_x = roll_tapa - MG996R.orejas_desde_tapa
    abraz = (caja(xr0, roll_tapa, y - 30.5 - 3, y - 30.5, tope_roll - 20, 171)
             + caja(oreja_x, roll_tapa - 0.5, y + 12.5, y + 10.2 + (MG996R.largo_orejas - MG996R.largo) / 2,
                    Z_ROLL - 4, 171))
    eje = cil((0, y, 171), (0, y, Z_PISO[1] + 0.5), 4.0) + cil((0, y, Z_PISO[1] - 0.5), (0, y, Z_PISO[1] + 0.5), 7)
    add('placa_yaw', 'cadera_yaw', placa + abraz + eje, COLORES['pla'], 'impresa', 5,
        descripcion='Placa yaw con abrazaderas del servo roll y eje al rulemán')

    # Servo yaw de cadera y rulemán (paso 6), en el tronco
    tapa_yaw = Z_PISO[1] + 0.5 + MG90S.alto_horn
    caja_y, horn_y = servo_en(MG90S, (0, y, tapa_yaw), (0, 0, -1), (1, 0, 0))
    r.agregar(Pieza(f'servo_cadera_yaw_{lado}', 'tronco', espejo(caja_y), COLORES['mg90s'], 'comprada', 6,
                    masa=MG90S.masa, descripcion='MG90S cadera yaw'))
    r.agregar(Pieza(f'horn_cadera_yaw_{lado}', nombres['cadera_yaw'], espejo(horn_y), COLORES['horn'],
                    'comprada', 6, masa=1.0))
    rul = (cil((0, y, Z_PISO[0] + 0.5), (0, y, Z_PISO[0] + 7.5), RULEMAN_608[0] / 2)
           - cil((0, y, Z_PISO[0]), (0, y, Z_PISO[1]), RULEMAN_608[1] / 2))
    r.agregar(Pieza(f'ruleman_cadera_{lado}', 'tronco', espejo(rul), COLORES['acero'], 'comprada', 6,
                    masa=RULEMAN_608[3], descripcion='Rulemán 608ZZ'))

    # Articulaciones (eje del lado derecho = -espejo(eje izquierdo): mismos ángulos, movimiento simétrico)
    def eje_lado(e):
        e = np.array(e, float)
        return e if s == 1 else -np.array([e[0], -e[1], e[2]])
    a = lambda z: np.array([0.0, s * y, z])
    r.articulaciones += [
        Articulacion(f'cadera_yaw_{lado}', 'tronco', nombres['cadera_yaw'], a(Z_PISO[1]), eje_lado((0, 0, 1)),
                     (-0.4, 0.4), MG90S),
        Articulacion(f'cadera_roll_{lado}', nombres['cadera_yaw'], nombres['cadera_roll'], a(Z_ROLL),
                     eje_lado((1, 0, 0)), (-0.4, 0.4), MG996R),
        Articulacion(f'cadera_pitch_{lado}', nombres['cadera_roll'], nombres['muslo'], a(Z_CADERA),
                     eje_lado((0, 1, 0)), (-1.6, 1.0), MG996R, parado=-0.35),
        Articulacion(f'rodilla_{lado}', nombres['muslo'], nombres['canilla'], a(Z_RODILLA),
                     eje_lado((0, 1, 0)), (-0.1, 2.2), MG996R, parado=0.70),
        Articulacion(f'tobillo_{lado}', nombres['canilla'], nombres['pie'], a(Z_TOBILLO),
                     eje_lado((0, 1, 0)), (-1.2, 1.2), MG996R, parado=-0.35),
    ]
    for art in r.articulaciones[-5:]:
        r.anclas[art.hijo] = art.ancla
        r.padres[art.hijo] = art.padre
    for k in ('cadera_yaw', 'cadera_roll', 'muslo', 'canilla', 'pie'):
        r.masas_extra[nombres[k]] = (3.0, None)  # tornillos y cables


def tronco(r: Robot):
    r.anclas['tronco'] = np.array([0.0, 0.0, 220.0])
    r.padres['tronco'] = None
    t = 'tronco'
    # Piso con alojamientos de rulemanes (paso 6)
    piso = placa_redondeada((TRONCO_X[0], TRONCO_X[1]), (-50, 50), Z_PISO, 10)
    for s in (1, -1):
        piso -= cil((0, s * Y_PATA, Z_PISO[0] + 0.5), (0, s * Y_PATA, Z_PISO[1] + 1), 11.1)
        piso -= cil((0, s * Y_PATA, Z_PISO[0] - 1), (0, s * Y_PATA, Z_PISO[1] + 1), 6)
    r.agregar(Pieza('piso_tronco', t, piso, COLORES['pla'], 'impresa', 6, relleno=0.4,
                    descripcion='Piso del tronco con alojamientos 608'))
    # Batería, IMU y estante A (paso 7)
    lx, ly, lz, lm = LIPO_2S_1500
    r.agregar(Pieza('lipo_2s_1500', t, caja(-lx / 2, lx / 2, -ly / 2, ly / 2, 186, 186 + lz), COLORES['lipo'],
                    'comprada', 7, masa=lm, descripcion='LiPo 2S 1500 mAh'))
    r.agregar(Pieza('tapa_bateria', t, caja(-lx / 2 - 2, lx / 2 + 2, -ly / 2 - 2, ly / 2 + 2, 201, 202.5),
                    COLORES['pla'], 'impresa', 7, relleno=0.5, descripcion='Tapa y traba de la batería'))
    gx, gy, gz, gm = GY521
    r.agregar(Pieza('imu_gy521', t, caja(-gx / 2, gx / 2, -gy / 2, gy / 2, 202.5, 202.5 + gz),
                    COLORES['pcb_azul'], 'comprada', 7, masa=gm, descripcion='IMU MPU-6050 (GY-521)'))
    estante_a = placa_redondeada((TRONCO_X[0] + 3.5, TRONCO_X[1] - 3.5), (TRONCO_Y[0] + 3.5, TRONCO_Y[1] - 3.5), (218, 221), 9)
    for s in (1, -1):  # paso de los servos yaw de cadera
        estante_a -= caja(-8, 19, s * Y_PATA - 8, s * Y_PATA + 8, 210, 230)
    r.agregar(Pieza('estante_a', t, estante_a, COLORES['pla'], 'impresa', 7, relleno=0.4,
                    descripcion='Estante A'))
    xl, xw, xh, xm = XL4016
    for i, s in enumerate((1, -1)):
        y0, y1 = sorted((s * 1, s * (1 + xw)))
        r.agregar(Pieza(f'xl4016_{"izq" if s > 0 else "der"}', t, caja(-xl / 2, xl / 2, y0, y1, 221, 221 + xh),
                        COLORES['pcb_azul'], 'comprada', 7, masa=xm,
                        descripcion=f'Regulador XL4016 6,0 V (pata {"izquierda" if s > 0 else "derecha"})'))
    mx_, my_, mz_, mm_ = PLACA_MCP3008
    r.agregar(Pieza('placa_mcp3008', t, caja(33, 33 + my_, -mx_ / 2, mx_ / 2, 221, 221 + mz_),
                    COLORES['pcb_verde'], 'comprada', 7, masa=mm_, descripcion='2× MCP3008 + divisores'))
    # Estante B (paso 8)
    estante_b = (placa_redondeada((TRONCO_X[0] + 3.5, TRONCO_X[1] - 3.5), (TRONCO_Y[0] + 3.5, TRONCO_Y[1] - 3.5), (244, 247), 9)
                 - caja(20, 50, -9, 9, 240, 250))
    r.agregar(Pieza('estante_b', t, estante_b, COLORES['pla'], 'impresa', 8, relleno=0.4,
                    descripcion='Estante B'))
    px, py, pz, pm = PI_ZERO_2W
    r.agregar(Pieza('pi_zero_2w', t, caja(-14, -14 + py, -px / 2, px / 2, 247, 247 + pz), COLORES['pcb_verde'],
                    'comprada', 8, masa=pm + 3, descripcion='Raspberry Pi Zero 2 W'))
    cx_, cy_, cz_, cm_ = PCA9685
    r.agregar(Pieza('pca9685', t, caja(-44, -44 + cy_, -cx_ / 2, cx_ / 2, 247, 247 + cz_),
                    COLORES['pcb_azul'], 'comprada', 8, masa=cm_, descripcion='PCA9685 (16 PWM)'))
    nx, ny, nz, nm = MINI560
    r.agregar(Pieza('mini560_5v', t, caja(20, 20 + nx, 12, 12 + ny, 247, 247 + nz), COLORES['pcb_violeta'],
                    'comprada', 8, masa=nm, descripcion='Mini560 5 V (Pi)'))
    # Servo yaw del cuello (paso 9)
    caja_c, _ = servo_en(MG90S, (X_CUELLO, 0, 268), (0, 0, 1), (-1, 0, 0), 'lejos')
    r.agregar(Pieza('servo_cuello_yaw', t, caja_c, COLORES['mg90s'], 'comprada', 9, masa=MG90S.masa,
                    descripcion='MG90S cuello yaw'))
    # Carcasa (paso 11): cajita redondeada abierta abajo (apoya sobre el piso del tronco)
    cuerpo = (caja_redondeada(TRONCO_X, TRONCO_Y, TRONCO_Z, TRONCO_R)
              - caja_redondeada(TRONCO_X, TRONCO_Y, (TRONCO_Z[0] - 5, TRONCO_Z[1]), TRONCO_R, encoger=PARED))
    cuerpo -= caja(-200, 200, -200, 200, 0, TRONCO_Z[0] + 0.01)
    cuerpo -= cil((X_CUELLO, 0, 260), (X_CUELLO, 0, 300), 17)        # paso del cuello
    inferior = cuerpo - caja(-200, 200, -200, 200, Z_CORTE_TRONCO, 400)
    superior = cuerpo - caja(-200, 200, -200, 200, 0, Z_CORTE_TRONCO)
    r.agregar(Pieza('cuerpo_inferior', t, inferior, COLORES['carcasa'], 'impresa', 11, relleno=1.0,
                    descripcion='Carcasa del cuerpo, mitad inferior'))
    r.agregar(Pieza('cuerpo_superior', t, superior, COLORES['carcasa'], 'impresa', 11, relleno=1.0,
                    descripcion='Carcasa del cuerpo, mitad superior'))
    r.masas_extra['tronco'] = (40.0, None)  # cables, conectores, tornillos


def cuello_y_cabeza(r: Robot):
    x = X_CUELLO
    # Cuello (paso 9): dos servos a la vista entre el tronco y la cabeza, como el original
    r.anclas['cuello'] = np.array([x, 0, 272.0])
    r.padres['cuello'] = 'tronco'
    _, horn_cy = servo_en(MG90S, (x, 0, 268), (0, 0, 1), (-1, 0, 0), 'lejos')
    r.agregar(Pieza('horn_cuello_yaw', 'cuello', horn_cy, COLORES['horn'], 'comprada', 9, masa=1.0))
    caja_cp, horn_cp = servo_en(MG90S, (x, 8.7, Z_PITCH), (0, 1, 0), (0, 0, -1), 'eje')
    r.agregar(Pieza('servo_cuello_pitch', 'cuello', caja_cp, COLORES['mg90s'], 'comprada', 9, masa=MG90S.masa,
                    descripcion='MG90S cuello pitch'))
    base = cil((x, 0, 275.8), (x, 0, 278), 14)
    lados = caja(x - 8.5, x - 6.3, -14, 8.7, 278, 300) + caja(x + 6.3, x + 8.5, -14, 8.7, 278, 300)
    r.agregar(Pieza('soporte_cuello', 'cuello', base + lados, COLORES['pla'], 'impresa', 9,
                    descripcion='Soporte del cuello (yaw → pitch)'))

    # Cabeza (paso 10)
    r.anclas['cabeza'] = np.array([x, 0, Z_PITCH])
    r.padres['cabeza'] = 'cuello'
    r.agregar(Pieza('horn_cuello_pitch', 'cabeza', horn_cp, COLORES['horn'], 'comprada', 9, masa=1.0))
    z0, z1 = Z_CAB
    # Cúpula: cáscara abierta abajo, aristas de arriba bien redondeadas
    cupula = (caja_redondeada(CAB_X, CAB_Y, Z_CAB, R_CAB_ARRIBA, R_CAB_ABAJO)
              - caja_redondeada(CAB_X, CAB_Y, (z0 - 5, z1), R_CAB_ARRIBA, R_CAB_ABAJO, encoger=PARED))
    cupula -= caja(-200, 200, -200, 200, 0, z0 + 0.01)
    cupula -= cil((CAB_X[1] - 10, 0, 328), (CAB_X[1] + 5, 0, 328), 4.5)       # lente
    r.agregar(Pieza('cupula', 'cabeza', cupula, COLORES['carcasa'], 'impresa', 10, relleno=1.0,
                    descripcion='Cúpula de la cabeza'))
    # Base estructural dentro de la cúpula: sostiene el servo de la boca y se une al cuello
    base_cab = placa_redondeada((0, CAB_X[1] - 12), (-44, 44), (302, 305), 8)
    base_cab -= caja(x - 10, x + 20, -22, 22, 300, 306)                        # paso del servo pitch
    soporte = (placa_y(16.5, 19.5, [(x, Z_PITCH, 9)], [(x - 6, x + 10, 302, 305)])
               + placa_y(-18, -15, [(x, Z_PITCH, 7)], [(x - 6, x + 10, 302, 305)])
               + cil((x, -15, Z_PITCH), (x, -14, Z_PITCH), 3))
    bx, bz = BISAGRA_BOCA
    soporte_bisagra = caja(bx - 4, bx + 4, -16.5, -14, 302, bz + 4) + cil((bx, -16.5, bz), (bx, -19, bz), 2)
    r.agregar(Pieza('base_cabeza', 'cabeza', base_cab + soporte + soporte_bisagra, COLORES['pla'], 'impresa', 10,
                    descripcion='Base de la cabeza: une el cuello, el servo de la boca y la cúpula'))
    # Servo de la boca, en el fondo de la cabeza, eje lateral
    caja_b, horn_b = servo_en(MG90S, (bx, 9, bz), (0, 1, 0), (1, 0, 0), 'lejos')
    r.agregar(Pieza('servo_boca', 'cabeza', caja_b, COLORES['mg90s'], 'comprada', 10, masa=MG90S.masa,
                    descripcion='MG90S boca'))
    # Pico superior: lámina amarilla ancha en el borde de abajo de la cúpula
    pico_sup = placa_redondeada((CAB_X[1] - 12, CAB_X[1] + 24), (-48, 48), (302, 306), 12)
    r.agregar(Pieza('pico_superior', 'cabeza', pico_sup, COLORES['naranja'], 'impresa', 10, relleno=0.4,
                    descripcion='Pico superior'))
    # Ojos: aro amarillo con disco claro, uno por lado
    for s, lado in ((1, 'izq'), (-1, 'der')):
        yc = CAB_Y[1] if s > 0 else CAB_Y[0]
        aro = (cil((55, yc, 334), (55, yc + 3 * s, 334), 18)
               - cil((55, yc - 1 * s, 334), (55, yc + 4 * s, 334), 11))
        disco = cil((55, yc, 334), (55, yc + 2 * s, 334), 11)
        r.agregar(Pieza(f'ojo_aro_{lado}', 'cabeza', aro, COLORES['naranja'], 'impresa', 10, relleno=0.4,
                        descripcion='Aro del ojo'))
        r.agregar(Pieza(f'ojo_disco_{lado}', 'cabeza', disco, COLORES['amarillo_claro'], 'impresa', 10, relleno=0.4,
                        descripcion='Disco del ojo'))
    # Cámara en la frente, detrás del agujero del lente
    xf = CAB_X[1] - PARED
    r.agregar(Pieza('camara', 'cabeza', caja(xf - 7, xf - 6, -12.5, 12.5, 316, 340), COLORES['pcb_verde'], 'comprada',
                    10, masa=3.0, descripcion='Cámara de la Pi (en la frente)'))
    r.agregar(Pieza('lente', 'cabeza', cil((xf - 6, 0, 328), (xf + 1, 0, 328), 4), COLORES['negro'], 'comprada', 10,
                    masa=1.0))
    r.agregar(Pieza('soporte_camara', 'cabeza', caja(xf - 10, xf - 7, -14, 14, 306, 342), COLORES['pla'], 'impresa', 10,
                    descripcion='Soporte de la cámara'))

    # Pico inferior (paso 10): es todo el piso de la cabeza, abre como una tapa desde atrás
    r.anclas['pico_inferior'] = np.array([bx, 0, bz])
    r.padres['pico_inferior'] = 'cabeza'
    r.agregar(Pieza('horn_boca', 'pico_inferior', horn_b, COLORES['horn'], 'comprada', 10, masa=0.5))
    mand = placa_redondeada((-12, CAB_X[1] - 12), (-50, 50), (296, 301), 10)
    mand -= caja(x - 12, x + 22, -22, 22, 290, 310)                             # paso del cuello
    brazo_h = placa_y(16.8, 18.8, [(bx, bz, 6)], [(bx - 3, bx + 3, 296, bz)])
    brazo_p = placa_y(-18.8, -16.8, [(bx, bz, 6)], [(bx - 3, bx + 3, 296, bz)])
    r.agregar(Pieza('mandibula', 'pico_inferior', mand + brazo_h + brazo_p, COLORES['carcasa'], 'impresa', 10,
                    relleno=0.4, descripcion='Mandíbula: piso de la cabeza, la mueve el servo de la boca'))
    pico_inf = placa_redondeada((CAB_X[1] - 12, CAB_X[1] + 22), (-46, 46), (296, 301), 12)
    r.agregar(Pieza('pico_inferior', 'pico_inferior', pico_inf, COLORES['naranja'], 'impresa', 10, relleno=0.4,
                    descripcion='Pico inferior (amarillo, va pegado a la mandíbula)'))
    r.articulaciones += [
        Articulacion('cuello_yaw', 'tronco', 'cuello', r.anclas['cuello'], np.array([0, 0, 1.0]), (-1.5, 1.5), MG90S),
        Articulacion('cuello_pitch', 'cuello', 'cabeza', r.anclas['cabeza'], np.array([0, 1.0, 0]), (-0.8, 0.8),
                     MG90S),
        Articulacion('boca', 'cabeza', 'pico_inferior', r.anclas['pico_inferior'], np.array([0, 1.0, 0]),
                     (0.0, 0.55), MG90S),
    ]


def construir() -> Robot:
    r = Robot()
    tronco(r)
    pata(r, 'izq')
    pata(r, 'der')
    cuello_y_cabeza(r)
    for p in r.piezas:
        if p.malla.is_empty():
            raise ValueError(f'pieza vacía: {p.nombre}')
    return r


# ---------------------------------------------------------------------------
# Exportación
# ---------------------------------------------------------------------------
def a_trimesh(m: Manifold) -> trimesh.Trimesh:
    mesh = m.to_mesh()
    return trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts), process=True)


def color_rgba(hexa, alfa=255):
    h = hexa.lstrip('#')
    return [int(h[i:i + 2], 16) for i in (0, 2, 4)] + [alfa]


def propiedades_eslabon(r: Robot, eslabon, mallas):
    """Masa (kg), centro de masa e inercia (kg·m²) en el marco del eslabón (m)."""
    ancla = r.anclas[eslabon]
    total, momento, partes = 0.0, np.zeros(3), []
    for p in r.piezas:
        if p.eslabon != eslabon:
            continue
        mt = mallas[p.nombre]
        m = p.masa_g() / 1000.0
        vol = mt.volume
        if vol <= 0:
            raise ValueError(f'volumen no positivo: {p.nombre}')
        com = (mt.center_mass - ancla) / 1000.0
        # Inercia de un sólido homogéneo con esa masa, en su centro de masa.
        inertia = mt.moment_inertia * (m / (vol * 1.0)) / 1e6  # (mm² · kg) -> kg·m²
        partes.append((m, com, inertia))
        total += m
        momento += m * com
    extra = r.masas_extra.get(eslabon)
    com = momento / total
    if extra:
        m = extra[0] / 1000.0
        partes.append((m, com.copy(), np.eye(3) * m * 1e-6))
        total += m
    inercia = np.zeros((3, 3))
    for m, c, ii in partes:
        d = c - com
        inercia += ii + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return total, com, inercia


def exportar(r: Robot):
    SALIDA.mkdir(parents=True, exist_ok=True)
    (SALIDA / 'piezas_impresas').mkdir(exist_ok=True)
    (SALIDA / 'mjcf' / 'mallas').mkdir(parents=True, exist_ok=True)
    mallas = {p.nombre: a_trimesh(p.malla) for p in r.piezas}

    # STL de las piezas impresas, en coordenadas de mundo (pose cero)
    for p in r.piezas:
        if p.tipo in ('impresa', 'tpu'):
            mallas[p.nombre].export(SALIDA / 'piezas_impresas' / f'{p.nombre}.stl')

    # GLB: un nodo por eslabón (en su ancla) y las piezas dentro, en metros.
    escena = trimesh.Scene()
    orden = ['tronco'] + [a.hijo for a in r.articulaciones]
    for esl in orden:
        padre = r.padres[esl] or 'world'
        rel = r.anclas[esl] - (r.anclas[padre] if padre != 'world' else 0)
        tf = np.eye(4)
        tf[:3, 3] = rel / 1000.0
        escena.graph.update(frame_to=esl, frame_from=padre, matrix=tf)
    for p in r.piezas:
        mt = mallas[p.nombre].copy()
        mt.apply_translation(-r.anclas[p.eslabon])
        mt.apply_scale(1 / 1000.0)
        mt = trimesh.graph.smooth_shade(mt, angle=math.radians(35))
        lineal = [(c / 255.0) ** 2.2 for c in color_rgba(p.color)[:3]] + [1.0]  # glTF usa color lineal
        mat = trimesh.visual.material.PBRMaterial(
            name=p.nombre, baseColorFactor=lineal,
            metallicFactor=0.35 if p.color in (COLORES['aluminio'], COLORES['acero']) else 0.0,
            roughnessFactor=0.55)
        mt.visual = trimesh.visual.TextureVisuals(material=mat)
        escena.add_geometry(mt, node_name=p.nombre, geom_name=p.nombre, parent_node_name=p.eslabon)
    (SALIDA / 'pato.glb').write_bytes(trimesh.exchange.gltf.export_glb(escena, include_normals=True))

    # robot.json para el visor
    datos = {
        'eslabones': {e: {'padre': r.padres[e], 'ancla_mm': r.anclas[e].tolist()} for e in orden},
        'articulaciones': [dict(nombre=a.nombre, padre=a.padre, hijo=a.hijo, ancla_mm=a.ancla.tolist(),
                                eje=a.eje.tolist(), rango=list(a.rango), servo=a.servo.nombre, parado=a.parado)
                           for a in r.articulaciones],
        'piezas': [dict(nombre=p.nombre, eslabon=p.eslabon, tipo=p.tipo, paso=p.paso, color=p.color,
                        masa_g=round(p.masa_g(), 1), descripcion=p.descripcion) for p in r.piezas],
        'pasos': PASOS,
    }
    (SALIDA / 'robot.json').write_text(json.dumps(datos, ensure_ascii=False, indent=1))

    # MJCF
    props = {e: propiedades_eslabon(r, e, mallas) for e in orden}
    hijos = {e: [a for a in r.articulaciones if a.padre == e] for e in orden}
    assets, lineas = [], []
    for p in r.piezas:
        mt = mallas[p.nombre].copy()
        mt.apply_translation(-r.anclas[p.eslabon])
        mt.apply_scale(1 / 1000.0)
        mt.export(SALIDA / 'mjcf' / 'mallas' / f'{p.nombre}.stl')
        assets.append(f'    <mesh name="{p.nombre}" file="mallas/{p.nombre}.stl"/>')

    def cuerpo(esl, art, sangria):
        sp = ' ' * sangria
        if art is None:
            pos = r.anclas[esl] / 1000.0
            lineas.append(f'{sp}<body name="{esl}" pos="{pos[0]:.5f} {pos[1]:.5f} {pos[2]:.5f}">')
            lineas.append(f'{sp}  <freejoint name="base"/>')
        else:
            rel = (r.anclas[esl] - r.anclas[art.padre]) / 1000.0
            lineas.append(f'{sp}<body name="{esl}" pos="{rel[0]:.5f} {rel[1]:.5f} {rel[2]:.5f}">')
            e = art.eje
            lineas.append(f'{sp}  <joint name="{art.nombre}" axis="{e[0]:g} {e[1]:g} {e[2]:g}" '
                          f'range="{art.rango[0]} {art.rango[1]}" class="{art.servo.nombre.lower()}"/>')
        m, com, ii = props[esl]
        lineas.append(f'{sp}  <inertial pos="{com[0]:.5f} {com[1]:.5f} {com[2]:.5f}" mass="{m:.5f}" '
                      f'fullinertia="{ii[0,0]:.3e} {ii[1,1]:.3e} {ii[2,2]:.3e} {ii[0,1]:.3e} {ii[0,2]:.3e} {ii[1,2]:.3e}"/>')
        for p in r.piezas:
            if p.eslabon == esl:
                c = color_rgba(p.color)
                rgba = ' '.join(f'{v / 255:.3f}' for v in c)
                clase = 'suela' if p.nombre.startswith('suela') else 'visual'
                lineas.append(f'{sp}  <geom mesh="{p.nombre}" class="{clase}" rgba="{rgba}"/>')
        for a in hijos[esl]:
            cuerpo(a.hijo, a, sangria + 2)
        lineas.append(f'{sp}</body>')

    cuerpo('tronco', None, 4)
    nm = lambda s: 9.81 * s.stall_kgcm_6v / 100.0
    actuadores = '\n'.join(f'    <position name="{a.nombre}" joint="{a.nombre}" class="{a.servo.nombre.lower()}"/>'
                           for a in r.articulaciones)
    xml = f'''<mujoco model="pato_criollo_v0">
  <compiler angle="radian" meshdir="."/>
  <option timestep="0.002"/>
  <default>
    <geom contype="0" conaffinity="0" density="0"/>
    <default class="visual"><geom group="2"/></default>
    <default class="suela"><geom contype="1" conaffinity="1" friction="0.8 0.02 0.01" group="2"/></default>
    <default class="mg996r">
      <joint damping="0.05" frictionloss="0.02" armature="0.002"/>
      <position kp="8" forcerange="-{nm(MG996R):.3f} {nm(MG996R):.3f}"/>
    </default>
    <default class="mg90s">
      <joint damping="0.01" frictionloss="0.005" armature="0.0005"/>
      <position kp="2" forcerange="-{nm(MG90S):.3f} {nm(MG90S):.3f}"/>
    </default>
  </default>
  <asset>
{chr(10).join(assets)}
  </asset>
  <worldbody>
    <light pos="0 0 2" dir="0 0 -1" directional="true"/>
    <geom name="suelo" type="plane" size="0 0 0.05" rgba="0.75 0.78 0.8 1" contype="1" conaffinity="1"/>
{chr(10).join(lineas)}
  </worldbody>
  <actuator>
{actuadores}
  </actuator>
</mujoco>
'''
    (SALIDA / 'mjcf' / 'pato.xml').write_text(xml)

    # Resumen de masas
    filas = sorted(r.piezas, key=lambda p: (p.paso, p.nombre))
    total = sum(p.masa_g() for p in r.piezas) + sum(v[0] for v in r.masas_extra.values())
    impresas = [p for p in r.piezas if p.tipo in ('impresa', 'tpu')]
    md = ['# Resumen del modelo v0', '',
          f'- Masa total estimada: **{total / 1000:.2f} kg**',
          f'- Servos: 8× MG996R, 5× MG90S',
          f'- Piezas impresas: {len(impresas)} '
          f'({sum(p.masa_g() for p in impresas):.0f} g de PLA/TPU, con el relleno indicado)', '',
          '| Paso | Pieza | Tipo | Masa (g) | Descripción |', '| ---: | --- | --- | ---: | --- |']
    md += [f'| {p.paso} | `{p.nombre}` | {p.tipo} | {p.masa_g():.1f} | {p.descripcion} |' for p in filas]
    md += ['', f'Más {sum(v[0] for v in r.masas_extra.values()):.0f} g de cables, conectores y tornillos repartidos.']
    (SALIDA / 'resumen.md').write_text('\n'.join(md) + '\n')
    return total


if __name__ == '__main__':
    robot = construir()
    total = exportar(robot)
    print(f'{len(robot.piezas)} piezas, {len(robot.articulaciones)} articulaciones, masa {total:.0f} g')
