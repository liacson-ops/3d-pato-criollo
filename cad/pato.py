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
Y_PATA = 42.0          # eje de cada pata respecto del centro
# Servos de rodilla, tobillo y cadera pitch acostados: eje lateral, cuerpo hacia atrás.
# Así cada segmento mide 32 mm en vez de 45.
Z_TOBILLO = 22.0
SEGMENTO = 32.0
Z_RODILLA = Z_TOBILLO + SEGMENTO
Z_CADERA = Z_RODILLA + SEGMENTO
Z_ROLL = Z_CADERA + 36.0       # con 30 la cuna del muslo rozaba el servo roll al balancear
Z_PISO = (Z_ROLL + 24.0, Z_ROLL + 32.0)
# Tronco: cajita redondeada, como el original (la electrónica va en estantes adentro).
TRONCO_X = (-54.0, 70.0)
TRONCO_Y = (-55.0, 55.0)          # los MG996R de cadera yaw (19,7 de ancho en y = ±42) van adentro
Z_TAPA_YAW_CADERA = Z_PISO[1] + 0.5 + MG996R.alto_horn      # horn sobre el eje del rulemán
TRONCO_Z = (Z_PISO[1] + 1.0, Z_PISO[1] + 71.0)
TRONCO_R = 12.0
Z_CORTE_TRONCO = TRONCO_Z[0] + 38.0
X_CUELLO = 40.0
Z_YAW_TAPA = TRONCO_Z[1] - 3.0        # tapa del servo yaw del cuello (el horn asoma arriba)
Z_PITCH = Z_YAW_TAPA + 47.0   # cuello más largo: con la boca abierta y la cabeza girada, el pico no entra en el tronco
# Cabeza: cúpula tipo "pan" con el piso plano y pico ancho que abre como una tapa.
# Se modela en el marco de referencia viejo (pitch en z=295) y se baja al final.
CAB_X = (-15.0, 115.0)
CAB_Y = (-57.0, 57.0)             # más ancha que el tronco, como el original
Z_CAB = (305.0, 365.0)
R_CAB_ARRIBA, R_CAB_ABAJO = 26.0, 10.0
BISAGRA_BOCA = (-5.0, 312.0)       # (x, z) del eje del pico inferior, atrás de la cabeza
PARED = 1.6

# Servos de eje lateral: tapa hacia afuera, horn afuera; pivote impreso del lado de adentro.
Y_TAPA = Y_PATA + 12.0                            # 54
Y_FONDO = Y_TAPA - MG996R.alto                    # 17.2
Y_HORN = Y_TAPA + MG996R.alto_horn                # 62.5
PLACA_EXT = (Y_HORN, Y_HORN + 3.0)                # placa exterior del eslabón de abajo, sobre el horn
PLACA_PIV = (10.2, 12.7)                          # placa interior inferior de cada eslabón (lleva el agujero del pin)
PLACA_PIV_U = (8.0, 11.0)                         # ídem en el soporte U
Y_CUNA_INT = (13.7, 16.7)                         # placa interior de la cuna (0,5 mm del cuerpo)
Y_PAD = (Y_TAPA - MG996R.orejas_desde_tapa - MG996R.espesor_orejas - 2.9,
         Y_TAPA - MG996R.orejas_desde_tapa - MG996R.espesor_orejas)   # apoyo de las orejas
PARED_CUNA = (10.2, 14.0)                         # paredes arriba/abajo del cuerpo, desde el eje

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
    1: 'Pies: suela de TPU, paleta y la cuna con el servo del tobillo',
    2: 'Canilla: cuna con el servo de la rodilla; abajo se une al horn del tobillo',
    3: 'Muslo: cuna con el servo pitch de cadera; abajo se une al horn de la rodilla',
    4: 'Soporte U de cadera: cuelga del servo roll y recibe el muslo',
    5: 'Servo roll de cadera, placa yaw y eje',
    6: 'Piso del tronco: rulemanes 608 y servos yaw de cadera; se cuelgan las patas',
    7: 'Batería, IMU y estante A (reguladores XL4016 y placa de los MCP3008)',
    8: 'Estante B: Pi Zero 2 W, PCA9685 y regulador de 5 V',
    9: 'Cuello: servo yaw y servo pitch',
    10: 'Cabeza: cúpula, base, servo de la boca, cámara, ojos y pico',
    11: 'Carcasa del cuerpo (dos mitades)',
}


def cuna(zs, direccion, z_inf=None, y_pin=PLACA_PIV[0], base=None):
    """Cuna impresa de un servo MG996R acostado (eje lateral en z = zs, que es la articulación
    de ARRIBA del eslabón). El cuerpo apunta hacia atrás (direccion = -1) o hacia adelante (+1)
    y del lado del eje la cuna es corta (10,7 mm), así al girar su esquina nunca alcanza al
    eslabón de arriba. Del lado del eje: el horn va al eslabón de arriba y un pin pivota en su
    placa interior. Abajo (z_inf) cuelgan las placas que reciben el horn y el pin del eslabón
    siguiente. base = z de la paleta cuando la cuna apoya en el pie."""
    a, b = PARED_CUNA
    y_int, y_tapa = Y_CUNA_INT[0], Y_TAPA
    corto, largo = MG996R.eje_desde_punta + 0.5, MG996R.largo - MG996R.eje_desde_punta + 7.5
    x0, x1 = (-largo, corto) if direccion < 0 else (-corto, largo)
    m = caja(x0, x1, y_int, y_tapa, zs + a, zs + b)                         # pared de arriba
    m += caja(x0, x1, y_int, y_tapa, zs - b, zs - a)                        # pared de abajo
    m += caja(x0, x1, Y_CUNA_INT[0], Y_CUNA_INT[1], zs - b, zs + b)         # placa interior
    lejos = MG996R.largo - MG996R.eje_desde_punta                           # oreja del lado lejano al eje
    xt0, xt1 = (-lejos - 6.9, -lejos) if direccion < 0 else (lejos, lejos + 6.9)
    m += caja(xt0, xt1, Y_PAD[0], Y_PAD[1], zs - a, zs + a)                 # apoyo de esa oreja
    m += cil((0, Y_CUNA_INT[0], zs), (0, y_pin, zs), 3.5)                   # pin hacia el eslabón de arriba
    if z_inf is not None:
        m += caja(x0, x1, y_tapa, PLACA_EXT[1], zs - b, zs - 12.5)          # escalón bajo el horn
        m += placa_y(*PLACA_EXT, [(0, z_inf, 11.5)], [(-10, 10, z_inf, zs - 12.5)])
        m += (placa_y(*PLACA_PIV, [(0, z_inf, 10)], [(-8, 8, z_inf, zs - b + 3.5)])
              - cil((0, PLACA_PIV[0] - 1, z_inf), (0, PLACA_PIV[1] + 1, z_inf), 3.8))
        # Puente a la placa interior, por encima de lo que barre la esquina de la cuna de abajo (r = 17,6).
        m += caja(-10, 10, PLACA_PIV[0], Y_CUNA_INT[1], zs - b + 1.0, zs - b + 3.5)
    if base is not None:
        m += caja(x0, x1, y_int, y_tapa, base, zs - b)                      # apoyo sobre la paleta
    return m


def servo_acostado(zs, direccion):
    return servo_en(MG996R, (0, Y_TAPA, zs), (0, 1, 0), (direccion, 0, 0), 'lejos')


def pata(r: Robot, lado: str):
    """Construye la pata izquierda y, para la derecha, la espeja en Y."""
    s = 1 if lado == 'izq' else -1
    espejo = (lambda m: m) if s == 1 else (lambda m: m.mirror([0, 1, 0]))
    y = Y_PATA
    nombres = {k: f'{k}_{lado}' for k in ('cadera_yaw', 'cadera_roll', 'muslo', 'canilla', 'pie')}

    def add(nombre, eslabon, malla, color, tipo, paso, **kw):
        r.agregar(Pieza(f'{nombre}_{lado}', nombres[eslabon], espejo(malla), color, tipo, paso, **kw))

    # Pie (paso 1): paleta, suela y la cuna del servo del tobillo (apunta hacia atrás)
    talon, dedos = (-26, y, 13), [(42, y - 20, 7), (48, y, 7), (42, y + 20, 7)]
    contorno = Manifold.batch_hull([cil((px, py, 0), (px, py, 1), pr) for px, py, pr in [talon, *dedos]])
    muescas = [cil((54, y + d, -1), (54, y + d, 20), 5) for d in (-10, 10)]
    paleta = contorno - muescas[0] - muescas[1]
    add('suela', 'pie', paleta.scale([1, 1, 2]), COLORES['tpu'], 'tpu', 1, relleno=0.5,
        descripcion='Suela de TPU')
    add('paleta_pie', 'pie', paleta.scale([1, 1, 5]).translate([0, 0, 2]), COLORES['naranja'], 'impresa', 1,
        relleno=0.5, descripcion='Pie de pato (paleta)')
    # La cuna del tobillo apunta hacia atrás (hacia adelante sube contra la canilla al caminar).
    # Límite: con la rodilla a más de 75° y la cadera a -0,65 el muslo la alcanza (no hay cuclillas profundas).
    caja_t, horn_t = servo_acostado(Z_TOBILLO, -1)
    add('servo_tobillo', 'pie', caja_t, COLORES['mg996r'], 'comprada', 1, masa=MG996R.masa,
        descripcion='MG996R tobillo (acostado, en el pie)')
    add('soporte_pie', 'pie', cuna(Z_TOBILLO, -1, base=7), COLORES['pla'], 'impresa', 1,
        descripcion='Cuna del servo del tobillo, sobre la paleta')

    # Canilla (paso 2): cuna del servo de la rodilla (hacia adelante) y placas al horn del tobillo
    add('horn_tobillo', 'canilla', horn_t, COLORES['horn'], 'comprada', 2, masa=2.0)
    caja_r, horn_r = servo_acostado(Z_RODILLA, +1)
    add('servo_rodilla', 'canilla', caja_r, COLORES['mg996r'], 'comprada', 2, masa=MG996R.masa,
        descripcion='MG996R rodilla (acostado, en la canilla)')
    add('soporte_canilla', 'canilla', cuna(Z_RODILLA, +1, Z_TOBILLO), COLORES['pla'], 'impresa', 2,
        descripcion='Canilla: cuna del servo de la rodilla; abajo recibe el horn del tobillo')

    # Muslo (paso 3): cuna del servo pitch de cadera (hacia atrás) y placas al horn de la rodilla
    add('horn_rodilla', 'muslo', horn_r, COLORES['horn'], 'comprada', 3, masa=2.0)
    caja_p, horn_p = servo_acostado(Z_CADERA, -1)
    add('servo_cadera_pitch', 'muslo', caja_p, COLORES['mg996r'], 'comprada', 3, masa=MG996R.masa,
        descripcion='MG996R cadera pitch (acostado, en el muslo)')
    add('soporte_muslo', 'muslo', cuna(Z_CADERA, -1, Z_RODILLA, y_pin=PLACA_PIV_U[0]), COLORES['pla'], 'impresa', 3,
        descripcion='Muslo: cuna del servo pitch de cadera; abajo recibe el horn de la rodilla')

    # Soporte U (paso 4): marco que cuelga del servo roll y recibe el horn y el pin del muslo
    add('horn_cadera_pitch', 'cadera_roll', horn_p, COLORES['horn'], 'comprada', 4, masa=2.0)
    roll_tapa = 12.0
    x_front = roll_tapa + MG996R.alto_horn                      # 20.5
    zc, zr = Z_CADERA, Z_ROLL
    yi0, yi1 = PLACA_PIV_U
    # Lado exterior: solo un disco en el horn y un cuello por detrás del eje, que es la zona que la
    # cuna del muslo nunca barre (su esquina delantera gira a 17,6 mm del eje, hacia adelante).
    u = placa_y(*PLACA_EXT, [(0, zc, 11.5)])
    u += caja(-8, -2, PLACA_EXT[0], PLACA_EXT[1], zc + 8, zc + 17)
    # Los bordes de arriba quedan en zr + 4: con el balanceo, la esquina exterior (23,5 mm del eje roll)
    # sube 6 mm y no llega a la placa yaw.
    u += caja(-42, x_front + 3, PLACA_EXT[0], PLACA_EXT[1], zc + 17, zr + 4)
    u += placa_x(x_front, x_front + 3, [(y, zr, 13)], [(yi0, PLACA_EXT[1], zc + 6, zr)])         # frente
    # Pivote interior: disco en el eje y una barra por arriba de lo que barre la placa interior del muslo.
    u += placa_y(yi0, yi1, [(0, zc, 10)]) - cil((0, yi0 - 1, zc), (0, yi1 + 1, zc), 3.8)
    u += caja(0, x_front + 3, yi0, yi1, zc + 6, zc + 9)
    u += caja(-42, -39, y - 14, PLACA_EXT[1], zr - 10, zr + 4)                                    # atrás
    u += cil((-45.5, y, zr), (-26, y, zr), 3.5)                                                   # pin trasero del eje roll
    add('soporte_u', 'cadera_roll', u, COLORES['pla'], 'impresa', 4,
        descripcion='Soporte U de cadera: cuelga del servo roll, recibe el horn y el pin del muslo')

    # Servo roll, placa yaw y eje (paso 5)
    caja_ro, horn_ro = servo_en(MG996R, (roll_tapa, y, Z_ROLL), (1, 0, 0), (0, -1, 0))
    add('servo_cadera_roll', 'cadera_yaw', caja_ro, COLORES['mg996r'], 'comprada', 5, masa=MG996R.masa,
        descripcion='MG996R cadera roll')
    add('horn_cadera_roll', 'cadera_roll', horn_ro, COLORES['horn'], 'comprada', 5, masa=2.0)
    x_tab = roll_tapa - MG996R.orejas_desde_tapa - MG996R.espesor_orejas   # cara interior de las orejas
    # Placa sobre el servo roll. Angosta hacia adentro (solo una lengüeta para la oreja interior) y
    # hacia atrás (solo la barra al soporte del pin): así no cruza la línea media al abrir las puntas.
    placa = caja(-26, 14, y - 30.5, y + 17.1, Z_ROLL + 10.2, Z_ROLL + 13.2)
    placa += caja(x_tab - 2.5, 4, y - 37.4, y - 30.5, Z_ROLL + 10.2, Z_ROLL + 13.2)            # lengüeta
    placa += caja(x_tab - 2.5, x_tab, y + 10.2, y + 17.1, Z_ROLL - 10.2, Z_ROLL + 10.2)
    placa += caja(x_tab - 2.5, x_tab, y - 37.4, y - 30.5, Z_ROLL - 10.2, Z_ROLL + 10.2)
    placa += caja(-45.5, -26, y - 7, y + 7, Z_ROLL + 10.2, Z_ROLL + 13.2)                     # barra al pin
    placa += caja(-45.5, -42, y - 7, y + 7, Z_ROLL - 7, Z_ROLL + 10.2) - cil((-46, y, Z_ROLL), (-41, y, Z_ROLL), 4)
    eje = (cil((0, y, Z_ROLL + 13.2), (0, y, Z_PISO[1] + 0.5), 4.0)
           + cil((0, y, Z_PISO[1] - 0.5), (0, y, Z_PISO[1] + 0.5), 7))
    add('placa_yaw', 'cadera_yaw', placa + eje, COLORES['pla'], 'impresa', 5,
        descripcion='Placa yaw: sostiene el servo roll por las orejas y sube al rulemán')

    # Servo yaw de cadera y rulemán (paso 6), en el tronco
    caja_y, horn_y = servo_en(MG996R, (0, y, Z_TAPA_YAW_CADERA), (0, 0, -1), (1, 0, 0))
    r.agregar(Pieza(f'servo_cadera_yaw_{lado}', 'tronco', espejo(caja_y), COLORES['mg996r'], 'comprada', 6,
                    masa=MG996R.masa, descripcion='MG996R cadera yaw (parado, las orejas van contra el estante A)'))
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
    anc = lambda z: np.array([0.0, s * y, z])
    r.articulaciones += [
        # Yaw y roll: negativo = hacia adentro (punta del pie o pie hacia el centro), positivo = hacia afuera.
        # Hacia adentro menos, para que las dos patas no se toquen entre sí.
        Articulacion(f'cadera_yaw_{lado}', 'tronco', nombres['cadera_yaw'], anc(Z_PISO[1]), eje_lado((0, 0, 1)),
                     (-0.15, 0.35), MG996R),
        Articulacion(f'cadera_roll_{lado}', nombres['cadera_yaw'], nombres['cadera_roll'], anc(Z_ROLL),
                     eje_lado((1, 0, 0)), (-0.09, 0.25), MG996R),
        # Rangos limitados por la mecánica (ver cad/choques.py): más allá, las cunas se tocan.
        Articulacion(f'cadera_pitch_{lado}', nombres['cadera_roll'], nombres['muslo'], anc(Z_CADERA),
                     eje_lado((0, 1, 0)), (-1.15, 0.15), MG996R, parado=-0.35),
        Articulacion(f'rodilla_{lado}', nombres['muslo'], nombres['canilla'], anc(Z_RODILLA),
                     eje_lado((0, 1, 0)), (-0.1, 1.15), MG996R, parado=0.70),
        Articulacion(f'tobillo_{lado}', nombres['canilla'], nombres['pie'], anc(Z_TOBILLO),
                     eje_lado((0, 1, 0)), (-0.8, 0.3), MG996R, parado=-0.35),
    ]
    for art in r.articulaciones[-5:]:
        r.anclas[art.hijo] = art.ancla
        r.padres[art.hijo] = art.padre
    for k in ('cadera_yaw', 'cadera_roll', 'muslo', 'canilla', 'pie'):
        r.masas_extra[nombres[k]] = (3.0, None)  # tornillos y cables


def tronco(r: Robot):
    r.anclas['tronco'] = np.array([0.0, 0.0, (TRONCO_Z[0] + TRONCO_Z[1]) / 2])
    r.padres['tronco'] = None
    t = 'tronco'
    zb = TRONCO_Z[0]
    inx = (TRONCO_X[0] + 3.5, TRONCO_X[1] - 3.5)
    iny = (TRONCO_Y[0] + 3.5, TRONCO_Y[1] - 3.5)
    # Piso con alojamientos de rulemanes (paso 6)
    piso = placa_redondeada(TRONCO_X, TRONCO_Y, Z_PISO, 10)
    for s in (1, -1):
        piso -= cil((0, s * Y_PATA, Z_PISO[0] + 0.5), (0, s * Y_PATA, Z_PISO[1] + 1), 11.1)
        piso -= cil((0, s * Y_PATA, Z_PISO[0] - 1), (0, s * Y_PATA, Z_PISO[1] + 1), 6)
    r.agregar(Pieza('piso_tronco', t, piso, COLORES['pla'], 'impresa', 6, relleno=0.4,
                    descripcion='Piso del tronco con alojamientos 608'))
    # Batería y estante A (paso 7)
    lx, ly, lz, lm = LIPO_2S_1500
    r.agregar(Pieza('lipo_2s_1500', t, caja(-lx / 2, lx / 2, -ly / 2, ly / 2, zb, zb + lz), COLORES['lipo'],
                    'comprada', 7, masa=lm, descripcion='LiPo 2S 1500 mAh'))
    r.agregar(Pieza('tapa_bateria', t, caja(-lx / 2 - 2, lx / 2 + 2, -ly / 2 - 2, ly / 2 + 2, zb + lz, zb + lz + 1.5),
                    COLORES['pla'], 'impresa', 7, relleno=0.5, descripcion='Tapa y traba de la batería'))
    # El estante A apoya sobre las orejas de los servos yaw de cadera (se atornillan desde abajo).
    za = max(zb + lz + 2.0, Z_TAPA_YAW_CADERA + MG996R.orejas_desde_tapa + MG996R.espesor_orejas + 0.1)
    estante_a = placa_redondeada(inx, iny, (za, za + 3), 9)
    for s in (1, -1):  # paso del cuerpo de los servos yaw de cadera
        y0, y1 = sorted((s * (Y_PATA - 10.4), s * (Y_PATA + 10.4)))
        estante_a -= caja(-MG996R.eje_desde_punta - 0.5, MG996R.largo - MG996R.eje_desde_punta + 0.5, y0, y1, za - 5, za + 8)
    r.agregar(Pieza('estante_a', t, estante_a, COLORES['pla'], 'impresa', 7, relleno=0.4,
                    descripcion='Estante A'))
    xl, xw, xh, xm = XL4016   # girados: el largo va a lo ancho del tronco, uno detrás del otro
    for nombre, x0 in (('izq', -50), ('der', -6)):
        r.agregar(Pieza(f'xl4016_{nombre}', t, caja(x0, x0 + xw, -xl / 2, xl / 2, za + 3, za + 3 + xh),
                        COLORES['pcb_azul'], 'comprada', 7, masa=xm,
                        descripcion=f'Regulador XL4016 6,0 V (pata {"izquierda" if nombre == "izq" else "derecha"})'))
    mx_, my_, mz_, mm_ = PLACA_MCP3008
    r.agregar(Pieza('placa_mcp3008', t, caja(38, 38 + my_, -mx_ / 2, mx_ / 2, za + 3, za + 3 + mz_),
                    COLORES['pcb_verde'], 'comprada', 7, masa=mm_, descripcion='2× MCP3008 + divisores'))
    # Estante B: Pi, PCA9685, Mini560 e IMU (paso 8)
    zB = za + 3 + xh + 2.0
    estante_b = placa_redondeada(inx, iny, (zB, zB + 3), 9) - caja(20, 50, -9, 9, zB - 5, zB + 8)
    r.agregar(Pieza('estante_b', t, estante_b, COLORES['pla'], 'impresa', 8, relleno=0.4,
                    descripcion='Estante B'))
    px, py, pz, pm = PI_ZERO_2W
    r.agregar(Pieza('pi_zero_2w', t, caja(-14, -14 + py, -px / 2, px / 2, zB + 3, zB + 3 + pz), COLORES['pcb_verde'],
                    'comprada', 8, masa=pm + 3, descripcion='Raspberry Pi Zero 2 W'))
    cx_, cy_, cz_, cm_ = PCA9685
    r.agregar(Pieza('pca9685', t, caja(-44, -44 + cy_, -cx_ / 2, cx_ / 2, zB + 3, zB + 3 + cz_),
                    COLORES['pcb_azul'], 'comprada', 8, masa=cm_, descripcion='PCA9685 (16 PWM)'))
    nx, ny, nz, nm = MINI560
    r.agregar(Pieza('mini560_5v', t, caja(20, 20 + nx, 12, 12 + ny, zB + 3, zB + 3 + nz), COLORES['pcb_violeta'],
                    'comprada', 8, masa=nm, descripcion='Mini560 5 V (Pi)'))
    gx, gy, gz, gm = GY521
    r.agregar(Pieza('imu_gy521', t, caja(20, 20 + gx, -30, -30 + gy, zB + 3, zB + 3 + gz),
                    COLORES['pcb_azul'], 'comprada', 8, masa=gm, descripcion='IMU MPU-6050 (GY-521)'))
    # Servo yaw del cuello (paso 9)
    caja_c, _ = servo_en(MG90S, (X_CUELLO, 0, Z_YAW_TAPA), (0, 0, 1), (-1, 0, 0), 'lejos')
    r.agregar(Pieza('servo_cuello_yaw', t, caja_c, COLORES['mg90s'], 'comprada', 9, masa=MG90S.masa,
                    descripcion='MG90S cuello yaw'))
    # Carcasa (paso 11): cajita redondeada abierta abajo (apoya sobre el piso del tronco)
    cuerpo = (caja_redondeada(TRONCO_X, TRONCO_Y, TRONCO_Z, TRONCO_R)
              - caja_redondeada(TRONCO_X, TRONCO_Y, (TRONCO_Z[0] - 5, TRONCO_Z[1]), TRONCO_R, encoger=PARED))
    cuerpo -= caja(-200, 200, -200, 200, 0, TRONCO_Z[0] + 0.01)
    cuerpo -= cil((X_CUELLO, 0, Z_YAW_TAPA - 10), (X_CUELLO, 0, Z_YAW_TAPA + 40), 17)   # paso del cuello
    inferior = cuerpo - caja(-200, 200, -200, 200, Z_CORTE_TRONCO, 400)
    superior = cuerpo - caja(-200, 200, -200, 200, 0, Z_CORTE_TRONCO)
    r.agregar(Pieza('cuerpo_inferior', t, inferior, COLORES['carcasa'], 'impresa', 11, relleno=1.0,
                    descripcion='Carcasa del cuerpo, mitad inferior'))
    r.agregar(Pieza('cuerpo_superior', t, superior, COLORES['carcasa'], 'impresa', 11, relleno=1.0,
                    descripcion='Carcasa del cuerpo, mitad superior'))
    r.masas_extra['tronco'] = (40.0, None)  # cables, conectores, tornillos


def cuello_y_cabeza(r: Robot):
    x = X_CUELLO
    ZP0 = 295.0   # marco viejo; al final se baja todo a Z_PITCH
    # Cuello (paso 9): dos servos a la vista entre el tronco y la cabeza, como el original
    zy = ZP0 - (Z_PITCH - Z_YAW_TAPA)   # servo yaw, en el marco viejo
    r.anclas['cuello'] = np.array([x, 0, zy + 4.0])
    r.padres['cuello'] = 'tronco'
    _, horn_cy = servo_en(MG90S, (x, 0, zy), (0, 0, 1), (-1, 0, 0), 'lejos')
    r.agregar(Pieza('horn_cuello_yaw', 'cuello', horn_cy, COLORES['horn'], 'comprada', 9, masa=1.0))
    caja_cp, horn_cp = servo_en(MG90S, (x, 8.7, ZP0), (0, 1, 0), (0, 0, -1), 'eje')
    r.agregar(Pieza('servo_cuello_pitch', 'cuello', caja_cp, COLORES['mg90s'], 'comprada', 9, masa=MG90S.masa,
                    descripcion='MG90S cuello pitch'))
    base = cil((x, 0, zy + 7.8), (x, 0, zy + 10), 14)
    lados = caja(x - 8.5, x - 6.3, -14, 8.7, zy + 10, 300) + caja(x + 6.3, x + 8.5, -14, 8.7, zy + 10, 300)
    r.agregar(Pieza('soporte_cuello', 'cuello', base + lados, COLORES['pla'], 'impresa', 9,
                    descripcion='Soporte del cuello (yaw → pitch)'))

    # Cabeza (paso 10)
    r.anclas['cabeza'] = np.array([x, 0, ZP0])
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
    w = CAB_Y[1]
    base_cab = placa_redondeada((0, CAB_X[1] - 12), (-w + 8, w - 8), (302, 305), 8)
    base_cab -= caja(x - 10, x + 20, -22, 22, 300, 306)                        # paso del servo pitch
    soporte = (placa_y(16.5, 19.5, [(x, ZP0, 9)], [(x - 6, x + 10, 302, 305)])
               + placa_y(-18, -15, [(x, ZP0, 7)], [(x - 6, x + 10, 302, 305)])
               + cil((x, -15, ZP0), (x, -14, ZP0), 3))
    bx, bz = BISAGRA_BOCA
    soporte_bisagra = caja(bx - 4, bx + 4, -16.5, -14, 302, bz + 4) + cil((bx, -16.5, bz), (bx, -19, bz), 2)
    r.agregar(Pieza('base_cabeza', 'cabeza', base_cab + soporte + soporte_bisagra, COLORES['pla'], 'impresa', 10,
                    descripcion='Base de la cabeza: une el cuello, el servo de la boca y la cúpula'))
    # Servo de la boca, en el fondo de la cabeza, eje lateral
    caja_b, horn_b = servo_en(MG90S, (bx, 9, bz), (0, 1, 0), (1, 0, 0), 'lejos')
    r.agregar(Pieza('servo_boca', 'cabeza', caja_b, COLORES['mg90s'], 'comprada', 10, masa=MG90S.masa,
                    descripcion='MG90S boca'))
    # Pico superior: lámina amarilla ancha en el borde de abajo de la cúpula
    pico_sup = placa_redondeada((CAB_X[1] - 12, CAB_X[1] + 24), (-w + 4, w - 4), (302, 306), 12)
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
    mand = placa_redondeada((-12, CAB_X[1] - 12), (-w + 2, w - 2), (296, 301), 10)
    mand -= caja(x - 12, x + 36, -22, 22, 290, 310)   # paso del cuello (largo adelante: la boca abre con la cabeza inclinada)
    brazo_h = placa_y(16.8, 18.8, [(bx, bz, 6)], [(bx - 3, bx + 3, 296, bz)])
    brazo_p = placa_y(-18.8, -16.8, [(bx, bz, 6)], [(bx - 3, bx + 3, 296, bz)]) - cil((bx, -20, bz), (bx, -15, bz), 2.3)
    r.agregar(Pieza('mandibula', 'pico_inferior', mand + brazo_h + brazo_p, COLORES['carcasa'], 'impresa', 10,
                    relleno=0.4, descripcion='Mandíbula: piso de la cabeza, la mueve el servo de la boca'))
    pico_inf = placa_redondeada((CAB_X[1] - 12, CAB_X[1] + 22), (-w + 6, w - 6), (296, 301), 12)
    r.agregar(Pieza('pico_inferior', 'pico_inferior', pico_inf, COLORES['naranja'], 'impresa', 10, relleno=0.4,
                    descripcion='Pico inferior (amarillo, va pegado a la mandíbula)'))
    r.articulaciones += [
        Articulacion('cuello_yaw', 'tronco', 'cuello', r.anclas['cuello'], np.array([0, 0, 1.0]), (-1.5, 1.5), MG90S),
        Articulacion('cuello_pitch', 'cuello', 'cabeza', r.anclas['cabeza'], np.array([0, 1.0, 0]), (-0.5, 0.6),
                     MG90S),
        Articulacion('boca', 'cabeza', 'pico_inferior', r.anclas['pico_inferior'], np.array([0, 1.0, 0]),
                     (0.0, 0.3), MG90S),
    ]
    # Todo lo anterior está en el marco viejo (pitch en z = 295): bajarlo al cuello real.
    dz = Z_PITCH - 295.0
    for esl in ('cuello', 'cabeza', 'pico_inferior'):
        r.anclas[esl] = r.anclas[esl] + np.array([0, 0, dz])
    for p in r.piezas:
        if p.eslabon in ('cuello', 'cabeza', 'pico_inferior'):
            p.malla = p.malla.translate([0, 0, dz])
    for art in r.articulaciones[-3:]:
        art.ancla = art.ancla + np.array([0, 0, dz])


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


# Envolvente de la cabeza: cuánto puede abrir la boca según cuánto mira hacia abajo (cuello_pitch > 0).
# Con la cabeza baja y girada, el pico inferior abierto entra en la carcasa del tronco. Es una regla
# para el controlador (como la de las caderas): el modelo se verifica en esos vértices (cad/choques.py).
BOCA_MAX = [(0.0, 0.3), (0.45, 0.15), (0.6, 0.0)]   # (cuello_pitch hasta, boca máxima)


def boca_max(cuello_pitch):
    for pitch, boca in BOCA_MAX:
        if cuello_pitch <= pitch + 1e-9:
            return boca
    return 0.0


def orden_mjcf(r: Robot, hijos):
    """Orden de los cuerpos tal como quedan en el XML (recorrido en profundidad)."""
    out = []

    def visitar(esl):
        out.append(esl)
        for a in hijos[esl]:
            visitar(a.hijo)
    visitar('tronco')
    return out


# Piezas que definen la caja de choque de cada eslabón en el simulador. Los pies usan
# solo la paleta (caja chata) y el tronco la carcasa; el resto, todas sus piezas.
CHOQUE = {'pie': ('suela', 'paleta_pie'), 'tronco': ('cuerpo_inferior', 'cuerpo_superior', 'piso_tronco'),
          'cabeza': ('cupula', 'pico_superior'), 'pico_inferior': ('mandibula', 'pico_inferior')}
# Piezas que chocan en el modelo de entrenamiento (envolvente convexa de cada una). MuJoCo ya
# descarta los pares padre-hijo; el resto (patas entre sí, patas-tronco, cabeza-tronco) se simula.
CHOQUE_RL = ('suela', 'paleta_pie', 'soporte_', 'servo_', 'placa_yaw', 'cuerpo_', 'piso_tronco',
             'cupula', 'pico_superior', 'mandibula', 'pico_inferior')


def mjcf(r: Robot, mallas, props, hijos, modo):
    lineas = []

    def geoms(esl, sp):
        if modo == 'mallas':
            for p in r.piezas:
                if p.eslabon == esl:
                    rgba = ' '.join(f'{v / 255:.3f}' for v in color_rgba(p.color))
                    clase = 'suela' if p.nombre.startswith('suela') else 'visual'
                    lineas.append(f'{sp}  <geom mesh="{p.nombre}" class="{clase}" rgba="{rgba}"/>')
            return
        if modo == 'cascos':
            # Mallas visuales (sin física) para verlo en el visor de MuJoCo, más las envolventes de choque.
            for p in r.piezas:
                if p.eslabon == esl:
                    rgba = ' '.join(f'{v / 255:.3f}' for v in color_rgba(p.color))
                    lineas.append(f'{sp}  <geom mesh="{p.nombre}" class="visual" rgba="{rgba}"/>')
            for p in r.piezas:
                if p.eslabon == esl and any(p.nombre.startswith(n) for n in CHOQUE_RL) \
                        and not p.nombre.startswith('servo_cuello') and not p.nombre.startswith('servo_boca'):
                    lineas.append(f'{sp}  <geom mesh="{p.nombre}" class="casco"/>')
            return
        clave = next((k for k in CHOQUE if esl == k or esl.startswith(k + '_')), None)
        if clave is not None:
            # Una sola caja con las piezas elegidas (pie: la paleta; tronco: la carcasa; cabeza).
            piezas = [p for p in r.piezas if p.eslabon == esl
                      and any(p.nombre == n or p.nombre.startswith(n + '_') for n in CHOQUE[clave])]
            grupos = [(f'caja_{esl}', piezas)]
        else:
            # Patas y cuello: una caja por pieza (servo, cuna, horn). La caja única del eslabón
            # entero sobresalía por debajo de la suela con la pata flexionada.
            # Las piezas altas (cunas) se parten a la mitad del eslabón: la caja entera, inclinada,
            # bajaba más que la suela.
            z_medio = None
            hijo = next((a for a in r.articulaciones if a.padre == esl), None)
            if hijo is not None:
                z_medio = (r.anclas[esl][2] + hijo.ancla[2]) / 2
            grupos = []
            for p in r.piezas:
                if p.eslabon != esl:
                    continue
                bb = p.malla.bounding_box()
                if z_medio is not None and bb[2] < z_medio - 5 and bb[5] > z_medio + 5:
                    arriba, abajo = p.malla.split_by_plane((0, 0, 1), z_medio)
                    grupos.append((f'caja_{p.nombre}_sup', [np.array(arriba.bounding_box()).reshape(2, 3)]))
                    grupos.append((f'caja_{p.nombre}_inf', [np.array(abajo.bounding_box()).reshape(2, 3)]))
                else:
                    grupos.append((f'caja_{p.nombre}', [np.array(bb).reshape(2, 3)]))
        for nombre, piezas in grupos:
            pts = np.vstack([q if isinstance(q, np.ndarray) else mallas[q.nombre].bounds for q in piezas])
            lo, hi = pts.min(0), pts.max(0)
            c = ((lo + hi) / 2 - r.anclas[esl]) / 1000.0
            h = (hi - lo) / 2000.0
            lineas.append(f'{sp}  <geom name="{nombre}" type="box" class="choque" pos="{c[0]:.4f} {c[1]:.4f} {c[2]:.4f}" '
                          f'size="{h[0]:.4f} {h[1]:.4f} {h[2]:.4f}"/>')

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
        geoms(esl, sp)
        if esl == 'cabeza' and modo == 'cascos':
            # Cámara de la Pi detrás del lente, mirando hacia adelante (+x de la cabeza). fovy de la cámara v2.
            cx, cz = (CAB_X[1] - PARED + 1 - X_CUELLO) / 1000.0, (328 - 295) / 1000.0
            lineas.append(f'{sp}  <camera name="camara" pos="{cx:.4f} 0 {cz:.4f}" xyaxes="0 -1 0 0 0 1" fovy="49"/>')
        for a in hijos[esl]:
            cuerpo(a.hijo, a, sangria + 2)
        lineas.append(f'{sp}</body>')

    cuerpo('tronco', None, 4)
    nm = lambda s: 9.81 * s.stall_kgcm_6v / 100.0
    actuadores = '\n'.join(f'    <position name="{a.nombre}" joint="{a.nombre}" class="{a.servo.nombre.lower()}"/>'
                           for a in r.articulaciones)
    parado = ' '.join(f'{a.parado:g}' for a in r.articulaciones)
    assets = ''
    if modo in ('mallas', 'cascos'):
        assets = '  <asset>\n' + '\n'.join(f'    <mesh name="{p.nombre}" file="mallas/{p.nombre}.stl"/>' for p in r.piezas) + '\n  </asset>\n'
    return f'''<mujoco model="pato_criollo_v0">
  <compiler angle="radian" meshdir="."/>
  <option timestep="0.002" integrator="implicitfast"/>
  <default>
    <geom contype="0" conaffinity="0" density="0"/>
    <default class="visual"><geom type="mesh" group="2"/></default>
    <default class="suela"><geom type="mesh" contype="1" conaffinity="0" friction="0.9 0.02 0.01" group="2"/></default>
    <default class="choque"><geom contype="1" conaffinity="0" friction="0.9 0.02 0.01" group="3" rgba="0.9 0.5 0.2 0.3"/></default>
    <default class="casco"><geom type="mesh" contype="1" conaffinity="1" friction="0.9 0.02 0.01" group="3" rgba="0.9 0.5 0.2 0.3"/></default>
    <default class="mg996r">
      <joint damping="0.08" frictionloss="0.02" armature="0.002"/>
      <position kp="15" forcerange="-{nm(MG996R):.3f} {nm(MG996R):.3f}"/>
    </default>
    <default class="mg90s">
      <joint damping="0.01" frictionloss="0.005" armature="0.0005"/>
      <position kp="4" forcerange="-{nm(MG90S):.3f} {nm(MG90S):.3f}"/>
    </default>
  </default>
{assets}  <worldbody>
    <light pos="0 0 2" dir="0 0 -1" directional="true"/>
    <geom name="suelo" type="plane" size="0 0 0.05" rgba="0.75 0.78 0.8 1" contype="1" conaffinity="1" friction="0.9 0.02 0.01"/>
{'''    <body name="pelota" mocap="true" pos="1 0 0.03">
      <geom type="sphere" size="0.03" rgba="0.9 0.2 0.2 1" contype="0" conaffinity="0"/>
    </body>
''' if modo == 'cascos' else ''}{chr(10).join(lineas)}
  </worldbody>
{'''  <contact>
    <!-- Abuelo-nieto: la envolvente convexa de la mandíbula (que es cóncava) roza el soporte del cuello. -->
    <exclude body1="cuello" body2="pico_inferior"/>
  </contact>
''' if modo == 'cascos' else ''}  <actuator>
{actuadores}
  </actuator>
  <keyframe>
    <key name="parado" qpos="0 0 {r.anclas['tronco'][2] / 1000.0:.4f} 1 0 0 0 {parado}" ctrl="{parado}"/>
  </keyframe>
</mujoco>
'''


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
        'envolventes': {'boca_max_por_cuello_pitch': BOCA_MAX},
    }
    (SALIDA / 'robot.json').write_text(json.dumps(datos, ensure_ascii=False, indent=1))

    # MJCF: pato.xml con las mallas (visual) y pato_sim.xml con cajas de choque (simulador web)
    props = {e: propiedades_eslabon(r, e, mallas) for e in orden}
    hijos = {e: [a for a in r.articulaciones if a.padre == e] for e in orden}
    for p in r.piezas:
        mt = mallas[p.nombre].copy()
        mt.apply_translation(-r.anclas[p.eslabon])
        mt.apply_scale(1 / 1000.0)
        mt.export(SALIDA / 'mjcf' / 'mallas' / f'{p.nombre}.stl')
    (SALIDA / 'mjcf' / 'pato.xml').write_text(mjcf(r, mallas, props, hijos, 'mallas'))
    (SALIDA / 'mjcf' / 'pato_sim.xml').write_text(mjcf(r, mallas, props, hijos, 'cajas'))
    (SALIDA / 'mjcf' / 'pato_rl.xml').write_text(mjcf(r, mallas, props, hijos, 'cascos'))
    datos['orden_mjcf'] = orden_mjcf(r, hijos)
    (SALIDA / 'robot.json').write_text(json.dumps(datos, ensure_ascii=False, indent=1))

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
