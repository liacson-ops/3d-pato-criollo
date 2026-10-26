// Simulador físico del Pato Criollo en el navegador: MuJoCo (WebAssembly) + three.js.
// Se empaqueta en salida/simulador.html con el motor, el modelo y las mallas adentro.
import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import loadMujoco from '@mujoco/mujoco';

const ROBOT = window.PATO_ROBOT;
const BOCA_MAX = (ROBOT.envolventes && ROBOT.envolventes.boca_max_por_cuello_pitch) || [[0, 0.3]];
function bocaMax(pitch) { for (const [p, b] of BOCA_MAX) if (pitch <= p + 1e-9) return b; return 0; }
const $ = (id) => document.getElementById(id);
const NGAIN = 10, NBIAS = 10;
const PARADO = { cadera_pitch: -0.35, rodilla: 0.7, tobillo: -0.35 };
const ART = ROBOT.articulaciones;                     // mismo orden que los actuadores del XML
const tipo = (n) => n.replace(/_(izq|der)$/, '');
const lado = (n) => (n.endsWith('_der') ? -1 : 1);

function b64ABytes(b64) {
  const bin = atob(b64);
  const out = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
  return out;
}

// ---------------------------------------------------------------------------
// Marcha programada a mano (la misma de sim/marcha.py)
// ---------------------------------------------------------------------------
const marcha = { frecuencia: 2.0, balanceo: 0.05, levante: 0.15, zancada: 0.24, giro: 0.0, rampa: 1.0 };

function objetivosMarcha(t, vel, q) {
  const fase = 2 * Math.PI * marcha.frecuencia * t;
  const g = Math.min(1, t / marcha.rampa) * vel;
  const sen = Math.sin(fase);
  for (const a of ART) {
    const s = lado(a.nombre), k = tipo(a.nombre);
    const swing = Math.max(0, s * sen);
    const lev = marcha.levante * swing * g;
    const zanc = marcha.zancada * g * s * Math.cos(fase);
    let v = 0;
    if (k === 'cadera_yaw') v = marcha.giro * g * s;
    else if (k === 'cadera_roll') v = -marcha.balanceo * sen * g * s;   // signo medido en MuJoCo: ver sim/marcha.py
    else if (k === 'cadera_pitch') v = PARADO.cadera_pitch - lev / 2 + zanc;
    else if (k === 'rodilla') v = PARADO.rodilla + lev;
    else if (k === 'tobillo') v = PARADO.tobillo - lev / 2 - zanc;
    q[a.nombre] = v;
  }
  return q;
}

function objetivosParado(q) {
  for (const a of ART) q[a.nombre] = PARADO[tipo(a.nombre)] ?? 0;
  return q;
}

// ---------------------------------------------------------------------------
// Escena
// ---------------------------------------------------------------------------
const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
$('lienzo').appendChild(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xf3f1ec);
const camera = new THREE.PerspectiveCamera(35, 1, 0.01, 20);
camera.position.set(0.55, 0.35, 0.65);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(0, 0.15, 0);
scene.add(new THREE.HemisphereLight(0xffffff, 0xb9b2a5, 1.3));
const sol = new THREE.DirectionalLight(0xffffff, 2.2);
sol.position.set(0.6, 1.2, 0.8);
sol.castShadow = true;
sol.shadow.mapSize.set(2048, 2048);
Object.assign(sol.shadow.camera, { left: -1, right: 1, top: 1, bottom: -1, near: 0.1, far: 4 });
scene.add(sol);
const piso = new THREE.Mesh(new THREE.PlaneGeometry(6, 6), new THREE.MeshStandardMaterial({ color: 0xe6e2d9 }));
piso.rotation.x = -Math.PI / 2;
piso.receiveShadow = true;
scene.add(piso);
const grilla = new THREE.GridHelper(6, 60, 0xcfc8bb, 0xdfdad0);
grilla.position.y = 0.0005;
scene.add(grilla);
const raiz = new THREE.Group();          // marco z-arriba (MuJoCo) dentro de three (y-arriba)
raiz.rotation.x = -Math.PI / 2;
scene.add(raiz);

// ---------------------------------------------------------------------------
// Estado
// ---------------------------------------------------------------------------
let mujoco, model, data, dt;
const nodos = new Map();                 // eslabón -> Object3D
let cajas = [];                          // wireframes de las cajas de choque
const estado = { modo: 'parado', vel: 1.0, velSim: 1.0, tMarcha: 0, empuje: 0, cayo: false, gains: null, manual: {} };
// Comportamientos de ejemplo (arriba de la marcha a mano): recorrido A→B, seguir la pelota, gestos, alzado.
const comp = { nombre: null, t0: 0, puntos: [], i: 0, cabeza: { yaw: 0, pitch: 0, boca: 0 }, gesto: null, tGesto: 0,
  yawSeg: 0, pitchSeg: 0, perdido: 0, pelota: [1, 0, 0.03], sinPies: 0, alzado: false, fase: 0 };
const SIGNO_GIRO = 1;                    // signo de marcha.giro que hace girar a la izquierda (se verifica con probar_sim)
const CAM = { ancho: 160, alto: 120, f: 60 / Math.tan(24.5 * Math.PI / 180) };   // cámara de la Pi, a escala
let pelotaMalla = null, marcadores = [];
let tSim = 0, acumulador = 0, ultimo = performance.now(), fpsReal = 1;
const q = {};
let idTronco = 1;

function cargarModelo() {
  if (model) { data.delete(); model.delete(); }
  model = mujoco.MjModel.from_xml_string(window.PATO_XML);
  data = new mujoco.MjData(model);
  dt = model.opt.timestep;
  idTronco = ROBOT.orden_mjcf.indexOf('tronco') + 1;
  estado.gains = { k: Array.from(model.actuator_gainprm), b: Array.from(model.actuator_biasprm) };
}

function reiniciar(pose = 'parado') {
  mujoco.mj_resetDataKeyframe(model, data, 0);
  if (pose === 'cero') for (let i = 7; i < model.nq; i++) data.qpos[i] = 0;
  mujoco.mj_forward(model, data);
  // Apoyar: bajar hasta que la caja más baja quede 3 mm sobre el piso.
  let zmin = Infinity;
  const size = model.geom_size, xpos = data.geom_xpos, tipos = model.geom_type;
  for (let g = 0; g < model.ngeom; g++) if (tipos[g] === 6) zmin = Math.min(zmin, xpos[3 * g + 2] - size[3 * g + 2]);
  data.qpos[2] -= zmin - 0.003;
  for (let i = 0; i < model.nv; i++) data.qvel[i] = 0;
  mujoco.mj_forward(model, data);
  tSim = 0; estado.tMarcha = 0; estado.cayo = false; estado.empuje = 0;
  comp.alzado = false; comp.sinPies = 0; comp.cabeza = { yaw: 0, pitch: 0, boca: 0 }; comp.gesto = null;
  const f = data.xfrc_applied; for (let k = 0; k < 6 * model.nbody; k++) f[k] = 0;
  restaurarTorque();
  if (estado.modo === 'suelto') estado.modo = 'parado';
  if (pose === 'cero') { estado.modo = 'manual'; for (const a of ART) estado.manual[a.nombre] = 0; sincronizarDeslizadores(); }
  else objetivosParado(estado.manual);
  actualizarBotones();
}

function soltarTorque() {
  const k = model.actuator_gainprm, b = model.actuator_biasprm;
  for (let i = 0; i < model.nu; i++) { k[i * NGAIN] = 0; b[i * NBIAS + 1] = 0; }
}
function restaurarTorque() {
  const k = model.actuator_gainprm, b = model.actuator_biasprm;
  for (let i = 0; i < model.nu; i++) { k[i * NGAIN] = estado.gains.k[i * NGAIN]; b[i * NBIAS + 1] = estado.gains.b[i * NBIAS + 1]; }
}

function paso() {
  comportamiento();
  // Objetivos de los servos
  if (estado.modo === 'caminar') { objetivosMarcha(estado.tMarcha, estado.vel, q); estado.tMarcha += dt; }
  else if (estado.modo === 'parado') objetivosParado(q);
  else if (estado.modo === 'manual') for (const a of ART) q[a.nombre] = estado.manual[a.nombre] ?? 0;
  if (comp.alzado) {                                        // en brazos: patas recogidas
    for (const a of ART) { const k = tipo(a.nombre); q[a.nombre] = k === 'cadera_pitch' ? -0.6 : k === 'rodilla' ? 1.0 : k === 'tobillo' ? -0.4 : 0; }
  }
  if (estado.modo !== 'manual') {                           // la cabeza la manejan los gestos / el seguidor
    q.cuello_yaw = comp.cabeza.yaw; q.cuello_pitch = comp.cabeza.pitch;
    q.boca = Math.min(comp.cabeza.boca, bocaMax(comp.cabeza.pitch));
  }
  if (estado.modo !== 'suelto') {
    const ctrl = data.ctrl;
    for (let i = 0; i < ART.length; i++) ctrl[i] = q[ART[i].nombre];
  }
  // Empujón: fuerza sobre el tronco durante 0,15 s
  const f = data.xfrc_applied;
  f[6 * idTronco] = estado.empuje > 0 ? estado.empujeN : 0;
  if (estado.empuje > 0) estado.empuje -= dt;
  mujoco.mj_step(model, data);
  tSim += dt;
  const xmat = data.xmat;
  if (xmat[9 * idTronco + 8] < 0.5 && !estado.cayo) { estado.cayo = true; $('aviso').textContent = 'Se cayó. Reiniciá o soltá el torque.'; }
}

// ---------------------------------------------------------------------------
// Comportamientos de ejemplo
// ---------------------------------------------------------------------------
function rumbo() { const m = data.xmat; return Math.atan2(m[9 * idTronco + 3], m[9 * idTronco]); }
function posicion() { return [data.xpos[3 * idTronco], data.xpos[3 * idTronco + 1]]; }
function envolver(a) { return ((a + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI; }
const acotar = (x, a, b) => Math.max(a, Math.min(b, x));
const rampa = (t, t0, t1, a, b) => (t <= t0 ? a : t >= t1 ? b : a + (b - a) * (0.5 - 0.5 * Math.cos(Math.PI * (t - t0) / (t1 - t0))));
const GESTOS = {
  cuac: { duracion: 3.0, f: (t) => ({ yaw: 0, pitch: t < 2.4 ? rampa(t, 0, 0.6, 0, -0.5) : rampa(t, 2.4, 3, -0.5, 0), boca: t < 1.9 ? rampa(t, 0.5, 0.9, 0, 0.3) : rampa(t, 1.9, 2.3, 0.3, 0) }) },
  mirar: { duracion: 3.0, f: (t) => ({ yaw: Math.sin(2 * Math.PI * t / 3), pitch: t < 2.5 ? rampa(t, 0, 0.5, 0, 0.25) : rampa(t, 2.5, 3, 0.25, 0), boca: 0 }) },
};

function pixelPelota() {
  // Proyección de la pelota en la cámara de la cabeza (detrás del lente, mirando +x de la cabeza).
  const b = ROBOT.orden_mjcf.indexOf('cabeza') + 1;
  const m = data.xmat, p = data.xpos;
  const R = [m[9 * b], m[9 * b + 1], m[9 * b + 2], m[9 * b + 3], m[9 * b + 4], m[9 * b + 5], m[9 * b + 6], m[9 * b + 7], m[9 * b + 8]];
  const cx = [0.0744, 0, 0.033];                     // cámara en el marco de la cabeza (m)
  const cam = [p[3 * b] + R[0] * cx[0] + R[2] * cx[2], p[3 * b + 1] + R[3] * cx[0] + R[5] * cx[2], p[3 * b + 2] + R[6] * cx[0] + R[8] * cx[2]];
  const d = [comp.pelota[0] - cam[0], comp.pelota[1] - cam[1], comp.pelota[2] - cam[2]];
  const adelante = R[0] * d[0] + R[3] * d[1] + R[6] * d[2];
  const izq = R[1] * d[0] + R[4] * d[1] + R[7] * d[2];
  const arriba = R[2] * d[0] + R[5] * d[1] + R[8] * d[2];
  if (adelante < 0.02) return null;
  const u = CAM.ancho / 2 - CAM.f * izq / adelante, v = CAM.alto / 2 - CAM.f * arriba / adelante;
  if (u < 0 || u >= CAM.ancho || v < 0 || v >= CAM.alto) return null;
  return { u, v, r: CAM.f * 0.03 / adelante, dist: adelante };
}

function dibujarCamara(det) {
  const c = $('camara'); if (!c) return;
  const g = c.getContext('2d');
  g.fillStyle = '#d8d4cb'; g.fillRect(0, 0, c.width, c.height);
  g.strokeStyle = '#9a9388'; g.beginPath(); g.moveTo(c.width / 2, 0); g.lineTo(c.width / 2, c.height); g.moveTo(0, c.height / 2); g.lineTo(c.width, c.height / 2); g.stroke();
  if (det) { g.fillStyle = '#d1343b'; g.beginPath(); g.arc(det.u, det.v, Math.max(2, det.r), 0, 2 * Math.PI); g.fill(); }
}

function piesEnElAire() {
  // Ningún pie a menos de 1 cm del piso (cajas de choque de los pies).
  let zmin = Infinity;
  const gp = data.geom_xpos, gs = model.geom_size;
  for (const gi of comp.geomPies || []) zmin = Math.min(zmin, gp[3 * gi + 2] - gs[3 * gi + 2]);
  return zmin > 0.01;
}

function comportamiento() {
  const t = tSim;
  // Gesto de la cabeza en curso
  if (comp.gesto) {
    const g = GESTOS[comp.gesto];
    Object.assign(comp.cabeza, g.f(t - comp.tGesto));
    if (t - comp.tGesto >= g.duracion) { comp.gesto = null; comp.cabeza = { yaw: 0, pitch: 0, boca: 0 }; if (comp.nombre === 'cuac' || comp.nombre === 'mirar') terminarComportamiento('listo'); }
  }
  // Alzado: medio segundo sin pies apoyados -> recoge las patas y mira arriba; apoyado -> sigue
  comp.sinPies = piesEnElAire() ? comp.sinPies + dt : 0;
  if (!comp.alzado && comp.sinPies > 0.5 && !estado.cayo) { comp.alzado = true; comp.cabeza = { yaw: 0, pitch: -0.4, boca: 0 }; $('comportamiento').textContent = 'Lo alzaron: recoge las patas y mira hacia arriba.'; }
  if (comp.alzado && comp.sinPies === 0 && t - comp.tApoyo > 0) { comp.alzado = false; comp.cabeza = { yaw: 0, pitch: 0, boca: 0 }; if (comp.nombre === 'alzar') terminarComportamiento('lo apoyaron: sigue'); }
  if (comp.nombre === 'alzar') mano(t - comp.t0);
  if (comp.nombre === 'recorrido') {
    const [x, y] = posicion();
    while (comp.i < comp.puntos.length && Math.hypot(comp.puntos[comp.i][0] - x, comp.puntos[comp.i][1] - y) < 0.1) comp.i++;
    if (comp.i >= comp.puntos.length) { terminarComportamiento(`llegó a los ${comp.puntos.length} puntos`); return; }
    const [px, py] = comp.puntos[comp.i];
    const e = envolver(Math.atan2(py - y, px - x) - rumbo());
    marcha.giro = SIGNO_GIRO * acotar(0.6 * e, -0.25, 0.25);
    estado.vel = acotar(1 - Math.abs(e) / 0.8, 0.25, 1);
    estado.modo = 'caminar';
    $('comportamiento').textContent = `Va al punto ${comp.i + 1} de ${comp.puntos.length} (a ${(Math.hypot(px - x, py - y) * 100).toFixed(0)} cm).`;
  }
  if (comp.nombre === 'seguir') {
    // La pelota da una vuelta lenta; la cabeza la centra, el cuerpo gira y se acerca.
    const ang = 0.25 * (t - comp.t0);
    comp.pelota = [0.9 * Math.cos(ang), 0.9 * Math.sin(ang), 0.03];
    const det = pixelPelota();
    dibujarCamara(det);
    if (det) {
      comp.perdido = 0;
      const ex = Math.atan((det.u - CAM.ancho / 2) / CAM.f), ey = Math.atan((det.v - CAM.alto / 2) / CAM.f);
      comp.cabeza.yaw = acotar(comp.cabeza.yaw - 0.6 * ex, -1.5, 1.5);
      comp.cabeza.pitch = acotar(comp.cabeza.pitch + 0.6 * ey, -0.5, 0.6);
      const lejos = det.dist - 0.45;
      marcha.giro = Math.abs(comp.cabeza.yaw) > 0.35 ? SIGNO_GIRO * acotar(0.5 * comp.cabeza.yaw, -0.25, 0.25) : 0;
      const vx = Math.abs(comp.cabeza.yaw) < 0.9 ? acotar(lejos / 0.4, 0, 1) : 0;
      const mover = vx > 0.1 || marcha.giro !== 0;
      estado.vel = mover ? Math.max(vx, 0.3) : 0;
      estado.modo = mover ? 'caminar' : 'parado';
      comp.cabeza.boca = lejos < 0.05 ? 0.3 : 0;
      $('comportamiento').textContent = `Pelota a ${(det.dist * 100).toFixed(0)} cm, ${Math.abs(det.u - 80) < 24 ? 'centrada' : 'buscándola con la cabeza'}.`;
    } else {
      comp.perdido += dt;
      if (comp.perdido > 0.5) { comp.cabeza.yaw = acotar(comp.cabeza.yaw + (Math.floor(comp.perdido / 1.5) % 2 === 0 ? 0.6 : -0.6) * dt, -1.2, 1.2); comp.cabeza.pitch = 0.1; }
      estado.modo = 'parado';
      $('comportamiento').textContent = 'No ve la pelota: barre con la cabeza.';
    }
  }
}

function mano(t, dur = 4.0, alto = 0.2) {
  // Una mano que lo levanta 20 cm en 1 s, lo sostiene y lo apoya. Fuerza sobre el tronco.
  const f = data.xfrc_applied, b = idTronco;
  if (t < 1.0 || t > 1.0 + dur + 1.5) { for (let k = 0; k < 6; k++) f[6 * b + k] = 0; if (t > 1.0 + dur + 1.5) comp.tApoyo = -1; return; }
  const s = t < 1 + dur ? Math.min(1, (t - 1) / 1) : Math.max(0, 1 - (t - 1 - dur) / 1);
  const z = data.xpos[3 * b + 2], zObj = 0.19 + alto * s;
  const masa = 1.36, vz = data.cvel[6 * b + 5];
  f[6 * b + 2] = masa * 9.81 + 400 * (zObj - z) - 4 * vz;
  f[6 * b] = -2 * data.cvel[6 * b + 3]; f[6 * b + 1] = -2 * data.cvel[6 * b + 4];
  const m = data.xmat;                                 // lo mantiene derecho: torque hacia la vertical
  const zx = m[9 * b + 2], zy = m[9 * b + 5];
  f[6 * b + 3] = 0.3 * (-zy) - 0.02 * data.cvel[6 * b]; f[6 * b + 4] = 0.3 * zx - 0.02 * data.cvel[6 * b + 1]; f[6 * b + 5] = 0;
  comp.tApoyo = tSim + 0.5;
}

function terminarComportamiento(msg) {
  if (comp.nombre === 'recorrido' || comp.nombre === 'seguir') { estado.modo = 'parado'; estado.vel = 1; marcha.giro = 0; }
  if (comp.nombre === 'seguir') { pelotaMalla.visible = false; dibujarCamara(null); }
  if (comp.nombre === 'recorrido') for (const m of marcadores) m.visible = false;
  if (comp.nombre === 'alzar') { const f = data.xfrc_applied; for (let k = 0; k < 6; k++) f[6 * idTronco + k] = 0; }
  comp.nombre = null;
  $('comportamiento').textContent = msg ? `Ejemplo terminado: ${msg}.` : '';
  actualizarBotones();
}

function iniciarComportamiento(nombre) {
  if (comp.nombre) terminarComportamiento();
  $('aviso').textContent = '';
  comp.nombre = nombre; comp.t0 = tSim; comp.tApoyo = -1;
  if (nombre === 'cuac' || nombre === 'mirar') { comp.gesto = nombre; comp.tGesto = tSim; }
  if (nombre === 'recorrido') {
    const [x, y] = posicion(), r = rumbo();
    const rel = [[0.6, 0], [0.6, 0.6], [0, 0.6]];
    comp.puntos = rel.map(([a, b]) => [x + a * Math.cos(r) - b * Math.sin(r), y + a * Math.sin(r) + b * Math.cos(r)]);
    comp.i = 0;
    comp.puntos.forEach(([px, py], i) => { marcadores[i].position.set(px, py, 0.001); marcadores[i].visible = true; });
  }
  if (nombre === 'seguir') { pelotaMalla.visible = true; comp.perdido = 0; comp.cabeza = { yaw: 0, pitch: 0.15, boca: 0 }; }
  if (nombre === 'alzar') { estado.modo = 'parado'; }
  actualizarBotones();
}

function sincronizarVista() {
  if (pelotaMalla) pelotaMalla.position.set(comp.pelota[0], comp.pelota[1], comp.pelota[2]);
  const xpos = data.xpos, xquat = data.xquat;
  for (let i = 0; i < ROBOT.orden_mjcf.length; i++) {
    const n = nodos.get(ROBOT.orden_mjcf[i]);
    if (!n) continue;
    const b = i + 1;
    n.position.set(xpos[3 * b], xpos[3 * b + 1], xpos[3 * b + 2]);
    n.quaternion.set(xquat[4 * b + 1], xquat[4 * b + 2], xquat[4 * b + 3], xquat[4 * b]);
  }
  if (cajas.length) {
    const gp = data.geom_xpos, gm = data.geom_xmat;
    for (const { malla, g } of cajas) {
      malla.position.set(gp[3 * g], gp[3 * g + 1], gp[3 * g + 2]);
      const m = new THREE.Matrix4().set(gm[9 * g], gm[9 * g + 1], gm[9 * g + 2], 0, gm[9 * g + 3], gm[9 * g + 4], gm[9 * g + 5], 0,
        gm[9 * g + 6], gm[9 * g + 7], gm[9 * g + 8], 0, 0, 0, 0, 1);
      malla.quaternion.setFromRotationMatrix(m);
    }
  }
  // La cámara sigue al tronco
  const t = new THREE.Vector3(data.xpos[3 * idTronco], data.xpos[3 * idTronco + 1], data.xpos[3 * idTronco + 2]);
  raiz.localToWorld(t);
  const d = t.clone().sub(controls.target);
  controls.target.add(d.multiplyScalar(0.08));
  camera.position.add(d);
}

function lecturas() {
  const fuerza = data.actuator_force, rango = model.actuator_forcerange;
  const filas = $('torques').children;
  for (let i = 0; i < ART.length; i++) {
    const uso = Math.min(1, Math.abs(fuerza[i]) / Math.abs(rango[2 * i + 1]));
    const fila = filas[i];
    fila.querySelector('.barra > i').style.width = `${(uso * 100).toFixed(0)}%`;
    fila.querySelector('.barra > i').style.background = uso > 0.9 ? '#d1343b' : uso > 0.6 ? '#e8730c' : '#2e7d32';
    fila.querySelector('.valor').textContent = `${(uso * 100).toFixed(0)} %`;
  }
  const x = data.xpos[3 * idTronco], z = data.xpos[3 * idTronco + 2];
  $('estado').textContent = `t = ${tSim.toFixed(1)} s · ${fpsReal.toFixed(2)}× tiempo real · avance ${(x * 100).toFixed(1)} cm · tronco a ${(z * 100).toFixed(1)} cm`;
}

// ---------------------------------------------------------------------------
// Interfaz
// ---------------------------------------------------------------------------
function actualizarBotones() {
  for (const b of document.querySelectorAll('[data-modo]')) b.setAttribute('aria-pressed', b.dataset.modo === estado.modo);
  for (const b of document.querySelectorAll('[data-comp]')) b.setAttribute('aria-pressed', b.dataset.comp === comp.nombre);
  $('manual').hidden = estado.modo !== 'manual';
}

function sincronizarDeslizadores() {
  for (const a of ART) {
    const s = $(`j_${a.nombre}`);
    if (s) { s.value = estado.manual[a.nombre] ?? 0; s.nextElementSibling.textContent = `${((estado.manual[a.nombre] ?? 0) * 180 / Math.PI).toFixed(0)}°`; }
  }
}

function armarInterfaz() {
  for (const b of document.querySelectorAll('[data-modo]')) b.onclick = () => {
    const m = b.dataset.modo;
    $('aviso').textContent = '';
    if (m === 'suelto') soltarTorque(); else if (estado.modo === 'suelto') restaurarTorque();
    if (m === 'caminar' && estado.modo !== 'caminar') estado.tMarcha = 0;
    if (m === 'manual') { objetivosParado(estado.manual); sincronizarDeslizadores(); }
    estado.modo = m;
    actualizarBotones();
  };
  for (const b of document.querySelectorAll('[data-comp]')) b.onclick = () => { if (comp.nombre === b.dataset.comp) terminarComportamiento(); else iniciarComportamiento(b.dataset.comp); };
  $('reiniciar').onclick = () => { $('aviso').textContent = ''; if (comp.nombre) terminarComportamiento(); reiniciar('parado'); };
  $('acostar').onclick = () => { $('aviso').textContent = ''; reiniciar('cero'); };
  $('empujar').onclick = () => { estado.empuje = 0.15; estado.empujeN = +$('fuerza').value; };
  $('fuerza').oninput = (e) => { e.target.nextElementSibling.textContent = `${e.target.value} N`; };
  for (const [id, clave] of [['frecuencia', 'frecuencia'], ['balanceo', 'balanceo'], ['levante', 'levante'], ['zancada', 'zancada'], ['giro', 'giro']]) {
    const s = $(id);
    s.value = marcha[clave];
    s.oninput = () => { marcha[clave] = +s.value; s.nextElementSibling.textContent = s.value; };
    s.nextElementSibling.textContent = s.value;
  }
  $('velsim').oninput = (e) => { estado.velSim = +e.target.value; e.target.nextElementSibling.textContent = `${e.target.value}×`; };
  $('cajas').onchange = (e) => { for (const c of cajas) c.malla.visible = e.target.checked; };
  // Barras de torque y deslizadores manuales
  const ul = $('torques');
  for (const a of ART) {
    const li = document.createElement('li');
    li.innerHTML = `<span>${a.nombre.replace(/_/g, ' ')}</span><span class="barra"><i></i></span><span class="valor">0 %</span>`;
    ul.appendChild(li);
  }
  const man = $('manual_lista');
  for (const a of ART) {
    const div = document.createElement('div');
    div.className = 'desl';
    div.innerHTML = `<label for="j_${a.nombre}">${a.nombre.replace(/_/g, ' ')}</label>` +
      `<input id="j_${a.nombre}" type="range" min="${a.rango[0]}" max="${a.rango[1]}" step="0.01" value="0"><output>0°</output>`;
    man.appendChild(div);
    const s = div.querySelector('input');
    s.oninput = () => {
      let v = +s.value;
      // Envolvente de la cabeza: con la cabeza baja, la boca abre menos (ver cad/pato.py, BOCA_MAX).
      if (a.nombre === 'boca') { v = Math.min(v, bocaMax(estado.manual.cuello_pitch ?? 0)); s.value = v; }
      estado.manual[a.nombre] = v;
      s.nextElementSibling.textContent = `${(v * 180 / Math.PI).toFixed(0)}°`;
      if (a.nombre === 'cuello_pitch') { const b = $('j_boca'); if (b && +b.value > bocaMax(v)) { b.value = bocaMax(v); b.oninput(); } }
    };
  }
  window.addEventListener('keydown', (e) => {
    if (e.key === 'ArrowUp') $('b_caminar').click();
    if (e.key === 'ArrowDown') $('b_parado').click();
    if (e.key === 'ArrowLeft') { $('giro').value = 0.15; $('giro').oninput(); }
    if (e.key === 'ArrowRight') { $('giro').value = -0.15; $('giro').oninput(); }
    if (e.key === ' ') { e.preventDefault(); $('empujar').click(); }
  });
  window.addEventListener('keyup', (e) => { if (e.key === 'ArrowLeft' || e.key === 'ArrowRight') { $('giro').value = 0; $('giro').oninput(); } });
}

function crearPelotaYMarcadores() {
  pelotaMalla = new THREE.Mesh(new THREE.SphereGeometry(0.03, 24, 16), new THREE.MeshStandardMaterial({ color: 0xd1343b }));
  pelotaMalla.castShadow = true; pelotaMalla.visible = false;
  raiz.add(pelotaMalla);
  const colores = [0xe8730c, 0x2e7d32, 0x1e62b5];
  for (let i = 0; i < 3; i++) {
    const m = new THREE.Mesh(new THREE.RingGeometry(0.05, 0.07, 32), new THREE.MeshBasicMaterial({ color: colores[i], side: THREE.DoubleSide }));
    m.visible = false; raiz.add(m); marcadores.push(m);
  }
  comp.geomPies = [];
  for (let g = 0; g < model.ngeom; g++) {
    const b = model.geom_bodyid[g], nombre = ROBOT.orden_mjcf[b - 1];
    if (nombre === 'pie_izq' || nombre === 'pie_der') comp.geomPies.push(g);
  }
}

function crearCajas() {
  const size = model.geom_size, tipos = model.geom_type;
  const mat = new THREE.MeshBasicMaterial({ color: 0xe8730c, wireframe: true, transparent: true, opacity: 0.6 });
  for (let g = 0; g < model.ngeom; g++) {
    if (tipos[g] !== 6) continue;
    const malla = new THREE.Mesh(new THREE.BoxGeometry(2 * size[3 * g], 2 * size[3 * g + 1], 2 * size[3 * g + 2]), mat);
    malla.visible = false;
    raiz.add(malla);
    cajas.push({ malla, g });
  }
}

function ajustar() {
  const el = $('lienzo');
  renderer.setSize(el.clientWidth, el.clientHeight, false);
  camera.aspect = el.clientWidth / el.clientHeight;
  camera.updateProjectionMatrix();
}

async function principal() {
  mujoco = await loadMujoco({ wasmBinary: b64ABytes(window.MJ_WASM_B64).buffer });
  cargarModelo();
  await new Promise((ok, err) => new GLTFLoader().parse(b64ABytes(window.PATO_GLB).buffer, '', (gltf) => {
    const links = [];
    gltf.scene.traverse((o) => { if (ROBOT.eslabones[o.name]) links.push(o); });
    // Cada eslabón pasa a colgar directo de la raíz, con la pose que diga MuJoCo.
    for (const o of links) { raiz.add(o); o.position.set(0, 0, 0); o.quaternion.identity(); nodos.set(o.name, o); }
    gltf.scene.traverse((o) => { if (o.isMesh) { o.castShadow = o.receiveShadow = true; } });
    ok();
  }, err));
  crearCajas();
  crearPelotaYMarcadores();
  armarInterfaz();
  reiniciar('parado');
  new ResizeObserver(ajustar).observe($('lienzo'));
  ajustar();
  $('cargando').remove();
  renderer.setAnimationLoop(() => {
    const ahora = performance.now();
    const real = Math.min(0.05, (ahora - ultimo) / 1000);
    ultimo = ahora;
    acumulador += real * estado.velSim;
    const t0 = performance.now();
    let n = 0;
    while (acumulador >= dt && n < 200) { paso(); acumulador -= dt; n++; }
    const costo = (performance.now() - t0) / 1000;
    fpsReal = 0.9 * fpsReal + 0.1 * (n * dt / Math.max(real, 1e-3));
    if (costo > real && n >= 200) acumulador = 0;      // la máquina no da: no acumular atraso
    sincronizarVista();
    lecturas();
    controls.update();
    renderer.render(scene, camera);
  });
  window.PATO_SIM = {
    set(op) { Object.assign(estado, op); actualizarBotones(); },
    reiniciar,
    marcha, estado, comp, iniciar: iniciarComportamiento, terminar: terminarComportamiento,
    get t() { return tSim; }, get x() { return data.xpos[3 * idTronco]; }, get cayo() { return estado.cayo; },
    get rumbo() { return rumbo(); }, get posicion() { return posicion(); }, get alzado() { return comp.alzado; },
    pasos(n) { for (let i = 0; i < n; i++) paso(); sincronizarVista(); lecturas(); renderer.render(scene, camera); },
  };
  window.PATO_LISTO = true;
}

principal().catch((e) => { $('cargando').textContent = 'No se pudo cargar el simulador: ' + e.message; console.error(e); });
