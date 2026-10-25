// Simulador físico del Pato Criollo en el navegador: MuJoCo (WebAssembly) + three.js.
// Se empaqueta en salida/simulador.html con el motor, el modelo y las mallas adentro.
import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import loadMujoco from '@mujoco/mujoco';

const ROBOT = window.PATO_ROBOT;
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
const marcha = { frecuencia: 1.2, balanceo: 0.12, levante: 0.35, zancada: 0.26, giro: 0.0, rampa: 1.0 };

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
    else if (k === 'cadera_roll') v = marcha.balanceo * sen * g;
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
  // Objetivos de los servos
  if (estado.modo === 'caminar') { objetivosMarcha(estado.tMarcha, estado.vel, q); estado.tMarcha += dt; }
  else if (estado.modo === 'parado') objetivosParado(q);
  else if (estado.modo === 'manual') for (const a of ART) q[a.nombre] = estado.manual[a.nombre] ?? 0;
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

function sincronizarVista() {
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
  $('reiniciar').onclick = () => { $('aviso').textContent = ''; reiniciar('parado'); };
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
    s.oninput = () => { estado.manual[a.nombre] = +s.value; s.nextElementSibling.textContent = `${(+s.value * 180 / Math.PI).toFixed(0)}°`; };
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
    marcha, estado, get t() { return tSim; }, get x() { return data.xpos[3 * idTronco]; }, get cayo() { return estado.cayo; },
    pasos(n) { for (let i = 0; i < n; i++) paso(); sincronizarVista(); lecturas(); renderer.render(scene, camera); },
  };
  window.PATO_LISTO = true;
}

principal().catch((e) => { $('cargando').textContent = 'No se pudo cargar el simulador: ' + e.message; console.error(e); });
