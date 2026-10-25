// Visor 3D del Pato Criollo. Se empaqueta con esbuild dentro de salida/pato.html,
// junto con el modelo (GLB) y robot.json, así el HTML abre sin internet.
import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

const ROBOT = window.PATO_ROBOT;
const GLB_B64 = window.PATO_GLB;
const $ = (id) => document.getElementById(id);

const POSES = {
  cero: {},
  parado: { cadera_pitch: -0.35, rodilla: 0.7, tobillo: -0.35 },
  cuclillas: { cadera_pitch: -0.65, rodilla: 1.3, tobillo: -0.65 },
  saludo: { cadera_pitch: -0.35, rodilla: 0.7, tobillo: -0.35, cuello_yaw: 0.6, cuello_pitch: -0.3, boca: 0.45 },
};

const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.05;
$('lienzo').appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0xf3f1ec);
const camera = new THREE.PerspectiveCamera(35, 1, 0.005, 10);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.target.set(0, 0.17, 0);

scene.add(new THREE.HemisphereLight(0xffffff, 0xb9b2a5, 1.3));
const sol = new THREE.DirectionalLight(0xffffff, 2.2);
sol.position.set(0.6, 1.2, 0.8);
sol.castShadow = true;
sol.shadow.mapSize.set(2048, 2048);
Object.assign(sol.shadow.camera, { left: -0.4, right: 0.4, top: 0.5, bottom: -0.2, near: 0.1, far: 3 });
scene.add(sol);
const relleno = new THREE.DirectionalLight(0xdfe8ff, 0.7);
relleno.position.set(-0.8, 0.5, -0.6);
scene.add(relleno);

const piso = new THREE.Mesh(new THREE.CircleGeometry(0.6, 64), new THREE.ShadowMaterial({ opacity: 0.18 }));
piso.rotation.x = -Math.PI / 2;
piso.receiveShadow = true;
scene.add(piso);
const grilla = new THREE.GridHelper(1.2, 24, 0xcfc8bb, 0xe2ddd3);
grilla.position.y = 0.0005;
scene.add(grilla);

// El modelo está en Z arriba (como el CAD y MuJoCo); three.js usa Y arriba.
const raiz = new THREE.Group();
raiz.rotation.x = -Math.PI / 2;
scene.add(raiz);

const piezas = new Map();      // nombre -> { obj, info, material, base }
const eslabones = new Map();   // nombre -> Object3D
let estado = { modo: 'armado', pose: 'parado', paso: 11, explota: 0 };

function b64ABuffer(b64) {
  const bin = atob(b64);
  const out = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
  return out.buffer;
}

new GLTFLoader().parse(b64ABuffer(GLB_B64), '', (gltf) => {
  raiz.add(gltf.scene);
  const info = Object.fromEntries(ROBOT.piezas.map((p) => [p.nombre, p]));
  gltf.scene.traverse((o) => {
    if (ROBOT.eslabones[o.name]) eslabones.set(o.name, o);
    if (o.isMesh) {
      const nombre = info[o.name] ? o.name : o.parent && info[o.parent.name] ? o.parent.name : null;
      if (!nombre) return;
      o.castShadow = o.receiveShadow = true;
      o.material = o.material.clone();
      o.material.transparent = true;
      piezas.set(nombre, { obj: o, info: info[nombre], base: o.position.clone() });
      o.userData.pieza = nombre;
    }
  });
  calcularExplosion();
  armarInterfaz();
  aplicar();
  encuadrar('tres_cuartos');
  window.PATO_LISTO = true;
});

// Dirección de explosión de cada pieza: desde el centro del robot hacia afuera,
// con más recorrido vertical para que las capas del tronco se separen.
function calcularExplosion() {
  aplicarPose('cero');
  raiz.updateMatrixWorld(true);
  const centro = new THREE.Vector3(0, 0.2, 0);
  for (const p of piezas.values()) {
    const caja = new THREE.Box3().setFromObject(p.obj);
    const c = caja.getCenter(new THREE.Vector3());
    const d = c.clone().sub(centro);
    d.y *= 1.6;
    if (p.info.eslabon === 'tronco') d.y += (p.info.paso - 7) * 0.012;
    p.dirMundo = d.multiplyScalar(0.9);
  }
}

function aplicarPose(nombre) {
  const valores = POSES[nombre] || {};
  for (const a of ROBOT.articulaciones) {
    const tipo = a.nombre.replace(/_(izq|der)$/, '');
    const ang = valores[a.nombre] ?? valores[tipo] ?? 0;
    const nodo = eslabones.get(a.hijo);
    if (nodo) nodo.quaternion.setFromAxisAngle(new THREE.Vector3(...a.eje), ang);
  }
  raiz.position.y = 0;
  raiz.updateMatrixWorld(true);
  // Apoyar en el piso: sube o baja el robot hasta que lo más bajo toque y = 0.
  const caja = new THREE.Box3();
  for (const p of piezas.values()) if (p.info.eslabon.startsWith('pie')) caja.expandByObject(p.obj);
  if (!caja.isEmpty()) raiz.position.y = -caja.min.y;
}

function aplicar() {
  for (const p of piezas.values()) p.obj.position.copy(p.base);
  aplicarPose(estado.pose);
  raiz.updateMatrixWorld(true);
  const enPasos = estado.modo === 'pasos';
  for (const p of piezas.values()) {
    const m = p.obj.material;
    const visible = !enPasos || p.info.paso <= estado.paso;
    p.obj.visible = visible;
    let opacidad = 1;
    if (estado.modo === 'rayos' && p.info.descripcion.startsWith('Carcasa')) opacidad = 0.12;
    if (estado.modo === 'rayos' && p.info.nombre === 'cupula') opacidad = 0.15;
    if (enPasos && p.info.paso < estado.paso) opacidad = 0.28;
    // En el paso a paso, las carcasas nuevas se ven translúcidas para mostrar lo que va adentro.
    if (enPasos && p.info.paso === estado.paso && estado.paso < 11 && /^(cupula|cuerpo_(inferior|superior))$/.test(p.info.nombre))
      opacidad = 0.3;
    m.opacity = opacidad;
    m.depthWrite = opacidad > 0.5;
    m.emissive = new THREE.Color(0, 0, 0);
    p.obj.castShadow = opacidad > 0.5;
  }
  // Explosión: en coordenadas de mundo, convertidas al marco local de cada pieza.
  const f = estado.modo === 'explotado' ? estado.explota : 0;
  if (f > 0) {
    for (const p of piezas.values()) {
      const destino = p.obj.getWorldPosition(new THREE.Vector3()).add(p.dirMundo.clone().multiplyScalar(f));
      p.obj.position.copy(p.obj.parent.worldToLocal(destino));
    }
    raiz.updateMatrixWorld(true);
  }
  $('paso-texto').textContent = enPasos ? `Paso ${estado.paso}: ${ROBOT.pasos[estado.paso]}` : '';
  $('paso-control').hidden = !enPasos;
  $('explota-control').hidden = estado.modo !== 'explotado';
  for (const b of document.querySelectorAll('[data-modo]')) b.setAttribute('aria-pressed', b.dataset.modo === estado.modo);
  for (const b of document.querySelectorAll('[data-pose]')) b.setAttribute('aria-pressed', b.dataset.pose === estado.pose);
  listaPaso();
}

function listaPaso() {
  const ul = $('lista');
  ul.innerHTML = '';
  if (estado.modo !== 'pasos') { ul.hidden = true; return; }
  ul.hidden = false;
  const vistos = new Set();
  for (const p of ROBOT.piezas) {
    if (p.paso !== estado.paso) continue;
    const clave = (p.descripcion || p.nombre).replace(/ \((izquierda|derecha)\)/, '');
    if (!p.descripcion || vistos.has(clave)) continue;
    vistos.add(clave);
    const li = document.createElement('li');
    const pares = ROBOT.piezas.filter((q) => q.paso === p.paso && q.descripcion === p.descripcion).length;
    li.innerHTML = `<span class="punto" style="background:${p.color}"></span>${clave}${pares > 1 ? ` <b>×${pares}</b>` : ''}`;
    ul.appendChild(li);
  }
}

const VISTAS = {
  tres_cuartos: [[0.42, 0.34, 0.5], [0, 0.17, 0]],
  frente: [[0.75, 0.2, 0], [0, 0.18, 0]],
  costado: [[0, 0.2, 0.75], [0, 0.18, 0]],
  atras: [[-0.45, 0.4, -0.45], [0, 0.18, 0]],
  arriba: [[0.05, 0.85, 0.02], [0, 0.17, 0]],
  pata: [[0.32, 0.12, 0.38], [0, 0.08, 0.04]],
  tronco: [[0.3, 0.42, 0.34], [0, 0.22, 0]],
  cabeza: [[0.3, 0.38, 0.28], [0.05, 0.33, 0]],
};

// Encuadra las piezas visibles (o solo las del paso actual) desde una vista ¾.
function encuadrarAuto(soloPaso) {
  const caja = new THREE.Box3();
  for (const p of piezas.values()) {
    if (!p.obj.visible) continue;
    if (soloPaso && p.info.paso !== estado.paso) continue;
    caja.expandByObject(p.obj);
  }
  if (caja.isEmpty()) return encuadrar('tres_cuartos');
  const c = caja.getCenter(new THREE.Vector3());
  const r = caja.getSize(new THREE.Vector3()).length() / 2;
  const dist = Math.max(0.2, (r / Math.sin(THREE.MathUtils.degToRad(camera.fov / 2))) * (soloPaso ? 1.5 : 0.95));
  camera.position.copy(c).add(new THREE.Vector3(0.55, 0.42, 0.72).normalize().multiplyScalar(dist));
  controls.target.copy(c);
  controls.update();
}

function encuadrar(nombre) {
  if (nombre === 'auto') return encuadrarAuto(true);
  if (nombre === 'todo') return encuadrarAuto(false);
  const [pos, obj] = VISTAS[nombre] || VISTAS.tres_cuartos;
  camera.position.set(...pos);
  controls.target.set(...obj);
  controls.update();
}

function armarInterfaz() {
  for (const b of document.querySelectorAll('[data-modo]')) b.onclick = () => { estado.modo = b.dataset.modo; aplicar(); };
  for (const b of document.querySelectorAll('[data-pose]')) b.onclick = () => { estado.pose = b.dataset.pose; aplicar(); };
  for (const b of document.querySelectorAll('[data-vista]')) b.onclick = () => encuadrar(b.dataset.vista);
  const paso = $('paso');
  paso.max = Object.keys(ROBOT.pasos).length;
  paso.oninput = () => { estado.paso = +paso.value; aplicar(); };
  $('explota').oninput = (e) => { estado.explota = +e.target.value; aplicar(); };
  $('anterior').onclick = () => { paso.value = Math.max(1, +paso.value - 1); paso.oninput(); };
  $('siguiente').onclick = () => { paso.value = Math.min(+paso.max, +paso.value + 1); paso.oninput(); };
  const masa = ROBOT.piezas.reduce((s, p) => s + p.masa_g, 0);
  $('masa').textContent = `${ROBOT.piezas.length} piezas · ${(masa / 1000).toFixed(2)} kg sin cables`;
}

// Info de la pieza bajo el puntero.
const rayo = new THREE.Raycaster();
const puntero = new THREE.Vector2();
renderer.domElement.addEventListener('pointermove', (e) => {
  const r = renderer.domElement.getBoundingClientRect();
  puntero.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
  rayo.setFromCamera(puntero, camera);
  const objs = [...piezas.values()].filter((p) => p.obj.visible && p.obj.material.opacity > 0.5).map((p) => p.obj);
  const hit = rayo.intersectObjects(objs, false)[0];
  const tip = $('tip');
  if (!hit) { tip.hidden = true; return; }
  const p = piezas.get(hit.object.userData.pieza).info;
  tip.hidden = false;
  tip.style.left = `${e.clientX - r.left + 14}px`;
  tip.style.top = `${e.clientY - r.top + 14}px`;
  tip.innerHTML = `<b>${p.descripcion || p.nombre}</b><br><code>${p.nombre}</code> · ${p.tipo} · ${p.masa_g} g · paso ${p.paso}`;
});

function ajustar() {
  const el = $('lienzo');
  const w = el.clientWidth, h = el.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  // Con el panel abierto a la izquierda (pantallas anchas), corre el centro de la imagen a la derecha.
  const panel = document.querySelector('.panel');
  const corrido = panel && panel.style.display !== 'none' && w > 700 ? panel.offsetWidth / 2 + 8 : 0;
  if (corrido) camera.setViewOffset(w, h, -corrido, 0, w, h);
  else camera.clearViewOffset();
  camera.updateProjectionMatrix();
}
new ResizeObserver(ajustar).observe($('lienzo'));
ajustar();
renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera); });

// Para las capturas automáticas (visor/capturas.mjs).
window.PATO = {
  set(op) { ajustar(); Object.assign(estado, op); aplicar(); if (op.vista) encuadrar(op.vista); renderer.render(scene, camera); },
};
