// Arma salida/simulador.html: three.js + MuJoCo (WebAssembly) + modelo + mallas en un solo archivo.
import { build } from 'esbuild';
import { readFileSync, writeFileSync } from 'node:fs';
import { createRequire } from 'node:module';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const aqui = dirname(fileURLToPath(import.meta.url));
const salida = join(aqui, '..', 'salida');
const require = createRequire(import.meta.url);
const wasm = readFileSync(join(dirname(require.resolve('@mujoco/mujoco')), 'mujoco.wasm'));

const js = await build({
  entryPoints: [join(aqui, 'sim.js')], bundle: true, minify: true, format: 'iife', write: false, target: 'es2020',
  platform: 'browser', legalComments: 'none', logLevel: 'error',
  // mujoco.js trae ramas para Node que no se usan en el navegador
  define: { 'import.meta.url': '"https://pato.local/mujoco.js"' },
  external: ['module', 'fs', 'path', 'url', 'worker_threads', 'crypto'],
});
const esc = (s) => s.replaceAll('</script', '<\\/script');
const reemplazos = {
  '/*SIM*/': esc(js.outputFiles[0].text),
  '/*ROBOT*/': esc(readFileSync(join(salida, 'robot.json'), 'utf8')),
  '/*XML*/': esc(JSON.stringify(readFileSync(join(salida, 'mjcf', 'pato_sim.xml'), 'utf8'))),
  '/*GLB*/': readFileSync(join(salida, 'pato.glb')).toString('base64'),
  '/*WASM*/': wasm.toString('base64'),
};
let html = readFileSync(join(aqui, 'sim.html'), 'utf8');
for (const [marca, valor] of Object.entries(reemplazos)) html = html.split(marca).join(valor);
writeFileSync(join(salida, 'simulador.html'), html);
console.log(`salida/simulador.html: ${(html.length / 1e6).toFixed(1)} MB`);
