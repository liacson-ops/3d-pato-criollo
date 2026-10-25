// Arma salida/pato.html: visor + modelo + datos en un solo archivo, sin dependencias externas.
import { build } from 'esbuild';
import { readFileSync, writeFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const aqui = dirname(fileURLToPath(import.meta.url));
const salida = join(aqui, '..', 'salida');
const js = await build({ entryPoints: [join(aqui, 'main.js')], bundle: true, minify: true, format: 'iife',
  write: false, target: 'es2020', legalComments: 'none' });
const reemplazos = {
  '/*VISOR*/': js.outputFiles[0].text.replaceAll('</script', '<\\/script'),
  '/*ROBOT*/': readFileSync(join(salida, 'robot.json'), 'utf8').replaceAll('</', '<\\/'),
  '/*GLB*/': readFileSync(join(salida, 'pato.glb')).toString('base64'),
};
let html = readFileSync(join(aqui, 'plantilla.html'), 'utf8');
for (const [marca, valor] of Object.entries(reemplazos)) html = html.split(marca).join(valor);
writeFileSync(join(salida, 'pato.html'), html);
console.log(`salida/pato.html: ${(html.length / 1e6).toFixed(1)} MB`);
