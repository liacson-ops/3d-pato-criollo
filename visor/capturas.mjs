// Saca las imágenes de docs/img a partir de salida/pato.html (necesita Playwright con Chromium).
import { createRequire } from 'node:module';
import { mkdirSync } from 'node:fs';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { dirname, join } from 'node:path';

const require = createRequire(import.meta.url);
const { chromium } = require('playwright');
const aqui = dirname(fileURLToPath(import.meta.url));
const html = pathToFileURL(join(aqui, '..', 'salida', 'pato.html')).href;
const dir = join(aqui, '..', 'docs', 'img');
mkdirSync(dir, { recursive: true });

const browser = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
const page = await browser.newPage({ viewport: { width: 1400, height: 1000 }, deviceScaleFactor: 1 });
page.on('pageerror', (e) => console.error('error en la página:', e.message));
await page.goto(html);
await page.waitForFunction(() => window.PATO_LISTO === true, null, { timeout: 120000 });

async function foto(nombre, estado, { panel = false } = {}) {
  await page.evaluate(([e, p]) => {
    document.querySelector('.panel').style.display = p ? '' : 'none';
    window.PATO.set(e);
  }, [estado, panel]);
  await page.waitForTimeout(400);
  await page.screenshot({ path: join(dir, `${nombre}.png`) });
  console.log(nombre);
}

await foto('pato_tres_cuartos', { modo: 'armado', pose: 'parado', vista: 'tres_cuartos' });
await foto('pato_frente', { modo: 'armado', pose: 'parado', vista: 'frente' });
await foto('pato_costado', { modo: 'armado', pose: 'parado', vista: 'costado' });
await foto('pato_rayos_x', { modo: 'rayos', pose: 'parado', vista: 'tres_cuartos' });
await foto('pato_rayos_x_atras', { modo: 'rayos', pose: 'parado', vista: 'atras' });
await foto('pato_explotado', { modo: 'explotado', explota: 0.3, pose: 'cero', vista: 'todo' });
await foto('pato_saludo', { modo: 'armado', pose: 'saludo', vista: 'tres_cuartos' });
for (let n = 1; n <= 11; n++) {
  await foto(`armado_paso_${String(n).padStart(2, '0')}`, { modo: 'pasos', paso: n, pose: n === 11 ? 'parado' : 'cero', vista: n === 11 ? 'tres_cuartos' : 'auto' },
    { panel: true });
}
await browser.close();
