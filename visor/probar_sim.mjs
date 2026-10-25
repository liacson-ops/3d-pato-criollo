// Prueba automática del simulador: carga, camina 5 s y saca una captura.
import { createRequire } from 'node:module';
import { mkdirSync } from 'node:fs';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { dirname, join } from 'node:path';
const { chromium } = createRequire(import.meta.url)('playwright');
const aqui = dirname(fileURLToPath(import.meta.url));
const browser = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
const page = await browser.newPage({ viewport: { width: 1400, height: 1000 } });
page.on('pageerror', (e) => console.error('error en la página:', e.message));
page.on('console', (m) => { if (m.type() === 'error') console.error('[consola]', m.text()); });
const t0 = Date.now();
await page.goto(pathToFileURL(join(aqui, '..', 'salida', 'simulador.html')).href);
await page.waitForFunction(() => window.PATO_LISTO === true, null, { timeout: 180000 });
console.log(`cargó en ${((Date.now() - t0) / 1000).toFixed(1)} s`);
const r = await page.evaluate(async () => {
  const S = window.PATO_SIM;
  S.pasos(500);                                  // medio segundo parado
  const parado = { z: S.estado, x0: S.x };
  S.set({ modo: 'caminar', tMarcha: 0 });
  const t1 = performance.now();
  S.pasos(2500);                                 // 5 s de marcha
  const ms = performance.now() - t1;
  return { avance_cm: (S.x - parado.x0) * 100, cayo: S.cayo, t: S.t, ms_por_segundo_simulado: ms / 5 };
});
console.log(JSON.stringify(r));
mkdirSync(join(aqui, '..', 'docs', 'img'), { recursive: true });
await page.screenshot({ path: join(aqui, '..', 'docs', 'img', 'simulador.png') });
const r2 = await page.evaluate(() => { const S = window.PATO_SIM; S.set({ empuje: 0.15, empujeN: 6 }); S.pasos(1500); return { cayo_tras_empujon_6N: S.cayo, x: S.x }; });
console.log(JSON.stringify(r2));
await browser.close();
