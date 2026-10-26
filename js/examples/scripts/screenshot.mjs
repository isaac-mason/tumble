// node scripts/screenshot.mjs <example> <out.png> [milliseconds] [key codes held...]: the built example (vite build) in
// headless chromium with webgpu, the page saved after the given time, the keys held from the start. no window opens
import { readFile } from 'node:fs/promises';
import { createServer } from 'node:http';
import { extname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';

const [example, out, wait = '3000', ...held] = process.argv.slice(2);
const root = fileURLToPath(new URL('../dist', import.meta.url));
const types = { '.html': 'text/html', '.js': 'text/javascript', '.wasm': 'application/wasm', '.css': 'text/css' };
const server = createServer(async (request, response) => {
    try {
        const path = decodeURIComponent(new URL(request.url, 'http://localhost').pathname);
        const body = await readFile(join(root, path));
        response.writeHead(200, { 'content-type': types[extname(path)] ?? 'application/octet-stream' });
        response.end(body);
    } catch {
        response.writeHead(404);
        response.end();
    }
}).listen(0, '127.0.0.1');
await new Promise((resolve) => server.once('listening', resolve));

const browser = await chromium.launch({ args: ['--enable-unsafe-webgpu', '--use-angle=metal', '--enable-gpu'] });
const page = await browser.newPage({ viewport: { width: 1280, height: 800 } });
const errors = [];
page.on('console', (message) => {
    if (message.type() === 'error') errors.push(`console: ${message.text()}`);
});
page.on('pageerror', (error) => errors.push(`pageerror: ${error.message}`));
await page.goto(`http://127.0.0.1:${server.address().port}/${example}.html`);
// the example listens once its wasm and renderer are up
await page.waitForTimeout(held.length > 0 ? 1000 : 0);
for (const key of held) await page.keyboard.down(key);
await page.waitForTimeout(Number(wait));
await page.screenshot({ path: out });
console.log(`saved ${out}`);
for (const line of errors.slice(0, 20)) console.log(line);
await browser.close();
server.close();
