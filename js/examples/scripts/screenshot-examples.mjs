// node scripts/screenshot-examples.mjs [key...]: each example in src/examples.json, or those named, from the built
// examples (vite build) in headless chromium with webgpu, its interface hidden, saved after a few seconds to
// public/screenshots/<key>.png for the gallery's cards. no window opens
import { mkdir, readFile } from 'node:fs/promises';
import { createServer } from 'node:http';
import { extname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';

const WAIT = Number(process.env.SCREENSHOT_WAIT ?? 3000);
const site = fileURLToPath(new URL('../dist', import.meta.url));
const out = fileURLToPath(new URL('../public/screenshots', import.meta.url));
const examples = JSON.parse(await readFile(new URL('../src/examples.json', import.meta.url), 'utf8'));
const named = process.argv.slice(2);
const keys = named.length > 0 ? named : Object.keys(examples);
for (const key of keys) if (!(key in examples)) throw new Error(`no example ${key}`);
await mkdir(out, { recursive: true });

const types = { '.html': 'text/html', '.js': 'text/javascript', '.wasm': 'application/wasm', '.css': 'text/css' };
const server = createServer(async (request, response) => {
    try {
        const path = decodeURIComponent(new URL(request.url, 'http://localhost').pathname);
        const body = await readFile(join(site, path));
        response.writeHead(200, { 'content-type': types[extname(path)] ?? 'application/octet-stream' });
        response.end(body);
    } catch {
        response.writeHead(404);
        response.end();
    }
}).listen(0, '127.0.0.1');
await new Promise((resolve) => server.once('listening', resolve));

const browser = await chromium.launch({ args: ['--enable-unsafe-webgpu', '--use-angle=metal', '--enable-gpu'] });
const page = await browser.newPage({ viewport: { width: 1280, height: 720 } });
const errors = [];
page.on('pageerror', (error) => errors.push(error.message));
try {
    for (const key of keys) {
        errors.length = 0;
        await page.goto(`http://127.0.0.1:${server.address().port}/${key}.html`);
        await page.addStyleTag({
            // the scene's canvas only: the panel's graphs are canvases too
            content: 'body * { visibility: hidden !important; } body > canvas { visibility: visible !important; }',
        });
        await page.waitForTimeout(WAIT);
        if (errors.length > 0) throw new Error(`${key}: ${errors.join('\n')}`);
        await page
            .locator('body > canvas')
            .first()
            .screenshot({ path: join(out, `${key}.png`) });
        console.log(`saved ${key}`);
    }
} finally {
    await browser.close();
    server.close();
}
