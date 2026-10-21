// node screenshot.mjs <web build dir> <out.png> <app args...>: the labs page in headless chromium, the picture taken
// once the app says its frames are done
import { readFile } from 'node:fs/promises';
import { createServer } from 'node:http';
import { extname, join } from 'node:path';
import { chromium } from 'playwright';

const [root, out, ...args] = process.argv.slice(2);
const types = { '.html': 'text/html', '.js': 'text/javascript', '.wasm': 'application/wasm', '.data': 'application/octet-stream' };
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

const browser = await chromium.launch();
const page = await browser.newPage({ viewport: { width: 1280, height: 800 } });
const errors = [];
page.on('console', (message) => {
    // the picture's own read back stalls the gpu, which the driver reports
    if (message.text().includes('GPU stall due to ReadPixels')) return;
    if (message.type() === 'error' || message.type() === 'warning') errors.push(`${message.type()}: ${message.text()}`);
});
page.on('pageerror', (error) => errors.push(`pageerror: ${error.message}`));
const query = new URLSearchParams(args.map((arg) => ['a', arg])).toString();
await page.goto(`http://127.0.0.1:${server.address().port}/tumble_labs.html?${query}`);
try {
    await page.waitForFunction(() => window.labsScreenshotReady === true, null, { timeout: 120_000 });
    await page.screenshot({ path: out });
    console.log(`saved ${out}`);
} catch (error) {
    console.log(`no screenshot: ${error.message}`);
    process.exitCode = 1;
}
for (const line of errors.slice(0, 30)) console.log(line);
await browser.close();
server.close();
