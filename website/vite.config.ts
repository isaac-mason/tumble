import { readdirSync } from 'node:fs';
import { defineConfig } from 'vite';

// the site's page and a page per example
const input: Record<string, string> = { index: './index.html' };
for (const file of readdirSync('./examples')) {
    if (file.endsWith('.html')) input[`examples/${file.replace('.html', '')}`] = `./examples/${file}`;
}

export default defineConfig({
    base: './',
    // the physics worker awaits the wasm at its top level, which only a module worker can
    worker: { format: 'es' },
    build: { outDir: './dist', target: 'esnext', rollupOptions: { input } },
});
