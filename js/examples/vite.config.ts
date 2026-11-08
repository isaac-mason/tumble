import { readdirSync } from 'node:fs';
import { defineConfig } from 'vite';

// the gallery and a page per example
const input: Record<string, string> = {};
for (const file of readdirSync('./')) {
    if (file.endsWith('.html')) input[file.replace('.html', '')] = `./${file}`;
}

export default defineConfig({
    base: './',
    build: { outDir: './dist', target: 'esnext', rollupOptions: { input } },
    // the examples load the c++ samples' models from samples/assets
    server: { fs: { allow: ['../..'] }, open: '/index.html' },
});
