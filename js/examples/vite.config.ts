import { readdirSync } from 'node:fs';
import { defineConfig } from 'vite';

// a page per example
const input: Record<string, string> = {};
for (const file of readdirSync('./')) {
    if (file.endsWith('.html')) input[file.replace('.html', '')] = `./${file}`;
}

export default defineConfig({
    base: './',
    build: { outDir: './dist', target: 'esnext', rollupOptions: { input } },
});
