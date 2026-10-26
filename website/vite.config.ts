import { defineConfig } from 'vite';

export default defineConfig({
    base: './',
    // the page imports the examples' common code, whose own imports must find the same packages as the page's
    resolve: { dedupe: ['gpucat', 'math', 'tumble.js'] },
    build: { outDir: './dist', target: 'esnext' },
});
