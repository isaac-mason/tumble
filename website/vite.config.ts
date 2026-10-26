import { defineConfig } from 'vite';

export default defineConfig({
    base: './',
    // the page imports the examples' common code, whose own imports must find the same packages as the page's
    resolve: { dedupe: ['gpucat', 'math', 'tumble.js'] },
    // the worker awaits the wasm at its top level, which only a module worker can
    worker: { format: 'es' },
    build: { outDir: './dist', target: 'esnext' },
});
