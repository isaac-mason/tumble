import { defineConfig } from 'vite';

export default defineConfig({
    base: './',
    // the physics worker awaits the wasm at its top level, which only a module worker can
    worker: { format: 'es' },
    build: { outDir: './dist', target: 'esnext' },
    server: { fs: { allow: ['..'] } },
});
