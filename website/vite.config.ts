import { defineConfig } from 'vite';

// the site's page: build.sh puts the built examples under examples/
export default defineConfig({
    base: './',
    // the physics worker awaits the wasm at its top level, which only a module worker can
    worker: { format: 'es' },
    build: { outDir: './dist', target: 'esnext' },
    // tumble.js, and its wasm, come from ../js
    server: { fs: { allow: ['..'] } },
});
