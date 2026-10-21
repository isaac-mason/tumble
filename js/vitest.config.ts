import { fileURLToPath } from 'node:url';
import { defineConfig } from 'vitest/config';

// the docs' programs import `tumble` as a consumer would: here, the package's source
export default defineConfig({
    resolve: { alias: { tumble: fileURLToPath(new URL('./src/index.ts', import.meta.url)) } },
});
