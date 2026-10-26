import { describe, test } from 'vitest';

declare global {
    interface ImportMeta {
        /** vite's: the modules matching the pattern, each loaded when called */
        glob(pattern: string): Record<string, () => Promise<unknown>>;
    }
}

// every program the js docs show code from: each checks what it shows, throwing when it does not
const programs = import.meta.glob('../docs/*.ts');

describe('the js docs', () => {
    for (const [path, load] of Object.entries(programs)) {
        test(path.replace('../docs/', ''), async () => {
            await load();
        });
    }
});
