import { hasWasm } from './generated/tumble.js';
import { instantiate } from './module.js';

export type InitOptions = {
    /** the wasm itself, its bytes or compiled module, or where to fetch it from; by default the package's dist/tumble.wasm */
    wasm?: BufferSource | WebAssembly.Module | URL | string;
};

let booting: Promise<void> | null = null;

/** the package's own wasm, beside its modules in dist/ (from src/ while developing, one level up as from dist/) */
function defaultWasmUrl(): URL {
    return new URL('../dist/tumble.wasm', import.meta.url);
}

async function bytesFrom(url: URL): Promise<ArrayBuffer> {
    if (url.protocol === 'file:') {
        const fsModule = 'node:fs/promises';
        const { readFile } = await import(/* @vite-ignore */ fsModule);
        const bytes: Uint8Array = await readFile(url);
        return bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength) as ArrayBuffer;
    }
    const response = await fetch(url);
    if (!response.ok) throw new Error(`tumble: fetching ${url}: ${response.status}`);
    return response.arrayBuffer();
}

/**
 * loads and starts the wasm, once per realm, before anything else: a second call, during or after the first, waits for
 * the same one
 */
export function init(options: InitOptions = {}): Promise<void> {
    if (hasWasm()) return Promise.resolve();
    if (booting === null) {
        booting = (async () => {
            const source = options.wasm ?? defaultWasmUrl();
            // a relative string resolves against the page, as fetch would resolve it
            const url = typeof source === 'string' ? new URL(source, globalThis.location?.href) : source;
            const wasm = url instanceof URL ? await bytesFrom(url) : url;
            await instantiate(wasm);
        })();
    }
    return booting;
}
