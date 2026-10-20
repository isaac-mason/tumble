// the wasm module: one instance per js realm, instantiated with a memory this side creates. it imports nothing else,
// so it never calls back: it reads and writes the memory and returns numbers, and every result bigger than a number
// comes out through the views. the memory starts small and the wasm grows it as it allocates, which replaces the buffer
// under every view; `grew()` re-derives them, and the plumbing calls it after any call that can allocate.

import { SCHEMA_HASH, SCRATCH_BYTES, setWasm, WasmError, type WasmExports } from './generated/tumble';
import { createWasmMemory, refreshViews, type WasmMemory } from './memory';

export type Instance = {
    module: WebAssembly.Module;
    exports: WasmExports;
    mem: WasmMemory;
    /** the address of the region value params cross through: SCRATCH_BYTES, owned by this side. */
    scratch: number;
    /** re-derives the views if the memory grew. */
    grew(): void;
    /** throws the wasm's pending error, if any, and clears it. */
    check(): void;
};

const PAGE_BYTES = 65536;
/** the wasm's own data and stack sit under 2 MiB; 4 MiB to start leaves room for the heap's first tables. */
const INITIAL_PAGES = 64;
/** the memory's ceiling: the whole 32-bit space where the host allows, else what it does. */
const MAXIMUM_PAGES_PREFERRED = [65536, 32768, 16384, 8192];

function createMemory(): WebAssembly.Memory {
    for (const maximum of MAXIMUM_PAGES_PREFERRED) {
        try {
            return new WebAssembly.Memory({ initial: INITIAL_PAGES, maximum });
        } catch {
            // the host refused that ceiling; try a lower one
        }
    }
    throw new Error(`tumble: the host cannot reserve ${(MAXIMUM_PAGES_PREFERRED.at(-1)! * PAGE_BYTES) >> 20} MiB of wasm memory`);
}

const ERROR_NAMES = Object.fromEntries(Object.entries(WasmError).map(([name, code]) => [code, name]));

export function errorName(code: number): string {
    return ERROR_NAMES[code] ?? `unknown(${code})`;
}

/** the wasm instantiated from its bytes or compiled module, initialised and bound as the realm's wasm. */
export async function instantiate(source: BufferSource | WebAssembly.Module): Promise<Instance> {
    const memory = createMemory();
    const imports = { env: { memory } };
    const module = source instanceof WebAssembly.Module ? source : await WebAssembly.compile(source);
    const instance = await WebAssembly.instantiate(module, imports);
    const exports = instance.exports as unknown as WasmExports;
    const hash = exports.tumble_wasm_schema_hash() >>> 0;
    if (hash !== SCHEMA_HASH) {
        throw new Error(
            `tumble: the wasm was built from schema 0x${hash.toString(16)}, this package from 0x${SCHEMA_HASH.toString(16)}`,
        );
    }
    const mem = createWasmMemory(memory);
    const wasm: Instance = {
        module,
        exports,
        mem,
        scratch: 0,
        grew: () => refreshViews(mem),
        check: () => {
            const code = exports.tumble_wasm_error();
            if (code === WasmError.NONE) return;
            exports.tumble_wasm_error_clear();
            throw new Error(`tumble: ${errorName(code)}`);
        },
    };
    if (exports.tumble_wasm_init(exports.__heap_base.value as number) === 0) {
        throw new Error(`tumble: init failed: ${errorName(exports.tumble_wasm_error())}`);
    }
    wasm.scratch = exports.tumble_wasm_alloc_bytes(SCRATCH_BYTES);
    if (wasm.scratch === 0) throw new Error(`tumble: no memory for ${SCRATCH_BYTES} bytes of scratch`);
    wasm.grew();
    setWasm(wasm);
    return wasm;
}
