// fails the build unless the module imports exactly `env.memory` and nothing else: the wasm can
// never call JS because there is no function import to call. also refuses `__wasm_call_ctors`
// in the exports. a global constructor that stays internal is worse: with no entry point the
// linker wraps every export to run the constructor list on each call, which -Wglobal-constructors
// in toolchain.sh is what prevents.
//
// usage: node check-imports.mjs <file.wasm>
import { readFileSync } from 'node:fs';

const path = process.argv[2];
if (!path) {
    console.error('usage: check-imports.mjs <file.wasm>');
    process.exit(2);
}
const bytes = new Uint8Array(readFileSync(path));

let pos = 0;
const u8 = () => bytes[pos++];
const leb = () => {
    let result = 0;
    let shift = 0;
    for (;;) {
        const b = u8();
        result |= (b & 0x7f) << shift;
        if ((b & 0x80) === 0) return result >>> 0;
        shift += 7;
    }
};
const name = () => {
    const len = leb();
    const s = new TextDecoder().decode(bytes.subarray(pos, pos + len));
    pos += len;
    return s;
};

if (bytes[0] !== 0 || bytes[1] !== 0x61 || bytes[2] !== 0x73 || bytes[3] !== 0x6d) {
    console.error(`${path}: not a wasm module`);
    process.exit(1);
}
pos = 8;

const imports = [];
const exports = [];
while (pos < bytes.length) {
    const id = u8();
    const size = leb();
    const end = pos + size;
    if (id === 2) {
        const count = leb();
        for (let i = 0; i < count; i++) {
            const module = name();
            const field = name();
            const kind = u8();
            imports.push({ module, field, kind });
            // skip the descriptor
            if (kind === 0) leb();
            else if (kind === 1) {
                u8();
                const flags = u8();
                leb();
                if (flags & 1) leb();
            } else if (kind === 2) {
                const flags = u8();
                leb();
                if (flags & 1) leb();
            } else if (kind === 3) {
                u8();
                u8();
            }
        }
    } else if (id === 7) {
        const count = leb();
        for (let i = 0; i < count; i++) {
            const field = name();
            const kind = u8();
            leb();
            exports.push({ field, kind });
        }
    }
    pos = end;
}

const KIND = ['function', 'table', 'memory', 'global'];
const bad = imports.filter((i) => !(i.module === 'env' && i.field === 'memory' && i.kind === 2));
if (imports.length > 1 || bad.length > 0) {
    console.error(`${path}: the wasm may import only env.memory, but imports:`);
    for (const i of imports) console.error(`  ${i.module}.${i.field} (${KIND[i.kind]})`);
    process.exit(1);
}
if (exports.some((e) => e.field === '__wasm_call_ctors')) {
    console.error(`${path}: exports __wasm_call_ctors, so something has a global constructor; the wasm has none`);
    process.exit(1);
}
console.log(`${path}: imports ${imports.length === 0 ? 'nothing' : 'env.memory only'}; ${exports.length} exports`);
