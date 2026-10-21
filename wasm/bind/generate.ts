// runs the schema binary, reads the json the manifest (wasm/bindings.cpp) describes, and writes:
//   js/src/generated/tumble.ts    the plumbing: a namespace of functions per C++ namespace, a reader and writer per
//                                 value type, enums, the exports type. the package's own modules import it; users never do
//   wasm/generated/exports.cpp    the extern "C" layer, one forward per bound function
//   wasm/generated/schema_hash.h
// run by build-schema.sh; the outputs are committed.
//
// usage: node generate.ts <tumble-schema.wasm> [--check]

import { mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

type Scalar = 'u8' | 'u16' | 'u32' | 'i32' | 'f32' | 'f64' | 'bool' | 'void';
type StructKind = 'value' | 'embedded' | 'handle' | 'value-in' | 'value-out' | 'ptr-array';
type TypeRef =
    | { kind: 'void'; count: number; ref: boolean; const: boolean }
    | { kind: 'scalar' | 'ptr-scalar'; scalar: Scalar; count: number; ref: boolean; const: boolean; bits?: boolean }
    | { kind: StructKind; cpp: string | null; count: number; ref: boolean; const: boolean };
type ParamDesc = { name: string | null; type: TypeRef };
type FunctionDesc = {
    cpp: string;
    returns: TypeRef;
    params: ParamDesc[];
    grows: boolean;
    doc: string | null;
    jsNamespace: string | null;
};
type FieldDesc = { name: string; offset: number; type: TypeRef; writable: boolean; doc: string | null };
type StructDesc = {
    cpp: string;
    js: string | null;
    value: boolean;
    tuple: boolean;
    size: number;
    align: number;
    doc: string | null;
    fields: FieldDesc[];
};
type EnumDesc = { name: string; doc: string | null; values: Array<{ name: string; value: number; doc: string | null }> };
/** a float constant's value is its bits */
type ConstantDesc = { name: string; value: number; float: boolean; doc: string | null };
type Schema = {
    includes: string[];
    structs: StructDesc[];
    enums: EnumDesc[];
    functions: FunctionDesc[];
    constants: ConstantDesc[];
};

const here = dirname(fileURLToPath(import.meta.url));
const TS_OUT = resolve(here, '../../js/src/generated/tumble.ts');
const EXPORTS_OUT = resolve(here, '../generated/exports.cpp');
const HASH_OUT = resolve(here, '../generated/schema_hash.h');

const wasmPath = process.argv[2];
if (!wasmPath) {
    console.error('usage: generate.ts <tumble-schema.wasm> [--check]');
    process.exit(2);
}
const check = process.argv.includes('--check');

// ---- run the schema binary ----

const memory = new WebAssembly.Memory({ initial: 64, maximum: 4096 });
const { instance } = await WebAssembly.instantiate(readFileSync(wasmPath), { env: { memory } });
const exports = instance.exports as { schema_json(): number };

function readCString(address: number): string {
    const u8 = new Uint8Array(memory.buffer);
    let end = address;
    while (u8[end] !== 0) end++;
    return new TextDecoder().decode(u8.subarray(address, end));
}

const json = readCString(exports.schema_json());
const schema = JSON.parse(json) as Schema;

// fnv-1a over the description: both sides carry it and the loader refuses a mismatch
let hash = 0x811c9dc5;
for (let i = 0; i < json.length; i++) {
    hash ^= json.charCodeAt(i);
    hash = Math.imul(hash, 0x01000193) >>> 0;
}
const hashHex = `0x${(hash >>> 0).toString(16).padStart(8, '0')}`;

// ---- names: the C++ decides, the case changes ----

function camel(snake: string): string {
    return snake.replace(/_([a-z0-9])/g, (_, c: string) => c.toUpperCase());
}

function pascal(snake: string): string {
    const c = camel(snake);
    return c[0]!.toUpperCase() + c.slice(1);
}

function capitalize(name: string): string {
    return name[0]!.toUpperCase() + name.slice(1);
}

/**
 * `adapters::rigid_body::get_position` -> namespace RigidBody, name getPosition, symbol tumble_rigid_body_get_position:
 * the innermost namespace names it. a js namespace the manifest names replaces it in both: `runtime::init` bound in
 * `wasm` is Wasm.init, tumble_wasm_init.
 */
function functionNames(f: FunctionDesc): { ns: string; name: string; symbol: string } {
    const parts = f.cpp.split('::');
    if (parts.length < 2) throw new Error(`${f.cpp}: a bound function needs a namespace; it becomes the js one`);
    const bare = parts[parts.length - 1]!;
    const ns = f.jsNamespace ?? parts[parts.length - 2]!;
    return { ns: pascal(ns), name: camel(bare), symbol: `tumble_${ns}_${bare}` };
}

/** a record's js name is its type name; a value's drops the trailing f. */
function structJsName(s: StructDesc): string {
    if (s.js !== null) return s.js;
    const bare = s.cpp.split('::').pop()!;
    if (bare.includes('<')) throw new Error(`${s.cpp}: a template needs RECORD_AS with a js name`);
    return s.value ? bare.replace(/f$/, '') : bare;
}

// ---- the description, checked ----

const structsByCpp = new Map<string, StructDesc>();
for (const s of schema.structs) structsByCpp.set(s.cpp, s);

function structOf(t: TypeRef, where: string): StructDesc {
    if (t.kind === 'void' || t.kind === 'scalar' || t.kind === 'ptr-scalar') throw new Error(`${where}: not a struct type`);
    if (t.cpp === null) throw new Error(`${where}: uses a struct that the manifest does not register`);
    const s = structsByCpp.get(t.cpp);
    if (!s) throw new Error(`${where}: unknown struct ${t.cpp}`);
    return s;
}

function jsOf(t: TypeRef, where: string): string {
    return structJsName(structOf(t, where));
}

const VIEW: Record<Exclude<Scalar, 'void'>, { view: string; shift: number }> = {
    u8: { view: 'u8', shift: 0 },
    u16: { view: 'u16', shift: 1 },
    u32: { view: 'u32', shift: 2 },
    i32: { view: 'i32', shift: 2 },
    f32: { view: 'f32', shift: 2 },
    f64: { view: 'f64', shift: 3 },
    bool: { view: 'u8', shift: 0 },
};

const CPP_SCALAR: Record<Scalar, string> = {
    u8: 'uint8_t',
    u16: 'uint16_t',
    u32: 'uint32_t',
    i32: 'int32_t',
    f32: 'float',
    f64: 'double',
    bool: 'bool',
    void: 'void',
};

const FALLBACK_NAMES = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l'];
/** biome's lineWidth in js/biome.json; the output is kept within it so the formatter leaves it alone. */
const LINE_WIDTH = 130;

/** an unnamed handle or value param is named after its type (`voxels`); a scalar falls back to a, b, c. a trailing ? marks a nullable handle. */
function paramName(p: ParamDesc, i: number): string {
    if (p.name !== null) return p.name.replace(/\?$/, '');
    const t = p.type;
    if (t.kind === 'handle' || t.kind === 'value-in' || t.kind === 'value-out') {
        const js = structJsName(structOf(t, 'param'));
        return js[0]!.toLowerCase() + js.slice(1);
    }
    return FALLBACK_NAMES[i]!;
}

function paramNullable(p: ParamDesc): boolean {
    return p.name?.endsWith('?') ?? false;
}

function docLines(doc: string | null, indent: string): string[] {
    return doc ? [`${indent}/** ${doc} */`] : [];
}

function align16(n: number): number {
    return (n + 15) & ~15;
}

/** a scalar read at a byte address expression, over `mem`. */
function scalarRead(scalar: Exclude<Scalar, 'void'>, addr: string): string {
    const v = VIEW[scalar];
    const read = v.shift === 0 ? `mem.${v.view}[${addr}]!` : `mem.${v.view}[(${addr}) >> ${v.shift}]!`;
    return scalar === 'bool' ? `${read} !== 0` : read;
}

function scalarWrite(scalar: Exclude<Scalar, 'void'>, addr: string, value: string): string {
    const v = VIEW[scalar];
    const stored = scalar === 'bool' ? `${value} ? 1 : 0` : value;
    return v.shift === 0 ? `mem.${v.view}[${addr}] = ${stored};` : `mem.${v.view}[(${addr}) >> ${v.shift}] = ${stored};`;
}

const DATA_VIEW: Record<Exclude<Scalar, 'void'>, string> = {
    u8: 'Uint8',
    u16: 'Uint16',
    u32: 'Uint32',
    i32: 'Int32',
    f32: 'Float32',
    f64: 'Float64',
    bool: 'Uint8',
};

/** a scalar read at `offset + at` of a little-endian DataView named `view`. */
function viewRead(scalar: Exclude<Scalar, 'void'>, at: number): string {
    const read = `view.get${DATA_VIEW[scalar]}(offset + ${at}${scalarBytes(scalar) > 1 ? ', true' : ''})`;
    return scalar === 'bool' ? `${read} !== 0` : read;
}

/** the statements writing a scalar at `offset + at` of a little-endian DataView named `view`. */
function viewWrite(scalar: Exclude<Scalar, 'void'>, at: number, value: string): string[] {
    const stored = scalar === 'bool' ? `${value} ? 1 : 0` : value;
    return [`view.set${DATA_VIEW[scalar]}(offset + ${at}, ${stored}${scalarBytes(scalar) > 1 ? ', true' : ''});`];
}

function scalarBytes(scalar: Exclude<Scalar, 'void'>): number {
    return 1 << VIEW[scalar].shift;
}

function scalarTsType(scalar: Scalar): string {
    return scalar === 'bool' ? 'boolean' : 'number';
}

/** scalars' address typed by the scalar, an array of handles' by the handle; a void * is a plain number, which any address converts to. */
function pointerTsType(t: TypeRef, where: string): string {
    const pointer = 'Pointer';
    if (t.kind === 'ptr-scalar') return t.scalar === 'void' ? 'number' : `${pointer}<'${t.scalar}'>`;
    if (t.kind === 'ptr-array') return `${pointer}<${jsOf(t, where)}>`;
    throw new Error(`${where}: ${t.kind} is not a pointer`);
}

// ---- value types: a js type, a create(), and flattened read/write over memory ----

/** `address` marks an address word: the js type it reads as, and whether 0 reads as null (a handle). */
type Leaf = { offset: number; scalar: Exclude<Scalar, 'void'>; expr: string; address?: { type: string; nullable: boolean } };

/** what a leaf stores: a handle's null as 0. */
function leafStored(leaf: Leaf): string {
    return leaf.address?.nullable ? `${leaf.expr} ?? 0` : leaf.expr;
}

/** a leaf's raw read typed as the leaf: an address word as its pointer or handle type, a handle's 0 as null. */
function leafRead(leaf: Leaf, raw: string): string {
    const address = leaf.address;
    if (address === undefined || address.type === 'number') return raw;
    return address.nullable ? `(${raw} || null) as ${address.type} | null` : `${raw} as ${address.type}`;
}

/** every scalar of a value, with its byte offset and the js expression that holds it under `path`. */
function leavesOf(s: StructDesc, base: number, path: string): Leaf[] {
    const out: Leaf[] = [];
    let index = 0;
    for (const f of s.fields) {
        const t = f.type;
        const where = `${s.cpp}::${f.name}`;
        const js = camel(f.name);
        if (t.kind === 'scalar') {
            if (t.scalar === 'void') throw new Error(`${where}: void field`);
            for (let k = 0; k < t.count; k++) {
                const expr = s.tuple ? `${path}[${index++}]` : t.count === 1 ? `${path}.${js}` : `${path}.${js}[${k}]!`;
                out.push({ offset: base + f.offset + k * scalarBytes(t.scalar), scalar: t.scalar, expr });
            }
        } else if (t.kind === 'value') {
            const expr = s.tuple ? `${path}[${index++}]` : `${path}.${js}`;
            const value = structOf(t, where);
            if (t.count === 1) out.push(...leavesOf(value, base + f.offset, expr));
            else for (let k = 0; k < t.count; k++) out.push(...leavesOf(value, base + f.offset + k * value.size, `${expr}[${k}]!`));
        } else if (t.kind === 'handle' && t.count === 1) {
            const expr = s.tuple ? `${path}[${index++}]` : `${path}.${js}`;
            out.push({ offset: base + f.offset, scalar: 'u32', expr, address: { type: jsOf(t, where), nullable: true } });
        } else if (t.kind === 'ptr-scalar' || t.kind === 'ptr-array') {
            const expr = s.tuple ? `${path}[${index++}]` : `${path}.${js}`;
            out.push({ offset: base + f.offset, scalar: 'u32', expr, address: { type: pointerTsType(t, where), nullable: false } });
        } else {
            throw new Error(`${where}: a value holds scalars, values, handles and pointers, not ${t.kind}`);
        }
    }
    return out;
}

function valueFieldType(t: TypeRef, where: string): string {
    if (t.kind === 'scalar') return t.count === 1 ? scalarTsType(t.scalar) : 'number[]';
    if (t.kind === 'value') return t.count === 1 ? jsOf(t, where) : `${jsOf(t, where)}[]`;
    if (t.kind === 'handle') return `${jsOf(t, where)} | null`;
    if (t.kind === 'ptr-scalar' || t.kind === 'ptr-array') return pointerTsType(t, where);
    throw new Error(`${where}: ${t.kind} cannot be a value field`);
}

/** elements packed onto as few lines as fit at `indent`, as biome fills an array of numbers. */
function fillLines(elements: string[], indent: string): string[] {
    const lines: string[] = [];
    let line = '';
    for (const element of elements) {
        const next = line === '' ? element : `${line}, ${element}`;
        if (line !== '' && `${indent}${next},`.length > LINE_WIDTH) {
            lines.push(`${indent}${line},`);
            line = element;
        } else line = next;
    }
    if (line !== '') lines.push(`${indent}${line},`);
    return lines;
}

/** `create()`'s body: one line when it fits, otherwise a field per line as biome would print it. */
function valueCreate(s: StructDesc): string[] {
    const parts: Array<{ js: string; expr: string; elements: string[] | null }> = [];
    for (const f of s.fields) {
        const t = f.type;
        const js = camel(f.name);
        if (t.kind === 'scalar' && t.count > 1) {
            const zeros = new Array<string>(t.count).fill('0');
            if (s.tuple) for (const zero of zeros) parts.push({ js, expr: zero, elements: null });
            else parts.push({ js, expr: `[${zeros.join(', ')}]`, elements: zeros });
            continue;
        }
        let expr: string;
        if (t.kind === 'scalar') expr = t.scalar === 'bool' ? 'false' : '0';
        else if (t.kind === 'handle') expr = 'null';
        else if (t.kind === 'ptr-scalar' || t.kind === 'ptr-array') {
            const type = pointerTsType(t, s.cpp);
            expr = type === 'number' ? '0' : `0 as ${type}`;
        } else if (t.count > 1) expr = `Array.from({ length: ${t.count} }, () => ${jsOf(t, s.cpp)}.create())`;
        else expr = `${jsOf(t, s.cpp)}.create()`;
        parts.push({ js, expr, elements: null });
    }
    const flat = s.tuple
        ? `[${parts.map((p) => p.expr).join(', ')}]`
        : `{ ${parts.map((p) => `${p.js}: ${p.expr}`).join(', ')} }`;
    if (`        return ${flat};`.length <= LINE_WIDTH || s.tuple) return [`        return ${flat};`];
    const lines = ['        return {'];
    for (const p of parts) {
        if (p.elements !== null && `            ${p.js}: ${p.expr},`.length > LINE_WIDTH) {
            lines.push(`            ${p.js}: [`, ...fillLines(p.elements, '                '), '            ],');
        } else lines.push(`            ${p.js}: ${p.expr},`);
    }
    lines.push('        };');
    return lines;
}

function valueTsType(s: StructDesc): string {
    const name = structJsName(s);
    if (s.tuple) {
        const parts: string[] = [];
        for (const f of s.fields) {
            const t = f.type;
            if (t.kind === 'scalar' && t.count > 1) parts.push(...new Array<string>(t.count).fill('number'));
            else parts.push(`${camel(f.name)}: ${valueFieldType(t, s.cpp)}`);
        }
        // one element per line past the formatter's width, as biome would print it
        const flat = `[${parts.join(', ')}]`;
        return `export type ${name} = ${flat};`.length <= LINE_WIDTH ? flat : `[\n${parts.map((p) => `    ${p},`).join('\n')}\n]`;
    }
    const fields = s.fields.map((f) => `${camel(f.name)}: ${valueFieldType(f.type, s.cpp)}`);
    const flat = `{ ${fields.join('; ')} }`;
    return `export type ${name} = ${flat};`.length <= LINE_WIDTH ? flat : `{\n${fields.map((f) => `    ${f};`).join('\n')}\n}`;
}

/**
 * `writeTo` / `readFrom`: the leaves at a byte offset of any buffer through a DataView, laid out as they are in the
 * wasm's memory, so a value or record crosses to another realm's memory with no layout written by hand.
 */
function emitViewAccess(name: string, leaves: Leaf[]): string[] {
    const lines: string[] = [];
    lines.push("    /** writes it at a byte offset of any buffer (little-endian), laid out as in the wasm's memory. */");
    lines.push(`    writeTo(view: DataView, offset: number, value: ${name}): void {`);
    for (const leaf of leaves)
        for (const statement of viewWrite(leaf.scalar, leaf.offset, leafStored(leaf))) lines.push(`        ${statement}`);
    lines.push('    },');
    lines.push('    /** reads it at a byte offset of any buffer (little-endian) into out, and returns out. */');
    lines.push(`    readFrom(out: ${name}, view: DataView, offset: number): ${name} {`);
    for (const leaf of leaves)
        lines.push(`        ${leaf.expr.replace(/^value/, 'out')} = ${leafRead(leaf, viewRead(leaf.scalar, leaf.offset))};`);
    lines.push('        return out;');
    lines.push('    },');
    return lines;
}

function emitValue(s: StructDesc): string[] {
    const name = structJsName(s);
    const lines: string[] = [];
    const leaves = leavesOf(s, 0, 'value');
    lines.push(...docLines(s.doc, ''));
    lines.push(`export type ${name} = ${valueTsType(s)};`);
    lines.push(`export const ${name} = {`);
    lines.push(`    size: ${s.size},`);
    lines.push(`    create(): ${name} {`);
    lines.push(...valueCreate(s));
    lines.push('    },');
    lines.push("    /** reads the value at a byte address of the wasm's memory into out, and returns out. */");
    lines.push(`    read(out: ${name}, address: number): ${name} {`);
    lines.push('        const mem = wasm.mem;');
    for (const leaf of leaves) {
        const read = leafRead(leaf, scalarRead(leaf.scalar, `address + ${leaf.offset}`));
        lines.push(`        ${leaf.expr.replace(/^value/, 'out')} = ${read};`);
    }
    lines.push('        return out;');
    lines.push('    },');
    lines.push("    /** writes the value at a byte address of the wasm's memory. */");
    lines.push(`    write(address: number, value: ${name}): void {`);
    lines.push('        const mem = wasm.mem;');
    for (const leaf of leaves) lines.push(`        ${scalarWrite(leaf.scalar, `address + ${leaf.offset}`, leafStored(leaf))}`);
    lines.push('    },');
    lines.push(...emitViewAccess(name, leaves));
    lines.push('};');
    return lines;
}

// ---- functions: scalars and handles by number, values through scratch ----

/** a value returned by reference is the out-param again; it crosses as nothing. */
function returnsNothing(f: FunctionDesc): boolean {
    const r = f.returns;
    return r.kind === 'void' || (r.ref && (r.kind === 'value-out' || r.kind === 'value-in'));
}

/** the bytes of scratch a call needs: every value param, 16-aligned. */
function scratchBytes(f: FunctionDesc): number {
    let total = 0;
    for (const p of f.params) {
        if (p.type.kind === 'value-in' || p.type.kind === 'value-out') total += align16(structOf(p.type, f.cpp).size);
    }
    return total;
}

function jsParamType(p: ParamDesc, where: string): string {
    const t = p.type;
    switch (t.kind) {
        case 'scalar':
            if (t.scalar === 'f64') throw new Error(`${where}: an f64 crosses only as a field`);
            return scalarTsType(t.scalar);
        case 'ptr-scalar':
        case 'ptr-array':
            return pointerTsType(t, where);
        case 'handle':
            return paramNullable(p) ? `${jsOf(t, where)} | null` : jsOf(t, where);
        case 'value-in':
        case 'value-out':
            return jsOf(t, where);
        default:
            throw new Error(`${where}: ${t.kind} cannot be a parameter`);
    }
}

/**
 * the body and signature of a bound function as a property of its namespace object, or as an exported function of the
 * module (`moduleLevel`). out-params come first, then in-params, each in c order; a function returning nothing with one
 * out-param returns it, filled. a handle returned by a call that allocates is never null: a 0 there is an error, thrown.
 */
function emitFunction(f: FunctionDesc, indent: string, moduleLevel: boolean): string[] {
    const where = f.cpp;
    const { name } = functionNames(f);
    const named = f.params.map((p, i) => ({ ...p, js: paramName(p, i) }));
    const ins = named.filter((p) => p.type.kind !== 'value-out');
    const outs = named.filter((p) => p.type.kind === 'value-out');
    const nothing = returnsNothing(f);
    const returnsOut = nothing && outs.length === 1;

    const signatureParams: string[] = [];
    for (const p of outs) signatureParams.push(`${p.js}: ${jsOf(p.type, where)}`);
    for (const p of ins) signatureParams.push(`${p.js}: ${jsParamType(p, where)}`);
    let returnType: string;
    const r = f.returns;
    if (returnsOut) returnType = jsOf(outs[0]!.type, where);
    else if (nothing) returnType = 'void';
    else if (r.kind === 'scalar') {
        if (r.scalar === 'f64') throw new Error(`${where}: an f64 crosses only as a field`);
        returnType = scalarTsType(r.scalar);
    } else if (r.kind === 'ptr-scalar' || r.kind === 'ptr-array') returnType = pointerTsType(r, where);
    else if (r.kind === 'handle') returnType = f.grows ? jsOf(r, where) : `${jsOf(r, where)} | null`;
    else throw new Error(`${where}: ${r.kind} cannot be returned; use an out-param`);

    // scratch offsets for the value params, in c order
    const offsets = new Map<string, number>();
    let scratchUsed = 0;
    for (const p of named) {
        if (p.type.kind === 'value-in' || p.type.kind === 'value-out') {
            offsets.set(p.js, scratchUsed);
            scratchUsed += align16(structOf(p.type, where).size);
        }
    }

    const body: string[] = [];
    if (scratchUsed > 0) {
        body.push('const scratch = wasm.scratch;');
        for (const p of named) {
            if (p.type.kind === 'value-in') body.push(`${jsOf(p.type, where)}.write(scratch + ${offsets.get(p.js)}, ${p.js});`);
        }
    }
    const args: string[] = [];
    for (const p of named) {
        switch (p.type.kind) {
            case 'handle':
                args.push(paramNullable(p) ? `${p.js} === null ? 0 : ${p.js}` : p.js);
                break;
            case 'value-in':
            case 'value-out':
                args.push(`scratch + ${offsets.get(p.js)}`);
                break;
            case 'scalar':
                args.push(p.type.scalar === 'bool' ? `${p.js} ? 1 : 0` : p.js);
                break;
            default:
                args.push(p.js);
        }
    }
    const exportsOf = 'wasm.exports';
    const callee = `${nothing ? '' : 'const r = '}${exportsOf}.${functionNames(f).symbol}(`;
    const flatCall = `${callee}${args.join(', ')});`;
    // an argument per line past the formatter's width, as biome would print it
    if (`${indent}    ${flatCall}`.length <= LINE_WIDTH || args.length === 0) body.push(flatCall);
    else body.push(callee, ...args.map((arg) => `    ${arg},`), ');');
    if (f.grows) body.push('wasm.grew();');
    for (const p of outs) body.push(`${jsOf(p.type, where)}.read(${p.js}, scratch + ${offsets.get(p.js)});`);
    if (returnsOut) body.push(`return ${outs[0]!.js};`);
    else if (r.kind === 'handle' && f.grows) {
        body.push('if (r === 0) wasm.check();');
        body.push(`return r as ${jsOf(r, where)};`);
    } else if (r.kind === 'handle') body.push(`return r === 0 ? null : (r as ${jsOf(r, where)});`);
    else if (r.kind === 'scalar' && r.scalar === 'bool') body.push('return r !== 0;');
    // wasm returns i32; a u32 of 2^31 or more would read negative
    else if (r.kind === 'scalar' && r.scalar === 'u32') body.push('return r >>> 0;');
    else if ((r.kind === 'ptr-scalar' || r.kind === 'ptr-array') && returnType !== 'number')
        body.push(`return r as ${returnType};`);
    else if (!nothing) body.push('return r;');

    const lines: string[] = [];
    lines.push(...docLines(f.doc, indent));
    const head = moduleLevel ? `export function ${name}` : name;
    const signature = `${indent}${head}(${signatureParams.join(', ')}): ${returnType} {`;
    if (signature.length <= LINE_WIDTH) lines.push(signature);
    else {
        // a parameter per line past the formatter's width, as biome would print it
        lines.push(`${indent}${head}(`);
        for (const param of signatureParams) lines.push(`${indent}    ${param},`);
        lines.push(`${indent}): ${returnType} {`);
    }
    for (const line of body) lines.push(`${indent}    ${line}`);
    lines.push(moduleLevel ? `${indent}}` : `${indent}},`);
    return lines;
}

// ---- records: readers (and writers) over the fields ----

/** the plain object a record's fields read into, for `writeTo` / `readFrom`: `Record` reads into `RecordValue`. */
function recordValueName(s: StructDesc): string {
    return `${structJsName(s)}Value`;
}

/** a record field in its plain object: pointers and handles typed as the field getters type them, embedded records as their own. */
function recordFieldType(f: FieldDesc, where: string): string {
    const t = f.type;
    switch (t.kind) {
        case 'scalar':
            if (t.count > 1) throw new Error(`${where}: scalar arrays on records are not supported yet`);
            return scalarTsType(t.scalar);
        case 'ptr-scalar':
        case 'ptr-array':
            return pointerTsType(t, where);
        case 'handle':
            return `${jsOf(t, where)} | null`;
        case 'value':
            return t.count > 1 ? `${jsOf(t, where)}[]` : jsOf(t, where);
        case 'embedded':
            return recordValueName(structOf(t, where));
        default:
            throw new Error(`${where}: ${t.kind} cannot be a field of a record`);
    }
}

function recordFieldCreate(f: FieldDesc, where: string): string {
    const t = f.type;
    switch (t.kind) {
        case 'scalar':
            return t.scalar === 'bool' ? 'false' : '0';
        case 'value':
            return t.count > 1
                ? `Array.from({ length: ${t.count} }, () => ${jsOf(t, where)}.create())`
                : `${jsOf(t, where)}.create()`;
        case 'embedded':
            return `${structJsName(structOf(t, where))}.createValue()`;
        case 'handle':
            return 'null';
        case 'ptr-scalar':
        case 'ptr-array': {
            const type = pointerTsType(t, where);
            return type === 'number' ? '0' : `0 as ${type}`;
        }
        default:
            return '0';
    }
}

/** every scalar of a record's plain object, as leavesOf does for a value. */
function recordLeavesOf(s: StructDesc, base: number, path: string): Leaf[] {
    const out: Leaf[] = [];
    for (const f of s.fields) {
        const t = f.type;
        const where = `${s.cpp}::${f.name}`;
        const expr = `${path}.${camel(f.name)}`;
        const offset = base + f.offset;
        switch (t.kind) {
            case 'scalar':
                if (t.scalar === 'void') throw new Error(`${where}: void field`);
                if (t.count > 1) throw new Error(`${where}: scalar arrays on records are not supported yet`);
                out.push({ offset, scalar: t.scalar, expr });
                break;
            case 'ptr-scalar':
            case 'ptr-array':
                out.push({ offset, scalar: 'u32', expr, address: { type: pointerTsType(t, where), nullable: false } });
                break;
            case 'handle':
                out.push({ offset, scalar: 'u32', expr, address: { type: jsOf(t, where), nullable: true } });
                break;
            case 'value': {
                const value = structOf(t, where);
                if (t.count === 1) out.push(...leavesOf(value, offset, expr));
                else for (let k = 0; k < t.count; k++) out.push(...leavesOf(value, offset + k * value.size, `${expr}[${k}]`));
                break;
            }
            case 'embedded':
                out.push(...recordLeavesOf(structOf(t, where), offset, expr));
                break;
            default:
                throw new Error(`${where}: ${t.kind} cannot be a field of a record`);
        }
    }
    return out;
}

function recordValueType(s: StructDesc): string {
    const name = recordValueName(s);
    const fields = s.fields.map((f) => `${camel(f.name)}: ${recordFieldType(f, `${s.cpp}::${f.name}`)}`);
    const flat = `{ ${fields.join('; ')} }`;
    return `export type ${name} = ${flat};`.length <= LINE_WIDTH ? flat : `{\n${fields.map((f) => `    ${f};`).join('\n')}\n}`;
}

function recordValueCreate(s: StructDesc): string[] {
    const parts = s.fields.map((f) => `${camel(f.name)}: ${recordFieldCreate(f, `${s.cpp}::${f.name}`)}`);
    const flat = `        return { ${parts.join(', ')} };`;
    if (flat.length <= LINE_WIDTH) return [flat];
    return ['        return {', ...parts.map((p) => `            ${p},`), '        };'];
}

function emitFields(s: StructDesc): string[] {
    const name = structJsName(s);
    const self = name[0]!.toLowerCase() + name.slice(1);
    const lines: string[] = [];
    lines.push(`    size: ${s.size},`);
    lines.push(`    align: ${s.align},`);
    lines.push('    /** byte offset of each field, for hot paths that read the views directly. */');
    lines.push('    offset: {');
    for (const f of s.fields) lines.push(`        ${camel(f.name)}: ${f.offset},`);
    lines.push('    },');
    lines.push(`    createValue(): ${recordValueName(s)} {`);
    lines.push(...recordValueCreate(s));
    lines.push('    },');
    lines.push(...emitViewAccess(recordValueName(s), recordLeavesOf(s, 0, 'value')));
    for (const f of s.fields) {
        const t = f.type;
        const where = `${s.cpp}::${f.name}`;
        const js = camel(f.name);
        const addr = `${self} + ${f.offset}`;
        lines.push('');
        lines.push(...docLines(f.doc, '    '));
        switch (t.kind) {
            case 'scalar': {
                if (t.count > 1) throw new Error(`${where}: scalar arrays on records are not supported yet`);
                if (t.scalar === 'void') throw new Error(`${where}: void field`);
                lines.push(`    ${js}(${self}: ${name}): ${scalarTsType(t.scalar)} {`);
                lines.push('        const mem = wasm.mem;');
                lines.push(`        return ${scalarRead(t.scalar, addr)};`);
                lines.push('    },');
                if (f.writable) {
                    lines.push(`    set${capitalize(js)}(${self}: ${name}, value: ${scalarTsType(t.scalar)}): void {`);
                    lines.push('        const mem = wasm.mem;');
                    lines.push(`        ${scalarWrite(t.scalar, addr, 'value')}`);
                    lines.push('    },');
                }
                break;
            }
            case 'ptr-scalar':
            case 'ptr-array': {
                const type = pointerTsType(t, where);
                const read = scalarRead('u32', addr);
                lines.push(`    ${js}(${self}: ${name}): ${type} {`);
                lines.push('        const mem = wasm.mem;');
                lines.push(`        return ${type === 'number' ? read : `${read} as ${type}`};`);
                lines.push('    },');
                if (f.writable) {
                    lines.push(`    set${capitalize(js)}(${self}: ${name}, value: ${type}): void {`);
                    lines.push('        const mem = wasm.mem;');
                    lines.push(`        ${scalarWrite('u32', addr, 'value')}`);
                    lines.push('    },');
                }
                break;
            }
            case 'handle': {
                const target = jsOf(t, where);
                lines.push(`    ${js}(${self}: ${name}): ${target} | null {`);
                lines.push('        const mem = wasm.mem;');
                lines.push(`        const a = ${scalarRead('u32', addr)};`);
                lines.push(`        return a === 0 ? null : (a as ${target});`);
                lines.push('    },');
                if (f.writable) {
                    lines.push(`    set${capitalize(js)}(${self}: ${name}, value: ${target} | null): void {`);
                    lines.push('        const mem = wasm.mem;');
                    lines.push(`        ${scalarWrite('u32', addr, 'value === null ? 0 : value')}`);
                    lines.push('    },');
                }
                break;
            }
            case 'embedded': {
                const target = jsOf(t, where);
                lines.push(`    ${js}(${self}: ${name}): ${target} {`);
                lines.push(`        return (${addr}) as ${target};`);
                lines.push('    },');
                break;
            }
            case 'value': {
                const target = jsOf(t, where);
                if (t.count > 1) {
                    // an array of values: the element at index, structOf(t).size bytes apart
                    const stride = structOf(t, where).size;
                    lines.push(`    read${capitalize(js)}(out: ${target}, ${self}: ${name}, index: number): ${target} {`);
                    lines.push(`        return ${target}.read(out, ${addr} + index * ${stride});`);
                    lines.push('    },');
                    if (f.writable) {
                        lines.push(`    write${capitalize(js)}(${self}: ${name}, index: number, value: ${target}): void {`);
                        lines.push(`        ${target}.write(${addr} + index * ${stride}, value);`);
                        lines.push('    },');
                    }
                    break;
                }
                lines.push(`    read${capitalize(js)}(out: ${target}, ${self}: ${name}): ${target} {`);
                lines.push(`        return ${target}.read(out, ${addr});`);
                lines.push('    },');
                if (f.writable) {
                    lines.push(`    write${capitalize(js)}(${self}: ${name}, value: ${target}): void {`);
                    lines.push(`        ${target}.write(${addr}, value);`);
                    lines.push('    },');
                }
                break;
            }
            default:
                throw new Error(`${where}: ${t.kind} cannot be a field of a record`);
        }
    }
    return lines;
}

// ---- namespaces: a record's fields and the functions of the C++ namespace with that name ----

type Namespace = { name: string; record: StructDesc | null; functions: FunctionDesc[] };

/** the functions bound in js namespace `wasm` are the module's own, at its top level: `init`, `error`. */
const ENTRY_NAMESPACE = 'Wasm';

function namespaces(): Namespace[] {
    const byName = new Map<string, Namespace>();
    const get = (name: string) => {
        let ns = byName.get(name);
        if (!ns) {
            ns = { name, record: null, functions: [] };
            byName.set(name, ns);
        }
        return ns;
    };
    for (const s of schema.structs) if (!s.value) get(structJsName(s)).record = s;
    for (const f of schema.functions) get(functionNames(f).ns).functions.push(f);
    return [...byName.values()];
}

function emitNamespace(ns: Namespace): string[] {
    const lines: string[] = [];
    if (ns.name === ENTRY_NAMESPACE) {
        if (ns.record !== null)
            throw new Error(`${ns.record.cpp}: a record cannot be named ${ENTRY_NAMESPACE}, the entry's own namespace`);
        for (let i = 0; i < ns.functions.length; i++) {
            if (i > 0) lines.push('');
            lines.push(...emitFunction(ns.functions[i]!, '', true));
        }
        return lines;
    }
    const members = new Set<string>();
    const claim = (member: string, where: string) => {
        if (members.has(member)) throw new Error(`${ns.name}.${member}: defined twice (${where})`);
        members.add(member);
    };
    if (ns.record !== null) {
        lines.push(...docLines(ns.record.doc, ''));
        lines.push(`export type ${ns.name} = Handle<'${ns.name}'>;`);
    }
    // a record with no bound fields is an opaque handle: no plain object, no layout
    if (ns.record !== null && ns.record.fields.length > 0) {
        const valueName = recordValueName(ns.record);
        if (schema.structs.some((o) => structJsName(o) === valueName) || schema.enums.some((e) => e.name === valueName))
            throw new Error(`${ns.record.cpp}: its plain object ${valueName} is already a type's name`);
        lines.push(
            `/** ${ns.name}'s fields as a plain object, for ${ns.name}.writeTo / readFrom over another realm's buffer. */`,
        );
        lines.push(`export type ${valueName} = ${recordValueType(ns.record)};`);
        for (const member of ['size', 'align', 'offset', 'createValue', 'writeTo', 'readFrom']) claim(member, ns.record.cpp);
        for (const f of ns.record.fields) {
            const js = camel(f.name);
            claim(f.type.kind === 'value' ? `read${capitalize(js)}` : js, `${ns.record.cpp}::${f.name}`);
            if (f.writable)
                claim(f.type.kind === 'value' ? `write${capitalize(js)}` : `set${capitalize(js)}`, `${ns.record.cpp}::${f.name}`);
        }
    }
    for (const f of ns.functions) claim(functionNames(f).name, f.cpp);
    lines.push(`export const ${ns.name} = {`);
    const fields = ns.record !== null && ns.record.fields.length > 0;
    if (fields) lines.push(...emitFields(ns.record!));
    for (let i = 0; i < ns.functions.length; i++) {
        if (fields || i > 0) lines.push('');
        lines.push(...emitFunction(ns.functions[i]!, '    ', false));
    }
    lines.push('};');
    return lines;
}

/** a const object of the values and a type of the same name: one name is both, and node runs the module as it is */
function emitEnum(e: EnumDesc): string[] {
    const lines: string[] = [];
    lines.push(...docLines(e.doc, ''));
    lines.push(`export const ${e.name} = {`);
    for (const v of e.values) {
        lines.push(...docLines(v.doc, '    '));
        lines.push(`    ${v.name}: ${v.value},`);
    }
    lines.push('} as const;');
    lines.push(`export type ${e.name} = (typeof ${e.name})[keyof typeof ${e.name}];`);
    return lines;
}

/** a bound C++ constant as a module-level const; masks and sentinels past 16 bits in hex, as the C++ writes them. */
function emitConstant(c: ConstantDesc): string[] {
    const value = c.float
        ? String(new Float32Array(new Uint32Array([c.value]).buffer)[0])
        : c.value > 0xffff
          ? `0x${c.value.toString(16)}`
          : String(c.value);
    return [...docLines(c.doc, ''), `export const ${c.name} = ${value};`];
}

function emitExportsType(): string[] {
    const lines: string[] = [];
    lines.push('/** the extern "C" surface, every function taking and returning numbers; the namespaces above call these. */');
    lines.push('export type WasmExports = {');
    lines.push('    __heap_base: WebAssembly.Global;');
    lines.push('    __data_end: WebAssembly.Global;');
    for (const f of schema.functions) {
        const params = f.params.map((p, i) => `${paramName(p, i)}: number`);
        const { symbol } = functionNames(f);
        const returns = returnsNothing(f) ? 'void' : 'number';
        const flat = `    ${symbol}(${params.join(', ')}): ${returns};`;
        if (flat.length <= LINE_WIDTH) lines.push(flat);
        else lines.push(`    ${symbol}(`, ...params.map((p) => `        ${p},`), `    ): ${returns};`);
    }
    lines.push('};');
    return lines;
}

function tsModule(): string {
    const scratch = Math.max(0, ...schema.functions.map((f) => scratchBytes(f)));
    const parts: string[] = [
        '// generated by wasm/bind/generate.ts from wasm/bindings.cpp; do not edit.',
        '// the plumbing under the package: a namespace per C++ namespace the wasm exposes. a handle is the address of an',
        '// object the wasm owns, the field functions read and write its record through the views, the others call into',
        '// the wasm. a value type is a plain object or tuple with a reader and writer over memory; a call passes one',
        "// through the wasm's scratch region. the realm has one wasm, bound here by init.",
        '',
        "import type { Instance } from '../module.js';",
        '',
        '/** fnv-1a of the description the manifest produced; the wasm carries the same and the loader refuses a mismatch. */',
        `export const SCHEMA_HASH = ${hashHex};`,
        '/** bytes of scratch the loader allocates for value params: the most any one call needs. */',
        `export const SCRATCH_BYTES = ${scratch};`,
        '',
        '/** the address of an object the wasm owns, typed by what it is; 0 never occurs, null stands for no object. */',
        'export type Handle<Name extends string> = number & { readonly [HANDLE_OF]: Name };',
        'declare const HANDLE_OF: unique symbol;',
        "/** the address of scalars (`Pointer<'u16'>`) or of handles (`Pointer<Chunk>`) in the wasm's memory; 0 when there are none. */",
        'export type Pointer<To> = number & { readonly [POINTER_TO]: To };',
        'declare const POINTER_TO: unique symbol;',
        '',
        "/** before init, the wasm: any use of it throws, saying what to do, rather than failing on undefined */",
        'const UNBOOTED = new Proxy({} as Instance, {',
        '    get() {',
        "        throw new Error('tumble: await init() before anything else');",
        '    },',
        '});',
        '',
        'let wasm: Instance = UNBOOTED;',
        '',
        "/** binds the realm's wasm; the loader calls it once the wasm is initialised. */",
        'export function setWasm(booted: Instance): void {',
        '    wasm = booted;',
        '}',
        '',
        "/** the realm's wasm; throws when none has booted, which is a boot-order bug, not a runtime condition. */",
        'export function getWasm(): Instance {',
        "    if (wasm === UNBOOTED) throw new Error('tumble: await init() before anything else');",
        '    return wasm;',
        '}',
        '',
        'export function hasWasm(): boolean {',
        '    return wasm !== UNBOOTED;',
        '}',
    ];
    for (const c of schema.constants) parts.push('', ...emitConstant(c));
    for (const e of schema.enums) parts.push('', ...emitEnum(e));
    for (const s of schema.structs) if (s.value) parts.push('', ...emitValue(s));
    for (const ns of namespaces()) parts.push('', ...emitNamespace(ns));
    parts.push('', ...emitExportsType(), '');
    return parts.join('\n');
}

// ---- the extern "C" layer ----

/** the C++ spelling of a parameter as the wrapper takes it: references become pointers. */
function cppParam(t: TypeRef, where: string): string {
    const constness = t.const ? 'const ' : '';
    switch (t.kind) {
        case 'void':
            throw new Error(`${where}: void parameter`);
        case 'scalar':
            return `${constness}${CPP_SCALAR[t.scalar]}`;
        case 'ptr-scalar':
            return `${constness}${CPP_SCALAR[t.scalar]} *`;
        case 'handle':
        case 'value-in':
        case 'value-out':
            return `${constness}${t.cpp} *`;
        case 'ptr-array':
            return `${constness}${t.cpp} **`;
        default:
            throw new Error(`${where}: ${t.kind} cannot be a parameter`);
    }
}

function cppReturn(f: FunctionDesc): string {
    if (returnsNothing(f)) return 'void';
    const r = f.returns;
    const constness = r.const ? 'const ' : '';
    switch (r.kind) {
        case 'scalar':
            return CPP_SCALAR[r.scalar];
        case 'ptr-scalar':
            return `${constness}${CPP_SCALAR[r.scalar]} *`;
        case 'handle':
            return `${constness}${r.cpp} *`;
        case 'ptr-array':
            return `${constness}${r.cpp} **`;
        default:
            throw new Error(`${f.cpp}: ${r.kind} cannot be returned`);
    }
}

/** the wrapper's argument for a parameter: a reference dereferenced, a handle struct braced from its u32, a pointer to handle structs converted. */
function wrapperArg(t: TypeRef, name: string): string {
    if (t.kind === 'scalar' && t.bits) return `{${name}}`;
    if (t.kind === 'ptr-scalar' && t.bits) return `bind_bits(${name})`;
    return t.ref ? `*${name}` : name;
}

function exportsModule(): string {
    const lines: string[] = [
        '// generated by wasm/bind/generate.ts from wasm/bindings.cpp; do not edit.',
        '// the extern "C" layer: one forward per bound function, the shipped binary\'s whole surface.',
        '// the schema build leaves it out (WASM_SCHEMA), as it is what the schema build produces.',
        '#ifndef WASM_SCHEMA',
        '',
        '#include <stdint.h>',
        '',
        '#include "wasm/runtime/export.h"',
    ];
    for (const inc of schema.includes) lines.push(`#include "${inc}"`);
    lines.push('', 'using namespace tumble;', '');
    for (const f of schema.functions) {
        const { symbol } = functionNames(f);
        const params = f.params.map((p, i) => `${cppParam(p.type, f.cpp)} ${paramName(p, i)}`);
        const args = f.params.map((p, i) => wrapperArg(p.type, paramName(p, i)));
        const r = f.returns;
        const call = `${f.cpp}(${args.join(', ')})${r.kind === 'scalar' && r.bits ? '.bits' : ''}`;
        lines.push(
            `WASM_EXPORT ${cppReturn(f)} ${symbol}(${params.join(', ')}) { ${returnsNothing(f) ? call : `return ${call}`}; }`,
        );
    }
    lines.push('', '#endif', '');
    return lines.join('\n');
}

function hashHeader(): string {
    return [
        '// generated by wasm/bind/generate.ts; do not edit. the js side carries the same hash.',
        '#pragma once',
        '',
        '#include <stdint.h>',
        '',
        'namespace tumble::runtime {',
        `constexpr uint32_t SCHEMA_HASH = ${hashHex}u;`,
        '}',
        '',
    ].join('\n');
}

const outputs: Array<[string, string]> = [
    [TS_OUT, tsModule()],
    [EXPORTS_OUT, exportsModule()],
    [HASH_OUT, hashHeader()],
];
let changed = false;
for (const [path, text] of outputs) {
    let current = '';
    try {
        current = readFileSync(path, 'utf8');
    } catch {
        current = '';
    }
    if (current === text) continue;
    changed = true;
    if (check) {
        console.error(`${path} is stale; run sh wasm/toolchain/build-schema.sh`);
    } else {
        mkdirSync(dirname(path), { recursive: true });
        writeFileSync(path, text);
        console.log(`wrote ${path}`);
    }
}
if (check && changed) process.exit(1);
if (!changed) console.log(`schema ${hashHex} up to date`);
