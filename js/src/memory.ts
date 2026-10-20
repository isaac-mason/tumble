// typed-array views over the wasm's memory. a view is bound to one ArrayBuffer, and the wasm replaces the buffer
// whenever it grows the memory: `refreshViews` re-derives the views after any call that can allocate.

export type WasmMemory = {
    memory: WebAssembly.Memory;
    /** the buffer every view was made on; a mismatch means re-derive. */
    buffer: ArrayBuffer;
    u8: Uint8Array;
    u16: Uint16Array;
    u32: Uint32Array;
    i32: Int32Array;
    f32: Float32Array;
    f64: Float64Array;
};

export function createWasmMemory(memory: WebAssembly.Memory): WasmMemory {
    const buffer = memory.buffer;
    return {
        memory,
        buffer,
        u8: new Uint8Array(buffer),
        u16: new Uint16Array(buffer),
        u32: new Uint32Array(buffer),
        i32: new Int32Array(buffer),
        f32: new Float32Array(buffer),
        f64: new Float64Array(buffer),
    };
}

/** rebinds the views when the buffer changed. */
export function refreshViews(mem: WasmMemory): void {
    if (mem.memory.buffer === mem.buffer) return;
    const buffer = mem.memory.buffer;
    mem.buffer = buffer;
    mem.u8 = new Uint8Array(buffer);
    mem.u16 = new Uint16Array(buffer);
    mem.u32 = new Uint32Array(buffer);
    mem.i32 = new Int32Array(buffer);
    mem.f32 = new Float32Array(buffer);
    mem.f64 = new Float64Array(buffer);
}
