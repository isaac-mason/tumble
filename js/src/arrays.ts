import * as Low from './generated/tumble';

/**
 * the floats copied into a block of the wasm's memory for the length of `use`, which takes its address: how an array
 * crosses into a call
 */
export function withFloats<T>(values: ArrayLike<number>, use: (address: number) => T): T {
    const bytes = values.length * 4;
    const address = bytes === 0 ? 0 : Low.allocBytes(bytes);
    if (bytes > 0 && address === 0) Low.getWasm().check();
    const mem = Low.getWasm().mem;
    for (let i = 0; i < values.length; i++) mem.f32[(address >> 2) + i] = values[i]!;
    try {
        return use(address);
    } finally {
        if (address !== 0) Low.freeBytes(address, bytes);
    }
}
