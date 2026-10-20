import * as Low from './generated/tumble';

/** copies arrays into blocks of the wasm's memory, each freed when the withCopies that made it returns */
export type Copier = {
    f32(values: ArrayLike<number>): Low.Pointer<'f32'>;
    u32(values: ArrayLike<number>): Low.Pointer<'u32'>;
    i32(values: ArrayLike<number>): Low.Pointer<'i32'>;
    /** `count` bytes, as the wasm left them */
    bytes(count: number): Low.Pointer<'u8'>;
};

/** how an array crosses into a call: `use` takes the copies' addresses, and the blocks are freed once it returns */
export function withCopies<T>(use: (copy: Copier) => T): T {
    const blocks: number[] = [];
    const block = (bytes: number): number => {
        if (bytes === 0) return 0;
        Low.errorClear();
        const address = Low.allocBytes(bytes);
        if (address === 0) Low.getWasm().check();
        blocks.push(address, bytes);
        return address;
    };
    const copier: Copier = {
        f32(values) {
            const address = block(values.length * 4);
            Low.getWasm().mem.f32.set(values, address >> 2);
            return address as Low.Pointer<'f32'>;
        },
        u32(values) {
            const address = block(values.length * 4);
            Low.getWasm().mem.u32.set(values, address >> 2);
            return address as Low.Pointer<'u32'>;
        },
        i32(values) {
            const address = block(values.length * 4);
            Low.getWasm().mem.i32.set(values, address >> 2);
            return address as Low.Pointer<'i32'>;
        },
        bytes(count) {
            return block(count) as Low.Pointer<'u8'>;
        },
    };
    try {
        return use(copier);
    } finally {
        for (let i = 0; i < blocks.length; i += 2) Low.freeBytes(blocks[i]!, blocks[i + 1]!);
    }
}
