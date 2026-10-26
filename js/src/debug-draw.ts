import * as Low from './generated/tumble.js';

/** floats per vertex: x, y, z, r, g, b */
export const DEBUG_VERTEX_STRIDE = Low.DEBUG_VERTEX_STRIDE;

/**
 * what read hands back: views straight onto the wasm's memory, x y z r g b per vertex (two a line, three a
 * triangle), valid until the next draw or clear, so take them from read each time rather than keeping them
 */
export type DebugViews = {
    lines: Float32Array;
    lineCount: number;
    triangles: Float32Array;
    triangleCount: number;
};

/** wireframes drawn into buffers in the wasm, kept between frames: clear, draw, read; free with debugDraw.destroy */
export type DebugDraw = {
    handle: Low.DebugDraw;
    views: DebugViews;
    buffer: ArrayBufferLike | null;
    linesAddress: number;
    trianglesAddress: number;
};

const EMPTY = new Float32Array(0);

export function create(): DebugDraw {
    Low.errorClear();
    return {
        handle: Low.Debug.create(),
        views: { lines: EMPTY, lineCount: 0, triangles: EMPTY, triangleCount: 0 },
        buffer: null,
        linesAddress: 0,
        trianglesAddress: 0,
    };
}

export function destroy(buffers: DebugDraw): void {
    Low.Debug.destroy(buffers.handle);
}

/** empty, keeping the memory, so drawing the same scene again does not grow it */
export function clear(buffers: DebugDraw): void {
    Low.Debug.clear(buffers.handle);
}

export function read(buffers: DebugDraw): DebugViews {
    const { handle, views } = buffers;
    const buffer = Low.getWasm().mem.memory.buffer;
    const lineCount = Low.Debug.lineCount(handle);
    const triangleCount = Low.Debug.triangleCount(handle);
    const linesAddress = Low.Debug.lines(handle);
    const trianglesAddress = Low.Debug.triangles(handle);
    const lineFloats = lineCount * 2 * DEBUG_VERTEX_STRIDE;
    const triangleFloats = triangleCount * 3 * DEBUG_VERTEX_STRIDE;
    const moved = buffer !== buffers.buffer;
    if (moved || linesAddress !== buffers.linesAddress || lineFloats !== views.lines.length) {
        views.lines = lineFloats === 0 ? EMPTY : new Float32Array(buffer, linesAddress, lineFloats);
        buffers.linesAddress = linesAddress;
    }
    if (moved || trianglesAddress !== buffers.trianglesAddress || triangleFloats !== views.triangles.length) {
        views.triangles = triangleFloats === 0 ? EMPTY : new Float32Array(buffer, trianglesAddress, triangleFloats);
        buffers.trianglesAddress = trianglesAddress;
    }
    views.lineCount = lineCount;
    views.triangleCount = triangleCount;
    buffers.buffer = buffer;
    return views;
}
