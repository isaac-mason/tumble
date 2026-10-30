import * as Low from './generated/tumble.js';

/** bytes per vertex: x, y, z as f32s, then r, g, b, a as bytes */
export const DEBUG_VERTEX_BYTES = Low.DEBUG_VERTEX_BYTES;
/** where a vertex's colour starts */
export const DEBUG_VERTEX_COLOR_OFFSET = Low.DEBUG_VERTEX_COLOR_OFFSET;

/**
 * what read hands back: views straight onto the wasm's memory, valid until the next draw or clear, so take them from
 * read each time rather than keeping them. a line is two vertices and a triangle three, each DEBUG_VERTEX_BYTES: vertex
 * i's x y z are lines[i * 4] to lines[i * 4 + 2], its r g b a lineColors[i * 16 + 12] to lineColors[i * 16 + 15]. the
 * two views of a buffer cover the same bytes, laid out for a gpu as float32x3 and unorm8x4
 */
export type DebugViews = {
    lines: Float32Array;
    lineColors: Uint8Array;
    lineCount: number;
    triangles: Float32Array;
    triangleColors: Uint8Array;
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
const EMPTY_BYTES = new Uint8Array(0);

export function create(): DebugDraw {
    Low.errorClear();
    return {
        handle: Low.Debug.create(),
        views: {
            lines: EMPTY,
            lineColors: EMPTY_BYTES,
            lineCount: 0,
            triangles: EMPTY,
            triangleColors: EMPTY_BYTES,
            triangleCount: 0,
        },
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
    const lineBytes = lineCount * 2 * DEBUG_VERTEX_BYTES;
    const triangleBytes = triangleCount * 3 * DEBUG_VERTEX_BYTES;
    const moved = buffer !== buffers.buffer;
    if (moved || linesAddress !== buffers.linesAddress || lineBytes !== views.lineColors.length) {
        views.lines = lineBytes === 0 ? EMPTY : new Float32Array(buffer, linesAddress, lineBytes / 4);
        views.lineColors = lineBytes === 0 ? EMPTY_BYTES : new Uint8Array(buffer, linesAddress, lineBytes);
        buffers.linesAddress = linesAddress;
    }
    if (moved || trianglesAddress !== buffers.trianglesAddress || triangleBytes !== views.triangleColors.length) {
        views.triangles = triangleBytes === 0 ? EMPTY : new Float32Array(buffer, trianglesAddress, triangleBytes / 4);
        views.triangleColors = triangleBytes === 0 ? EMPTY_BYTES : new Uint8Array(buffer, trianglesAddress, triangleBytes);
        buffers.trianglesAddress = trianglesAddress;
    }
    views.lineCount = lineCount;
    views.triangleCount = triangleCount;
    buffers.buffer = buffer;
    return views;
}
