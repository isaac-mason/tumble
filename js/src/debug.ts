import * as Low from './generated/tumble';
import type { KCC } from './kcc';
import type { Quat, Vec3 } from './math';
import type { BodyId } from './rigid-body';
import { defaultsOf, over } from './settings';
import type { Shape } from './shapes';
import type { World } from './world';

export const BodyColorMode = Low.BodyColorMode;
export type BodyColorMode = Low.BodyColorMode;

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

/** wireframes drawn into buffers in the wasm, kept between frames: clear, draw, read; free with DebugBuffers.destroy */
export type DebugBuffers = {
    handle: Low.DebugBuffers;
    views: DebugViews;
    buffer: ArrayBufferLike | null;
    linesAddress: number;
    trianglesAddress: number;
};

const EMPTY = new Float32Array(0);

export const DebugBuffers = {
    create(): DebugBuffers {
        Low.errorClear();
        return {
            handle: Low.Debug.create(),
            views: { lines: EMPTY, lineCount: 0, triangles: EMPTY, triangleCount: 0 },
            buffer: null,
            linesAddress: 0,
            trianglesAddress: 0,
        };
    },
    destroy(buffers: DebugBuffers): void {
        Low.Debug.destroy(buffers.handle);
    },
    /** empty, keeping the memory, so drawing the same scene again does not grow it */
    clear(buffers: DebugBuffers): void {
        Low.Debug.clear(buffers.handle);
    },
    read(buffers: DebugBuffers): DebugViews {
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
    },
};

export type BodiesOptions = {
    colorMode?: BodyColorMode;
    showLinearVelocity?: boolean;
    showAngularVelocity?: boolean;
    /** a bit per object layer whose bodies are not drawn */
    excludedObjectLayers?: number;
};

export type BodyOptions = {
    /** in place of the colour mode's */
    color?: Vec3 | null;
    colorMode?: BodyColorMode;
    showLinearVelocity?: boolean;
    showAngularVelocity?: boolean;
};

export type ShapeOptions = { color?: Vec3 };

export type JointsOptions = {
    /** the length of an axis's arm */
    size?: number;
    /** angular and translation limits */
    drawLimits?: boolean;
};

const bodiesDefaults = defaultsOf(Low.Debug.defaultBodiesOptions, Low.DebugBodiesOptions.create);
const bodyDefaults = defaultsOf(Low.Debug.defaultBodyOptions, Low.DebugBodyOptions.create);
const shapeDefaults = defaultsOf(Low.Debug.defaultShapeOptions, Low.DebugShapeOptions.create);
const jointsDefaults = defaultsOf(Low.Debug.defaultJointsOptions, Low.DebugJointsOptions.create);

function drawing(call: () => void): void {
    Low.errorClear();
    call();
    Low.getWasm().check();
}

// each draw appends to the buffers
export const Debug = {
    /** every body's shape outside the excluded layers, coloured by the mode, with its velocities when asked */
    bodies(out: DebugBuffers, world: World, options: BodiesOptions = {}): void {
        const native = over(bodiesDefaults(), options);
        native.excludedObjectLayers >>>= 0;
        drawing(() => Low.Debug.bodies(out.handle, world, native));
    },
    /** one body's shape and velocities; false when the id names no body */
    body(out: DebugBuffers, world: World, body: BodyId, options: BodyOptions = {}): boolean {
        const { color, ...rest } = options;
        const native = over(bodyDefaults(), rest);
        native.hasColor = color !== undefined && color !== null;
        if (color !== undefined && color !== null) native.color = color;
        Low.errorClear();
        const drawn = Low.Debug.body(out.handle, world, body, native);
        // a bad id is the answer, not an error to throw; anything else is
        if (!drawn) Low.errorClear();
        else Low.getWasm().check();
        return drawn;
    },
    line(out: DebugBuffers, a: Vec3, b: Vec3, color: Vec3): void {
        drawing(() => Low.Debug.line(out.handle, a, b, color));
    },
    /** a shape at a world pose */
    shape(out: DebugBuffers, shape: Shape, position: Vec3, quaternion: Quat, options: ShapeOptions = {}): void {
        const native = over(shapeDefaults(), options);
        drawing(() => Low.Debug.shape(out.handle, shape, position, quaternion, native));
    },
    /** each contact's points on body a, with the normal */
    contacts(out: DebugBuffers, world: World): void {
        drawing(() => Low.Debug.contacts(out.handle, world));
    },
    /** each contact's manifold: both points, the outline, the normal and the tangents */
    contactConstraints(out: DebugBuffers, world: World): void {
        drawing(() => Low.Debug.contactConstraints(out.handle, world));
    },
    /** the character's contacts from its last move: each point, and its normal toward the character */
    characterContacts(out: DebugBuffers, character: KCC): void {
        drawing(() => Low.Debug.characterContacts(out.handle, character.handle));
    },
    /** every enabled constraint's anchors and axes, and its limits when asked */
    joints(out: DebugBuffers, world: World, options: JointsOptions = {}): void {
        const native = over(jointsDefaults(), options);
        drawing(() => Low.Debug.joints(out.handle, world, native));
    },
};
