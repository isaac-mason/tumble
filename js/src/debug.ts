import type { BodyId } from './body.js';
import type { DebugDraw } from './debug-draw.js';
import * as Low from './generated/tumble.js';
import type { KCC } from './kcc.js';
import type { Quat, Vec3 } from './math.js';
import { defaultsOf, over } from './settings.js';
import type { Shape } from './shapes/shapes.js';
import type { World } from './world.js';

export const BodyColorMode = Low.BodyColorMode;
export type BodyColorMode = Low.BodyColorMode;

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
/** every body's shape outside the excluded layers, coloured by the mode, with its velocities when asked */
export function bodies(out: DebugDraw, world: World, options: BodiesOptions = {}): void {
    const native = over(bodiesDefaults(), options);
    native.excludedObjectLayers >>>= 0;
    drawing(() => Low.Debug.bodies(out.handle, world, native));
}

/** one body's shape and velocities; false when the id names no body */
export function body(out: DebugDraw, world: World, body: BodyId, options: BodyOptions = {}): boolean {
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
}

export function line(out: DebugDraw, a: Vec3, b: Vec3, color: Vec3): void {
    drawing(() => Low.Debug.line(out.handle, a, b, color));
}

/** a shape at a world pose */
export function shape(out: DebugDraw, shape: Shape, position: Vec3, quaternion: Quat, options: ShapeOptions = {}): void {
    const native = over(shapeDefaults(), options);
    drawing(() => Low.Debug.shape(out.handle, shape, position, quaternion, native));
}

/** each contact's points on body a, with the normal */
export function contacts(out: DebugDraw, world: World): void {
    drawing(() => Low.Debug.contacts(out.handle, world));
}

/** each contact's manifold: both points, the outline, the normal and the tangents */
export function contactConstraints(out: DebugDraw, world: World): void {
    drawing(() => Low.Debug.contactConstraints(out.handle, world));
}

/** the character's contacts from its last move: each point, and its normal toward the character */
export function characterContacts(out: DebugDraw, character: KCC): void {
    drawing(() => Low.Debug.characterContacts(out.handle, character.handle));
}

/** every enabled constraint's anchors and axes, and its limits when asked */
export function joints(out: DebugDraw, world: World, options: JointsOptions = {}): void {
    const native = over(jointsDefaults(), options);
    drawing(() => Low.Debug.joints(out.handle, world, native));
}
