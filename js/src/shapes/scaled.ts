import * as Low from '../generated/tumble.js';
import type { Vec3 } from '../math.js';
import type { Shape } from './shapes.js';

export type ScaledShapeSettings = { shape: Shape; scale: Vec3 };

/**
 * the shape scaled per axis, which it holds. a sphere, capsule or cylinder takes a uniform scale only, and a
 * compound's children must not be rotated under a non-uniform one
 */
export function create(settings: ScaledShapeSettings): Shape {
    Low.errorClear();
    return Low.Scaled.create(settings);
}

// a scaled shape's fields: the getters read a scaled shape only

/** the shape it scales, which it holds: not to be released here */
export function getInnerShape(shape: Shape): Shape {
    return Low.Scaled.shape(shape as number as Low.Scaled)!;
}

export function getScale(out: Vec3, shape: Shape): Vec3 {
    return Low.Scaled.readScale(out, shape as number as Low.Scaled);
}
