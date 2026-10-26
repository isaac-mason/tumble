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
