import * as Low from '../generated/tumble.js';
import type { Vec3 } from '../math.js';
import type { Shape } from './shapes.js';

export type OffsetCenterOfMassShapeSettings = { shape: Shape; offset: Vec3 };

/** the shape's centre of mass moved by the offset and its geometry left where it is, to steady a car by lowering it; it holds the shape */
export function create(settings: OffsetCenterOfMassShapeSettings): Shape {
    Low.errorClear();
    return Low.OffsetCenterOfMass.create(settings);
}

// an offset centre of mass shape's fields: the getters read one only

/** the shape whose centre of mass it moves, which it holds: not to be released here */
export function getInnerShape(shape: Shape): Shape {
    return Low.OffsetCenterOfMass.shape(shape as number as Low.OffsetCenterOfMass)!;
}

export function getOffset(out: Vec3, shape: Shape): Vec3 {
    return Low.OffsetCenterOfMass.readOffset(out, shape as number as Low.OffsetCenterOfMass);
}
