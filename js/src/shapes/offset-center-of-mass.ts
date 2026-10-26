import * as Low from '../generated/tumble.js';
import type { Vec3 } from '../math.js';
import type { Shape } from './shapes.js';

export type OffsetCenterOfMassShapeSettings = { shape: Shape; offset: Vec3 };

/** the shape's centre of mass moved by the offset and its geometry left where it is, to steady a car by lowering it; it holds the shape */
export function create(settings: OffsetCenterOfMassShapeSettings): Shape {
    Low.errorClear();
    return Low.OffsetCenterOfMass.create(settings);
}
