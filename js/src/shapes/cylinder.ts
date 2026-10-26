import * as Low from '../generated/tumble.js';
import { defaultsOf, over } from '../settings.js';
import type { Density, Material, Shape } from './shapes.js';

/** along y */
export type CylinderShapeSettings = {
    halfHeight: number;
    radius: number;
    /** rounds the rims; clamped to the half height and the radius */
    convexRadius?: number;
} & Density &
    Material;

const cylinderDefaults = defaultsOf(Low.Cylinder.defaultSettings, Low.CylinderSettings.create);

export function create(settings: CylinderShapeSettings): Shape {
    Low.errorClear();
    return Low.Cylinder.create(over(cylinderDefaults(), settings));
}

/** the shape changed in place, to the settings as create takes them; each body using it takes it again with rigidBody.setShape */
export function update(shape: Shape, settings: CylinderShapeSettings): void {
    Low.Cylinder.update(shape, over(cylinderDefaults(), settings));
}
