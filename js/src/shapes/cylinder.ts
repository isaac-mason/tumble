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

// a cylinder's fields, as it holds them: the getters read a cylinder only

export function getHalfHeight(shape: Shape): number {
    return Low.Cylinder.halfHeight(shape as number as Low.Cylinder);
}

export function getRadius(shape: Shape): number {
    return Low.Cylinder.radius(shape as number as Low.Cylinder);
}

/** clamped to the half height and the radius */
export function getConvexRadius(shape: Shape): number {
    return Low.Cylinder.convexRadius(shape as number as Low.Cylinder);
}

export function getDensity(shape: Shape): number {
    return Low.Cylinder.density(shape as number as Low.Cylinder);
}
