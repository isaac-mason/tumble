import * as Low from '../generated/tumble.js';
import { defaultsOf, over } from '../settings.js';
import type { Density, Material, Shape } from './shapes.js';

export type SphereShapeSettings = { radius: number } & Density & Material;

const sphereDefaults = defaultsOf(Low.Sphere.defaultSettings, Low.SphereSettings.create);

export function create(settings: SphereShapeSettings): Shape {
    Low.errorClear();
    return Low.Sphere.create(over(sphereDefaults(), settings));
}

/** the shape changed in place, to the settings as create takes them; each body using it takes it again with rigidBody.setShape */
export function update(shape: Shape, settings: SphereShapeSettings): void {
    Low.Sphere.update(shape, over(sphereDefaults(), settings));
}

// a sphere's fields, as it holds them: the getters read a sphere only

export function getRadius(shape: Shape): number {
    return Low.Sphere.radius(shape as number as Low.Sphere);
}

export function getDensity(shape: Shape): number {
    return Low.Sphere.density(shape as number as Low.Sphere);
}
