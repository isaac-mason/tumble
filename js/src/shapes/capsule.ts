import * as Low from '../generated/tumble.js';
import { defaultsOf, over } from '../settings.js';
import type { Density, Material, Shape } from './shapes.js';

/** along y */
export type CapsuleShapeSettings = {
    /** half the height of the cylinder between the hemispheres */
    halfHeightOfCylinder: number;
    radius: number;
} & Density &
    Material;

const capsuleDefaults = defaultsOf(Low.Capsule.defaultSettings, Low.CapsuleSettings.create);

export function create(settings: CapsuleShapeSettings): Shape {
    Low.errorClear();
    return Low.Capsule.create(over(capsuleDefaults(), settings));
}

/** the shape changed in place, to the settings as create takes them; each body using it takes it again with rigidBody.setShape */
export function update(shape: Shape, settings: CapsuleShapeSettings): void {
    Low.Capsule.update(shape, over(capsuleDefaults(), settings));
}
