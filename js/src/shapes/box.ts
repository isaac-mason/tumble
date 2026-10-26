import * as Low from '../generated/tumble.js';
import type { Vec3 } from '../math.js';
import { defaultsOf, over } from '../settings.js';
import type { Density, Material, Shape } from './shapes.js';

export type BoxShapeSettings = {
    halfExtents: Vec3;
    /** rounds the edges a little, for faster and steadier collisions; clamped to the smallest half extent */
    convexRadius?: number;
} & Density &
    Material;

const boxDefaults = defaultsOf(Low.Box.defaultSettings, Low.BoxSettings.create);

export function create(settings: BoxShapeSettings): Shape {
    Low.errorClear();
    return Low.Box.create(over(boxDefaults(), settings));
}

/** the shape changed in place, to the settings as create takes them; each body using it takes it again with rigidBody.setShape */
export function update(shape: Shape, settings: BoxShapeSettings): void {
    Low.Box.update(shape, over(boxDefaults(), settings));
}
