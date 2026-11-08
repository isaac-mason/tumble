import * as Low from '../generated/tumble.js';
import type { Quat, Vec3 } from '../math.js';
import { defaultsOf, over } from '../settings.js';
import type { Shape } from './shapes.js';

export type TransformedShapeSettings = { shape: Shape; position?: Vec3; quaternion?: Quat };

const transformedDefaults = defaultsOf(Low.Transformed.defaultSettings, Low.TransformedSettings.create);

/** the shape moved and turned within the body; it holds the shape */
export function create(settings: TransformedShapeSettings): Shape {
    Low.errorClear();
    return Low.Transformed.create(over(transformedDefaults(), settings));
}

// a transformed shape's fields: the getters read a transformed shape only

/** the shape it places, which it holds: not to be released here */
export function getInnerShape(shape: Shape): Shape {
    return Low.Transformed.shape(shape as number as Low.Transformed)!;
}

export function getPosition(out: Vec3, shape: Shape): Vec3 {
    return Low.Transformed.readPosition(out, shape as number as Low.Transformed);
}

export function getQuaternion(out: Quat, shape: Shape): Quat {
    return Low.Transformed.readQuaternion(out, shape as number as Low.Transformed);
}
