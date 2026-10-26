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
