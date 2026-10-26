import * as Low from '../generated/tumble.js';
import type { Vec3 } from '../math.js';
import { defaultsOf } from '../settings.js';
import type { Material, Shape } from './shapes.js';

/** a half space, solid behind the plane; for static bodies */
export type PlaneShapeSettings = {
    /** out of the solid side, normalised by create; up by default */
    normal?: Vec3;
    /** point . normal + constant = 0 */
    constant?: number;
    /** how far the plane reaches, for its bounds */
    halfExtent?: number;
} & Material;

const planeDefaults = defaultsOf(Low.Plane.defaultSettings, Low.PlaneSettings.create);

export function create(settings: PlaneShapeSettings = {}): Shape {
    const defaults = planeDefaults();
    Low.errorClear();
    return Low.Plane.create({
        plane: { normal: settings.normal ?? defaults.plane.normal, constant: settings.constant ?? defaults.plane.constant },
        halfExtent: settings.halfExtent ?? defaults.halfExtent,
        materialId: settings.materialId ?? defaults.materialId,
    });
}
