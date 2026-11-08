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
    const normal = settings.normal ?? defaults.plane;
    Low.errorClear();
    return Low.Plane.create({
        // the normal in x, y and z, the constant in w
        plane: [normal[0], normal[1], normal[2], settings.constant ?? defaults.plane[3]],
        halfExtent: settings.halfExtent ?? defaults.halfExtent,
        materialId: settings.materialId ?? defaults.materialId,
    });
}

// a plane's fields, as it holds them: the getters read a plane only

const _plane: Low.Planef = [0, 0, 0, 0];

/** normalised */
export function getNormal(out: Vec3, shape: Shape): Vec3 {
    Low.Plane.readPlane(_plane, shape as number as Low.Plane);
    out[0] = _plane[0];
    out[1] = _plane[1];
    out[2] = _plane[2];
    return out;
}

export function getConstant(shape: Shape): number {
    return Low.Plane.readPlane(_plane, shape as number as Low.Plane)[3];
}

export function getHalfExtent(shape: Shape): number {
    return Low.Plane.halfExtent(shape as number as Low.Plane);
}
