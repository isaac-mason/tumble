import * as Low from './generated/tumble';
import type { Vec3 } from './math';

/** a shape, yours to free with its type's destroy once no body uses it */
export type Shape = Low.Shape;

const DEFAULT_DENSITY = 1000;
const DEFAULT_CONVEX_RADIUS = 0.05;

export type BoxSettings = {
    halfExtents: Vec3;
    /** rounds the edges a little, for faster and steadier collisions */
    convexRadius?: number;
    /** kg/m3 */
    density?: number;
    materialId?: number;
};

export const Box = {
    create(settings: BoxSettings): Shape {
        return Low.Box.create({
            halfExtents: settings.halfExtents,
            convexRadius: settings.convexRadius ?? DEFAULT_CONVEX_RADIUS,
            density: settings.density ?? DEFAULT_DENSITY,
            materialId: settings.materialId ?? -1,
        });
    },
    destroy(shape: Shape): void {
        Low.Box.destroy(shape);
    },
};

export type SphereSettings = {
    radius: number;
    density?: number;
    materialId?: number;
};

export const Sphere = {
    create(settings: SphereSettings): Shape {
        return Low.Sphere.create({
            radius: settings.radius,
            density: settings.density ?? DEFAULT_DENSITY,
            materialId: settings.materialId ?? -1,
        });
    },
    destroy(shape: Shape): void {
        Low.Sphere.destroy(shape);
    },
};
