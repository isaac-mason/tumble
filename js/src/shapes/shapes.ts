import { withCopies } from '../arrays.js';
import * as Low from '../generated/tumble.js';
import type { Box3, Quat, Vec3 } from '../math.js';

/** a shape: counted, so releaseShape lets go of yours and the last holder frees it */
export type Shape = Low.Shape;

export const ShapeType = Low.ShapeType;
export type ShapeType = Low.ShapeType;

/** mass and inertia about the centre of mass; the inertia is the upper 3x3 of a column-major 4x4, kg m2 */
export type MassProperties = Low.MassProperties;

export function getShapeType(shape: Shape): ShapeType {
    return Low.Shape.getType(shape) as ShapeType;
}

/** local space bounds */
export function getShapeLocalBounds(out: Box3, shape: Shape): Box3 {
    return Low.Shape.getLocalBounds(out, shape);
}

/** local space */
export function getShapeCenterOfMass(out: Vec3, shape: Shape): Vec3 {
    return Low.Shape.getCenterOfMass(out, shape);
}

/** m3 */
export function getShapeVolume(shape: Shape): number {
    return Low.Shape.getVolume(shape);
}

/** at the shape's density, about its centre of mass */
export function computeMassProperties(out: MassProperties, shape: Shape): MassProperties {
    return Low.Shape.getMassProperties(out, shape);
}

/** how many references hold the shape: bodies', characters', the shapes built over it, and this one until release */
export function getShapeRefCount(shape: Shape): number {
    return Low.Shape.getRefCount(shape);
}

/**
 * lets go of the shape made here: bodies, characters and shapes built over it hold their own, and the last to let go
 * frees it. a shape is released once, after which it is not to be used here
 */
export function releaseShape(shape: Shape): void {
    Low.Shape.release(shape);
}

/** kg/m3 for every solid shape; a body's mass, when given, scales it away */
export type Density = { density?: number };
/** the surface a hit on the shape reports; -1 (the default) for none, the body's friction and restitution standing */
export type Material = { materialId?: number };

export const ConvexHullBuildResult = Low.ConvexHullBuildResult;
export type ConvexHullBuildResult = Low.ConvexHullBuildResult;
export const BakeSupportAdjacency = Low.BakeSupportAdjacency;
export type BakeSupportAdjacency = Low.BakeSupportAdjacency;
/** why the points made no hull */
export class ConvexHullBuildError extends Error {
    readonly result: ConvexHullBuildResult;
    constructor(result: ConvexHullBuildResult) {
        const name = Object.entries(ConvexHullBuildResult).find(([, value]) => value === result)?.[0] ?? String(result);
        super(`tumble: no convex hull: ${name}`);
        this.result = result;
    }
}
export const BvhSplitStrategy = Low.BvhSplitStrategy;
export type BvhSplitStrategy = Low.BvhSplitStrategy;
/** a child of a compound: a shape the compound holds, placed in the compound's space */
export type CompoundChild = { shape: Shape; position?: Vec3; quaternion?: Quat };

const ORIGIN: Vec3 = [0, 0, 0];
const IDENTITY: Quat = [0, 0, 0, 1];

export function withChildren<T>(children: CompoundChild[], use: (address: Low.Pointer<'u8'>) => T): T {
    if (children.length === 0) throw new Error('tumble: a compound needs at least one child');
    return withCopies((copy) => {
        const size = Low.CompoundChild.size;
        const address = copy.bytes(children.length * size);
        for (let i = 0; i < children.length; i++) {
            const child = children[i]!;
            Low.CompoundChild.write(address + i * size, {
                shape: child.shape,
                position: child.position ?? ORIGIN,
                quaternion: child.quaternion ?? IDENTITY,
            });
        }
        Low.errorClear();
        return use(address);
    });
}
