import { withCopies } from './arrays.js';
import * as Low from './generated/tumble.js';
import type { Box3, Quat, Vec3 } from './math.js';
import { defaultsOf, over } from './settings.js';

/** a shape, yours to free with its type's destroy once no body or other shape uses it */
export type Shape = Low.Shape;

export const ShapeType = Low.ShapeType;
export type ShapeType = Low.ShapeType;

/** mass and inertia about the centre of mass; the inertia is the upper 3x3 of a column-major 4x4, kg m2 */
export type MassProperties = Low.MassProperties;

/** what every shape has */
export const Shape = {
    getType(shape: Shape): ShapeType {
        return Low.Shape.getType(shape) as ShapeType;
    },
    /** local space bounds */
    getLocalBounds(out: Box3, shape: Shape): Box3 {
        return Low.Shape.getLocalBounds(out, shape);
    },
    /** local space */
    getCenterOfMass(out: Vec3, shape: Shape): Vec3 {
        return Low.Shape.getCenterOfMass(out, shape);
    },
    /** m3 */
    getVolume(shape: Shape): number {
        return Low.Shape.getVolume(shape);
    },
    /** at the shape's density, about its centre of mass */
    getMassProperties(out: MassProperties, shape: Shape): MassProperties {
        return Low.Shape.getMassProperties(out, shape);
    },
};

/** kg/m3 for every solid shape; a body's mass, when given, scales it away */
type Density = { density?: number };
/** the surface a hit on the shape reports; -1 (the default) for none, the body's friction and restitution standing */
type Material = { materialId?: number };

export type BoxSettings = {
    halfExtents: Vec3;
    /** rounds the edges a little, for faster and steadier collisions; clamped to the smallest half extent */
    convexRadius?: number;
} & Density &
    Material;

const boxDefaults = defaultsOf(Low.Box.defaultSettings, Low.BoxSettings.create);

export const Box = {
    create(settings: BoxSettings): Shape {
        Low.errorClear();
        return Low.Box.create(over(boxDefaults(), settings));
    },
    destroy(shape: Shape): void {
        Low.Box.destroy(shape);
    },
};

export type SphereSettings = { radius: number } & Density & Material;

const sphereDefaults = defaultsOf(Low.Sphere.defaultSettings, Low.SphereSettings.create);

export const Sphere = {
    create(settings: SphereSettings): Shape {
        Low.errorClear();
        return Low.Sphere.create(over(sphereDefaults(), settings));
    },
    destroy(shape: Shape): void {
        Low.Sphere.destroy(shape);
    },
};

/** along y */
export type CapsuleSettings = {
    /** half the height of the cylinder between the hemispheres */
    halfHeightOfCylinder: number;
    radius: number;
} & Density &
    Material;

const capsuleDefaults = defaultsOf(Low.Capsule.defaultSettings, Low.CapsuleSettings.create);

export const Capsule = {
    create(settings: CapsuleSettings): Shape {
        Low.errorClear();
        return Low.Capsule.create(over(capsuleDefaults(), settings));
    },
    destroy(shape: Shape): void {
        Low.Capsule.destroy(shape);
    },
};

/** along y */
export type CylinderSettings = {
    halfHeight: number;
    radius: number;
    /** rounds the rims; clamped to the half height and the radius */
    convexRadius?: number;
} & Density &
    Material;

const cylinderDefaults = defaultsOf(Low.Cylinder.defaultSettings, Low.CylinderSettings.create);

export const Cylinder = {
    create(settings: CylinderSettings): Shape {
        Low.errorClear();
        return Low.Cylinder.create(over(cylinderDefaults(), settings));
    },
    destroy(shape: Shape): void {
        Low.Cylinder.destroy(shape);
    },
};

/** a half space, solid behind the plane; for static bodies */
export type PlaneSettings = {
    /** out of the solid side, normalised by create; up by default */
    normal?: Vec3;
    /** point . normal + constant = 0 */
    constant?: number;
    /** how far the plane reaches, for its bounds */
    halfExtent?: number;
} & Material;

const planeDefaults = defaultsOf(Low.Plane.defaultSettings, Low.PlaneSettings.create);

export const Plane = {
    create(settings: PlaneSettings = {}): Shape {
        const defaults = planeDefaults();
        Low.errorClear();
        return Low.Plane.create({
            plane: { normal: settings.normal ?? defaults.plane.normal, constant: settings.constant ?? defaults.plane.constant },
            halfExtent: settings.halfExtent ?? defaults.halfExtent,
            materialId: settings.materialId ?? defaults.materialId,
        });
    },
    destroy(shape: Shape): void {
        Low.Plane.destroy(shape);
    },
};

/** no volume and no collisions: a placeholder for a body whose shape comes later */
export const Empty = {
    create(): Shape {
        Low.errorClear();
        return Low.Empty.create();
    },
    destroy(shape: Shape): void {
        Low.Empty.destroy(shape);
    },
};

/** a built hull, shared by any number of ConvexHull shapes; release it after them */
export type ConvexHullData = Low.ConvexHullData;

export const ConvexHullBuildResult = Low.ConvexHullBuildResult;
export type ConvexHullBuildResult = Low.ConvexHullBuildResult;
export const BakeSupportAdjacency = Low.BakeSupportAdjacency;
export type BakeSupportAdjacency = Low.BakeSupportAdjacency;

export type ConvexHullBuildSettings = {
    /** xyz per point, interior points allowed */
    positions: ArrayLike<number>;
    convexRadius?: number;
    /** points may lie this far outside the hull; bigger gives fewer vertices */
    hullTolerance?: number;
    /** how far a vertex may shift at a sharp edge when the hull is shrunk by the convex radius */
    maxErrorConvexRadius?: number;
    bakeSupportAdjacency?: BakeSupportAdjacency;
};

/** why a hull build failed */
export class ConvexHullBuildError extends Error {
    readonly result: ConvexHullBuildResult;
    constructor(result: ConvexHullBuildResult) {
        const name = Object.entries(ConvexHullBuildResult).find(([, value]) => value === result)?.[0] ?? String(result);
        super(`tumble: no convex hull: ${name}`);
        this.result = result;
    }
}

const hullBuildDefaults = defaultsOf(Low.ConvexHullData.defaultSettings, Low.ConvexHullBuildSettings.create);
const hullBuild = Low.ConvexHullBuild.create();

export const ConvexHullData = {
    /** the hull around the points; throws ConvexHullBuildError when they make none */
    build(settings: ConvexHullBuildSettings): ConvexHullData {
        if (settings.positions.length % 3 !== 0) throw new Error('tumble: hull positions are xyz triples');
        const defaults = hullBuildDefaults();
        return withCopies((copy) => {
            const positions = copy.f32(settings.positions);
            Low.ConvexHullData.build(hullBuild, {
                positions,
                numPositions: settings.positions.length / 3,
                convexRadius: settings.convexRadius ?? defaults.convexRadius,
                hullTolerance: settings.hullTolerance ?? defaults.hullTolerance,
                maxErrorConvexRadius: settings.maxErrorConvexRadius ?? defaults.maxErrorConvexRadius,
                bakeSupportAdjacency: settings.bakeSupportAdjacency ?? defaults.bakeSupportAdjacency,
            });
            if (hullBuild.data === null) throw new ConvexHullBuildError(hullBuild.result as ConvexHullBuildResult);
            return hullBuild.data;
        });
    },
    release(data: ConvexHullData): void {
        Low.ConvexHullData.release(data);
    },
};

export type ConvexHullSettings = { data: ConvexHullData } & Density & Material;

const hullDefaults = defaultsOf(Low.ConvexHull.defaultSettings, Low.ConvexHullSettings.create);

export const ConvexHull = {
    /** a shape over built hull data, which it borrows */
    create(settings: ConvexHullSettings): Shape {
        Low.errorClear();
        return Low.ConvexHull.create(over(hullDefaults(), settings));
    },
    destroy(shape: Shape): void {
        Low.ConvexHull.destroy(shape);
    },
};

/** a built mesh, shared by any number of TriangleMesh shapes; release it after them */
export type TriangleMeshData = Low.TriangleMeshData;

export const BvhSplitStrategy = Low.BvhSplitStrategy;
export type BvhSplitStrategy = Low.BvhSplitStrategy;

export type TriangleMeshBuildSettings = {
    /** xyz per vertex */
    positions: ArrayLike<number>;
    /** three vertex indices per triangle, counter clockwise seen from the front */
    indices: ArrayLike<number>;
    /** one per triangle, the surface a hit on it reports; -1 for none */
    materialIds?: ArrayLike<number>;
    /** a triangle with |cross product| below this is dropped */
    degenerateTolerance?: number;
    bvhSplitStrategy?: BvhSplitStrategy;
    bvhMaxLeafTris?: number;
    /** a convex edge whose cos(dihedral angle) is below this is active, so bodies slide over the rest; negative keeps every edge active */
    activeEdgeCosThresholdAngle?: number;
};

const meshBuildDefaults = defaultsOf(Low.TriangleMeshData.defaultSettings, Low.TriangleMeshBuildSettings.create);

export const TriangleMeshData = {
    build(settings: TriangleMeshBuildSettings): TriangleMeshData {
        const { positions, indices, materialIds } = settings;
        if (positions.length % 3 !== 0) throw new Error('tumble: mesh positions are xyz triples');
        if (indices.length % 3 !== 0) throw new Error('tumble: mesh indices are triples');
        if (materialIds !== undefined && materialIds.length !== indices.length / 3) {
            throw new Error('tumble: one material id per triangle');
        }
        const defaults = meshBuildDefaults();
        return withCopies((copy) => {
            Low.errorClear();
            return Low.TriangleMeshData.build({
                positions: copy.f32(positions),
                numPositions: positions.length / 3,
                indices: copy.u32(indices),
                numIndices: indices.length,
                materialIds: materialIds === undefined ? (0 as Low.Pointer<'i32'>) : copy.i32(materialIds),
                degenerateTolerance: settings.degenerateTolerance ?? defaults.degenerateTolerance,
                bvhSplitStrategy: settings.bvhSplitStrategy ?? defaults.bvhSplitStrategy,
                bvhMaxLeafTris: settings.bvhMaxLeafTris ?? defaults.bvhMaxLeafTris,
                activeEdgeCosThresholdAngle: settings.activeEdgeCosThresholdAngle ?? defaults.activeEdgeCosThresholdAngle,
            });
        });
    },
    release(data: TriangleMeshData): void {
        Low.TriangleMeshData.release(data);
    },
};

/** for static bodies, or kinematic ones: a mesh has no volume, so a dynamic body over one needs massPropertiesOverride */
export type TriangleMeshSettings = {
    data: TriangleMeshData;
    /** the shape's own material, for queries that report it; hits report the hit triangle's */
    materialId?: number;
};

const meshDefaults = defaultsOf(Low.TriangleMesh.defaultSettings, Low.TriangleMeshSettings.create);

export const TriangleMesh = {
    /** a shape over built mesh data, which it borrows */
    create(settings: TriangleMeshSettings): Shape {
        Low.errorClear();
        return Low.TriangleMesh.create(over(meshDefaults(), settings));
    },
    destroy(shape: Shape): void {
        Low.TriangleMesh.destroy(shape);
    },
};

/** a child of a compound: a shape the compound borrows, placed in the compound's space */
export type CompoundChild = { shape: Shape; position?: Vec3; quaternion?: Quat };

const ORIGIN: Vec3 = [0, 0, 0];
const IDENTITY: Quat = [0, 0, 0, 1];

function withChildren<T>(children: CompoundChild[], use: (address: Low.Pointer<'u8'>) => T): T {
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

export type CompoundSettings = { children: CompoundChild[] };

export const Compound = {
    /** children placed together; it copies the list and borrows the shapes */
    create(settings: CompoundSettings): Shape {
        return withChildren(settings.children, (address) => Low.Compound.create(address, settings.children.length));
    },
    destroy(shape: Shape): void {
        Low.Compound.destroy(shape);
    },
};

export type StaticCompoundSettings = {
    children: CompoundChild[];
    /** children per leaf of the tree over them */
    bvhMaxLeafChildren?: number;
};

export const StaticCompound = {
    /** a compound with a tree over its children, for many children that never change */
    create(settings: StaticCompoundSettings): Shape {
        const leafChildren = settings.bvhMaxLeafChildren ?? Low.StaticCompound.defaultBvhMaxLeafChildren();
        return withChildren(settings.children, (address) =>
            Low.StaticCompound.create(address, settings.children.length, leafChildren),
        );
    },
    destroy(shape: Shape): void {
        Low.StaticCompound.destroy(shape);
    },
};

export type TransformedSettings = { shape: Shape; position?: Vec3; quaternion?: Quat };

const transformedDefaults = defaultsOf(Low.Transformed.defaultSettings, Low.TransformedSettings.create);

export const Transformed = {
    /** the shape moved and turned within the body; it borrows the shape */
    create(settings: TransformedSettings): Shape {
        Low.errorClear();
        return Low.Transformed.create(over(transformedDefaults(), settings));
    },
    destroy(shape: Shape): void {
        Low.Transformed.destroy(shape);
    },
};

export type ScaledSettings = { shape: Shape; scale: Vec3 };

export const Scaled = {
    /**
     * the shape scaled per axis, borrowing it. a sphere, capsule or cylinder takes a uniform scale only, and a
     * compound's children must not be rotated under a non-uniform one
     */
    create(settings: ScaledSettings): Shape {
        Low.errorClear();
        return Low.Scaled.create(settings);
    },
    destroy(shape: Shape): void {
        Low.Scaled.destroy(shape);
    },
};

export type OffsetCenterOfMassSettings = { shape: Shape; offset: Vec3 };

export const OffsetCenterOfMass = {
    /** the shape's centre of mass moved by the offset and its geometry left where it is, to steady a car by lowering it; it borrows the shape */
    create(settings: OffsetCenterOfMassSettings): Shape {
        Low.errorClear();
        return Low.OffsetCenterOfMass.create(settings);
    },
    destroy(shape: Shape): void {
        Low.OffsetCenterOfMass.destroy(shape);
    },
};
