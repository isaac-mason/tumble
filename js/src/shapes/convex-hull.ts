import { withCopies } from '../arrays.js';
import * as Low from '../generated/tumble.js';
import type { Vec3 } from '../math.js';
import { defaultsOf } from '../settings.js';
import {
    type BakeSupportAdjacency,
    ConvexHullBuildError,
    type ConvexHullBuildResult,
    type Density,
    type Material,
    type Shape,
} from './shapes.js';

/** the points to build the hull from, and how */
export type ConvexHullShapeSettings = {
    /** xyz per point, interior points allowed */
    positions: ArrayLike<number>;
    convexRadius?: number;
    /** points may lie this far outside the hull; bigger gives fewer vertices */
    hullTolerance?: number;
    /** how far a vertex may shift at a sharp edge when the hull is shrunk by the convex radius */
    maxErrorConvexRadius?: number;
    bakeSupportAdjacency?: BakeSupportAdjacency;
} & Density &
    Material;

const hullDefaults = defaultsOf(Low.ConvexHull.defaultSettings, Low.ConvexHullSettings.create);
const hullCreate = Low.ConvexHullCreate.create();

/** a shape over the hull of the points, which it builds; throws ConvexHullBuildError when they make none */
export function create(settings: ConvexHullShapeSettings): Shape {
    if (settings.positions.length % 3 !== 0) throw new Error('tumble: hull positions are xyz triples');
    const defaults = hullDefaults();
    return withCopies((copy) => {
        Low.errorClear();
        Low.ConvexHull.create(hullCreate, {
            positions: copy.f32(settings.positions),
            numPositions: settings.positions.length / 3,
            convexRadius: settings.convexRadius ?? defaults.convexRadius,
            hullTolerance: settings.hullTolerance ?? defaults.hullTolerance,
            maxErrorConvexRadius: settings.maxErrorConvexRadius ?? defaults.maxErrorConvexRadius,
            bakeSupportAdjacency: settings.bakeSupportAdjacency ?? defaults.bakeSupportAdjacency,
            density: settings.density ?? defaults.density,
            materialId: settings.materialId ?? defaults.materialId,
        });
        if (hullCreate.shape === null) throw new ConvexHullBuildError(hullCreate.result as ConvexHullBuildResult);
        return hullCreate.shape;
    });
}

// a hull's points and faces, as it holds them: the getters read a hull only, a point or face by an index below its count

function dataOf(shape: Shape): Low.ConvexHullData {
    return Low.ConvexHull.data(shape as number as Low.ConvexHull)!;
}

const _face = Low.ConvexHullFace.create();
const _plane: Low.Planef = [0, 0, 0, 0];

/** as built: reduced to fit the hull's size and maxErrorConvexRadius */
export function getConvexRadius(shape: Shape): number {
    return Low.ConvexHullData.convexRadius(dataOf(shape));
}

export function getDensity(shape: Shape): number {
    return Low.ConvexHull.density(shape as number as Low.ConvexHull);
}

/** the hull's vertices, which the faces index */
export function getPointCount(shape: Shape): number {
    return Low.ConvexHullData.pointsLength(dataOf(shape));
}

export function getPoint(out: Vec3, shape: Shape, index: number): Vec3 {
    return Low.ConvexHullData.readPoints(out, dataOf(shape), index);
}

export function getFaceCount(shape: Shape): number {
    return Low.ConvexHullData.facesLength(dataOf(shape));
}

/** at least three */
export function getFaceVertexCount(shape: Shape, face: number): number {
    return Low.ConvexHullData.readFaces(_face, dataOf(shape), face).numVertices;
}

/** the index of the face's vertex's point, its vertices counter clockwise seen from outside */
export function getFaceVertex(shape: Shape, face: number, vertex: number): number {
    const data = dataOf(shape);
    return Low.ConvexHullData.vertexIndicesAt(data, Low.ConvexHullData.readFaces(_face, data, face).firstVertex + vertex);
}

/** out of the hull */
export function getFaceNormal(out: Vec3, shape: Shape, face: number): Vec3 {
    Low.ConvexHullData.readPlanes(_plane, dataOf(shape), face);
    out[0] = _plane[0];
    out[1] = _plane[1];
    out[2] = _plane[2];
    return out;
}
