import { withCopies } from '../arrays.js';
import * as Low from '../generated/tumble.js';
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
