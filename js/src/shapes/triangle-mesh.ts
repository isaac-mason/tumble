import { withCopies } from '../arrays.js';
import * as Low from '../generated/tumble.js';
import type { Vec3 } from '../math.js';
import { defaultsOf } from '../settings.js';
import type { BvhSplitStrategy, Shape } from './shapes.js';

/**
 * the triangles to build the mesh from, and how. for static bodies, or kinematic ones: a mesh has no volume, so a
 * dynamic body over one needs massPropertiesOverride
 */
export type TriangleMeshShapeSettings = {
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

const meshDefaults = defaultsOf(Low.TriangleMesh.defaultSettings, Low.TriangleMeshSettings.create);

/** a shape over the mesh of the triangles, which it builds */
export function create(settings: TriangleMeshShapeSettings): Shape {
    const { positions, indices, materialIds } = settings;
    if (positions.length % 3 !== 0) throw new Error('tumble: mesh positions are xyz triples');
    if (indices.length % 3 !== 0) throw new Error('tumble: mesh indices are triples');
    if (materialIds !== undefined && materialIds.length !== indices.length / 3) {
        throw new Error('tumble: one material id per triangle');
    }
    const defaults = meshDefaults();
    return withCopies((copy) => {
        Low.errorClear();
        return Low.TriangleMesh.create({
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
}

// a mesh as it was built: coincident vertices welded, degenerate and duplicate triangles dropped, the triangles in the
// tree's order. the getters read a mesh only, a vertex or triangle by an index below its count

/** a built triangle: its vertices' indices, counter clockwise seen from the front, and its front's normal */
export type TriangleMeshTriangle = Low.TriangleMeshTriangle;

export function createTriangle(): TriangleMeshTriangle {
    return Low.TriangleMeshTriangle.create();
}

function dataOf(shape: Shape): Low.TriangleMeshData {
    return Low.TriangleMesh.data(shape as number as Low.TriangleMesh)!;
}

export function getVertexCount(shape: Shape): number {
    return Low.TriangleMeshData.positionsLength(dataOf(shape));
}

export function getVertex(out: Vec3, shape: Shape, index: number): Vec3 {
    return Low.TriangleMeshData.readPositions(out, dataOf(shape), index);
}

export function getTriangleCount(shape: Shape): number {
    return Low.TriangleMeshData.trianglesLength(dataOf(shape));
}

export function getTriangle(out: TriangleMeshTriangle, shape: Shape, index: number): TriangleMeshTriangle {
    return Low.TriangleMeshData.readTriangles(out, dataOf(shape), index);
}
