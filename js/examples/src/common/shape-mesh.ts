import { type Quat, quat, type Vec3, vec3 } from 'math';
import {
    box,
    capsule,
    compound,
    convexHull,
    cylinder,
    getShapeType,
    offsetCenterOfMass,
    plane,
    type Shape,
    ShapeType,
    scaled,
    sphere,
    staticCompound,
    transformed,
    triangleMesh,
} from 'tumble.js';

// a shape's surface as triangles in its own space, read from the shape: what a body of it looks like solid. the same
// as the c++ samples' shape_mesh

const SEGMENTS = 24;
const RINGS = 12;

/** three vertices a triangle, each a position and a normal */
export type ShapeTriangles = { positions: number[]; normals: number[] };

/** where a leaf sits in the outermost shape's space: scaled, then rotated, then moved, as the library places children */
type Placement = { position: Vec3; quaternion: Quat; scale: Vec3 };

type Builder = { out: ShapeTriangles; placement: Placement };

const _placed: Vec3 = [0, 0, 0];
const _turned: Vec3 = [0, 0, 0];

function vertex(builder: Builder, point: Vec3, normal: Vec3): void {
    const { position, quaternion, scale } = builder.placement;
    vec3.multiply(_placed, point, scale);
    vec3.transformQuat(_placed, _placed, quaternion);
    vec3.add(_placed, _placed, position);
    // a normal scales by the inverse of the scale
    vec3.set(_turned, normal[0] / scale[0], normal[1] / scale[1], normal[2] / scale[2]);
    vec3.transformQuat(_turned, _turned, quaternion);
    vec3.normalize(_turned, _turned);
    builder.out.positions.push(_placed[0], _placed[1], _placed[2]);
    builder.out.normals.push(_turned[0], _turned[1], _turned[2]);
}

function triangle(builder: Builder, a: Vec3, b: Vec3, c: Vec3, normal: Vec3): void {
    vertex(builder, a, normal);
    vertex(builder, b, normal);
    vertex(builder, c, normal);
}

function quad(builder: Builder, a: Vec3, b: Vec3, c: Vec3, d: Vec3, na: Vec3, nb: Vec3, nc: Vec3, nd: Vec3): void {
    vertex(builder, a, na);
    vertex(builder, b, nb);
    vertex(builder, c, nc);
    vertex(builder, a, na);
    vertex(builder, c, nc);
    vertex(builder, d, nd);
}

function boxSurface(builder: Builder, half: Vec3): void {
    for (let axis = 0; axis < 3; axis++) {
        for (const sign of [-1, 1]) {
            const u = (axis + 1) % 3;
            const v = (axis + 2) % 3;
            const corners: Vec3[] = [];
            for (let k = 0; k < 4; k++) {
                const c: Vec3 = [0, 0, 0];
                c[axis] = sign;
                c[u] = k === 1 || k === 2 ? 1 : -1;
                c[v] = k >= 2 ? 1 : -1;
                if (sign < 0) c[v] = -c[v];
                corners.push([c[0] * half[0], c[1] * half[1], c[2] * half[2]]);
            }
            const normal: Vec3 = [0, 0, 0];
            normal[axis] = sign;
            quad(builder, corners[0]!, corners[1]!, corners[2]!, corners[3]!, normal, normal, normal, normal);
        }
    }
}

function direction(theta: number, phi: number): Vec3 {
    return [Math.sin(theta) * Math.cos(phi), Math.cos(theta), Math.sin(theta) * Math.sin(phi)];
}

/** a sphere split at its equator, its halves moved halfHeight up and down: a capsule, or at 0 a sphere */
function rounded(builder: Builder, radius: number, halfHeight: number): void {
    // pole to pole, the equator twice when the halves part: once for each half, the cylinder between
    const rings: { theta: number; lift: number }[] = [];
    for (let k = 0; k <= RINGS / 2; k++) rings.push({ theta: (Math.PI * k) / RINGS, lift: halfHeight });
    for (let k = halfHeight > 0 ? RINGS / 2 : RINGS / 2 + 1; k <= RINGS; k++)
        rings.push({ theta: (Math.PI * k) / RINGS, lift: -halfHeight });
    for (let r = 0; r + 1 < rings.length; r++) {
        const upper = rings[r]!;
        const lower = rings[r + 1]!;
        for (let segment = 0; segment < SEGMENTS; segment++) {
            const phi0 = (2 * Math.PI * segment) / SEGMENTS;
            const phi1 = (2 * Math.PI * (segment + 1)) / SEGMENTS;
            const n00 = direction(upper.theta, phi0);
            const n01 = direction(upper.theta, phi1);
            const n10 = direction(lower.theta, phi0);
            const n11 = direction(lower.theta, phi1);
            const at = (n: Vec3, lift: number): Vec3 => [n[0] * radius, n[1] * radius + lift, n[2] * radius];
            quad(builder, at(n00, upper.lift), at(n01, upper.lift), at(n11, lower.lift), at(n10, lower.lift), n00, n01, n11, n10);
        }
    }
}

function cylinderSurface(builder: Builder, radius: number, halfHeight: number): void {
    const up: Vec3 = [0, 1, 0];
    const down: Vec3 = [0, -1, 0];
    for (let segment = 0; segment < SEGMENTS; segment++) {
        const phi0 = (2 * Math.PI * segment) / SEGMENTS;
        const phi1 = (2 * Math.PI * (segment + 1)) / SEGMENTS;
        const n0: Vec3 = [Math.cos(phi0), 0, Math.sin(phi0)];
        const n1: Vec3 = [Math.cos(phi1), 0, Math.sin(phi1)];
        const b0: Vec3 = [n0[0] * radius, -halfHeight, n0[2] * radius];
        const b1: Vec3 = [n1[0] * radius, -halfHeight, n1[2] * radius];
        const t0: Vec3 = [b0[0], halfHeight, b0[2]];
        const t1: Vec3 = [b1[0], halfHeight, b1[2]];
        quad(builder, b0, t0, t1, b1, n0, n0, n1, n1);
        triangle(builder, [0, halfHeight, 0], t1, t0, up);
        triangle(builder, [0, -halfHeight, 0], b0, b1, down);
    }
}

function hullSurface(builder: Builder, hull: Shape): void {
    const normal: Vec3 = [0, 0, 0];
    const first: Vec3 = [0, 0, 0];
    const b: Vec3 = [0, 0, 0];
    const c: Vec3 = [0, 0, 0];
    for (let face = 0; face < convexHull.getFaceCount(hull); face++) {
        convexHull.getFaceNormal(normal, hull, face);
        convexHull.getPoint(first, hull, convexHull.getFaceVertex(hull, face, 0));
        // a fan from the face's first vertex
        for (let k = 1; k + 1 < convexHull.getFaceVertexCount(hull, face); k++) {
            convexHull.getPoint(b, hull, convexHull.getFaceVertex(hull, face, k));
            convexHull.getPoint(c, hull, convexHull.getFaceVertex(hull, face, k + 1));
            triangle(builder, first, b, c, normal);
        }
    }
}

function meshSurface(builder: Builder, mesh: Shape): void {
    const built = triangleMesh.createTriangle();
    const a: Vec3 = [0, 0, 0];
    const b: Vec3 = [0, 0, 0];
    const c: Vec3 = [0, 0, 0];
    for (let i = 0; i < triangleMesh.getTriangleCount(mesh); i++) {
        triangleMesh.getTriangle(built, mesh, i);
        triangleMesh.getVertex(a, mesh, built.indexA);
        triangleMesh.getVertex(b, mesh, built.indexB);
        triangleMesh.getVertex(c, mesh, built.indexC);
        triangle(builder, a, b, c, built.normal);
    }
}

/** the shortest turn from +y onto a unit direction */
function turnFromUp(out: Quat, to: Vec3): Quat {
    // half way between up and to, as the quaternion (up x to, 1 + up . to) normalised; straight down turns about x
    if (to[1] < -0.999999) return quat.set(out, 1, 0, 0, 0);
    quat.set(out, to[2], 0, -to[0], 1 + to[1]);
    return quat.normalize(out, out);
}

function planeSurface(builder: Builder, shape: Shape): void {
    const normal = plane.getNormal([0, 0, 0], shape);
    const center = vec3.scale([0, 0, 0], normal, -plane.getConstant(shape));
    const turn = turnFromUp([0, 0, 0, 1], normal);
    const e = plane.getHalfExtent(shape);
    const u = vec3.transformQuat([0, 0, 0], [e, 0, 0], turn);
    const v = vec3.transformQuat([0, 0, 0], [0, 0, e], turn);
    const corner = (su: number, sv: number): Vec3 => [
        center[0] + su * u[0] + sv * v[0],
        center[1] + su * u[1] + sv * v[1],
        center[2] + su * u[2] + sv * v[2],
    ];
    quad(builder, corner(-1, -1), corner(-1, 1), corner(1, 1), corner(1, -1), normal, normal, normal, normal);
}

/** the inner shape placed by a local position and rotation, the scale passed on as it is */
function child(builder: Builder, inner: Shape, position: Vec3, quaternion: Quat): void {
    const outer = builder.placement;
    const moved = vec3.multiply([0, 0, 0], position, outer.scale);
    vec3.transformQuat(moved, moved, outer.quaternion);
    vec3.add(moved, moved, outer.position);
    builder.placement = {
        position: moved,
        quaternion: quat.multiply([0, 0, 0, 1], outer.quaternion, quaternion),
        scale: outer.scale,
    };
    surface(builder, inner);
    builder.placement = outer;
}

function surface(builder: Builder, shape: Shape): void {
    switch (getShapeType(shape)) {
        case ShapeType.SPHERE:
            rounded(builder, sphere.getRadius(shape), 0);
            break;
        case ShapeType.BOX:
            boxSurface(builder, box.getHalfExtents([0, 0, 0], shape));
            break;
        case ShapeType.CAPSULE:
            rounded(builder, capsule.getRadius(shape), capsule.getHalfHeightOfCylinder(shape));
            break;
        case ShapeType.CYLINDER:
            cylinderSurface(builder, cylinder.getRadius(shape), cylinder.getHalfHeight(shape));
            break;
        case ShapeType.CONVEX_HULL:
            hullSurface(builder, shape);
            break;
        case ShapeType.TRIANGLE_MESH:
            meshSurface(builder, shape);
            break;
        case ShapeType.PLANE:
            planeSurface(builder, shape);
            break;
        case ShapeType.COMPOUND:
            for (let i = 0; i < compound.getChildCount(shape); i++) {
                const at = compound.getChildPosition([0, 0, 0], shape, i);
                child(builder, compound.getChildShape(shape, i), at, compound.getChildQuaternion([0, 0, 0, 1], shape, i));
            }
            break;
        case ShapeType.STATIC_COMPOUND:
            for (let i = 0; i < staticCompound.getChildCount(shape); i++) {
                const at = staticCompound.getChildPosition([0, 0, 0], shape, i);
                child(
                    builder,
                    staticCompound.getChildShape(shape, i),
                    at,
                    staticCompound.getChildQuaternion([0, 0, 0, 1], shape, i),
                );
            }
            break;
        case ShapeType.TRANSFORMED:
            child(
                builder,
                transformed.getInnerShape(shape),
                transformed.getPosition([0, 0, 0], shape),
                transformed.getQuaternion([0, 0, 0, 1], shape),
            );
            break;
        case ShapeType.SCALED: {
            const outer = builder.placement;
            const inner = vec3.multiply([0, 0, 0], outer.scale, scaled.getScale([0, 0, 0], shape));
            builder.placement = { ...outer, scale: inner };
            surface(builder, scaled.getInnerShape(shape));
            builder.placement = outer;
            break;
        }
        case ShapeType.OFFSET_CENTER_OF_MASS:
            surface(builder, offsetCenterOfMass.getInnerShape(shape));
            break;
        default:
            // empty has no surface
            break;
    }
}

export function buildShapeTriangles(shape: Shape): ShapeTriangles {
    const builder: Builder = {
        out: { positions: [], normals: [] },
        placement: { position: [0, 0, 0], quaternion: [0, 0, 0, 1], scale: [1, 1, 1] },
    };
    surface(builder, shape);
    return builder.out;
}
