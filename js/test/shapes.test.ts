import { describe, expect, test } from 'vitest';
import {
    type Box3,
    box,
    CastRayStatus,
    ConvexHullBuildError,
    ConvexHullBuildResult,
    capsule,
    castRay,
    compound,
    computeMassProperties,
    convexHull,
    createClosestCastRayCollector,
    createDefaultCastRaySettings,
    cylinder,
    emptyShape,
    getShapeCenterOfMass,
    getShapeLocalBounds,
    getShapeMaterialId,
    getShapeRefCount,
    getShapeType,
    getShapeVolume,
    init,
    type Listener,
    type MassProperties,
    MotionType,
    offsetCenterOfMass,
    plane,
    releaseShape,
    rigidBody,
    ShapeType,
    scaled,
    sphere,
    staticCompound,
    transformed,
    triangleMesh,
    updateWorld,
    type Vec3,
} from '../src';
import * as Low from '../src/generated/tumble';
import { makeScene, releaseScene, steps } from './helpers';

const bounds = (): Box3 => ({ min: [0, 0, 0], max: [0, 0, 0] });
const CUBE = [-1, -1, -1, 1, -1, -1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1];

describe('primitives', () => {
    test('box: type, bounds, volume and mass at the default density', async () => {
        await init();
        const boxShape = box.create({ halfExtents: [1, 2, 3] });
        expect(getShapeType(boxShape)).toBe(ShapeType.BOX);
        const out = bounds();
        getShapeLocalBounds(out, boxShape);
        expect(out).toEqual({ min: [-1, -2, -3], max: [1, 2, 3] });
        expect(getShapeVolume(boxShape)).toBeCloseTo(48, 4);
        const mass: MassProperties = Low.MassProperties.create();
        computeMassProperties(mass, boxShape);
        expect(mass.mass).toBeCloseTo(48 * 1000, 0);
        releaseShape(boxShape);

        const light = box.create({ halfExtents: [1, 1, 1], density: 10 });
        computeMassProperties(mass, light);
        expect(mass.mass).toBeCloseTo(80, 3);
        releaseShape(light);
    });

    test('sphere, capsule and cylinder volumes', async () => {
        await init();
        const sphereShape = sphere.create({ radius: 2 });
        expect(getShapeVolume(sphereShape)).toBeCloseTo((4 / 3) * Math.PI * 8, 3);
        const capsuleShape = capsule.create({ halfHeightOfCylinder: 1, radius: 0.5 });
        expect(getShapeType(capsuleShape)).toBe(ShapeType.CAPSULE);
        expect(getShapeVolume(capsuleShape)).toBeCloseTo(Math.PI * 0.25 * 2 + (4 / 3) * Math.PI * 0.125, 4);
        const out = bounds();
        getShapeLocalBounds(out, capsuleShape);
        expect(out.max[1]).toBeCloseTo(1.5, 5);
        const cylinderShape = cylinder.create({ halfHeight: 1, radius: 1 });
        expect(getShapeType(cylinderShape)).toBe(ShapeType.CYLINDER);
        expect(getShapeVolume(cylinderShape)).toBeCloseTo(Math.PI * 2, 3);
        releaseShape(sphereShape);
        releaseShape(capsuleShape);
        releaseShape(cylinderShape);
    });

    test('a plane holds a ball up; empty has nothing', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const planeShape = plane.create({ constant: -10 });
        expect(getShapeType(planeShape)).toBe(ShapeType.PLANE);
        const ball = sphere.create({ radius: 0.5 });
        rigidBody.create(world, { shape: planeShape, objectLayer: scene.notMoving, motionType: MotionType.STATIC });
        const body = rigidBody.create(world, {
            shape: ball,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 14, 0],
        });
        // the floor's top is y = 0, the plane at y = 10 is the one it lands on
        steps(world, 120);
        const position: Vec3 = [0, 0, 0];
        rigidBody.getPosition(position, world, body);
        expect(position[1]).toBeCloseTo(10.5, 1);

        const empty = emptyShape.create();
        expect(getShapeType(empty)).toBe(ShapeType.EMPTY);
        expect(getShapeVolume(empty)).toBe(0);
        releaseShape(empty);
        releaseScene(scene);
        releaseShape(ball);
        releaseShape(planeShape);
    });
});

describe('convex hulls', () => {
    test('a hull of a cube is the cube', async () => {
        await init();
        const heavy = convexHull.create({ positions: new Float32Array(CUBE) });
        const light = convexHull.create({ positions: CUBE, density: 1 });
        expect(getShapeType(heavy)).toBe(ShapeType.CONVEX_HULL);
        expect(getShapeVolume(heavy)).toBeCloseTo(8, 3);
        const mass = Low.MassProperties.create();
        computeMassProperties(mass, light);
        expect(mass.mass).toBeCloseTo(8, 3);
        releaseShape(heavy);
        releaseShape(light);
    });

    test('points that make no hull throw why', async () => {
        await init();
        let thrown: unknown = null;
        try {
            convexHull.create({ positions: [0, 0, 0, 1, 0, 0] });
        } catch (error) {
            thrown = error;
        }
        expect(thrown).toBeInstanceOf(ConvexHullBuildError);
        expect((thrown as ConvexHullBuildError).result).toBe(ConvexHullBuildResult.TOO_FEW_POINTS);
        expect((thrown as Error).message).toContain('TOO_FEW_POINTS');
        expect(() => convexHull.create({ positions: [0, 0, 0, 1, 0, 0, 2, 0, 0, 3, 0, 0] })).toThrow(ConvexHullBuildError);
        expect(() => convexHull.create({ positions: [0, 0] })).toThrow('xyz');
    });

    test('a hull frees its built hull with itself', async () => {
        await init();
        const before = Low.allocatedBytes();
        const hull = convexHull.create({ positions: CUBE, convexRadius: 0.1, density: 2 });
        expect(getShapeVolume(hull)).toBeCloseTo(8, 3);
        const mass = Low.MassProperties.create();
        computeMassProperties(mass, hull);
        expect(mass.mass).toBeCloseTo(16, 3);
        releaseShape(hull);
        expect(Low.allocatedBytes()).toBe(before);
        expect(() => convexHull.create({ positions: [0, 0, 0, 1, 0, 0] })).toThrow(ConvexHullBuildError);
        expect(Low.allocatedBytes()).toBe(before);
    });
});

describe('triangle meshes', () => {
    test("a mesh floor holds a box up and reports the hit triangle's material", async () => {
        const scene = await makeScene();
        const { world } = scene;
        // two triangles, a 10 x 10 square at y = 3, material 7 on the first and 8 on the second
        const mesh = triangleMesh.create({
            positions: [-5, 3, -5, 5, 3, -5, 5, 3, 5, -5, 3, 5],
            indices: new Uint32Array([0, 2, 1, 0, 3, 2]),
            materialIds: [7, 8],
        });
        expect(getShapeType(mesh)).toBe(ShapeType.TRIANGLE_MESH);
        const out = bounds();
        getShapeLocalBounds(out, mesh);
        expect(out.min).toEqual([-5, 3, -5]);
        const crate = box.create({ halfExtents: [0.5, 0.5, 0.5] });
        rigidBody.create(world, { shape: mesh, objectLayer: scene.notMoving, motionType: MotionType.STATIC });
        // over the second triangle, the one on the -x +z side of the diagonal
        const body = rigidBody.create(world, {
            shape: crate,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [-3, 4, 3],
        });
        let material = -2;
        const listener: Listener = {
            onContactAdded(contact) {
                material = contact.bodyIdA === body ? contact.materialIdB : contact.materialIdA;
            },
        };
        for (let i = 0; i < 60; i++) updateWorld(world, listener, 1 / 60);
        const position: Vec3 = [0, 0, 0];
        rigidBody.getPosition(position, world, body);
        expect(position[1]).toBeCloseTo(3.48, 1);
        expect(material).toBe(8);

        // a ray down onto the first triangle: its hit names the triangle, whose material the body gives
        const down = createClosestCastRayCollector();
        castRay(world, down, createDefaultCastRaySettings(), [3, 10, -3], [0, -1, 0], 20);
        expect(down.hit.status).toBe(CastRayStatus.COLLIDING);
        expect(rigidBody.getMaterial(world, down.hit.bodyIdB, down.hit.subShapeId)).toBe(7);
        releaseScene(scene);
        releaseShape(crate);
        releaseShape(mesh);
    });

    test('mismatched arrays are refused before the wasm sees them', async () => {
        await init();
        expect(() => triangleMesh.create({ positions: [0, 0, 0, 1, 0, 0, 0, 0, 1], indices: [0, 1] })).toThrow('triples');
        expect(() =>
            triangleMesh.create({ positions: [0, 0, 0, 1, 0, 0, 0, 0, 1], indices: [0, 1, 2], materialIds: [1, 2] }),
        ).toThrow('one material id per triangle');
    });
});

describe('compounds and wrappers', () => {
    test('compound and static compound: bounds, volume and centre of mass from their children', async () => {
        await init();
        const cube = box.create({ halfExtents: [0.5, 0.5, 0.5] });
        const children = [
            { shape: cube, position: [-2, 0, 0] as Vec3 },
            { shape: cube, position: [2, 0, 0] as Vec3, quaternion: [0, 0, 0, 1] as [number, number, number, number] },
        ];
        for (const shape of [compound.create({ children }), staticCompound.create({ children })]) {
            const out = bounds();
            getShapeLocalBounds(out, shape);
            expect(out.min[0]).toBeCloseTo(-2.5, 5);
            expect(out.max[0]).toBeCloseTo(2.5, 5);
            expect(getShapeVolume(shape)).toBeCloseTo(2, 4);
            const center: Vec3 = [1, 1, 1];
            getShapeCenterOfMass(center, shape);
            expect(center[0]).toBeCloseTo(0, 5);
            if (getShapeType(shape) === ShapeType.COMPOUND) releaseShape(shape);
            else {
                expect(getShapeType(shape)).toBe(ShapeType.STATIC_COMPOUND);
                releaseShape(shape);
            }
        }
        expect(() => compound.create({ children: [] })).toThrow('at least one child');
        releaseShape(cube);
    });

    test('transformed moves the bounds, scaled scales the volume, offset moves only the centre of mass', async () => {
        await init();
        const cube = box.create({ halfExtents: [1, 1, 1] });
        const moved = transformed.create({ shape: cube, position: [0, 5, 0] });
        const out = bounds();
        getShapeLocalBounds(out, moved);
        expect(out.min[1]).toBeCloseTo(4, 5);
        const scaledShape = scaled.create({ shape: cube, scale: [2, 2, 2] });
        expect(getShapeVolume(scaledShape)).toBeCloseTo(64, 3);
        const offset = offsetCenterOfMass.create({ shape: cube, offset: [0, -0.5, 0] });
        const center: Vec3 = [0, 0, 0];
        getShapeCenterOfMass(center, offset);
        expect(center[1]).toBeCloseTo(-0.5, 5);
        getShapeLocalBounds(out, offset);
        expect(out.min[1]).toBeCloseTo(-1, 5);
        releaseShape(moved);
        releaseShape(scaledShape);
        releaseShape(offset);
        releaseShape(cube);
    });
});

test('every shape, made and freed, leaves the heap as it was', async () => {
    await init();
    const before = Low.allocatedBytes();
    const cube = box.create({ halfExtents: [1, 1, 1] });
    const shapes = [
        convexHull.create({ positions: CUBE }),
        triangleMesh.create({ positions: [0, 0, 0, 1, 0, 0, 0, 0, 1], indices: [0, 2, 1] }),
        compound.create({ children: [{ shape: cube }] }),
        staticCompound.create({ children: [{ shape: cube }, { shape: cube, position: [3, 0, 0] }] }),
        transformed.create({ shape: cube }),
    ];
    releaseShape(shapes[0]!);
    releaseShape(shapes[1]!);
    releaseShape(shapes[2]!);
    releaseShape(shapes[3]!);
    releaseShape(shapes[4]!);
    releaseShape(cube);
    expect(Low.allocatedBytes()).toBe(before);
});

test('releaseShape frees any shape, and the data of those that own it', async () => {
    await init();
    const before = Low.allocatedBytes();
    const cube = box.create({ halfExtents: [1, 1, 1] });
    const shapes = [
        sphere.create({ radius: 1 }),
        capsule.create({ halfHeightOfCylinder: 1, radius: 0.5 }),
        cylinder.create({ halfHeight: 1, radius: 0.5 }),
        plane.create(),
        emptyShape.create(),
        convexHull.create({ positions: CUBE }),
        triangleMesh.create({ positions: [0, 0, 0, 1, 0, 0, 0, 0, 1], indices: [0, 2, 1] }),
        compound.create({ children: [{ shape: cube }] }),
        staticCompound.create({ children: [{ shape: cube }, { shape: cube, position: [3, 0, 0] }] }),
        transformed.create({ shape: cube }),
        scaled.create({ shape: cube, scale: [2, 2, 2] }),
        offsetCenterOfMass.create({ shape: cube, offset: [0, -1, 0] }),
    ];
    expect(new Set(shapes.map((shape) => getShapeType(shape))).size).toBe(shapes.length);
    for (const shape of shapes) releaseShape(shape);
    releaseShape(cube);
    expect(Low.allocatedBytes()).toBe(before);
});

describe('reference counts', () => {
    test('what holds a shape counts, and the last to let go frees it', async () => {
        const scene = await makeScene();
        const crate = box.create({ halfExtents: [0.5, 0.5, 0.5] });
        const a = rigidBody.create(scene.world, { shape: crate, objectLayer: scene.moving, motionType: MotionType.DYNAMIC });
        rigidBody.create(scene.world, {
            shape: crate,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [3, 0, 0],
        });
        const pair = compound.create({ children: [{ shape: crate }, { shape: crate, position: [2, 0, 0] }] });
        // js's own, two bodies' and the compound's two children
        expect(getShapeRefCount(crate)).toBe(5);

        // a removed body lets go, a released compound lets go of its children, and the world going lets go of the rest
        rigidBody.remove(scene.world, a);
        releaseShape(pair);
        expect(getShapeRefCount(crate)).toBe(2);
        releaseScene(scene);
        expect(getShapeRefCount(crate)).toBe(1);
        releaseShape(crate);
    });
});

describe('changing a shape in place', () => {
    test('update, then setShape with the same shape: the body follows, and a compound holds its new children', async () => {
        const scene = await makeScene();
        const crate = box.create({ halfExtents: [0.5, 0.5, 0.5] });
        const body = rigidBody.create(scene.world, { shape: crate, objectLayer: scene.moving, motionType: MotionType.DYNAMIC });
        box.update(crate, { halfExtents: [1, 2, 1] });
        rigidBody.setShape(scene.world, body, crate, true);
        const out = bounds();
        rigidBody.getBounds(out, scene.world, body);
        expect(out.max[1] - out.min[1]).toBeCloseTo(4, 4);
        expect(getShapeVolume(crate)).toBeCloseTo(16, 3);

        const ball = sphere.create({ radius: 0.5 });
        const pair = compound.create({ children: [{ shape: crate }] });
        compound.update(pair, { children: [{ shape: ball }, { shape: ball, position: [2, 0, 0] }] });
        expect(getShapeRefCount(crate)).toBe(2);
        expect(getShapeRefCount(ball)).toBe(3);
        releaseShape(pair);
        releaseScene(scene);
        releaseShape(crate);
        releaseShape(ball);
    });
});

describe('reading a shape back', () => {
    test('primitives read back what they hold, the convex radius as clamped', async () => {
        await init();
        const crate = box.create({ halfExtents: [1, 2, 0.04], convexRadius: 0.05, density: 500, materialId: 3 });
        expect(box.getHalfExtents([0, 0, 0], crate)).toEqual([1, 2, expect.closeTo(0.04, 6)]);
        expect(box.getConvexRadius(crate)).toBeCloseTo(0.04, 6);
        expect(box.getDensity(crate)).toBe(500);
        expect(getShapeMaterialId(crate)).toBe(3);

        const ball = sphere.create({ radius: 0.75, density: 200 });
        expect(sphere.getRadius(ball)).toBe(0.75);
        expect(sphere.getDensity(ball)).toBe(200);
        expect(getShapeMaterialId(ball)).toBe(-1);

        const pill = capsule.create({ halfHeightOfCylinder: 0.5, radius: 0.25 });
        expect(capsule.getHalfHeightOfCylinder(pill)).toBe(0.5);
        expect(capsule.getRadius(pill)).toBe(0.25);

        const can = cylinder.create({ halfHeight: 0.4, radius: 0.3, convexRadius: 0.02 });
        expect(cylinder.getHalfHeight(can)).toBeCloseTo(0.4, 6);
        expect(cylinder.getRadius(can)).toBeCloseTo(0.3, 6);
        expect(cylinder.getConvexRadius(can)).toBeCloseTo(0.02, 6);

        const ground = plane.create({ normal: [0, 2, 0], constant: 1.5, halfExtent: 40 });
        // normalised as a plane: the same plane, y = -0.75
        expect(plane.getNormal([0, 0, 0], ground)).toEqual([0, 1, 0]);
        expect(plane.getConstant(ground)).toBe(0.75);
        expect(plane.getHalfExtent(ground)).toBe(40);

        for (const shape of [crate, ball, pill, can, ground]) releaseShape(shape);
    });

    test('wrappers and compounds hand back the shapes they hold and where they hold them', async () => {
        await init();
        const crate = box.create({ halfExtents: [0.5, 0.5, 0.5] });
        const ball = sphere.create({ radius: 0.5 });

        const stretched = scaled.create({ shape: crate, scale: [2, 1, 3] });
        expect(scaled.getInnerShape(stretched)).toBe(crate);
        expect(scaled.getScale([0, 0, 0], stretched)).toEqual([2, 1, 3]);

        const turn = Math.SQRT1_2;
        const placed = transformed.create({ shape: ball, position: [1, 2, 3], quaternion: [0, turn, 0, turn] });
        expect(transformed.getInnerShape(placed)).toBe(ball);
        expect(transformed.getPosition([0, 0, 0], placed)).toEqual([1, 2, 3]);
        const quaternion = transformed.getQuaternion([0, 0, 0, 1], placed);
        expect(quaternion[1]).toBeCloseTo(turn, 6);
        expect(quaternion[3]).toBeCloseTo(turn, 6);

        const lowered = offsetCenterOfMass.create({ shape: crate, offset: [0, -0.4, 0] });
        expect(offsetCenterOfMass.getInnerShape(lowered)).toBe(crate);
        expect(offsetCenterOfMass.getOffset([0, 0, 0], lowered)).toEqual([0, expect.closeTo(-0.4, 6), 0]);

        const children = [
            { shape: crate, position: [-1, 0, 0] as Vec3 },
            { shape: ball, position: [1, 0.5, 0] as Vec3, quaternion: [0, 0, turn, turn] as [number, number, number, number] },
        ];
        for (const [module, shape] of [
            [compound, compound.create({ children })],
            [staticCompound, staticCompound.create({ children })],
        ] as const) {
            expect(module.getChildCount(shape)).toBe(2);
            expect(module.getChildShape(shape, 0)).toBe(crate);
            expect(module.getChildShape(shape, 1)).toBe(ball);
            expect(module.getChildPosition([0, 0, 0], shape, 1)).toEqual([1, 0.5, 0]);
            const childQuaternion = module.getChildQuaternion([0, 0, 0, 0], shape, 1);
            expect(childQuaternion[2]).toBeCloseTo(turn, 6);
            expect(childQuaternion[3]).toBeCloseTo(turn, 6);
            expect(module.getChildQuaternion([0, 0, 0, 0], shape, 0)).toEqual([0, 0, 0, 1]);
            releaseShape(shape);
        }

        for (const shape of [stretched, placed, lowered, crate, ball]) releaseShape(shape);
    });

    test('a hull of a cube reads back as six square faces, each wound counter clockwise about its outward normal', async () => {
        await init();
        // the corners and the centre, which the hull drops
        const hull = convexHull.create({ positions: [...CUBE, 0, 0, 0], convexRadius: 0 });
        expect(convexHull.getPointCount(hull)).toBe(8);
        expect(convexHull.getFaceCount(hull)).toBe(6);
        expect(convexHull.getConvexRadius(hull)).toBe(0);

        const normal: Vec3 = [0, 0, 0];
        const a: Vec3 = [0, 0, 0];
        const b: Vec3 = [0, 0, 0];
        const c: Vec3 = [0, 0, 0];
        for (let face = 0; face < 6; face++) {
            expect(convexHull.getFaceVertexCount(hull, face)).toBe(4);
            convexHull.getFaceNormal(normal, hull, face);
            expect(Math.abs(normal[0]) + Math.abs(normal[1]) + Math.abs(normal[2])).toBeCloseTo(1, 6);
            convexHull.getPoint(a, hull, convexHull.getFaceVertex(hull, face, 0));
            convexHull.getPoint(b, hull, convexHull.getFaceVertex(hull, face, 1));
            convexHull.getPoint(c, hull, convexHull.getFaceVertex(hull, face, 2));
            // every vertex on the face's side of the cube
            expect(a[0] * normal[0] + a[1] * normal[1] + a[2] * normal[2]).toBeCloseTo(1, 6);
            const ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
            const ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
            const winding =
                (ab[1]! * ac[2]! - ab[2]! * ac[1]!) * normal[0] +
                (ab[2]! * ac[0]! - ab[0]! * ac[2]!) * normal[1] +
                (ab[0]! * ac[1]! - ab[1]! * ac[0]!) * normal[2];
            expect(winding).toBeGreaterThan(0);
        }
        releaseShape(hull);
    });

    test('a mesh reads back as built: welded, the degenerate triangle dropped, its normals and materials kept', async () => {
        await init();
        // a 2 by 2 quad facing up from four vertices and a copy of one, and a triangle with no area
        const mesh = triangleMesh.create({
            positions: [-1, 0, -1, -1, 0, 1, 1, 0, 1, 1, 0, -1, -1, 0, -1],
            indices: [0, 1, 2, 4, 2, 3, 0, 0, 2],
            materialIds: [7, 8, 9],
        });
        expect(triangleMesh.getVertexCount(mesh)).toBe(4);
        expect(triangleMesh.getTriangleCount(mesh)).toBe(2);

        const triangle = triangleMesh.createTriangle();
        const vertex: Vec3 = [0, 0, 0];
        const materials: number[] = [];
        for (let i = 0; i < 2; i++) {
            triangleMesh.getTriangle(triangle, mesh, i);
            expect(triangle.normal).toEqual([0, 1, 0]);
            for (const index of [triangle.indexA, triangle.indexB, triangle.indexC]) {
                expect(index).toBeLessThan(4);
                triangleMesh.getVertex(vertex, mesh, index);
                expect(vertex[1]).toBe(0);
                expect(Math.abs(vertex[0])).toBe(1);
                expect(Math.abs(vertex[2])).toBe(1);
            }
            materials.push(triangle.materialId);
        }
        expect(materials.sort()).toEqual([7, 8]);
        releaseShape(mesh);
    });
});
