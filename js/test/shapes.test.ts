import { describe, expect, test } from 'vitest';
import {
    Box,
    type Box3,
    Capsule,
    Compound,
    ConvexHull,
    ConvexHullBuildError,
    ConvexHullBuildResult,
    ConvexHullData,
    Cylinder,
    Empty,
    init,
    type Listener,
    type MassProperties,
    MotionType,
    OffsetCenterOfMass,
    Plane,
    RigidBody,
    Scaled,
    Shape,
    ShapeType,
    Sphere,
    StaticCompound,
    Transformed,
    TriangleMesh,
    TriangleMeshData,
    type Vec3,
    World,
} from '../src';
import * as Low from '../src/generated/tumble';
import { makeScene, releaseScene, steps } from './helpers';

const bounds = (): Box3 => ({ min: [0, 0, 0], max: [0, 0, 0] });
const CUBE = [-1, -1, -1, 1, -1, -1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1];

describe('primitives', () => {
    test('box: type, bounds, volume and mass at the default density', async () => {
        await init();
        const box = Box.create({ halfExtents: [1, 2, 3] });
        expect(Shape.getType(box)).toBe(ShapeType.BOX);
        const out = bounds();
        Shape.getLocalBounds(out, box);
        expect(out).toEqual({ min: [-1, -2, -3], max: [1, 2, 3] });
        expect(Shape.getVolume(box)).toBeCloseTo(48, 4);
        const mass: MassProperties = Low.MassProperties.create();
        Shape.getMassProperties(mass, box);
        expect(mass.mass).toBeCloseTo(48 * 1000, 0);
        Box.destroy(box);

        const light = Box.create({ halfExtents: [1, 1, 1], density: 10 });
        Shape.getMassProperties(mass, light);
        expect(mass.mass).toBeCloseTo(80, 3);
        Box.destroy(light);
    });

    test('sphere, capsule and cylinder volumes', async () => {
        await init();
        const sphere = Sphere.create({ radius: 2 });
        expect(Shape.getVolume(sphere)).toBeCloseTo((4 / 3) * Math.PI * 8, 3);
        const capsule = Capsule.create({ halfHeightOfCylinder: 1, radius: 0.5 });
        expect(Shape.getType(capsule)).toBe(ShapeType.CAPSULE);
        expect(Shape.getVolume(capsule)).toBeCloseTo(Math.PI * 0.25 * 2 + (4 / 3) * Math.PI * 0.125, 4);
        const out = bounds();
        Shape.getLocalBounds(out, capsule);
        expect(out.max[1]).toBeCloseTo(1.5, 5);
        const cylinder = Cylinder.create({ halfHeight: 1, radius: 1 });
        expect(Shape.getType(cylinder)).toBe(ShapeType.CYLINDER);
        expect(Shape.getVolume(cylinder)).toBeCloseTo(Math.PI * 2, 3);
        Sphere.destroy(sphere);
        Capsule.destroy(capsule);
        Cylinder.destroy(cylinder);
    });

    test('a plane holds a ball up; empty has nothing', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const plane = Plane.create({ constant: -10 });
        expect(Shape.getType(plane)).toBe(ShapeType.PLANE);
        const ball = Sphere.create({ radius: 0.5 });
        RigidBody.create(world, { shape: plane, objectLayer: scene.notMoving, motionType: MotionType.STATIC });
        const body = RigidBody.create(world, {
            shape: ball,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 14, 0],
        });
        // the floor's top is y = 0, the plane at y = 10 is the one it lands on
        steps(world, 120);
        const position: Vec3 = [0, 0, 0];
        RigidBody.getPosition(position, world, body);
        expect(position[1]).toBeCloseTo(10.5, 1);

        const empty = Empty.create();
        expect(Shape.getType(empty)).toBe(ShapeType.EMPTY);
        expect(Shape.getVolume(empty)).toBe(0);
        Empty.destroy(empty);
        releaseScene(scene);
        Sphere.destroy(ball);
        Plane.destroy(plane);
    });
});

describe('convex hulls', () => {
    test('a hull of a cube is the cube, and shapes share its data', async () => {
        await init();
        const data = ConvexHullData.build({ positions: new Float32Array(CUBE) });
        const heavy = ConvexHull.create({ data });
        const light = ConvexHull.create({ data, density: 1 });
        expect(Shape.getType(heavy)).toBe(ShapeType.CONVEX_HULL);
        expect(Shape.getVolume(heavy)).toBeCloseTo(8, 3);
        const mass = Low.MassProperties.create();
        Shape.getMassProperties(mass, light);
        expect(mass.mass).toBeCloseTo(8, 3);
        ConvexHull.destroy(heavy);
        ConvexHull.destroy(light);
        ConvexHullData.release(data);
    });

    test('points that make no hull throw why', async () => {
        await init();
        let thrown: unknown = null;
        try {
            ConvexHullData.build({ positions: [0, 0, 0, 1, 0, 0] });
        } catch (error) {
            thrown = error;
        }
        expect(thrown).toBeInstanceOf(ConvexHullBuildError);
        expect((thrown as ConvexHullBuildError).result).toBe(ConvexHullBuildResult.TOO_FEW_POINTS);
        expect((thrown as Error).message).toContain('TOO_FEW_POINTS');
        expect(() => ConvexHullData.build({ positions: [0, 0, 0, 1, 0, 0, 2, 0, 0, 3, 0, 0] })).toThrow(ConvexHullBuildError);
        expect(() => ConvexHullData.build({ positions: [0, 0] })).toThrow('xyz');
    });
});

describe('triangle meshes', () => {
    test("a mesh floor holds a box up and reports the hit triangle's material", async () => {
        const scene = await makeScene();
        const { world } = scene;
        // two triangles, a 10 x 10 square at y = 3, material 7 on the first and 8 on the second
        const data = TriangleMeshData.build({
            positions: [-5, 3, -5, 5, 3, -5, 5, 3, 5, -5, 3, 5],
            indices: new Uint32Array([0, 2, 1, 0, 3, 2]),
            materialIds: [7, 8],
        });
        const mesh = TriangleMesh.create({ data });
        expect(Shape.getType(mesh)).toBe(ShapeType.TRIANGLE_MESH);
        const out = bounds();
        Shape.getLocalBounds(out, mesh);
        expect(out.min).toEqual([-5, 3, -5]);
        const crate = Box.create({ halfExtents: [0.5, 0.5, 0.5] });
        RigidBody.create(world, { shape: mesh, objectLayer: scene.notMoving, motionType: MotionType.STATIC });
        // over the second triangle, the one on the -x +z side of the diagonal
        const body = RigidBody.create(world, {
            shape: crate,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [-3, 4, 3],
        });
        let material = -2;
        const listener: Listener = {
            onContactAdded(contact) {
                material = contact.bodyA === body ? contact.materialB : contact.materialA;
            },
        };
        for (let i = 0; i < 60; i++) World.step(world, listener, 1 / 60);
        const position: Vec3 = [0, 0, 0];
        RigidBody.getPosition(position, world, body);
        expect(position[1]).toBeCloseTo(3.48, 1);
        expect(material).toBe(8);
        releaseScene(scene);
        Box.destroy(crate);
        TriangleMesh.destroy(mesh);
        TriangleMeshData.release(data);
    });

    test('mismatched arrays are refused before the wasm sees them', async () => {
        await init();
        expect(() => TriangleMeshData.build({ positions: [0, 0, 0, 1, 0, 0, 0, 0, 1], indices: [0, 1] })).toThrow('triples');
        expect(() =>
            TriangleMeshData.build({ positions: [0, 0, 0, 1, 0, 0, 0, 0, 1], indices: [0, 1, 2], materialIds: [1, 2] }),
        ).toThrow('one material id per triangle');
    });
});

describe('compounds and wrappers', () => {
    test('compound and static compound: bounds, volume and centre of mass from their children', async () => {
        await init();
        const cube = Box.create({ halfExtents: [0.5, 0.5, 0.5] });
        const children = [
            { shape: cube, position: [-2, 0, 0] as Vec3 },
            { shape: cube, position: [2, 0, 0] as Vec3, quaternion: [0, 0, 0, 1] as [number, number, number, number] },
        ];
        for (const shape of [Compound.create({ children }), StaticCompound.create({ children })]) {
            const out = bounds();
            Shape.getLocalBounds(out, shape);
            expect(out.min[0]).toBeCloseTo(-2.5, 5);
            expect(out.max[0]).toBeCloseTo(2.5, 5);
            expect(Shape.getVolume(shape)).toBeCloseTo(2, 4);
            const center: Vec3 = [1, 1, 1];
            Shape.getCenterOfMass(center, shape);
            expect(center[0]).toBeCloseTo(0, 5);
            if (Shape.getType(shape) === ShapeType.COMPOUND) Compound.destroy(shape);
            else {
                expect(Shape.getType(shape)).toBe(ShapeType.STATIC_COMPOUND);
                StaticCompound.destroy(shape);
            }
        }
        expect(() => Compound.create({ children: [] })).toThrow('at least one child');
        Box.destroy(cube);
    });

    test('transformed moves the bounds, scaled scales the volume, offset moves only the centre of mass', async () => {
        await init();
        const cube = Box.create({ halfExtents: [1, 1, 1] });
        const moved = Transformed.create({ shape: cube, position: [0, 5, 0] });
        const out = bounds();
        Shape.getLocalBounds(out, moved);
        expect(out.min[1]).toBeCloseTo(4, 5);
        const scaled = Scaled.create({ shape: cube, scale: [2, 2, 2] });
        expect(Shape.getVolume(scaled)).toBeCloseTo(64, 3);
        const offset = OffsetCenterOfMass.create({ shape: cube, offset: [0, -0.5, 0] });
        const center: Vec3 = [0, 0, 0];
        Shape.getCenterOfMass(center, offset);
        expect(center[1]).toBeCloseTo(-0.5, 5);
        Shape.getLocalBounds(out, offset);
        expect(out.min[1]).toBeCloseTo(-1, 5);
        Transformed.destroy(moved);
        Scaled.destroy(scaled);
        OffsetCenterOfMass.destroy(offset);
        Box.destroy(cube);
    });
});

test('every shape and its data, made and freed, leaves the heap as it was', async () => {
    await init();
    const before = Low.allocatedBytes();
    const cube = Box.create({ halfExtents: [1, 1, 1] });
    const hull = ConvexHullData.build({ positions: CUBE });
    const mesh = TriangleMeshData.build({ positions: [0, 0, 0, 1, 0, 0, 0, 0, 1], indices: [0, 2, 1] });
    const shapes = [
        ConvexHull.create({ data: hull }),
        TriangleMesh.create({ data: mesh }),
        Compound.create({ children: [{ shape: cube }] }),
        StaticCompound.create({ children: [{ shape: cube }, { shape: cube, position: [3, 0, 0] }] }),
        Transformed.create({ shape: cube }),
    ];
    ConvexHull.destroy(shapes[0]!);
    TriangleMesh.destroy(shapes[1]!);
    Compound.destroy(shapes[2]!);
    StaticCompound.destroy(shapes[3]!);
    Transformed.destroy(shapes[4]!);
    ConvexHullData.release(hull);
    TriangleMeshData.release(mesh);
    Box.destroy(cube);
    expect(Low.allocatedBytes()).toBe(before);
});
