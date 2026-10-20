import { describe, expect, test } from 'vitest';
import {
    Box,
    CastRayStatus,
    CastShapeStatus,
    castRay,
    castShape,
    collidePoint,
    collideShape,
    collideShapeWithInternalEdgeRemoval,
    createAllCastRayCollector,
    createAllCollidePointCollector,
    createAllCollideShapeCollector,
    createAnyCastRayCollector,
    createAnyCollidePointCollector,
    createClosestCastRayCollector,
    createClosestCastShapeCollector,
    createClosestCollideShapeCollector,
    createDefaultCastRaySettings,
    createDefaultCastShapeSettings,
    createDefaultCollidePointSettings,
    createDefaultCollideShapeSettings,
    Filter,
    MotionType,
    type Quat,
    RigidBody,
    Sphere,
    type Vec3,
    World,
} from '../src';
import * as Low from '../src/generated/tumble';
import { makeScene, releaseScene, type Scene } from './helpers';

const IDENTITY: Quat = [0, 0, 0, 1];
const ONE: Vec3 = [1, 1, 1];
const DOWN: Vec3 = [0, -1, 0];

type Stack = Scene & { crate: ReturnType<typeof Box.create>; lower: number; upper: number };

/** no gravity; box `lower` spans y 0.5 to 1.5 and `upper` 2.5 to 3.5, above the floor's top at 0 */
async function makeStack(): Promise<Stack> {
    const scene = await makeScene((settings) => {
        settings.gravity = [0, 0, 0];
    });
    const crate = Box.create({ halfExtents: [0.5, 0.5, 0.5] });
    const lower = RigidBody.create(scene.world, {
        shape: crate,
        objectLayer: scene.moving,
        motionType: MotionType.DYNAMIC,
        position: [0, 1, 0],
    });
    const upper = RigidBody.create(scene.world, {
        shape: crate,
        objectLayer: scene.moving,
        motionType: MotionType.DYNAMIC,
        position: [0, 3, 0],
        collisionGroups: 2,
        collisionMask: 5,
    });
    World.step(scene.world, undefined, 1 / 60);
    return { ...scene, crate, lower, upper };
}

function releaseStack(stack: Stack): void {
    releaseScene(stack);
    Box.destroy(stack.crate);
}

describe('queries', () => {
    test('a ray down the stack: the closest, any and every hit, closest first', async () => {
        const stack = await makeStack();
        const { world } = stack;
        const filter = Filter.create(world);
        const settings = createDefaultCastRaySettings();

        const closest = createClosestCastRayCollector();
        castRay(world, closest, settings, [0, 10, 0], DOWN, 20, filter);
        expect(closest.hit.status).toBe(CastRayStatus.COLLIDING);
        expect(closest.hit.bodyIdB).toBe(stack.upper);
        expect(closest.hit.fraction).toBeCloseTo(6.5 / 20, 3);

        const any = createAnyCastRayCollector();
        castRay(world, any, settings, [0, 10, 0], DOWN, 20, filter);
        expect(any.hit.status).toBe(CastRayStatus.COLLIDING);

        const all = createAllCastRayCollector();
        castRay(world, all, settings, [0, 10, 0], DOWN, 20, filter);
        all.sort();
        expect(all.hits.map((hit) => hit.fraction)).toEqual([
            expect.closeTo(6.5 / 20, 3),
            expect.closeTo(8.5 / 20, 3),
            expect.closeTo(10 / 20, 3),
        ]);
        expect(all.hits[1]!.bodyIdB).toBe(stack.lower);

        // a collector keeps what it had: the same ray again, the boxes filtered out, hits only the floor further along
        const floorOnly = Filter.create(world);
        Filter.disableObjectLayer(floorOnly, world, stack.moving);
        castRay(world, closest, settings, [0, 10, 0], DOWN, 20, floorOnly);
        expect(closest.hit.bodyIdB).toBe(stack.upper);
        Filter.destroy(floorOnly);
        closest.reset();
        expect(closest.hit.status).toBe(CastRayStatus.NOT_COLLIDING);
        castRay(world, closest, settings, [0, 2, 0], DOWN, 20, filter);
        expect(closest.hit.bodyIdB).toBe(stack.lower);
        expect(closest.hit.fraction).toBeCloseTo(0.5 / 20, 3);

        // a ray that misses
        closest.reset();
        castRay(world, closest, settings, [10, 10, 10], [0, 1, 0], 5, filter);
        expect(closest.hit.status).toBe(CastRayStatus.NOT_COLLIDING);
        Filter.destroy(filter);
        releaseStack(stack);
    });

    test('a filter passes over layers, groups, and what its body filter rejects', async () => {
        const stack = await makeStack();
        const { world } = stack;
        const filter = Filter.create(world);
        const settings = createDefaultCastRaySettings();
        const closest = createClosestCastRayCollector();

        expect(Filter.disableObjectLayer(filter, world, stack.moving)).toBe(true);
        expect(Filter.filterObjectLayer(filter, stack.moving)).toBe(false);
        castRay(world, closest, settings, [0, 10, 0], DOWN, 20, filter);
        expect(closest.hit.fraction).toBeCloseTo(0.5, 3);
        expect(Filter.enableObjectLayer(filter, world, 99)).toBe(false);

        // the upper box is in group 2 only: a mask without it passes over it
        Filter.setAllEnabled(filter, world);
        filter.collisionMask = ~2 >>> 0;
        closest.reset();
        castRay(world, closest, settings, [0, 10, 0], DOWN, 20, filter);
        expect(closest.hit.bodyIdB).toBe(stack.lower);

        // a body filter: the closest it passes, not the closest there is
        Filter.setAllEnabled(filter, world);
        let asked = 0;
        filter.bodyFilter = (body) => {
            asked++;
            return body !== stack.upper;
        };
        closest.reset();
        castRay(world, closest, settings, [0, 10, 0], DOWN, 20, filter);
        expect(asked).toBeGreaterThan(0);
        expect(closest.hit.bodyIdB).toBe(stack.lower);
        expect(closest.hit.fraction).toBeCloseTo(8.5 / 20, 3);

        // what the upper box collides with: its layer's layers, its groups and mask
        expect(Filter.setFromBody(filter, world, stack.upper)).toBe(true);
        expect(filter.collisionGroups).toBe(2);
        expect(filter.collisionMask).toBe(5);
        const copy = Filter.create(world);
        Filter.copy(copy, filter);
        expect(copy.collisionMask).toBe(5);
        expect(copy.bodyFilter).toBe(filter.bodyFilter);
        Filter.destroy(copy);
        Filter.destroy(filter);
        releaseStack(stack);
    });

    test('a sphere swept down stops on the upper box, the normal back up', async () => {
        const stack = await makeStack();
        const { world } = stack;
        const filter = Filter.create(world);
        const ball = Sphere.create({ radius: 0.5 });
        const closest = createClosestCastShapeCollector();
        castShape(world, closest, createDefaultCastShapeSettings(), ball, [0, 10, 0], IDENTITY, ONE, [0, -20, 0], filter);
        expect(closest.hit.status).toBe(CastShapeStatus.COLLIDING);
        expect(closest.hit.bodyIdB).toBe(stack.upper);
        // its centre 0.5 above the box's top at 3.5
        expect(closest.hit.fraction).toBeCloseTo(6 / 20, 2);
        expect(closest.hit.normal[1]).toBeCloseTo(1, 2);
        expect(closest.hit.pointB[1]).toBeCloseTo(3.5, 1);
        Sphere.destroy(ball);
        Filter.destroy(filter);
        releaseStack(stack);
    });

    test('a box overlapping both: every overlap, and the deepest', async () => {
        const stack = await makeStack();
        const { world } = stack;
        const filter = Filter.create(world);
        const probe = Box.create({ halfExtents: [0.4, 0.7, 0.4] });
        const settings = createDefaultCollideShapeSettings();
        // y 1.4 to 2.8: into the lower box by 0.1, the upper by 0.3
        const all = createAllCollideShapeCollector();
        collideShape(world, all, settings, probe, [0, 2.1, 0], IDENTITY, ONE, filter);
        expect(all.hits.map((hit) => hit.bodyIdB).sort()).toEqual([stack.lower, stack.upper].sort());
        const deepest = createClosestCollideShapeCollector();
        collideShape(world, deepest, settings, probe, [0, 2.1, 0], IDENTITY, ONE, filter);
        expect(deepest.hit?.bodyIdB).toBe(stack.upper);
        expect(deepest.hit?.penetration).toBeCloseTo(0.3, 2);
        deepest.reset();
        expect(deepest.hit).toBeNull();
        collideShapeWithInternalEdgeRemoval(world, deepest, settings, probe, [0, 2.1, 0], IDENTITY, ONE, filter);
        expect(deepest.hit?.bodyIdB).toBe(stack.upper);
        Box.destroy(probe);
        Filter.destroy(filter);
        releaseStack(stack);
    });

    test('a point in the lower box is in it alone', async () => {
        const stack = await makeStack();
        const { world } = stack;
        const filter = Filter.create(world);
        const settings = createDefaultCollidePointSettings();
        const any = createAnyCollidePointCollector();
        collidePoint(world, any, settings, [0, 1, 0], filter);
        expect(any.hit?.bodyIdB).toBe(stack.lower);
        const all = createAllCollidePointCollector();
        collidePoint(world, all, settings, [0, 2, 0], filter);
        expect(all.hits.length).toBe(0);
        Filter.destroy(filter);
        releaseStack(stack);
    });

    test('filters, made and freed, leave the heap as it was', async () => {
        const stack = await makeStack();
        const before = Low.allocatedBytes();
        const a = Filter.create(stack.world);
        const b = Filter.create(stack.world);
        Filter.disableAllLayers(a, stack.world);
        Filter.copy(b, a);
        Filter.destroy(a);
        Filter.destroy(b);
        expect(Low.allocatedBytes()).toBe(before);
        releaseStack(stack);
    });
});
