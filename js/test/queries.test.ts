import { describe, expect, test } from 'vitest';
import {
    box,
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
    filter,
    MotionType,
    type Quat,
    releaseShape,
    rigidBody,
    sphere,
    updateWorld,
    type Vec3,
} from '../src';
import * as Low from '../src/generated/tumble';
import { makeScene, releaseScene, type Scene } from './helpers';

const IDENTITY: Quat = [0, 0, 0, 1];
const ONE: Vec3 = [1, 1, 1];
const DOWN: Vec3 = [0, -1, 0];

type Stack = Scene & { crate: ReturnType<typeof box.create>; lower: number; upper: number };

/** no gravity; box `lower` spans y 0.5 to 1.5 and `upper` 2.5 to 3.5, above the floor's top at 0 */
async function makeStack(): Promise<Stack> {
    const scene = await makeScene((settings) => {
        settings.gravity = [0, 0, 0];
    });
    const crate = box.create({ halfExtents: [0.5, 0.5, 0.5] });
    const lower = rigidBody.create(scene.world, {
        shape: crate,
        objectLayer: scene.moving,
        motionType: MotionType.DYNAMIC,
        position: [0, 1, 0],
    });
    const upper = rigidBody.create(scene.world, {
        shape: crate,
        objectLayer: scene.moving,
        motionType: MotionType.DYNAMIC,
        position: [0, 3, 0],
        collisionGroups: 2,
        collisionMask: 5,
    });
    updateWorld(scene.world, undefined, 1 / 60);
    return { ...scene, crate, lower, upper };
}

function releaseStack(stack: Stack): void {
    releaseScene(stack);
    releaseShape(stack.crate);
}

describe('queries', () => {
    test('a ray down the stack: the closest, any and every hit, closest first', async () => {
        const stack = await makeStack();
        const { world } = stack;
        const queryFilter = filter.create(world);
        const settings = createDefaultCastRaySettings();

        const closest = createClosestCastRayCollector();
        castRay(world, closest, settings, [0, 10, 0], DOWN, 20, queryFilter);
        expect(closest.hit.status).toBe(CastRayStatus.COLLIDING);
        expect(closest.hit.bodyIdB).toBe(stack.upper);
        expect(closest.hit.fraction).toBeCloseTo(6.5 / 20, 3);

        const any = createAnyCastRayCollector();
        castRay(world, any, settings, [0, 10, 0], DOWN, 20, queryFilter);
        expect(any.hit.status).toBe(CastRayStatus.COLLIDING);

        const all = createAllCastRayCollector();
        castRay(world, all, settings, [0, 10, 0], DOWN, 20, queryFilter);
        all.sort();
        expect(all.hits.map((hit) => hit.fraction)).toEqual([
            expect.closeTo(6.5 / 20, 3),
            expect.closeTo(8.5 / 20, 3),
            expect.closeTo(10 / 20, 3),
        ]);
        expect(all.hits[1]!.bodyIdB).toBe(stack.lower);

        // a collector keeps what it had: the same ray again, the boxes filtered out, hits only the floor further along
        const floorOnly = filter.create(world);
        filter.disableObjectLayer(floorOnly, world, stack.moving);
        castRay(world, closest, settings, [0, 10, 0], DOWN, 20, floorOnly);
        expect(closest.hit.bodyIdB).toBe(stack.upper);
        filter.destroy(floorOnly);
        closest.reset();
        expect(closest.hit.status).toBe(CastRayStatus.NOT_COLLIDING);
        castRay(world, closest, settings, [0, 2, 0], DOWN, 20, queryFilter);
        expect(closest.hit.bodyIdB).toBe(stack.lower);
        expect(closest.hit.fraction).toBeCloseTo(0.5 / 20, 3);

        // a ray that misses
        closest.reset();
        castRay(world, closest, settings, [10, 10, 10], [0, 1, 0], 5, queryFilter);
        expect(closest.hit.status).toBe(CastRayStatus.NOT_COLLIDING);
        filter.destroy(queryFilter);
        releaseStack(stack);
    });

    test('a filter passes over layers, groups, and what its body filter rejects', async () => {
        const stack = await makeStack();
        const { world } = stack;
        const queryFilter = filter.create(world);
        const settings = createDefaultCastRaySettings();
        const closest = createClosestCastRayCollector();

        expect(filter.disableObjectLayer(queryFilter, world, stack.moving)).toBe(true);
        expect(filter.filterObjectLayer(queryFilter, stack.moving)).toBe(false);
        castRay(world, closest, settings, [0, 10, 0], DOWN, 20, queryFilter);
        expect(closest.hit.fraction).toBeCloseTo(0.5, 3);
        expect(filter.enableObjectLayer(queryFilter, world, 99)).toBe(false);

        // the upper box is in group 2 only: a mask without it passes over it
        filter.setAllEnabled(queryFilter, world);
        queryFilter.collisionMask = ~2 >>> 0;
        closest.reset();
        castRay(world, closest, settings, [0, 10, 0], DOWN, 20, queryFilter);
        expect(closest.hit.bodyIdB).toBe(stack.lower);

        // a body filter: the closest it passes, not the closest there is
        filter.setAllEnabled(queryFilter, world);
        let asked = 0;
        queryFilter.bodyFilter = (body) => {
            asked++;
            return body !== stack.upper;
        };
        closest.reset();
        castRay(world, closest, settings, [0, 10, 0], DOWN, 20, queryFilter);
        expect(asked).toBeGreaterThan(0);
        expect(closest.hit.bodyIdB).toBe(stack.lower);
        expect(closest.hit.fraction).toBeCloseTo(8.5 / 20, 3);

        // what the upper box collides with: its layer's layers, its groups and mask
        expect(filter.setFromBody(queryFilter, world, stack.upper)).toBe(true);
        expect(queryFilter.collisionGroups).toBe(2);
        expect(queryFilter.collisionMask).toBe(5);
        const copy = filter.create(world);
        filter.copy(copy, queryFilter);
        expect(copy.collisionMask).toBe(5);
        expect(copy.bodyFilter).toBe(queryFilter.bodyFilter);
        filter.destroy(copy);
        filter.destroy(queryFilter);
        releaseStack(stack);
    });

    test('a sphere swept down stops on the upper box, the normal back up', async () => {
        const stack = await makeStack();
        const { world } = stack;
        const queryFilter = filter.create(world);
        const ball = sphere.create({ radius: 0.5 });
        const closest = createClosestCastShapeCollector();
        castShape(world, closest, createDefaultCastShapeSettings(), ball, [0, 10, 0], IDENTITY, ONE, [0, -20, 0], queryFilter);
        expect(closest.hit.status).toBe(CastShapeStatus.COLLIDING);
        expect(closest.hit.bodyIdB).toBe(stack.upper);
        // its centre 0.5 above the box's top at 3.5
        expect(closest.hit.fraction).toBeCloseTo(6 / 20, 2);
        expect(closest.hit.normal[1]).toBeCloseTo(1, 2);
        expect(closest.hit.pointB[1]).toBeCloseTo(3.5, 1);
        releaseShape(ball);
        filter.destroy(queryFilter);
        releaseStack(stack);
    });

    test('a box overlapping both: every overlap, and the deepest', async () => {
        const stack = await makeStack();
        const { world } = stack;
        const queryFilter = filter.create(world);
        const probe = box.create({ halfExtents: [0.4, 0.7, 0.4] });
        const settings = createDefaultCollideShapeSettings();
        // y 1.4 to 2.8: into the lower box by 0.1, the upper by 0.3
        const all = createAllCollideShapeCollector();
        collideShape(world, all, settings, probe, [0, 2.1, 0], IDENTITY, ONE, queryFilter);
        expect(all.hits.map((hit) => hit.bodyIdB).sort()).toEqual([stack.lower, stack.upper].sort());
        const deepest = createClosestCollideShapeCollector();
        collideShape(world, deepest, settings, probe, [0, 2.1, 0], IDENTITY, ONE, queryFilter);
        expect(deepest.hit?.bodyIdB).toBe(stack.upper);
        expect(deepest.hit?.penetration).toBeCloseTo(0.3, 2);
        deepest.reset();
        expect(deepest.hit).toBeNull();
        collideShapeWithInternalEdgeRemoval(world, deepest, settings, probe, [0, 2.1, 0], IDENTITY, ONE, queryFilter);
        expect(deepest.hit?.bodyIdB).toBe(stack.upper);
        releaseShape(probe);
        filter.destroy(queryFilter);
        releaseStack(stack);
    });

    test('a point in the lower box is in it alone', async () => {
        const stack = await makeStack();
        const { world } = stack;
        const queryFilter = filter.create(world);
        const settings = createDefaultCollidePointSettings();
        const any = createAnyCollidePointCollector();
        collidePoint(world, any, settings, [0, 1, 0], queryFilter);
        expect(any.hit?.bodyIdB).toBe(stack.lower);
        // the hit lowered its early-out fraction to the bound: a query into it stops before running, until a reset
        collidePoint(world, any, settings, [0, 3, 0], queryFilter);
        expect(any.hit?.bodyIdB).toBe(stack.lower);
        any.reset();
        collidePoint(world, any, settings, [0, 3, 0], queryFilter);
        expect(any.hit?.bodyIdB).toBe(stack.upper);
        const all = createAllCollidePointCollector();
        collidePoint(world, all, settings, [0, 2, 0], queryFilter);
        expect(all.hits.length).toBe(0);
        filter.destroy(queryFilter);
        releaseStack(stack);
    });

    test('a query without a filter hits every layer and group', async () => {
        const stack = await makeStack();
        const { world } = stack;
        const all = createAllCastRayCollector();
        castRay(world, all, createDefaultCastRaySettings(), [0, 10, 0], DOWN, 20);
        expect(all.hits.length).toBe(3);
        const swept = createClosestCastShapeCollector();
        const ball = sphere.create({ radius: 0.5 });
        castShape(world, swept, createDefaultCastShapeSettings(), ball, [0, 10, 0], IDENTITY, ONE, [0, -20, 0]);
        expect(swept.hit.bodyIdB).toBe(stack.upper);
        const overlap = createAllCollideShapeCollector();
        collideShape(world, overlap, createDefaultCollideShapeSettings(), ball, [0, 2, 0], IDENTITY, ONE);
        expect(overlap.hits.map((hit) => hit.bodyIdB).sort()).toEqual([stack.lower, stack.upper].sort());
        const point = createAnyCollidePointCollector();
        collidePoint(world, point, createDefaultCollidePointSettings(), [0, 1, 0]);
        expect(point.hit?.bodyIdB).toBe(stack.lower);
        releaseShape(ball);
        releaseStack(stack);
    });

    test('filters, made and freed, leave the heap as it was', async () => {
        const stack = await makeStack();
        const before = Low.allocatedBytes();
        const a = filter.create(stack.world);
        const b = filter.create(stack.world);
        filter.disableAllLayers(a, stack.world);
        filter.copy(b, a);
        filter.destroy(a);
        filter.destroy(b);
        expect(Low.allocatedBytes()).toBe(before);
        releaseStack(stack);
    });
});
