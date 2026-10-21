import { describe, expect, test } from 'vitest';
import {
    Box,
    DEBUG_VERTEX_STRIDE,
    Debug,
    DebugBuffers,
    HingeConstraint,
    init,
    MotionType,
    RigidBody,
    Sphere,
    type Vec3,
    World,
    WorldSettings,
} from '../src';
import * as Low from '../src/generated/tumble';
import { makeScene, releaseScene, steps } from './helpers';

/** the vertices of the views' lines, as [x, y, z, r, g, b] */
function vertices(lines: Float32Array): number[][] {
    const out: number[][] = [];
    for (let i = 0; i < lines.length; i += DEBUG_VERTEX_STRIDE) out.push(Array.from(lines.subarray(i, i + DEBUG_VERTEX_STRIDE)));
    return out;
}

describe('debug drawing', () => {
    test("a box body's twelve edges, at its corners, in the dynamic green", async () => {
        const scene = await makeScene();
        const crate = Box.create({ halfExtents: [0.5, 0.25, 1] });
        RigidBody.create(scene.world, {
            shape: crate,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [3, 2, -1],
        });
        const buffers = DebugBuffers.create();
        // the floor out of it, by its layer's bit
        Debug.bodies(buffers, scene.world, { excludedObjectLayers: 1 << scene.notMoving });
        const views = DebugBuffers.read(buffers);
        expect(views.lineCount).toBe(12);
        expect(views.triangleCount).toBe(0);
        const points = vertices(views.lines);
        expect(points.length).toBe(24);
        const xs = points.map((p) => p[0]!);
        const ys = points.map((p) => p[1]!);
        const zs = points.map((p) => p[2]!);
        expect([Math.min(...xs), Math.max(...xs)]).toEqual([2.5, 3.5]);
        expect([Math.min(...ys), Math.max(...ys)]).toEqual([1.75, 2.25]);
        expect([Math.min(...zs), Math.max(...zs)]).toEqual([-2, 0]);
        for (const p of points) expect(p.slice(3)).toEqual([Math.fround(0.2), 1, Math.fround(0.2)]);
        DebugBuffers.destroy(buffers);
        releaseScene(scene);
        Box.destroy(crate);
    });

    test('clear empties the buffers and keeps their memory: the same scene drawn again is in the same place', async () => {
        const scene = await makeScene();
        const buffers = DebugBuffers.create();
        Debug.bodies(buffers, scene.world);
        const first = DebugBuffers.read(buffers);
        const count = first.lineCount;
        const address = first.lines.byteOffset;
        expect(count).toBeGreaterThan(0);
        DebugBuffers.clear(buffers);
        expect(DebugBuffers.read(buffers).lineCount).toBe(0);
        Debug.bodies(buffers, scene.world);
        const again = DebugBuffers.read(buffers);
        expect(again.lineCount).toBe(count);
        expect(again.lines.byteOffset).toBe(address);
        // the same views object, its views rebuilt only when something moved
        expect(again).toBe(first);
        DebugBuffers.destroy(buffers);
        releaseScene(scene);
    });

    test('a line, a body by id in its own colour, and a body id naming nothing', async () => {
        const scene = await makeScene();
        const ball = Sphere.create({ radius: 0.5 });
        const body = RigidBody.create(scene.world, {
            shape: ball,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 3, 0],
        });
        const buffers = DebugBuffers.create();
        const purple: Vec3 = [0.5, 0, 1];
        Debug.line(buffers, [0, 0, 0], [1, 2, 3], purple);
        expect(vertices(DebugBuffers.read(buffers).lines)).toEqual([
            [0, 0, 0, 0.5, 0, 1],
            [1, 2, 3, 0.5, 0, 1],
        ]);
        expect(Debug.body(buffers, scene.world, body, { color: purple })).toBe(true);
        const drawn = vertices(DebugBuffers.read(buffers).lines);
        expect(drawn.length).toBeGreaterThan(2);
        for (const p of drawn) expect(p.slice(3)).toEqual([0.5, 0, 1]);
        RigidBody.remove(scene.world, body);
        const before = DebugBuffers.read(buffers).lineCount;
        expect(Debug.body(buffers, scene.world, body)).toBe(false);
        expect(DebugBuffers.read(buffers).lineCount).toBe(before);
        DebugBuffers.destroy(buffers);
        releaseScene(scene);
        Sphere.destroy(ball);
    });

    test('contacts, contact constraints and joints draw once the world has them', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const crate = Box.create({ halfExtents: [0.5, 0.5, 0.5] });
        const resting = RigidBody.create(world, {
            shape: crate,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 0.5, 0],
        });
        const door = RigidBody.create(world, {
            shape: crate,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [4, 3, 0],
        });
        HingeConstraint.create(world, resting, door, { pointA: [4, 3.5, 0], pointB: [4, 3.5, 0] });
        const buffers = DebugBuffers.create();
        Debug.contacts(buffers, world);
        expect(DebugBuffers.read(buffers).lineCount).toBe(0);
        steps(world, 2);
        Debug.contacts(buffers, world);
        const contactLines = DebugBuffers.read(buffers).lineCount;
        expect(contactLines).toBeGreaterThan(0);
        Debug.contactConstraints(buffers, world);
        const withManifolds = DebugBuffers.read(buffers).lineCount;
        expect(withManifolds).toBeGreaterThan(contactLines);
        Debug.joints(buffers, world, { size: 1 });
        expect(DebugBuffers.read(buffers).lineCount).toBeGreaterThan(withManifolds);
        DebugBuffers.destroy(buffers);
        releaseScene(scene);
        Box.destroy(crate);
    });

    test('views follow the memory when a draw grows it past where they were', async () => {
        const scene = await makeScene();
        const ball = Sphere.create({ radius: 0.2 });
        const buffers = DebugBuffers.create();
        Debug.bodies(buffers, scene.world);
        const small = DebugBuffers.read(buffers).lines;
        for (let i = 0; i < 400; i++) {
            RigidBody.create(scene.world, {
                shape: ball,
                objectLayer: scene.moving,
                motionType: MotionType.DYNAMIC,
                position: [i, 5, 0],
            });
        }
        Debug.bodies(buffers, scene.world);
        const grown = DebugBuffers.read(buffers);
        // the memory grew: a view kept from before is detached, empty, and read hands back new ones
        expect(small.length).toBe(0);
        expect(grown.lines.length).toBe(grown.lineCount * 2 * DEBUG_VERTEX_STRIDE);
        // the last sphere's lines are about x 399
        const last = vertices(grown.lines).at(-1)!;
        expect(Math.abs(last[0]! - 399)).toBeLessThan(0.25);
        DebugBuffers.destroy(buffers);
        releaseScene(scene);
        Sphere.destroy(ball);
    });

    test('buffers, made, drawn into and freed, leave the heap as it was', async () => {
        await init();
        const settings = WorldSettings.create();
        const world = World.create(settings);
        const before = Low.allocatedBytes();
        const buffers = DebugBuffers.create();
        const ball = Sphere.create({ radius: 0.5 });
        Debug.shape(buffers, ball, [0, 0, 0], [0, 0, 0, 1], { color: [1, 0, 0] });
        Debug.bodies(buffers, world);
        DebugBuffers.destroy(buffers);
        Sphere.destroy(ball);
        expect(Low.allocatedBytes()).toBe(before);
        World.destroy(world);
    });
});
