import { describe, expect, test } from 'vitest';
import {
    box,
    type Color,
    createWorld,
    createWorldSettings,
    DEBUG_VERTEX_BYTES,
    DEBUG_VERTEX_COLOR_OFFSET,
    type DebugViews,
    debug,
    debugDraw,
    destroyWorld,
    hingeConstraint,
    init,
    MotionType,
    releaseShape,
    rigidBody,
    sphere,
} from '../src';
import * as Low from '../src/generated/tumble';
import { makeScene, releaseScene, steps } from './helpers';

/** the vertices of the views' lines, as [x, y, z, r, g, b, a] */
function vertices(views: DebugViews): number[][] {
    const out: number[][] = [];
    const floats = DEBUG_VERTEX_BYTES / 4;
    for (let i = 0; i < views.lineCount * 2; i++) {
        const color = i * DEBUG_VERTEX_BYTES + DEBUG_VERTEX_COLOR_OFFSET;
        out.push([...views.lines.subarray(i * floats, i * floats + 3), ...views.lineColors.subarray(color, color + 4)]);
    }
    return out;
}

describe('debug drawing', () => {
    test("a box body's twelve edges, at its corners, in the dynamic green", async () => {
        const scene = await makeScene();
        const crate = box.create({ halfExtents: [0.5, 0.25, 1] });
        rigidBody.create(scene.world, {
            shape: crate,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [3, 2, -1],
        });
        const buffers = debugDraw.create();
        // the floor out of it, by its layer's bit
        debug.bodies(buffers, scene.world, { excludedObjectLayers: 1 << scene.notMoving });
        const views = debugDraw.read(buffers);
        expect(views.lineCount).toBe(12);
        expect(views.triangleCount).toBe(0);
        const points = vertices(views);
        expect(points.length).toBe(24);
        const xs = points.map((p) => p[0]!);
        const ys = points.map((p) => p[1]!);
        const zs = points.map((p) => p[2]!);
        expect([Math.min(...xs), Math.max(...xs)]).toEqual([2.5, 3.5]);
        expect([Math.min(...ys), Math.max(...ys)]).toEqual([1.75, 2.25]);
        expect([Math.min(...zs), Math.max(...zs)]).toEqual([-2, 0]);
        for (const p of points) expect(p.slice(3)).toEqual([51, 255, 51, 255]);
        debugDraw.destroy(buffers);
        releaseScene(scene);
        releaseShape(crate);
    });

    test('clear empties the buffers and keeps their memory: the same scene drawn again is in the same place', async () => {
        const scene = await makeScene();
        const buffers = debugDraw.create();
        debug.bodies(buffers, scene.world);
        const first = debugDraw.read(buffers);
        const count = first.lineCount;
        const address = first.lines.byteOffset;
        expect(count).toBeGreaterThan(0);
        debugDraw.clear(buffers);
        expect(debugDraw.read(buffers).lineCount).toBe(0);
        debug.bodies(buffers, scene.world);
        const again = debugDraw.read(buffers);
        expect(again.lineCount).toBe(count);
        expect(again.lines.byteOffset).toBe(address);
        // the same views object, its views rebuilt only when something moved
        expect(again).toBe(first);
        debugDraw.destroy(buffers);
        releaseScene(scene);
    });

    test('a line, a body by id in its own colour, and a body id naming nothing', async () => {
        const scene = await makeScene();
        const ball = sphere.create({ radius: 0.5 });
        const body = rigidBody.create(scene.world, {
            shape: ball,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 3, 0],
        });
        const buffers = debugDraw.create();
        const purple: Color = [128, 0, 255, 255];
        debug.line(buffers, [0, 0, 0], [1, 2, 3], purple);
        expect(vertices(debugDraw.read(buffers))).toEqual([
            [0, 0, 0, 128, 0, 255, 255],
            [1, 2, 3, 128, 0, 255, 255],
        ]);
        expect(debug.body(buffers, scene.world, body, { color: purple })).toBe(true);
        const drawn = vertices(debugDraw.read(buffers));
        expect(drawn.length).toBeGreaterThan(2);
        for (const p of drawn) expect(p.slice(3)).toEqual([128, 0, 255, 255]);
        rigidBody.remove(scene.world, body);
        const before = debugDraw.read(buffers).lineCount;
        expect(debug.body(buffers, scene.world, body)).toBe(false);
        expect(debugDraw.read(buffers).lineCount).toBe(before);
        debugDraw.destroy(buffers);
        releaseScene(scene);
        releaseShape(ball);
    });

    test('contacts, contact constraints and joints draw once the world has them', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const crate = box.create({ halfExtents: [0.5, 0.5, 0.5] });
        const resting = rigidBody.create(world, {
            shape: crate,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 0.5, 0],
        });
        const door = rigidBody.create(world, {
            shape: crate,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [4, 3, 0],
        });
        hingeConstraint.create(world, resting, door, { pointA: [4, 3.5, 0], pointB: [4, 3.5, 0] });
        const buffers = debugDraw.create();
        debug.contacts(buffers, world);
        expect(debugDraw.read(buffers).lineCount).toBe(0);
        steps(world, 2);
        debug.contacts(buffers, world);
        const contactLines = debugDraw.read(buffers).lineCount;
        expect(contactLines).toBeGreaterThan(0);
        debug.contactConstraints(buffers, world);
        const withManifolds = debugDraw.read(buffers).lineCount;
        expect(withManifolds).toBeGreaterThan(contactLines);
        debug.joints(buffers, world, { size: 1 });
        expect(debugDraw.read(buffers).lineCount).toBeGreaterThan(withManifolds);
        debugDraw.destroy(buffers);
        releaseScene(scene);
        releaseShape(crate);
    });

    test('views follow the memory when it grows past where they were', async () => {
        const scene = await makeScene();
        const ball = sphere.create({ radius: 0.2 });
        const buffers = debugDraw.create();
        debug.bodies(buffers, scene.world);
        const small = debugDraw.read(buffers).lines;
        // more than the memory holds: it grows under the view kept above
        const fillerBytes = Low.getWasm().mem.buffer.byteLength;
        const filler = Low.allocBytes(fillerBytes);
        for (let i = 0; i < 400; i++) {
            rigidBody.create(scene.world, {
                shape: ball,
                objectLayer: scene.moving,
                motionType: MotionType.DYNAMIC,
                position: [i, 5, 0],
            });
        }
        debug.bodies(buffers, scene.world);
        const grown = debugDraw.read(buffers);
        // the memory grew: a view kept from before is detached, empty, and read hands back new ones
        expect(small.length).toBe(0);
        expect(grown.lineColors.length).toBe(grown.lineCount * 2 * DEBUG_VERTEX_BYTES);
        // the last sphere's lines are about x 399
        const last = vertices(grown).at(-1)!;
        expect(Math.abs(last[0]! - 399)).toBeLessThan(0.25);
        Low.freeBytes(filler, fillerBytes);
        debugDraw.destroy(buffers);
        releaseScene(scene);
        releaseShape(ball);
    });

    test('buffers, made, drawn into and freed, leave the heap as it was', async () => {
        await init();
        const settings = createWorldSettings();
        const world = createWorld(settings);
        const before = Low.allocatedBytes();
        const buffers = debugDraw.create();
        const ball = sphere.create({ radius: 0.5 });
        debug.shape(buffers, ball, [0, 0, 0], [0, 0, 0, 1], { color: [255, 0, 0, 255] });
        debug.bodies(buffers, world);
        debugDraw.destroy(buffers);
        releaseShape(ball);
        expect(Low.allocatedBytes()).toBe(before);
        destroyWorld(world);
    });
});
