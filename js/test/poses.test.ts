import { describe, expect, test } from 'vitest';
import { INVALID_BODY_ID, MotionType, Poses, RigidBody, Sphere, World } from '../src';
import { makeScene, releaseScene, steps } from './helpers';

describe('poses', () => {
    test('a slot per body, its pose and the changed slots from one read', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const ball = Sphere.create({ radius: 0.5 });
        const resting = RigidBody.create(world, {
            shape: ball,
            objectLayer: scene.notMoving,
            motionType: MotionType.STATIC,
            position: [5, 0.5, 0],
        });
        const falling = RigidBody.create(world, {
            shape: ball,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 5, 0],
        });
        const poses = Poses.create(world);
        const restingSlot = Poses.add(poses, resting);
        const fallingSlot = Poses.add(poses, falling);
        expect(Poses.body(poses, fallingSlot)).toBe(falling);

        // newly added slots read as changed, so their meshes get placed
        let views = Poses.read(poses);
        expect(Array.from(views.changed).sort()).toEqual([restingSlot, fallingSlot].sort());
        expect(Array.from(views.positions.subarray(restingSlot * 3, restingSlot * 3 + 3))).toEqual([5, 0.5, 0]);
        expect(Array.from(views.quaternions.subarray(fallingSlot * 4, fallingSlot * 4 + 4))).toEqual([0, 0, 0, 1]);

        // two steps between reads, as a fixed timestep loop makes: only the falling body changed, and its pose is the latest
        World.step(world, 1 / 60);
        World.step(world, 1 / 60);
        views = Poses.read(poses);
        expect(Array.from(views.changed)).toEqual([fallingSlot]);
        const y: [number] = [0];
        const position: [number, number, number] = [0, 0, 0];
        RigidBody.getPosition(world, position, falling);
        y[0] = views.positions[fallingSlot * 3 + 1]!;
        expect(y[0]).toBe(Math.fround(position[1]));

        // nothing between reads: nothing changed
        expect(Poses.read(poses).changed.length).toBe(0);

        // a teleport with no step between is seen
        RigidBody.setPosition(world, resting, [6, 0.5, 0], true);
        views = Poses.read(poses);
        expect(Array.from(views.changed)).toEqual([restingSlot]);
        expect(views.positions[restingSlot * 3]).toBe(6);

        Poses.destroy(poses);
        releaseScene(scene);
        Sphere.destroy(ball);
    });

    test('a gone body keeps its last pose; a freed slot is reused', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const ball = Sphere.create({ radius: 0.5 });
        const poses = Poses.create(world);
        const first = RigidBody.create(world, {
            shape: ball,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 3, 0],
        });
        const slot = Poses.add(poses, first);
        Poses.read(poses);
        RigidBody.remove(world, first);
        steps(world, 10);
        let views = Poses.read(poses);
        expect(views.changed.length).toBe(0);
        expect(views.positions[slot * 3 + 1]).toBe(3);

        expect(Poses.remove(poses, slot)).toBe(true);
        expect(Poses.remove(poses, slot)).toBe(false);
        expect(Poses.body(poses, slot)).toBe(INVALID_BODY_ID);
        const second = RigidBody.create(world, {
            shape: ball,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [2, 4, 0],
        });
        expect(Poses.add(poses, second)).toBe(slot);
        views = Poses.read(poses);
        expect(Array.from(views.changed)).toEqual([slot]);
        expect(views.positions[slot * 3]).toBe(2);

        expect(() => Poses.add(poses, first)).toThrow();
        Poses.destroy(poses);
        releaseScene(scene);
        Sphere.destroy(ball);
    });

    test('views follow the arrays when slots and memory grow', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const ball = Sphere.create({ radius: 0.2 });
        const poses = Poses.create(world);
        let kept: Float32Array = new Float32Array(0);
        const bodies: number[] = [];
        for (let i = 0; i < 5000; i++) {
            const body = RigidBody.create(world, {
                shape: ball,
                objectLayer: scene.notMoving,
                motionType: MotionType.STATIC,
                position: [i % 100, 1, Math.floor(i / 100)],
            });
            bodies.push(body);
            Poses.add(poses, body);
            if (i === 0) kept = Poses.read(poses).positions;
        }
        const views = Poses.read(poses);
        // the memory grew under the view kept from the first read: it reads as empty, which is why read is called for fresh ones
        expect(kept.length).toBe(0);
        expect(views.positions.length).toBe(5000 * 3);
        // slot 0 was read already
        expect(views.changed.length).toBe(4999);
        expect(views.positions[4999 * 3]).toBe(99);
        expect(views.positions[4999 * 3 + 2]).toBe(49);
        Poses.destroy(poses);
        releaseScene(scene);
        Sphere.destroy(ball);
    });
});
