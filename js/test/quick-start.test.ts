import { describe, expect, test } from 'vitest';
import { Box, init, MotionType, RigidBody, Sphere, type Vec3, World, WorldSettings } from '../src';

describe('a world from js', () => {
    test('a box and a ball dropped onto a floor come to rest on it', async () => {
        await init();
        const settings = WorldSettings.create();
        const moving = WorldSettings.addObjectLayer(settings, WorldSettings.addBroadphaseLayer(settings));
        const notMoving = WorldSettings.addObjectLayer(settings, WorldSettings.addBroadphaseLayer(settings));
        WorldSettings.enableCollision(settings, moving, moving);
        WorldSettings.enableCollision(settings, moving, notMoving);
        const world = World.create(settings);

        const floor = Box.create({ halfExtents: [20, 0.5, 20] });
        const crate = Box.create({ halfExtents: [0.5, 0.5, 0.5] });
        const ball = Sphere.create({ radius: 0.5 });
        RigidBody.create(world, { shape: floor, objectLayer: notMoving, motionType: MotionType.STATIC, position: [0, -0.5, 0] });
        const box = RigidBody.create(world, {
            shape: crate,
            objectLayer: moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 4, 0],
        });
        const sphere = RigidBody.create(world, {
            shape: ball,
            objectLayer: moving,
            motionType: MotionType.DYNAMIC,
            position: [3, 6, 0],
        });
        for (let i = 0; i < 120; i++) World.step(world, 1 / 60);

        const position: Vec3 = [0, 0, 0];
        expect(RigidBody.getPosition(position, world, box)).toBe(true);
        expect(position[1]).toBeCloseTo(0.48, 1);
        RigidBody.getPosition(position, world, sphere);
        expect(position[1]).toBeCloseTo(0.48, 1);

        // a removed body's id names nothing
        expect(RigidBody.remove(world, box)).toBe(true);
        position[1] = 123;
        expect(RigidBody.getPosition(position, world, box)).toBe(false);
        expect(position[1]).toBe(123);
        World.destroy(world);
        Box.destroy(floor);
        Box.destroy(crate);
        Sphere.destroy(ball);
    });
});
