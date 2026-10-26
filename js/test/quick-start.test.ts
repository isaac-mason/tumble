import { describe, expect, test } from 'vitest';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    createWorld,
    createWorldSettings,
    destroyWorld,
    enableCollision,
    init,
    MotionType,
    releaseShape,
    rigidBody,
    sphere,
    updateWorld,
    type Vec3,
} from '../src';

describe('a world from js', () => {
    test('a box and a ball dropped onto a floor come to rest on it', async () => {
        await init();
        const settings = createWorldSettings();
        const moving = addObjectLayer(settings, addBroadphaseLayer(settings));
        const notMoving = addObjectLayer(settings, addBroadphaseLayer(settings));
        enableCollision(settings, moving, moving);
        enableCollision(settings, moving, notMoving);
        const world = createWorld(settings);

        const floor = box.create({ halfExtents: [20, 0.5, 20] });
        const crate = box.create({ halfExtents: [0.5, 0.5, 0.5] });
        const ball = sphere.create({ radius: 0.5 });
        rigidBody.create(world, { shape: floor, objectLayer: notMoving, motionType: MotionType.STATIC, position: [0, -0.5, 0] });
        const crateBody = rigidBody.create(world, {
            shape: crate,
            objectLayer: moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 4, 0],
        });
        const ballBody = rigidBody.create(world, {
            shape: ball,
            objectLayer: moving,
            motionType: MotionType.DYNAMIC,
            position: [3, 6, 0],
        });
        for (let i = 0; i < 120; i++) updateWorld(world, undefined, 1 / 60);

        const position: Vec3 = [0, 0, 0];
        expect(rigidBody.getPosition(position, world, crateBody)).toBe(true);
        expect(position[1]).toBeCloseTo(0.48, 1);
        rigidBody.getPosition(position, world, ballBody);
        expect(position[1]).toBeCloseTo(0.48, 1);

        // a removed body's id names nothing
        expect(rigidBody.remove(world, crateBody)).toBe(true);
        position[1] = 123;
        expect(rigidBody.getPosition(position, world, crateBody)).toBe(false);
        expect(position[1]).toBe(123);
        destroyWorld(world);
        releaseShape(floor);
        releaseShape(crate);
        releaseShape(ball);
    });
});
