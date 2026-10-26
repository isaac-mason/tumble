import { describe, expect, test } from 'vitest';
import { type Box3, box, Dof, MotionType, type Quat, releaseShape, rigidBody, sphere, updateWorld, type Vec3 } from '../src';
import { makeScene, releaseScene, steps } from './helpers';

describe('rigid bodies', () => {
    test('settings reach the body: user data, layer, motion type, mass', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const crate = box.create({ halfExtents: [0.5, 0.5, 0.5] });
        const body = rigidBody.create(world, {
            shape: crate,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 2, 0],
            userData: 2 ** 45 + 3,
            mass: 12,
        });
        expect(rigidBody.exists(world, body)).toBe(true);
        expect(rigidBody.getUserData(world, body)).toBe(2 ** 45 + 3);
        expect(rigidBody.getObjectLayer(world, body)).toBe(scene.moving);
        expect(rigidBody.getMotionType(world, body)).toBe(MotionType.DYNAMIC);
        expect(rigidBody.getMass(world, body)).toBeCloseTo(12, 4);
        expect(rigidBody.getShape(world, body)).toBe(crate);
        const bounds: Box3 = { min: [0, 0, 0], max: [0, 0, 0] };
        expect(rigidBody.getBounds(bounds, world, body)).toBe(true);
        expect(bounds.min[1]).toBeCloseTo(1.5, 1);
        expect(bounds.max[1]).toBeCloseTo(2.5, 1);
        rigidBody.setUserData(world, body, 9);
        expect(rigidBody.getUserData(world, body)).toBe(9);
        releaseScene(scene);
        releaseShape(crate);
    });

    test('velocities, impulses and teleports', async () => {
        const scene = await makeScene((settings) => {
            settings.gravity = [0, 0, 0];
        });
        const { world } = scene;
        const ball = sphere.create({ radius: 0.5 });
        const body = rigidBody.create(world, {
            shape: ball,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 5, 0],
            linearDamping: 0,
        });
        const velocity: Vec3 = [0, 0, 0];
        rigidBody.setLinearVelocity(world, body, [1, 0, 0]);
        rigidBody.addImpulse(world, body, [0, 0, rigidBody.getMass(world, body) * 2]);
        rigidBody.getLinearVelocity(velocity, world, body);
        expect(velocity[0]).toBeCloseTo(1, 5);
        expect(velocity[2]).toBeCloseTo(2, 4);
        rigidBody.setAngularVelocity(world, body, [0, 3, 0]);
        rigidBody.getAngularVelocity(velocity, world, body);
        expect(velocity[1]).toBeCloseTo(3, 4);

        rigidBody.setPosition(world, body, [10, 5, 0], true);
        const position: Vec3 = [0, 0, 0];
        rigidBody.getPosition(position, world, body);
        expect(position).toEqual([10, 5, 0]);
        releaseScene(scene);
        releaseShape(ball);
    });

    test('a kinematic body reaches its target in the step it was given', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const crate = box.create({ halfExtents: [0.5, 0.5, 0.5] });
        const body = rigidBody.create(world, {
            shape: crate,
            objectLayer: scene.moving,
            motionType: MotionType.KINEMATIC,
            position: [0, 3, 0],
        });
        rigidBody.moveKinematic(world, body, [2, 3, 0], [0, 0, 0, 1], 1 / 60);
        updateWorld(world, undefined, 1 / 60);
        const position: Vec3 = [0, 0, 0];
        rigidBody.getPosition(position, world, body);
        expect(position[0]).toBeCloseTo(2, 4);
        releaseScene(scene);
        releaseShape(crate);
    });

    test('degrees of freedom: a body locked to y falls straight even when pushed', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const ball = sphere.create({ radius: 0.5 });
        const body = rigidBody.create(world, {
            shape: ball,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 5, 0],
            allowedDegreesOfFreedom: Dof.TRANSLATION_Y,
        });
        rigidBody.addImpulse(world, body, [50, 0, 50]);
        steps(world, 30);
        const position: Vec3 = [0, 0, 0];
        rigidBody.getPosition(position, world, body);
        expect(position[0]).toBe(0);
        expect(position[2]).toBe(0);
        expect(position[1]).toBeLessThan(5);
        releaseScene(scene);
        releaseShape(ball);
    });

    test('a resting body sleeps, and wakes when told', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const crate = box.create({ halfExtents: [0.5, 0.5, 0.5] });
        const body = rigidBody.create(world, {
            shape: crate,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 0.5, 0],
        });
        steps(world, 120);
        expect(rigidBody.isSleeping(world, body)).toBe(true);
        rigidBody.wake(world, body);
        expect(rigidBody.isSleeping(world, body)).toBe(false);
        releaseScene(scene);
        releaseShape(crate);
    });

    test('getTransform reads position and rotation together', async () => {
        const scene = await makeScene();
        const ball = sphere.create({ radius: 0.5 });
        const body = rigidBody.create(scene.world, {
            shape: ball,
            objectLayer: scene.moving,
            motionType: MotionType.STATIC,
            position: [1, 2, 3],
            quaternion: [0, 1, 0, 0],
        });
        const position: Vec3 = [0, 0, 0];
        const quaternion: Quat = [0, 0, 0, 0];
        expect(rigidBody.getTransform(position, quaternion, scene.world, body)).toBe(true);
        expect(position).toEqual([1, 2, 3]);
        expect(quaternion).toEqual([0, 1, 0, 0]);
        releaseScene(scene);
        releaseShape(ball);
    });

    test('every call on a removed id answers false or nothing', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const ball = sphere.create({ radius: 0.5 });
        const body = rigidBody.create(world, { shape: ball, objectLayer: scene.moving, motionType: MotionType.DYNAMIC });
        rigidBody.remove(world, body);
        expect(rigidBody.exists(world, body)).toBe(false);
        expect(rigidBody.remove(world, body)).toBe(false);
        expect(rigidBody.setLinearVelocity(world, body, [1, 0, 0])).toBe(false);
        expect(rigidBody.addForce(world, body, [1, 0, 0], true)).toBe(false);
        expect(rigidBody.getMass(world, body)).toBe(0);
        expect(rigidBody.getShape(world, body)).toBe(null);
        // a new body in the freed slot has a new id; the old one still names nothing
        const next = rigidBody.create(world, { shape: ball, objectLayer: scene.moving, motionType: MotionType.DYNAMIC });
        expect(next).not.toBe(body);
        expect(rigidBody.exists(world, body)).toBe(false);
        releaseScene(scene);
        releaseShape(ball);
    });
});
