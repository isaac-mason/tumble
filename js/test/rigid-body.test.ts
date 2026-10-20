import { describe, expect, test } from 'vitest';
import { Box, type Box3, Dof, MotionType, RigidBody, Sphere, type Vec3, World } from '../src';
import { makeScene, releaseScene, steps } from './helpers';

describe('rigid bodies', () => {
    test('settings reach the body: user data, layer, motion type, mass', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const crate = Box.create({ halfExtents: [0.5, 0.5, 0.5] });
        const body = RigidBody.create(world, {
            shape: crate,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 2, 0],
            userData: 2 ** 45 + 3,
            mass: 12,
        });
        expect(RigidBody.exists(world, body)).toBe(true);
        expect(RigidBody.getUserData(world, body)).toBe(2 ** 45 + 3);
        expect(RigidBody.getObjectLayer(world, body)).toBe(scene.moving);
        expect(RigidBody.getMotionType(world, body)).toBe(MotionType.DYNAMIC);
        expect(RigidBody.getMass(world, body)).toBeCloseTo(12, 4);
        expect(RigidBody.getShape(world, body)).toBe(crate);
        const bounds: Box3 = { min: [0, 0, 0], max: [0, 0, 0] };
        expect(RigidBody.getBounds(bounds, world, body)).toBe(true);
        expect(bounds.min[1]).toBeCloseTo(1.5, 1);
        expect(bounds.max[1]).toBeCloseTo(2.5, 1);
        RigidBody.setUserData(world, body, 9);
        expect(RigidBody.getUserData(world, body)).toBe(9);
        releaseScene(scene);
        Box.destroy(crate);
    });

    test('velocities, impulses and teleports', async () => {
        const scene = await makeScene((settings) => {
            settings.gravity = [0, 0, 0];
        });
        const { world } = scene;
        const ball = Sphere.create({ radius: 0.5 });
        const body = RigidBody.create(world, {
            shape: ball,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 5, 0],
            linearDamping: 0,
        });
        const velocity: Vec3 = [0, 0, 0];
        RigidBody.setLinearVelocity(world, body, [1, 0, 0]);
        RigidBody.addImpulse(world, body, [0, 0, RigidBody.getMass(world, body) * 2]);
        RigidBody.getLinearVelocity(velocity, world, body);
        expect(velocity[0]).toBeCloseTo(1, 5);
        expect(velocity[2]).toBeCloseTo(2, 4);
        RigidBody.setAngularVelocity(world, body, [0, 3, 0]);
        RigidBody.getAngularVelocity(velocity, world, body);
        expect(velocity[1]).toBeCloseTo(3, 4);

        RigidBody.setPosition(world, body, [10, 5, 0]);
        const position: Vec3 = [0, 0, 0];
        RigidBody.getPosition(position, world, body);
        expect(position).toEqual([10, 5, 0]);
        releaseScene(scene);
        Sphere.destroy(ball);
    });

    test('a kinematic body reaches its target in the step it was given', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const crate = Box.create({ halfExtents: [0.5, 0.5, 0.5] });
        const body = RigidBody.create(world, {
            shape: crate,
            objectLayer: scene.moving,
            motionType: MotionType.KINEMATIC,
            position: [0, 3, 0],
        });
        RigidBody.moveKinematic(world, body, [2, 3, 0], [0, 0, 0, 1], 1 / 60);
        World.step(world, 1 / 60);
        const position: Vec3 = [0, 0, 0];
        RigidBody.getPosition(position, world, body);
        expect(position[0]).toBeCloseTo(2, 4);
        releaseScene(scene);
        Box.destroy(crate);
    });

    test('degrees of freedom: a body locked to y falls straight even when pushed', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const ball = Sphere.create({ radius: 0.5 });
        const body = RigidBody.create(world, {
            shape: ball,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 5, 0],
            allowedDegreesOfFreedom: Dof.TRANSLATION_Y,
        });
        RigidBody.addImpulse(world, body, [50, 0, 50]);
        steps(world, 30);
        const position: Vec3 = [0, 0, 0];
        RigidBody.getPosition(position, world, body);
        expect(position[0]).toBe(0);
        expect(position[2]).toBe(0);
        expect(position[1]).toBeLessThan(5);
        releaseScene(scene);
        Sphere.destroy(ball);
    });

    test('a resting body sleeps, and wakes when told', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const crate = Box.create({ halfExtents: [0.5, 0.5, 0.5] });
        const body = RigidBody.create(world, {
            shape: crate,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 0.5, 0],
        });
        steps(world, 120);
        expect(RigidBody.isSleeping(world, body)).toBe(true);
        RigidBody.wake(world, body);
        expect(RigidBody.isSleeping(world, body)).toBe(false);
        releaseScene(scene);
        Box.destroy(crate);
    });

    test('getTransforms reads many poses at once, leaving gone bodies as they were', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const ball = Sphere.create({ radius: 0.5 });
        const bodies = [0, 1, 2].map((i) =>
            RigidBody.create(world, {
                shape: ball,
                objectLayer: scene.moving,
                motionType: MotionType.DYNAMIC,
                position: [i * 2, 3, 0],
            }),
        );
        RigidBody.remove(world, bodies[1]!);
        const out = new Float32Array(21).fill(-1);
        expect(RigidBody.getTransforms(out, world, bodies)).toBe(2);
        expect(Array.from(out.subarray(0, 7))).toEqual([0, 3, 0, 0, 0, 0, 1]);
        expect(Array.from(out.subarray(7, 14))).toEqual([-1, -1, -1, -1, -1, -1, -1]);
        expect(out[14]).toBe(4);
        releaseScene(scene);
        Sphere.destroy(ball);
    });

    test('every call on a removed id answers false or nothing', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const ball = Sphere.create({ radius: 0.5 });
        const body = RigidBody.create(world, { shape: ball, objectLayer: scene.moving, motionType: MotionType.DYNAMIC });
        RigidBody.remove(world, body);
        expect(RigidBody.exists(world, body)).toBe(false);
        expect(RigidBody.remove(world, body)).toBe(false);
        expect(RigidBody.setLinearVelocity(world, body, [1, 0, 0])).toBe(false);
        expect(RigidBody.addForce(world, body, [1, 0, 0])).toBe(false);
        expect(RigidBody.getMass(world, body)).toBe(0);
        expect(RigidBody.getShape(world, body)).toBe(null);
        // a new body in the freed slot has a new id; the old one still names nothing
        const next = RigidBody.create(world, { shape: ball, objectLayer: scene.moving, motionType: MotionType.DYNAMIC });
        expect(next).not.toBe(body);
        expect(RigidBody.exists(world, body)).toBe(false);
        releaseScene(scene);
        Sphere.destroy(ball);
    });
});
