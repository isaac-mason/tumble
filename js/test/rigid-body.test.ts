import { describe, expect, test } from 'vitest';
import {
    type Box3,
    box,
    CastRayStatus,
    castRay,
    constraints,
    createClosestCastRayCollector,
    createDefaultCastRaySettings,
    Dof,
    type Mat4,
    MotionType,
    pointConstraint,
    type Quat,
    releaseShape,
    rigidBody,
    sphere,
    updateWorld,
    type Vec3,
} from '../src';
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
        expect(rigidBody.getGravityFactor(world, body)).toBe(-1);
        expect(rigidBody.setGravityFactor(world, body, 2)).toBe(false);
        expect(rigidBody.bodiesShareConstraint(world, body, next)).toBe(false);
        releaseScene(scene);
        releaseShape(ball);
    });

    test("a ray's hit on a turned box: the surface normal there is the face's, turned with the body", async () => {
        const scene = await makeScene();
        const { world } = scene;
        const crate = box.create({ halfExtents: [1, 1, 1] });
        // a quarter turn about z: the box's +x face looks up
        const turned: Quat = [0, 0, Math.SQRT1_2, Math.SQRT1_2];
        const body = rigidBody.create(world, {
            shape: crate,
            objectLayer: scene.notMoving,
            motionType: MotionType.STATIC,
            position: [0, 5, 0],
            quaternion: turned,
        });
        const collector = createClosestCastRayCollector();
        castRay(world, collector, createDefaultCastRaySettings(), [0.3, 10, 0.2], [0, -1, 0], 10);
        expect(collector.hit.status).toBe(CastRayStatus.COLLIDING);
        expect(collector.hit.bodyIdB).toBe(body);
        const at: Vec3 = [0.3, 10 - collector.hit.fraction * 10, 0.2];
        expect(at[1]).toBeCloseTo(6, 4);
        const normal: Vec3 = [0, 0, 0];
        expect(rigidBody.getSurfaceNormal(normal, world, body, collector.hit.subShapeId, at)).toBe(true);
        expect(normal[0]).toBeCloseTo(0, 5);
        expect(normal[1]).toBeCloseTo(1, 5);
        expect(normal[2]).toBeCloseTo(0, 5);
        releaseScene(scene);
        releaseShape(crate);
    });

    test('inverse inertia: a ball of 10 kg and radius 0.5 has an inertia of 1, and a static body none', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const ball = sphere.create({ radius: 0.5 });
        const dynamic = rigidBody.create(world, {
            shape: ball,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            mass: 10,
        });
        const fixed = rigidBody.create(world, { shape: ball, objectLayer: scene.notMoving, motionType: MotionType.STATIC });
        const inverse: Mat4 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        expect(rigidBody.getInverseInertia(inverse, world, dynamic)).toBe(true);
        // 2/5 m r^2, the same about every axis
        for (const diagonal of [0, 5, 10]) expect(inverse[diagonal]).toBeCloseTo(1, 4);
        for (const off of [1, 2, 4, 6, 8, 9]) expect(inverse[off]).toBeCloseTo(0, 6);
        expect(rigidBody.getInverseInertia(inverse, world, fixed)).toBe(true);
        expect(inverse.slice(0, 11).every((value) => value === 0)).toBe(true);
        releaseScene(scene);
        releaseShape(ball);
    });

    test('a gravity factor of 0 floats, and one set later takes over from the one made with', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const ball = sphere.create({ radius: 0.5 });
        const floating = rigidBody.create(world, {
            shape: ball,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 10, 0],
            gravityFactor: 0.5,
        });
        expect(rigidBody.getGravityFactor(world, floating)).toBe(0.5);
        expect(rigidBody.setGravityFactor(world, floating, 0)).toBe(true);
        expect(rigidBody.getGravityFactor(world, floating)).toBe(0);
        steps(world, 30);
        const at: Vec3 = [0, 0, 0];
        rigidBody.getPosition(at, world, floating);
        expect(at[1]).toBeCloseTo(10, 5);
        rigidBody.setGravityFactor(world, floating, 1);
        rigidBody.wake(world, floating);
        steps(world, 30);
        rigidBody.getPosition(at, world, floating);
        expect(at[1]).toBeLessThan(9);
        releaseScene(scene);
        releaseShape(ball);
    });

    test('bodies share a constraint while one joins them', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const ball = sphere.create({ radius: 0.5 });
        const create = (x: number) =>
            rigidBody.create(world, {
                shape: ball,
                objectLayer: scene.moving,
                motionType: MotionType.DYNAMIC,
                position: [x, 2, 0],
            });
        const a = create(0);
        const b = create(1);
        const c = create(2);
        const joint = pointConstraint.create(world, a, b, { pointA: [0.5, 2, 0], pointB: [0.5, 2, 0] });
        expect(rigidBody.bodiesShareConstraint(world, a, b)).toBe(true);
        expect(rigidBody.bodiesShareConstraint(world, b, a)).toBe(true);
        expect(rigidBody.bodiesShareConstraint(world, a, c)).toBe(false);
        constraints.remove(world, joint);
        expect(rigidBody.bodiesShareConstraint(world, a, b)).toBe(false);
        releaseScene(scene);
        releaseShape(ball);
    });
});
