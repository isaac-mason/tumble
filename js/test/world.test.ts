import { describe, expect, test } from 'vitest';
import {
    Box,
    type ContactEvent,
    type ContactRemovedEvent,
    init,
    type Listener,
    MotionType,
    RigidBody,
    Sphere,
    type Vec3,
    World,
    WorldSettings,
} from '../src';
import { makeScene, releaseScene, steps } from './helpers';

describe('world settings', () => {
    test('start from the wasm defaults', async () => {
        await init();
        const settings = WorldSettings.create();
        expect(settings.gravity).toEqual([0, expect.closeTo(-9.81, 5), 0]);
        expect(settings.solver.velocityIterations).toBe(10);
        expect(settings.solver.penetrationSlop).toBeCloseTo(0.02, 6);
        expect(settings.sleeping.allowSleeping).toBe(true);
        expect(settings.narrowphase.speculativeContactDistance).toBeCloseTo(0.02, 6);
    });

    test('reach the world: no gravity, nothing falls', async () => {
        const scene = await makeScene((settings) => {
            settings.gravity = [0, 0, 0];
        });
        const ball = Sphere.create({ radius: 0.5 });
        const body = RigidBody.create(scene.world, {
            shape: ball,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 3, 0],
        });
        steps(scene.world, 60);
        const position: Vec3 = [0, 0, 0];
        RigidBody.getPosition(position, scene.world, body);
        expect(position[1]).toBe(3);
        releaseScene(scene);
        Sphere.destroy(ball);
    });
});

describe('the step', () => {
    test('a listener answers: a rejected pair passes through, a contact with no restitution does not bounce', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const ball = Sphere.create({ radius: 0.5 });
        const ghost = RigidBody.create(world, {
            shape: ball,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [2, 0.5, 0],
        });
        const bouncer = RigidBody.create(world, {
            shape: ball,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 3, 0],
            restitution: 1,
        });
        let rejected = 0;
        const listener: Listener = {
            onBodyPairValidate(bodyA, bodyB) {
                const collide = bodyA !== ghost && bodyB !== ghost;
                if (!collide) rejected++;
                return collide;
            },
            onContactAdded(_contact, settings) {
                settings.combinedRestitution = 0;
            },
            onContactPersisted(_contact, settings) {
                settings.combinedRestitution = 0;
            },
        };
        for (let i = 0; i < 120; i++) World.step(world, listener, 1 / 60);
        const position: Vec3 = [0, 0, 0];
        RigidBody.getPosition(position, world, ghost);
        expect(rejected).toBeGreaterThan(0);
        expect(position[1]).toBeLessThan(-1);
        // restitution 1 would bounce it back up; the listener kept it down
        RigidBody.getPosition(position, world, bouncer);
        expect(position[1]).toBeCloseTo(0.48, 1);
        releaseScene(scene);
        Sphere.destroy(ball);
    });

    test('a contact the listener keeps rejecting is never solved', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const crate = Box.create({ halfExtents: [0.5, 0.5, 0.5] });
        const body = RigidBody.create(world, {
            shape: crate,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 2, 0],
        });
        let validated = 0;
        const listener: Listener = {
            onContactValidate(contact) {
                validated++;
                return contact.bodyA !== body && contact.bodyB !== body;
            },
        };
        for (let i = 0; i < 90; i++) World.step(world, listener, 1 / 60);
        const position: Vec3 = [0, 0, 0];
        RigidBody.getPosition(position, world, body);
        expect(validated).toBeGreaterThan(1);
        expect(position[1]).toBeLessThan(-1);
        releaseScene(scene);
        Box.destroy(crate);
    });

    test("contact events come whole, and a removed body's contacts are removed", async () => {
        const scene = await makeScene();
        const { world } = scene;
        const crate = Box.create({ halfExtents: [0.5, 0.5, 0.5] });
        const body = RigidBody.create(world, {
            shape: crate,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 0.5, 0],
            userData: 2 ** 40 + 5,
        });
        const added: ContactEvent[] = [];
        const removed: ContactRemovedEvent[] = [];
        const listener: Listener = {
            onContactAdded(contact) {
                added.push(structuredClone(contact));
            },
            onContactRemoved(event) {
                removed.push({ ...event });
            },
        };
        World.step(world, listener, 1 / 60);
        expect(added.length).toBe(1);
        const event = added[0]!;
        expect(event.bodyA === body || event.bodyB === body).toBe(true);
        expect(event.bodyA === body ? event.userDataA : event.userDataB).toBe(2 ** 40 + 5);
        expect(event.points).toBe(4);
        expect(Math.abs(event.normal[1])).toBeCloseTo(1, 5);
        for (let i = 0; i < 4; i++) expect(Math.abs(event.pointsA[i]![1])).toBeLessThan(0.05);

        RigidBody.remove(world, body);
        World.step(world, listener, 1 / 60);
        expect(removed.length).toBe(1);
        expect(removed[0]!.bodyA === body || removed[0]!.bodyB === body).toBe(true);
        releaseScene(scene);
        Box.destroy(crate);
    });

    test('a hook sees the combined material, the material standing in for the body on its side', async () => {
        const scene = await makeScene();
        const { world } = scene;
        World.setMaterials(world, [{ friction: 0.81, restitution: 0 }]);
        const ground = Box.create({ halfExtents: [5, 0.5, 5], materialId: 0 });
        const crate = Box.create({ halfExtents: [0.5, 0.5, 0.5] });
        RigidBody.create(world, {
            shape: ground,
            objectLayer: scene.notMoving,
            motionType: MotionType.STATIC,
            position: [20, 5, 0],
        });
        const body = RigidBody.create(world, {
            shape: crate,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [20, 6, 0],
            friction: 0.25,
        });
        let friction = 0;
        const listener: Listener = {
            onContactAdded(_contact, settings) {
                friction = settings.combinedFriction;
            },
        };
        for (let i = 0; i < 60; i++) World.step(world, listener, 1 / 60);
        expect(friction).toBeCloseTo(0.45, 6);
        const position: Vec3 = [0, 0, 0];
        RigidBody.getPosition(position, world, body);
        expect(position[1]).toBeCloseTo(6, 1);
        releaseScene(scene);
        Box.destroy(ground);
        Box.destroy(crate);
    });

    test('a hook that throws: the step finishes, then throws it', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const ball = Sphere.create({ radius: 0.5 });
        RigidBody.create(world, {
            shape: ball,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 0.5, 0],
        });
        const listener: Listener = {
            onBodyPairValidate() {
                throw new Error('from the hook');
            },
        };
        expect(() => World.step(world, listener, 1 / 60)).toThrow('from the hook');
        // the world is between steps again, and steps on
        World.step(world, undefined, 1 / 60);
        World.step(world, {}, 1 / 60);
        releaseScene(scene);
        Sphere.destroy(ball);
    });
});
