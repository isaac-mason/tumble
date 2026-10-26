import { describe, expect, test } from 'vitest';
import {
    box,
    type ContactEvent,
    type ContactRemovedEvent,
    createWorldSettings,
    getGravity,
    init,
    type Listener,
    MotionType,
    releaseShape,
    rigidBody,
    setGravity,
    setMaterials,
    sphere,
    updateWorld,
    type Vec3,
} from '../src';
import { makeScene, releaseScene, steps } from './helpers';

describe('world settings', () => {
    test('start from the wasm defaults', async () => {
        await init();
        const settings = createWorldSettings();
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
        const ball = sphere.create({ radius: 0.5 });
        const body = rigidBody.create(scene.world, {
            shape: ball,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 3, 0],
        });
        steps(scene.world, 60);
        const position: Vec3 = [0, 0, 0];
        rigidBody.getPosition(position, scene.world, body);
        expect(position[1]).toBe(3);
        releaseScene(scene);
        releaseShape(ball);
    });
});

describe('the step', () => {
    test('a listener answers: a rejected pair passes through, a contact with no restitution does not bounce', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const ball = sphere.create({ radius: 0.5 });
        const ghost = rigidBody.create(world, {
            shape: ball,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [2, 0.5, 0],
        });
        const bouncer = rigidBody.create(world, {
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
        for (let i = 0; i < 120; i++) updateWorld(world, listener, 1 / 60);
        const position: Vec3 = [0, 0, 0];
        rigidBody.getPosition(position, world, ghost);
        expect(rejected).toBeGreaterThan(0);
        expect(position[1]).toBeLessThan(-1);
        // restitution 1 would bounce it back up; the listener kept it down
        rigidBody.getPosition(position, world, bouncer);
        expect(position[1]).toBeCloseTo(0.48, 1);
        releaseScene(scene);
        releaseShape(ball);
    });

    test('a contact the listener keeps rejecting is never solved', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const crate = box.create({ halfExtents: [0.5, 0.5, 0.5] });
        const body = rigidBody.create(world, {
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
        for (let i = 0; i < 90; i++) updateWorld(world, listener, 1 / 60);
        const position: Vec3 = [0, 0, 0];
        rigidBody.getPosition(position, world, body);
        expect(validated).toBeGreaterThan(1);
        expect(position[1]).toBeLessThan(-1);
        releaseScene(scene);
        releaseShape(crate);
    });

    test("contact events come whole, and a removed body's contacts are removed", async () => {
        const scene = await makeScene();
        const { world } = scene;
        const crate = box.create({ halfExtents: [0.5, 0.5, 0.5] });
        const body = rigidBody.create(world, {
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
        updateWorld(world, listener, 1 / 60);
        expect(added.length).toBe(1);
        const event = added[0]!;
        expect(event.bodyA === body || event.bodyB === body).toBe(true);
        expect(event.bodyA === body ? event.userDataA : event.userDataB).toBe(2 ** 40 + 5);
        expect(event.points).toBe(4);
        expect(Math.abs(event.normal[1])).toBeCloseTo(1, 5);
        for (let i = 0; i < 4; i++) expect(Math.abs(event.pointsA[i]![1])).toBeLessThan(0.05);

        rigidBody.remove(world, body);
        updateWorld(world, listener, 1 / 60);
        expect(removed.length).toBe(1);
        expect(removed[0]!.bodyA === body || removed[0]!.bodyB === body).toBe(true);
        releaseScene(scene);
        releaseShape(crate);
    });

    test('a hook sees the combined material, the material standing in for the body on its side', async () => {
        const scene = await makeScene();
        const { world } = scene;
        setMaterials(world, [{ friction: 0.81, restitution: 0 }]);
        const ground = box.create({ halfExtents: [5, 0.5, 5], materialId: 0 });
        const crate = box.create({ halfExtents: [0.5, 0.5, 0.5] });
        rigidBody.create(world, {
            shape: ground,
            objectLayer: scene.notMoving,
            motionType: MotionType.STATIC,
            position: [20, 5, 0],
        });
        const body = rigidBody.create(world, {
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
        for (let i = 0; i < 60; i++) updateWorld(world, listener, 1 / 60);
        expect(friction).toBeCloseTo(0.45, 6);
        const position: Vec3 = [0, 0, 0];
        rigidBody.getPosition(position, world, body);
        expect(position[1]).toBeCloseTo(6, 1);
        releaseScene(scene);
        releaseShape(ground);
        releaseShape(crate);
    });

    test('a hook that throws: the step finishes, then throws it', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const ball = sphere.create({ radius: 0.5 });
        rigidBody.create(world, {
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
        expect(() => updateWorld(world, listener, 1 / 60)).toThrow('from the hook');
        // the world is between steps again, and steps on
        updateWorld(world, undefined, 1 / 60);
        updateWorld(world, {}, 1 / 60);
        releaseScene(scene);
        releaseShape(ball);
    });

    test('gravity changed after the world is made, from the next step', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const ball = sphere.create({ radius: 0.5 });
        const body = rigidBody.create(world, {
            shape: ball,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 50, 0],
        });
        const gravity: Vec3 = [0, 0, 0];
        expect(getGravity(gravity, world)).toEqual([0, Math.fround(-9.81), 0]);
        setGravity(world, [3, 0, 0]);
        expect(getGravity(gravity, world)).toEqual([3, 0, 0]);
        steps(world, 60);
        const velocity: Vec3 = [0, 0, 0];
        rigidBody.getLinearVelocity(velocity, world, body);
        // a second sideways at 3 m/s/s, less a little damping, and no fall
        expect(velocity[0]).toBeGreaterThan(2.8);
        expect(Math.abs(velocity[1])).toBeLessThan(1e-6);
        releaseScene(scene);
        releaseShape(ball);
    });
});
