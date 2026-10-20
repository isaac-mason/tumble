import { describe, expect, test } from 'vitest';
import {
    Answers,
    Box,
    ContactEvent,
    ContactRemovedEvent,
    init,
    MotionType,
    PairEvent,
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
        RigidBody.getPosition(scene.world, position, body);
        expect(position[1]).toBe(3);
        releaseScene(scene);
        Sphere.destroy(ball);
    });
});

describe('the step', () => {
    test('phases with answers: a rejected pair passes through, an overridden contact does not bounce', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const ball = Sphere.create({ radius: 0.5 });
        const ghost = RigidBody.create(world, {
            shape: ball,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [2, 0.5, 0],
            userData: 7,
        });
        const bouncer = RigidBody.create(world, {
            shape: ball,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 3, 0],
            restitution: 1,
        });
        const pair = PairEvent.create();
        let rejected = 0;
        for (let i = 0; i < 120; i++) {
            World.stepBegin(world, 1 / 60);
            do {
                World.findPairs(world);
                for (let p = 0; p < World.pairs(world); p++) {
                    PairEvent.read(world, pair, p);
                    if (pair.userDataA === 7 || pair.userDataB === 7) rejected += Answers.rejectPair(world, p) ? 1 : 0;
                }
                World.narrowphase(world);
            } while (World.woken(world) > 0);
            const contacts = World.contactsAdded(world) + World.contactsPersisted(world);
            const event = ContactEvent.create();
            for (let c = 0; c < contacts; c++) {
                const added = c < World.contactsAdded(world);
                if (added) ContactEvent.readAdded(world, event, c);
                else ContactEvent.readPersisted(world, event, c - World.contactsAdded(world));
                event.settings.combinedRestitution = 0;
                Answers.overrideContact(world, c, event.settings);
            }
            World.solve(world);
            World.stepEnd(world);
        }
        const position: Vec3 = [0, 0, 0];
        RigidBody.getPosition(world, position, ghost);
        expect(rejected).toBeGreaterThan(0);
        expect(position[1]).toBeLessThan(-1);
        // restitution 1 would bounce it back up; the overrides kept it down
        RigidBody.getPosition(world, position, bouncer);
        expect(position[1]).toBeCloseTo(0.48, 1);
        releaseScene(scene);
        Sphere.destroy(ball);
    });

    test('contact events read whole, removals after a body goes, moved bodies', async () => {
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
        World.step(world, 1 / 60);
        expect(World.contactsAdded(world)).toBe(1);
        const event = ContactEvent.create();
        expect(ContactEvent.readAdded(world, event, 0)).toBe(true);
        expect(event.bodyA === body || event.bodyB === body).toBe(true);
        expect(event.bodyA === body ? event.userDataA : event.userDataB).toBe(2 ** 40 + 5);
        expect(event.points).toBe(4);
        expect(Math.abs(event.normal[1])).toBeCloseTo(1, 5);
        for (let i = 0; i < 4; i++) expect(Math.abs(event.pointsA[i]![1])).toBeLessThan(0.05);
        expect(ContactEvent.readAdded(world, event, 1)).toBe(false);
        expect(World.moved(world)).toBe(1);
        expect(World.movedBody(world, 0)).toBe(body);

        RigidBody.remove(world, body);
        World.step(world, 1 / 60);
        expect(World.contactsRemoved(world)).toBe(1);
        const removed = ContactRemovedEvent.create();
        expect(ContactRemovedEvent.read(world, removed, 0)).toBe(true);
        expect(removed.bodyA === body || removed.bodyB === body).toBe(true);
        releaseScene(scene);
        Box.destroy(crate);
    });

    test('materials stand in for the body on their side', async () => {
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
        RigidBody.create(world, {
            shape: crate,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [20, 6, 0],
            friction: 0.25,
        });
        World.step(world, 1 / 60);
        const event = ContactEvent.create();
        ContactEvent.readAdded(world, event, 0);
        expect(event.settings.combinedFriction).toBeCloseTo(0.45, 6);
        releaseScene(scene);
        Box.destroy(ground);
        Box.destroy(crate);
    });
});
