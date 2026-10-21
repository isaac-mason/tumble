import { describe, expect, test } from 'vitest';
import {
    Capsule,
    ContactPairs,
    createDefaultUpdateSettings,
    Filter,
    KCC,
    MotionType,
    RigidBody,
    Sphere,
    Transformed,
    type Vec3,
    World,
} from '../src';
import { makeScene, releaseScene } from './helpers';

describe('contact pairs', () => {
    test('a bouncing ball observing its contacts: its pair with the floor added, removed in the air, and added again', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const ball = Sphere.create({ radius: 0.5 });
        const body = RigidBody.create(world, {
            shape: ball,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 2, 0],
            restitution: 0.6,
            userData: 77,
            observesContacts: true,
        });
        // an unobserved ball beside it is never a pair's side
        RigidBody.create(world, { shape: ball, objectLayer: scene.moving, motionType: MotionType.DYNAMIC, position: [3, 2, 0] });
        const pair = ContactPairs.create();
        let added = 0;
        let removed = 0;
        let handle = -1;
        for (let frame = 0; frame < 120; frame++) {
            // a frame of two steps
            ContactPairs.begin(world);
            for (let step = 0; step < 2; step++) {
                World.step(world, undefined, 1 / 120);
                ContactPairs.recordWorldContacts(world);
            }
            ContactPairs.end(world);
            for (let i = 0; i < ContactPairs.activeCount(world); i++) {
                expect(ContactPairs.readActive(pair, world, i)).toBe(true);
                expect(pair.bodyA === body || pair.bodyB === body).toBe(true);
                if (pair.added) {
                    added++;
                    handle = pair.handle;
                    expect(pair.bodyA === body ? pair.userDataA : pair.userDataB).toBe(77);
                } else {
                    // a pair keeps its handle while it lasts
                    expect(pair.handle).toBe(handle);
                }
            }
            removed += ContactPairs.removedCount(world);
        }
        expect(ContactPairs.readActive(pair, world, ContactPairs.activeCount(world))).toBe(false);
        expect(added).toBeGreaterThanOrEqual(2);
        expect(removed).toBeGreaterThanOrEqual(1);
        releaseScene(scene);
        Sphere.destroy(ball);
    });

    test("a character's contacts, through its inner body, its normal from the character into the floor", async () => {
        const scene = await makeScene();
        const { world } = scene;
        const capsule = Capsule.create({ halfHeightOfCylinder: 0.5, radius: 0.3 });
        const standing = Transformed.create({ shape: capsule, position: [0, 0.8, 0] });
        const character = KCC.create(
            { shape: standing, observesContacts: true, innerRigidBody: { shape: standing, objectLayer: scene.moving } },
            [0, 0.01, 0],
            [0, 0, 0, 1],
        );
        KCC.add(world, character);
        const filter = Filter.create(world);
        const gravity: Vec3 = [0, -10, 0];
        for (let frame = 0; frame < 10; frame++) {
            character.linearVelocity = [0, -0.2, 0];
            KCC.update(world, character, 1 / 60, gravity, createDefaultUpdateSettings(), undefined, filter);
        }
        ContactPairs.begin(world);
        KCC.refreshContacts(world, character, filter, undefined);
        ContactPairs.recordCharacterContacts(world, character);
        ContactPairs.end(world);
        expect(ContactPairs.activeCount(world)).toBe(1);
        const pair = ContactPairs.create();
        ContactPairs.readActive(pair, world, 0);
        expect(pair.bodyA).toBe(character.innerRigidBodyId);
        expect(pair.normal[1]).toBeLessThan(-0.99);
        KCC.remove(world, character);
        KCC.destroy(character);
        Filter.destroy(filter);
        releaseScene(scene);
        Transformed.destroy(standing);
        Capsule.destroy(capsule);
    });
});
