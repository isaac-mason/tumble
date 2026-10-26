import { describe, expect, test } from 'vitest';
import {
    capsule,
    contactPairs,
    filter,
    kcc,
    MotionType,
    releaseShape,
    rigidBody,
    sphere,
    transformed,
    updateWorld,
    type Vec3,
} from '../src';
import { makeScene, releaseScene } from './helpers';

describe('contact pairs', () => {
    test('a bouncing ball observing its contacts: its pair with the floor added, removed in the air, and added again', async () => {
        const scene = await makeScene();
        const { world } = scene;
        const ball = sphere.create({ radius: 0.5 });
        const body = rigidBody.create(world, {
            shape: ball,
            objectLayer: scene.moving,
            motionType: MotionType.DYNAMIC,
            position: [0, 2, 0],
            restitution: 0.6,
            userData: 77,
            observesContacts: true,
        });
        // an unobserved ball beside it is never a pair's side
        rigidBody.create(world, { shape: ball, objectLayer: scene.moving, motionType: MotionType.DYNAMIC, position: [3, 2, 0] });
        const pair = contactPairs.create();
        let added = 0;
        let removed = 0;
        let handle = -1;
        for (let frame = 0; frame < 120; frame++) {
            // a frame of two steps
            contactPairs.begin(world);
            for (let step = 0; step < 2; step++) {
                updateWorld(world, undefined, 1 / 120);
                contactPairs.recordWorldContacts(world);
            }
            contactPairs.end(world);
            for (let i = 0; i < contactPairs.activeCount(world); i++) {
                expect(contactPairs.readActive(pair, world, i)).toBe(true);
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
            removed += contactPairs.removedCount(world);
        }
        expect(contactPairs.readActive(pair, world, contactPairs.activeCount(world))).toBe(false);
        expect(added).toBeGreaterThanOrEqual(2);
        expect(removed).toBeGreaterThanOrEqual(1);
        releaseScene(scene);
        releaseShape(ball);
    });

    test("a character's contacts, through its inner body, its normal from the character into the floor", async () => {
        const scene = await makeScene();
        const { world } = scene;
        const capsuleShape = capsule.create({ halfHeightOfCylinder: 0.5, radius: 0.3 });
        const standing = transformed.create({ shape: capsuleShape, position: [0, 0.8, 0] });
        const character = kcc.create(
            { shape: standing, observesContacts: true, innerRigidBody: { shape: standing, objectLayer: scene.moving } },
            [0, 0.01, 0],
            [0, 0, 0, 1],
        );
        kcc.add(world, character);
        const queryFilter = filter.create(world);
        const gravity: Vec3 = [0, -10, 0];
        for (let frame = 0; frame < 10; frame++) {
            character.linearVelocity = [0, -0.2, 0];
            kcc.update(world, character, 1 / 60, gravity, kcc.createDefaultUpdateSettings(), undefined, queryFilter);
        }
        contactPairs.begin(world);
        kcc.refreshContacts(world, character, queryFilter, undefined);
        contactPairs.recordCharacterContacts(world, character);
        contactPairs.end(world);
        expect(contactPairs.activeCount(world)).toBe(1);
        const pair = contactPairs.create();
        contactPairs.readActive(pair, world, 0);
        expect(pair.bodyA).toBe(character.innerRigidBodyId);
        expect(pair.normal[1]).toBeLessThan(-0.99);
        kcc.remove(world, character);
        kcc.release(character);
        filter.destroy(queryFilter);
        releaseScene(scene);
        releaseShape(standing);
        releaseShape(capsuleShape);
    });
});
