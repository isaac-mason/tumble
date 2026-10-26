import { describe, expect, test } from 'vitest';
import {
    type BodyId,
    box,
    createActiveBody,
    getActiveBodies,
    getActiveBodyAt,
    getNumActiveBodies,
    type Listener,
    MotionType,
    rigidBody,
    updateWorld,
    type Vec3,
} from '../src';
import { makeScene, releaseScene, type Scene } from './helpers';

/** a listener that writes down what it hears, as `+id` for an activation and `-id` for a deactivation */
function recording(): { heard: string[]; listener: Listener } {
    const heard: string[] = [];
    return {
        heard,
        listener: {
            onBodyActivated: (body) => heard.push(`+${body}`),
            onBodyDeactivated: (body) => heard.push(`-${body}`),
        },
    };
}

function addBox(scene: Scene, position: Vec3): BodyId {
    const shape = box.create({ halfExtents: [0.5, 0.5, 0.5] });
    return rigidBody.create(scene.world, { shape, objectLayer: scene.moving, motionType: MotionType.DYNAMIC, position });
}

describe('activation', () => {
    test('a change between steps is heard after the next step, before that step’s own', async () => {
        const scene = await makeScene();
        const { heard, listener } = recording();
        const boxBody = addBox(scene, [0, 0.5, 0]);
        // created active, heard at the first step
        updateWorld(scene.world, listener, 1 / 60);
        expect(heard).toEqual([`+${boxBody}`]);
        heard.length = 0;

        // put to sleep, then woken by the step: falling onto it from above
        rigidBody.sleep(scene.world, boxBody);
        const falling = addBox(scene, [0, 1.6, 0]);
        for (let i = 0; i < 60 && rigidBody.isSleeping(scene.world, boxBody); i++) updateWorld(scene.world, listener, 1 / 60);
        expect(heard.slice(0, 3)).toEqual([`-${boxBody}`, `+${falling}`, `+${boxBody}`]);
        releaseScene(scene);
    });

    test('a body that wakes and falls asleep within a frame of steps is heard both times', async () => {
        const scene = await makeScene();
        const boxBody = addBox(scene, [0, 0.5, 0]);
        for (let i = 0; i < 240 && !rigidBody.isSleeping(scene.world, boxBody); i++) updateWorld(scene.world, undefined, 1 / 60);
        expect(rigidBody.isSleeping(scene.world, boxBody)).toBe(true);

        // woken by hand and left to settle again, all before anything listens: the next step tells it all, in order
        rigidBody.wake(scene.world, boxBody);
        for (let i = 0; i < 240 && !rigidBody.isSleeping(scene.world, boxBody); i++) updateWorld(scene.world, undefined, 1 / 60);
        rigidBody.wake(scene.world, boxBody);
        rigidBody.sleep(scene.world, boxBody);
        const { heard, listener } = recording();
        updateWorld(scene.world, listener, 1 / 60);
        expect(heard).toEqual([`+${boxBody}`, `-${boxBody}`]);
        heard.length = 0;
        updateWorld(scene.world, listener, 1 / 60);
        expect(heard).toEqual([]);
        releaseScene(scene);
    });

    test('a hook can wake another body, and that is heard in the same delivery', async () => {
        const scene = await makeScene();
        const a = addBox(scene, [0, 0.5, 0]);
        const b = addBox(scene, [3, 0.5, 0]);
        updateWorld(scene.world, undefined, 1 / 60);
        rigidBody.sleep(scene.world, a);
        rigidBody.sleep(scene.world, b);
        updateWorld(scene.world, undefined, 1 / 60);
        const heard: BodyId[] = [];
        rigidBody.wake(scene.world, a);
        updateWorld(
            scene.world,
            {
                onBodyActivated: (body) => {
                    heard.push(body);
                    if (body === a) rigidBody.wake(scene.world, b);
                },
            },
            1 / 60,
        );
        expect(heard).toEqual([a, b]);
        releaseScene(scene);
    });

    test('a hook that throws ends the delivery, and the step throws it', async () => {
        const scene = await makeScene();
        addBox(scene, [0, 0.5, 0]);
        addBox(scene, [3, 0.5, 0]);
        let calls = 0;
        const throwing: Listener = {
            onBodyActivated: () => {
                calls++;
                throw new Error('no');
            },
        };
        expect(() => updateWorld(scene.world, throwing, 1 / 60)).toThrow('no');
        expect(calls).toBe(1);
        // what was not heard is gone with the step
        calls = 0;
        updateWorld(scene.world, throwing, 1 / 60);
        expect(calls).toBe(0);
        releaseScene(scene);
    });
});

describe('active bodies', () => {
    test('read after a step: every active body, in body index order, as the getters have it', async () => {
        const scene = await makeScene();
        const boxes: BodyId[] = [];
        for (let i = 0; i < 40; i++) boxes.push(addBox(scene, [(i % 8) * 2 - 8, 1 + Math.floor(i / 8) * 2, 0]));
        for (let i = 0; i < 20; i++) updateWorld(scene.world, undefined, 1 / 60);
        rigidBody.sleep(scene.world, boxes[3]!);

        const count = getActiveBodies(scene.world);
        expect(count).toBe(getNumActiveBodies(scene.world));
        expect(count).toBe(39);
        const activeBody = createActiveBody();
        const position: Vec3 = [0, 0, 0];
        const velocity: Vec3 = [0, 0, 0];
        let lastIndex = -1;
        for (let i = 0; i < count; i++) {
            getActiveBodyAt(activeBody, scene.world, i);
            expect(activeBody.body).not.toBe(boxes[3]);
            const index = activeBody.body & 0x7fffff;
            expect(index).toBeGreaterThan(lastIndex);
            lastIndex = index;
            rigidBody.getPosition(position, scene.world, activeBody.body);
            rigidBody.getLinearVelocity(velocity, scene.world, activeBody.body);
            expect(activeBody.position).toEqual(position);
            expect(activeBody.linearVelocity).toEqual(velocity);
        }
        releaseScene(scene);
    });

    test('a world at rest reads nothing', async () => {
        const scene = await makeScene();
        const boxBody = addBox(scene, [0, 0.5, 0]);
        for (let i = 0; i < 240 && !rigidBody.isSleeping(scene.world, boxBody); i++) updateWorld(scene.world, undefined, 1 / 60);
        expect(getActiveBodies(scene.world)).toBe(0);
        releaseScene(scene);
    });
});
