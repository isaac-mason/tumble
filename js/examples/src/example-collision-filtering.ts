import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    createWorld,
    createWorldSettings,
    enableCollision,
    init,
    MotionType,
    rigidBody,
    updateWorld,
} from 'tumble.js';
import { addBody, createBodies, drawBodies, readBodies } from './common/bodies';
import { createInfo } from './common/info';
import { loop } from './common/loop';
import { addOverlayControls, beginOverlay, createOverlay, endOverlay } from './common/overlay';
import { createPanel } from './common/panel';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';
import { spectrum } from './common/theme';

await init();
const renderer = await createRenderer([10, 5, 10], [0, 0, 0]);
const worldSettings = createWorldSettings();
const moving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
const notMoving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
enableCollision(worldSettings, moving, moving);
enableCollision(worldSettings, moving, notMoving);
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world);

// two bodies collide when each is in a group the other's mask takes. bodies in neither group, as the ground is by
// default, are in every group and take every one
const GROUP_A = 1 << 0;
const GROUP_B = 1 << 1;
const COLOR_A = spectrum[1];
const COLOR_B = spectrum[6];

const ground = rigidBody.create(world, {
    shape: box.create({ halfExtents: [5, 0.1, 5] }),
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
});
addBody(bodies, ground);

// two platforms over the ground: the lower meets only group a, the upper only group b
const platform = box.create({ halfExtents: [1, 0.1, 1] });
for (const [y, group, color] of [
    [1, GROUP_A, COLOR_A],
    [2, GROUP_B, COLOR_B],
] as const) {
    const body = rigidBody.create(world, {
        shape: platform,
        objectLayer: notMoving,
        motionType: MotionType.STATIC,
        position: [0, y, 0],
        collisionGroups: group,
        collisionMask: group,
    });
    addBody(bodies, body, color);
}

// a block of small cubes above them, alternate rows in each group: group b lands on the upper platform, and group a
// falls through it onto the lower
const COUNT = 4;
const HALF = 0.1;
const cube = box.create({ halfExtents: [HALF, HALF, HALF] });
const offset = HALF * 2 * (COUNT / 2);
for (let j = 0; j < 4; j++) {
    for (let i = 0; i < COUNT; i++) {
        for (let k = 0; k < COUNT; k++) {
            const group = k % 2 === 0 ? GROUP_A : GROUP_B;
            const body = rigidBody.create(world, {
                shape: cube,
                objectLayer: moving,
                motionType: MotionType.DYNAMIC,
                position: [i * HALF * 2 - offset, j * HALF * 2 + 2.5, k * HALF * 2 - offset],
                collisionGroups: group,
                collisionMask: group,
                friction: 1,
                restitution: 0,
            });
            addBody(bodies, body, group === GROUP_A ? COLOR_A : COLOR_B);
        }
    }
}

const panel = createPanel('collision filtering');
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo(
    'the coral cubes and platform are in one group and the blue ones in another. each cube only collides with its own platform',
);

loop(
    stepTime,
    (deltaTime) => {
        updateWorld(world, undefined, deltaTime);
        readBodies(bodies);
    },
    (alpha) => {
        drawBodies(bodies, alpha);
        beginOverlay(overlay);
        endOverlay(overlay);
        render(renderer);
    },
);
