import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    createWorld,
    createWorldSettings,
    enableCollision,
    init,
    MotionQuality,
    MotionType,
    rigidBody,
    sphere,
    updateWorld,
} from 'tumble.js';
import { addBody, createBodies, drawBodies, readBodies } from './common/bodies';
import { createInfo } from './common/info';
import { loop } from './common/loop';
import { addOverlayControls, beginOverlay, createOverlay, endOverlay } from './common/overlay';
import { createPanel } from './common/panel';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';
import { palette } from './common/theme';

await init();
const renderer = await createRenderer([-31.96, 19.73, -27.86], [0, -0.4, 0]);
const worldSettings = createWorldSettings();
const moving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
const notMoving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
enableCollision(worldSettings, moving, moving);
enableCollision(worldSettings, moving, notMoving);
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world);

const GROUND_TOP = 0.1;
const ground = rigidBody.create(world, {
    shape: box.create({ halfExtents: [30, 0.1, 30] }),
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    restitution: 0,
});
addBody(bodies, ground);

// walls of bricks, each a pyramid of stacked rows, side by side along x
const brick = box.create({ halfExtents: [0.5, 0.5, 1] });
const ROWS = 8;
for (let wall = 0; wall < 5; wall++) {
    for (let i = 0; i < ROWS; i++) {
        for (let j = i; j < ROWS; j++) {
            const body = rigidBody.create(world, {
                shape: brick,
                objectLayer: moving,
                motionType: MotionType.DYNAMIC,
                position: [wall * 6, i + GROUND_TOP + 0.5, i + (j - i) * 2 - ROWS],
                restitution: 0,
            });
            addBody(bodies, body);
        }
    }
}

// a heavy ball fired through them: far further a step than its own size, so without its swept motion quality it would
// pass straight through the first walls between one step and the next
const ballShape = sphere.create({ radius: 1, density: 10000 });
fire();

function fire(): void {
    const body = rigidBody.create(world, {
        shape: ballShape,
        objectLayer: moving,
        motionType: MotionType.DYNAMIC,
        position: [-20, GROUND_TOP + 2.5, 0],
        restitution: 0,
        motionQuality: MotionQuality.LINEAR_CAST,
    });
    // 42 tonnes at 3.6 km/s, 60 m a step
    rigidBody.addImpulse(world, body, [150_000_000, 0, 0]);
    addBody(bodies, body, palette.light);
}

const panel = createPanel('continuous collision');
panel.button('fire another', fire);
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo('a ball fired at 3.6 km/s hits the walls because its linear cast motion quality sweeps it each step');

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
