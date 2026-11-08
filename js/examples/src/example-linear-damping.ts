import {
    addBroadphaseLayer,
    addObjectLayer,
    type BodyId,
    box,
    createWorld,
    createWorldSettings,
    enableCollision,
    init,
    MotionType,
    rigidBody,
    updateWorld,
    type Vec3,
} from 'tumble.js';
import { addBody, createBodies, drawBodies, readBodies, snapBody } from './common/bodies';
import { createInfo } from './common/info';
import { createLabel, type Label, placeLabels } from './common/labels';
import { loop } from './common/loop';
import { addOverlayControls, beginOverlay, createOverlay, endOverlay } from './common/overlay';
import { createPanel } from './common/panel';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';

await init();
const renderer = await createRenderer([0, 8, 25], [0, 2, 0]);
// no gravity: each box slides on, slowed by its damping alone
const worldSettings = createWorldSettings();
const moving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
enableCollision(worldSettings, moving, moving);
worldSettings.gravity = [0, 0, 0];
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world);

const COUNT = 10;
const START_SPEED = 8;
const crate = box.create({ halfExtents: [0.5, 0.5, 0.5] });
const boxes: { body: BodyId; start: Vec3 }[] = [];
const labels: Label[] = [];
for (let i = 0; i < COUNT; i++) {
    const damping = i * 0.1;
    const start: Vec3 = [-10 + 2 * i, 0.5, 0];
    const body = rigidBody.create(world, {
        shape: crate,
        objectLayer: moving,
        motionType: MotionType.DYNAMIC,
        position: start,
        linearDamping: damping,
    });
    addBody(bodies, body);
    boxes.push({ body, start });
    labels.push(createLabel(`damping ${damping.toFixed(1)}`, [start[0], 2, 0]));
}

function launch(): void {
    for (const { body, start } of boxes) {
        rigidBody.setPosition(world, body, start, true);
        snapBody(bodies, body);
        rigidBody.setLinearVelocity(world, body, [0, 0, START_SPEED]);
        rigidBody.setAngularVelocity(world, body, [0, 0, 0]);
    }
}
launch();

// once every box has stopped or slid out of renderer, a second later they go again
const velocity: Vec3 = [0, 0, 0];
const position: Vec3 = [0, 0, 0];
let stoppedSteps = 0;
function relaunchWhenStopped(): void {
    const stopped = boxes.every(({ body }) => {
        rigidBody.getLinearVelocity(velocity, world, body);
        rigidBody.getPosition(position, world, body);
        return Math.abs(velocity[2]) < 0.01 || position[2] > 15;
    });
    stoppedSteps = stopped ? stoppedSteps + 1 : 0;
    if (stoppedSteps > 60) {
        stoppedSteps = 0;
        launch();
    }
}

const panel = createPanel('linear damping');
panel.button('launch', launch);
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo('the boxes are launched at the same speed with no gravity, with linear damping from none to 0.9');

loop(
    stepTime,
    (deltaTime) => {
        updateWorld(world, undefined, deltaTime);
        readBodies(bodies);
        relaunchWhenStopped();
    },
    (alpha) => {
        drawBodies(bodies, alpha);
        beginOverlay(overlay);
        endOverlay(overlay);
        render(renderer);
        placeLabels(renderer, labels);
    },
);
