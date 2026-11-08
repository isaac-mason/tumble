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
const renderer = await createRenderer([0, 0, 25], [0, 0, 0]);
// no gravity and nothing to touch: each box spins down by its damping alone
const worldSettings = createWorldSettings();
const moving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
enableCollision(worldSettings, moving, moving);
worldSettings.gravity = [0, 0, 0];
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world);

const COUNT = 10;
const SPIN: Vec3 = [3, 5, 2];
const crate = box.create({ halfExtents: [0.75, 0.75, 0.75] });
const boxes: { body: BodyId; start: Vec3 }[] = [];
const labels: Label[] = [];
for (let i = 0; i < COUNT; i++) {
    const damping = i * 0.1;
    const start: Vec3 = [-13.5 + 3 * i, 0, 0];
    const body = rigidBody.create(world, {
        shape: crate,
        objectLayer: moving,
        motionType: MotionType.DYNAMIC,
        position: start,
        angularDamping: damping,
    });
    addBody(bodies, body);
    boxes.push({ body, start });
    labels.push(createLabel(damping.toFixed(1), [start[0], 2, 0]));
}

function spin(): void {
    for (const { body, start } of boxes) {
        rigidBody.setTransform(world, body, start, [0, 0, 0, 1], true);
        snapBody(bodies, body);
        rigidBody.setLinearVelocity(world, body, [0, 0, 0]);
        rigidBody.setAngularVelocity(world, body, SPIN);
    }
}
spin();

// once every box has stopped turning, a second later they spin again
const angular: Vec3 = [0, 0, 0];
let stoppedSteps = 0;
function respinWhenStopped(): void {
    const stopped = boxes.every(({ body }) => {
        rigidBody.getAngularVelocity(angular, world, body);
        return Math.hypot(angular[0], angular[1], angular[2]) < 0.05;
    });
    stoppedSteps = stopped ? stoppedSteps + 1 : 0;
    if (stoppedSteps > 60) {
        stoppedSteps = 0;
        spin();
    }
}

const panel = createPanel('angular damping');
panel.button('spin', spin);
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo('the boxes start spinning at the same speed with no gravity, with angular damping from none to 0.9');

loop(
    stepTime,
    (deltaTime) => {
        updateWorld(world, undefined, deltaTime);
        readBodies(bodies);
        respinWhenStopped();
    },
    (alpha) => {
        drawBodies(bodies, alpha);
        beginOverlay(overlay);
        endOverlay(overlay);
        render(renderer);
        placeLabels(renderer, labels);
    },
);
