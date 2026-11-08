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
const renderer = await createRenderer([0, 8, 15], [0, 2, 0]);
const worldSettings = createWorldSettings();
const moving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
const notMoving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
enableCollision(worldSettings, moving, moving);
enableCollision(worldSettings, moving, notMoving);
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world);

const floor = rigidBody.create(world, {
    shape: box.create({ halfExtents: [20, 0.5, 10] }),
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    position: [0, -0.5, 0],
    restitution: 0.3,
});
addBody(bodies, floor);

// a body's gravity factor scales the world's gravity for it: 0 floats, 1 falls as usual, 2 falls twice as hard
const COUNT = 11;
const crate = box.create({ halfExtents: [0.5, 0.5, 0.5] });
const boxes: { body: BodyId; start: Vec3; gravityFactor: number }[] = [];
const labels: Label[] = [];
for (let i = 0; i < COUNT; i++) {
    const gravityFactor = i * 0.2;
    const start: Vec3 = [-10 + 2 * i, 8, 0];
    const body = rigidBody.create(world, {
        shape: crate,
        objectLayer: moving,
        motionType: MotionType.DYNAMIC,
        position: start,
        gravityFactor,
        restitution: 0.3,
    });
    addBody(bodies, body);
    boxes.push({ body, start, gravityFactor });
    labels.push(createLabel(gravityFactor.toFixed(1), [start[0], 9.5, 0]));
}

function drop(): void {
    for (const { body, start } of boxes) {
        rigidBody.setTransform(world, body, start, [0, 0, 0, 1], true);
        snapBody(bodies, body);
        rigidBody.setLinearVelocity(world, body, [0, 0, 0]);
        rigidBody.setAngularVelocity(world, body, [0, 0, 0]);
    }
}

// once every box that falls has landed, two seconds later they drop again
const velocity: Vec3 = [0, 0, 0];
const position: Vec3 = [0, 0, 0];
let settledSteps = 0;
function dropAgainWhenLanded(): void {
    const landed = boxes.every(({ body, gravityFactor }) => {
        if (gravityFactor === 0) return true;
        rigidBody.getLinearVelocity(velocity, world, body);
        rigidBody.getPosition(position, world, body);
        return position[1] < 1.5 && Math.abs(velocity[1]) < 0.1;
    });
    settledSteps = landed ? settledSteps + 1 : 0;
    if (settledSteps > 120) {
        settledSteps = 0;
        drop();
    }
}

const panel = createPanel('gravity factor');
panel.button('drop', drop);
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo('boxes dropped together, each with its own gravity factor from 0 to 2');

loop(
    stepTime,
    (deltaTime) => {
        updateWorld(world, undefined, deltaTime);
        readBodies(bodies);
        dropAgainWhenLanded();
    },
    (alpha) => {
        drawBodies(bodies, alpha);
        beginOverlay(overlay);
        endOverlay(overlay);
        render(renderer);
        placeLabels(renderer, labels);
    },
);
