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
    sphere,
    updateWorld,
} from 'tumble.js';
import { addBody, createBodies, drawBodies, readBodies, snapBody } from './common/bodies';
import { createInfo } from './common/info';
import { createLabel, placeLabels } from './common/labels';
import { loop } from './common/loop';
import { addOverlayControls, beginOverlay, createOverlay, endOverlay } from './common/overlay';
import { createPanel } from './common/panel';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';

await init();
const renderer = await createRenderer([0, 8, 20], [0, 3, 0]);
const worldSettings = createWorldSettings();
const moving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
const notMoving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
enableCollision(worldSettings, moving, moving);
enableCollision(worldSettings, moving, notMoving);
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world);

// a floor that gives nothing back, so each ball bounces by its own restitution alone
const floor = rigidBody.create(world, {
    shape: box.create({ halfExtents: [15, 1, 10] }),
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    restitution: 0,
    friction: 0.5,
});
addBody(bodies, floor);

const RESTITUTIONS = [0, 0.2, 0.4, 0.6, 0.8, 1];
const SPACING = 3;
const DROP_HEIGHT = 5;
const xOf = (i: number) => (i - (RESTITUTIONS.length - 1) / 2) * SPACING;

const ball = sphere.create({ radius: 0.5 });
const balls: BodyId[] = [];
const labels = RESTITUTIONS.map((restitution, i) => {
    const body = rigidBody.create(world, {
        shape: ball,
        objectLayer: moving,
        motionType: MotionType.DYNAMIC,
        position: [xOf(i), DROP_HEIGHT, 0],
        restitution,
        friction: 0.5,
    });
    addBody(bodies, body);
    balls.push(body);
    return createLabel(`e = ${restitution.toFixed(1)}`, [xOf(i), DROP_HEIGHT + 1.5, 0]);
});

function reset(): void {
    for (let i = 0; i < balls.length; i++) {
        const body = balls[i]!;
        rigidBody.setPosition(world, body, [xOf(i), DROP_HEIGHT, 0], true);
        snapBody(bodies, body);
        rigidBody.setLinearVelocity(world, body, [0, 0, 0]);
        rigidBody.setAngularVelocity(world, body, [0, 0, 0]);
    }
}
window.addEventListener('keydown', (event) => {
    if (event.key === 'r' || event.key === 'R') reset();
});

const panel = createPanel('restitution');
panel.button('reset [r]', reset);
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo("the higher a ball's restitution, the higher it bounces off the floor");

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
        placeLabels(renderer, labels);
    },
);
