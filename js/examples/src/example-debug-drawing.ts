import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    capsule,
    createWorld,
    createWorldSettings,
    emptyShape,
    enableCollision,
    hingeConstraint,
    init,
    MotionType,
    pointConstraint,
    rigidBody,
    sphere,
    updateWorld,
    type Vec3,
} from 'tumble.js';
import { addBody, createBodies, drawBodies, readBodies, setBodyVisible } from './common/bodies';
import { createInfo } from './common/info';
import { loop } from './common/loop';
import { addOverlayControls, beginOverlay, createOverlay, endOverlay } from './common/overlay';
import { createPanel } from './common/panel';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';

await init();
const renderer = await createRenderer([14, 10, 18], [0, 3, 0]);
const worldSettings = createWorldSettings();
const moving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
const notMoving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
enableCollision(worldSettings, moving, moving);
enableCollision(worldSettings, moving, notMoving);
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
// the library's own drawing, all on to start with
const overlay = createOverlay(renderer, world, { wireframes: true, contacts: true, joints: true });

const ground = rigidBody.create(world, {
    shape: box.create({ halfExtents: [12, 0.5, 12] }),
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    position: [0, -0.5, 0],
});
addBody(bodies, ground);

// a chain of planks hinged end to end from a fixed anchor, swinging down
const plank = box.create({ halfExtents: [0.5, 0.1, 0.25] });
let previous = rigidBody.create(world, {
    shape: emptyShape.create(),
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    position: [-4, 8, 0],
});
for (let i = 0; i < 8; i++) {
    const link = rigidBody.create(world, {
        shape: plank,
        objectLayer: moving,
        motionType: MotionType.DYNAMIC,
        position: [-3.5 + i, 8, 0],
    });
    addBody(bodies, link);
    const at: Vec3 = [-4 + i, 8, 0];
    hingeConstraint.create(world, previous, link, {
        pointA: at,
        pointB: at,
        hingeAxisA: [0, 0, 1],
        hingeAxisB: [0, 0, 1],
        normalAxisA: [1, 0, 0],
        normalAxisB: [1, 0, 0],
    });
    previous = link;
}

// a pendulum, and a pile for the contacts
const pivot = rigidBody.create(world, {
    shape: emptyShape.create(),
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    position: [5, 7, 0],
});
const bob = rigidBody.create(world, {
    shape: sphere.create({ radius: 0.6 }),
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [8, 7, 0],
});
addBody(bodies, bob);
pointConstraint.create(world, pivot, bob, { pointA: [5, 7, 0], pointB: [5, 7, 0] });
const crate = box.create({ halfExtents: [0.4, 0.4, 0.4] });
const pill = capsule.create({ halfHeightOfCylinder: 0.4, radius: 0.3 });
for (let i = 0; i < 24; i++) {
    const body = rigidBody.create(world, {
        shape: i % 2 === 0 ? crate : pill,
        objectLayer: moving,
        motionType: MotionType.DYNAMIC,
        position: [2 + (i % 3) * 0.3, 1 + i * 0.9, 4 + (i % 4) * 0.25],
    });
    addBody(bodies, body);
}

const settings = { solids: true };
const panel = createPanel('debug drawing');
panel.add(settings, 'solids').onChange((solids) => {
    for (const body of bodies.drawn.keys()) setBodyVisible(bodies, body, solids);
});
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo("the library's own debug drawing of wireframes, velocities, contacts and joints, read from debugDraw every frame");

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
