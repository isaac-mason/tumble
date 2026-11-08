import type { Vec3 } from 'math';
import {
    addBroadphaseLayer,
    addObjectLayer,
    type BodyId,
    box,
    createWorld,
    createWorldSettings,
    debug,
    enableCollision,
    init,
    type Listener,
    MotionType,
    rigidBody,
    updateWorld,
} from 'tumble.js';
import { addBody, createBodies, drawBodies, readBodies } from './common/bodies';
import { createInfo } from './common/info';
import { createLabel, placeLabels } from './common/labels';
import { loop } from './common/loop';
import { addOverlayControls, beginOverlay, createOverlay, endOverlay, overlayDraw } from './common/overlay';
import { createPanel } from './common/panel';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';
import { debugColor, spectrum } from './common/theme';

await init();
const renderer = await createRenderer([0, 5, 15], [0, 2, 0]);
const worldSettings = createWorldSettings();
const moving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
const notMoving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
enableCollision(worldSettings, moving, moving);
enableCollision(worldSettings, moving, notMoving);
worldSettings.gravity = [0, 0, 0];
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world);

// a dynamic box and a static one
const crate = box.create({ halfExtents: [0.5, 0.5, 0.5] });
for (const [x, motionType, objectLayer] of [
    [-2, MotionType.DYNAMIC, moving],
    [2, MotionType.STATIC, notMoving],
] as const) {
    const body = rigidBody.create(world, { shape: crate, objectLayer, motionType, position: [x, 1, 0], restitution: 0.3 });
    addBody(bodies, body, spectrum[x < 0 ? 5 : 2]);
}

// a kinematic sensor swung through them: it reports contacts and pushes nothing. a kinematic body meets only dynamic
// ones, unless it is made to collide with static and kinematic ones too
const sensorShape = box.create({ halfExtents: [1.5, 1, 1.5] });
const settings = { collideWithNonDynamic: false };
const position: Vec3 = [-8, 1, 0];
let sensor: BodyId = createSensor();

function createSensor(): BodyId {
    return rigidBody.create(world, {
        shape: sensorShape,
        objectLayer: moving,
        motionType: MotionType.KINEMATIC,
        position,
        sensor: true,
        collideKinematicVsNonDynamic: settings.collideWithNonDynamic,
    });
}

let touching = false;
const listener: Listener = {
    onContactAdded: (contact) => {
        if (contact.bodyIdA === sensor || contact.bodyIdB === sensor) touching = true;
    },
    onContactPersisted: (contact) => {
        if (contact.bodyIdA === sensor || contact.bodyIdB === sensor) touching = true;
    },
};

const label = createLabel('', [0, 3.5, 0]);
let time = 0;

const panel = createPanel('sensor');
panel.add(settings, 'collideWithNonDynamic', { label: 'meets static' }).onChange(() => {
    // set when a body is made
    rigidBody.remove(world, sensor);
    sensor = createSensor();
});
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo('a kinematic sensor swings through two boxes. it reports what it touches and pushes nothing');

const touchingColor = debugColor(spectrum[4]);
const clearColor = debugColor(spectrum[1]);

loop(
    stepTime,
    (deltaTime) => {
        time += deltaTime;
        position[0] = Math.sin(time) * 6;
        rigidBody.setPosition(world, sensor, position, false);
        touching = false;
        updateWorld(world, listener, deltaTime);
        readBodies(bodies);
    },
    (alpha) => {
        drawBodies(bodies, alpha);
        beginOverlay(overlay);
        debug.body(overlayDraw(overlay, true), world, sensor, { color: touching ? touchingColor : clearColor });
        endOverlay(overlay);
        render(renderer);
        label.element.textContent = touching ? 'touching' : 'clear';
        label.position[0] = position[0];
        placeLabels(renderer, [label]);
    },
);
