import {
    addBroadphaseLayer,
    addObjectLayer,
    type BodyId,
    box,
    capsule,
    createWorld,
    createWorldSettings,
    Dof,
    enableCollision,
    init,
    MotionType,
    type Quat,
    rigidBody,
    type Shape,
    sphere,
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
const worldSettings = createWorldSettings();
const moving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
const notMoving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
enableCollision(worldSettings, moving, moving);
enableCollision(worldSettings, moving, notMoving);
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world);

// a long floor of full friction: the pair's friction is the bodies' combined, so each body's own sets how it slides
const floor = rigidBody.create(world, {
    shape: box.create({ halfExtents: [50, 0.5, 300] }),
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    position: [0, -0.5, 0],
    restitution: 0,
    friction: 1,
});
addBody(bodies, floor);

const COUNT = 10;
const SPACING = 2;
const ROW_WIDTH = (COUNT - 1) * SPACING;
const SPEED = 10;
// lying on its side, to roll
const SIDEWAYS: Quat = [0, 0, Math.SQRT1_2, Math.SQRT1_2];

type Row = { shape: Shape; quaternion: Quat; allowedDegreesOfFreedom: number; firstX: number };
const rows: Row[] = [
    // spheres roll, so friction only spins them up
    {
        shape: sphere.create({ radius: 0.5 }),
        quaternion: [0, 0, 0, 1],
        allowedDegreesOfFreedom: Dof.ALL,
        firstX: -ROW_WIDTH * 1.5 - SPACING,
    },
    // boxes that cannot turn slide, so friction slows them
    {
        shape: box.create({ halfExtents: [0.5, 0.5, 0.5] }),
        quaternion: [0, 0, 0, 1],
        allowedDegreesOfFreedom: Dof.TRANSLATION_ONLY,
        firstX: -ROW_WIDTH / 2,
    },
    {
        shape: capsule.create({ halfHeightOfCylinder: 0.3, radius: 0.5 }),
        quaternion: SIDEWAYS,
        allowedDegreesOfFreedom: Dof.ALL,
        firstX: ROW_WIDTH / 2 + SPACING,
    },
];

const sliders: { body: BodyId; start: Vec3; quaternion: Quat }[] = [];
const labels: Label[] = [];
for (const row of rows) {
    for (let i = 0; i < COUNT; i++) {
        const friction = i / (COUNT - 1);
        const start: Vec3 = [row.firstX + SPACING * i, 0.5, 0];
        const body = rigidBody.create(world, {
            shape: row.shape,
            objectLayer: moving,
            motionType: MotionType.DYNAMIC,
            position: start,
            quaternion: row.quaternion,
            allowedDegreesOfFreedom: row.allowedDegreesOfFreedom,
            restitution: 0,
            friction,
        });
        addBody(bodies, body);
        sliders.push({ body, start, quaternion: row.quaternion });
        labels.push(createLabel(friction.toFixed(1), [start[0], 2, 0]));
    }
}

function launch(): void {
    for (const { body, start, quaternion } of sliders) {
        rigidBody.setTransform(world, body, start, quaternion, true);
        snapBody(bodies, body);
        rigidBody.setLinearVelocity(world, body, [0, 0, SPEED]);
        rigidBody.setAngularVelocity(world, body, [0, 0, 0]);
    }
}
launch();

const panel = createPanel('friction');
panel.button('launch', launch);
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo('spheres, boxes that cannot rotate and capsules on their side, launched at 10 m/s with friction from 0 to 1');

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
