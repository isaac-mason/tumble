import {
    addBroadphaseLayer,
    addObjectLayer,
    type BodyId,
    box,
    ConstraintSpace,
    compound,
    createWorld,
    createWorldSettings,
    enableCollision,
    hingeConstraint,
    init,
    type Listener,
    MotionType,
    MotorState,
    rigidBody,
    sliderConstraint,
    sphere,
    updateWorld,
} from 'tumble.js';
import { addBody, createBodies, drawBodies, readBodies } from './common/bodies';
import { createInfo } from './common/info';
import { createLabel, placeLabels } from './common/labels';
import { loop } from './common/loop';
import { addOverlayControls, beginOverlay, createOverlay, endOverlay } from './common/overlay';
import { createPanel } from './common/panel';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';
import { palette, spectrum } from './common/theme';

await init();
const renderer = await createRenderer([0, 15, 35], [0, 5, 0]);
const worldSettings = createWorldSettings();
const moving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
const notMoving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
enableCollision(worldSettings, moving, moving);
enableCollision(worldSettings, moving, notMoving);
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world);

// the windmill's wings turn through its hub, and the slider's box through the target it is joined to
const listener: Listener = { onBodyPairValidate: (a, b) => !rigidBody.bodiesShareConstraint(world, a, b) };

function addStatic(halfExtents: [number, number, number], position: [number, number, number], color?: string): BodyId {
    const body = rigidBody.create(world, {
        shape: box.create({ halfExtents }),
        objectLayer: notMoving,
        motionType: MotionType.STATIC,
        position,
        restitution: 0,
    });
    addBody(bodies, body, color);
    return body;
}

addStatic([200, 0.5, 50], [0, -0.5, 0]);

// a windmill: four wings on a hub, hinged about z and driven at a steady speed by a velocity motor. a heavy ball rolled
// at it is batted up against the walls
const HUB: [number, number, number] = [-20, 10, 0];
const hub = addStatic([0.25, 0.25, 0.25], HUB, palette.light);
const wing = (halfExtents: [number, number, number]) => box.create({ halfExtents, convexRadius: 0.05 });
const across = wing([4, 0.5, 0.5]);
const upright = wing([0.5, 4, 0.5]);
const wings = rigidBody.create(world, {
    shape: compound.create({
        children: [
            { shape: across, position: [5, 0, 0] },
            { shape: across, position: [-5, 0, 0] },
            { shape: upright, position: [0, 5, 0] },
            { shape: upright, position: [0, -5, 0] },
        ],
    }),
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: HUB,
    restitution: 0,
});
addBody(bodies, wings, spectrum[0]);
const windmill = hingeConstraint.create(world, hub, wings, {
    space: ConstraintSpace.WORLD,
    pointA: HUB,
    pointB: HUB,
    hingeAxisA: [0, 0, 1],
    hingeAxisB: [0, 0, 1],
    normalAxisA: [1, 0, 0],
    normalAxisB: [1, 0, 0],
});
const settings = { windmillSpeed: -5 };
hingeConstraint.setMotorState(world, windmill, MotorState.VELOCITY);
hingeConstraint.setTargetAngularVelocity(world, windmill, settings.windmillSpeed);

const ball = rigidBody.create(world, {
    shape: sphere.create({ radius: 2 }),
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [HUB[0] - 20, 2, 0],
    restitution: 0.45,
});
addBody(bodies, ball, spectrum[1]);
rigidBody.setLinearVelocity(world, ball, [25, 0, 0]);
addStatic([0.2, 15, 5], [HUB[0] - 25, 15, 0]);
addStatic([0.2, 150, 50], [50, 0, 0]);
addStatic([150, 0.2, 50], [0, 75, 0]);

// a slider with a position motor: the box is driven along x to a distance from the target it is joined to, set to the
// next marker every second
const Z = 0;
addStatic([12.5, 0.25, 2], [18, 0.25, Z]);
const target = addStatic([0.5, 2, 2], [30, 2, Z], palette.light);
const sliding = rigidBody.create(world, {
    shape: box.create({ halfExtents: [0.75, 0.75, 0.75] }),
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [15, 1.25, Z],
    restitution: 0,
    allowSleeping: false,
});
addBody(bodies, sliding, spectrum[4]);
const MARKERS = [0, 4, 13, 21];
MARKERS.forEach((offset, i) => {
    addStatic([1, 4, 0.75], [28 - offset, 2, -2.75], spectrum[(i * 2 + 1) % spectrum.length]);
});
const slider = sliderConstraint.create(world, sliding, target, {
    space: ConstraintSpace.WORLD,
    pointA: [15, 1.25, Z],
    pointB: [28, 1.25, Z],
    sliderAxisA: [1, 0, 0],
    sliderAxisB: [1, 0, 0],
    normalAxisA: [0, 1, 0],
    normalAxisB: [0, 1, 0],
});
sliderConstraint.setMotorState(world, slider, MotorState.POSITION);
sliderConstraint.setTargetPosition(world, slider, 0);

// to and fro along the markers: 0, 1, 2, 3, 2, 1, 0, ...
let steps = 0;
let next = 4;
let sliderTarget = 0;
function nextMarker(): void {
    if (++steps % 60 !== 0) return;
    const index = Math.abs((next++ % (MARKERS.length * 2 - 2)) - (MARKERS.length - 1));
    sliderTarget = MARKERS[index]!;
    sliderConstraint.setTargetPosition(world, slider, sliderTarget);
}

const labels = [createLabel('velocity motor', [HUB[0], HUB[1] + 11, 0]), createLabel('position motor', [18, 7, 0])];

const panel = createPanel('constraint motors');
panel
    .add(settings, 'windmillSpeed', { min: -20, max: 20, step: 0.5, label: 'windmill rad/s' })
    .onChange((speed) => hingeConstraint.setTargetAngularVelocity(world, windmill, speed));
panel.monitor(() => sliderTarget, { label: 'slider target' });
panel.monitor(() => sliderConstraint.getCurrentPosition(world, slider), {
    label: 'slider at',
    format: (value) => value.toFixed(2),
});
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo('a hinge velocity motor turns the windmill, and a slider position motor moves the box between the markers');

loop(
    stepTime,
    (deltaTime) => {
        nextMarker();
        updateWorld(world, listener, deltaTime);
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
