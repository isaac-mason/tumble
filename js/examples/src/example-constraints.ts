import { type Vec3, vec3 } from 'math';
import {
    addBroadphaseLayer,
    addObjectLayer,
    type BodyId,
    box,
    ConstraintSpace,
    coneConstraint,
    createWorld,
    createWorldSettings,
    distanceConstraint,
    enableCollision,
    fixedConstraint,
    hingeConstraint,
    init,
    type Listener,
    MotionType,
    pointConstraint,
    rigidBody,
    sixDOFConstraint,
    sliderConstraint,
    swingTwistConstraint,
    updateWorld,
} from 'tumble.js';
import { addBody, createBodies, drawBodies, readBodies } from './common/bodies';
import { createInfo } from './common/info';
import { createLabel, type Label, placeLabels } from './common/labels';
import { loop } from './common/loop';
import { addOverlayControls, beginOverlay, createOverlay, endOverlay } from './common/overlay';
import { createPanel } from './common/panel';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';
import { spectrum } from './common/theme';

await init();
const renderer = await createRenderer([10, 30, 30], [0, 5, 0]);
const worldSettings = createWorldSettings();
const moving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
const notMoving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
enableCollision(worldSettings, moving, moving);
enableCollision(worldSettings, moving, notMoving);
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world, { joints: true });

const ground = rigidBody.create(world, {
    shape: box.create({ halfExtents: [50, 0.5, 10] }),
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    position: [0, -0.5, 0],
    restitution: 0,
});
addBody(bodies, ground);

// joined bodies overlap at their joints, so the pairs a constraint joins are kept from colliding
const listener: Listener = { onBodyPairValidate: (a, b) => !rigidBody.bodiesShareConstraint(world, a, b) };

// a chain of ten boxes for each kind of constraint, hung from a static first box along z and knocked sideways by an
// impulse on the last. world space: every point and axis is given in the world
const LENGTH = 10;
const HEIGHT = 20;
const link = box.create({ halfExtents: [0.5, 0.5, 0.5], convexRadius: 0.05 });
const labels: Label[] = [];

function chain(x: number, name: string, color: string, connect: (a: BodyId, b: BodyId, z: number) => void): void {
    labels.push(createLabel(name, [x, HEIGHT + 2, LENGTH / 2]));
    let previous: BodyId | null = null;
    for (let z = 0; z < LENGTH; z++) {
        const first = z === 0;
        const body = rigidBody.create(world, {
            shape: link,
            objectLayer: first ? notMoving : moving,
            motionType: first ? MotionType.STATIC : MotionType.DYNAMIC,
            position: [x, HEIGHT, z],
            restitution: 0,
        });
        addBody(bodies, body, first ? undefined : color);
        if (previous !== null) connect(previous, body, z);
        if (z === LENGTH - 1) rigidBody.addImpulse(world, body, [1000, 0, 0]);
        previous = body;
    }
}

const space = ConstraintSpace.WORLD;
const between = (x: number, z: number): Vec3 => [x, HEIGHT, z - 0.5];

// welded
chain(-15, 'fixed', spectrum[0], (a, b, z) => {
    fixedConstraint.create(world, a, b, { space, pointA: between(-15, z), pointB: between(-15, z) });
});
// a ball and socket
chain(-10, 'point', spectrum[1], (a, b, z) => {
    pointConstraint.create(world, a, b, { space, pointA: between(-10, z), pointB: between(-10, z) });
});
// a rope: no further apart than 1
chain(-5, 'distance', spectrum[2], (a, b, z) => {
    distanceConstraint.create(world, a, b, {
        space,
        pointA: between(-5, z),
        pointB: between(-5, z),
        minDistance: 0,
        maxDistance: 1,
    });
});
// turning about x, at the links' bottom edges
chain(0, 'hinge', spectrum[3], (a, b, z) => {
    const at: Vec3 = [0, HEIGHT - 0.5, z - 0.5];
    hingeConstraint.create(world, a, b, {
        space,
        pointA: at,
        pointB: at,
        hingeAxisA: [1, 0, 0],
        hingeAxisB: [1, 0, 0],
        normalAxisA: [0, 1, 0],
        normalAxisB: [0, 1, 0],
    });
});
// sliding down and along, up to 1 apart
const slideAxis = vec3.normalize([0, 0, 0], [0, -1, 1]);
const slideNormal = vec3.perpendicular([0, 0, 0], slideAxis);
chain(5, 'slider', spectrum[4], (a, b, z) => {
    sliderConstraint.create(world, a, b, {
        space,
        pointA: between(5, z),
        pointB: between(5, z),
        sliderAxisA: slideAxis,
        sliderAxisB: slideAxis,
        normalAxisA: slideNormal,
        normalAxisB: slideNormal,
        limitsMin: 0,
        limitsMax: 1,
    });
});
// the links' z axes kept within 18 degrees of each other, twisting freely
chain(10, 'cone', spectrum[5], (a, b, z) => {
    const at: Vec3 = [10, HEIGHT, z];
    coneConstraint.create(world, a, b, {
        space,
        pointA: at,
        pointB: at,
        twistAxisA: [0, 0, 1],
        twistAxisB: [0, 0, 1],
        halfConeAngle: 0.1 * Math.PI,
    });
});
// swing limited apart about two axes, and twist about z within its own range
chain(15, 'swing twist', spectrum[6], (a, b, z) => {
    const at: Vec3 = [15, HEIGHT, z];
    swingTwistConstraint.create(world, a, b, {
        space,
        pointA: at,
        pointB: at,
        twistAxisA: [0, 0, 1],
        twistAxisB: [0, 0, 1],
        planeAxisA: [1, 0, 0],
        planeAxisB: [1, 0, 0],
        normalHalfConeAngle: 0.2 * Math.PI,
        planeHalfConeAngle: 0.1 * Math.PI,
        twistMinAngle: -0.3 * Math.PI,
        twistMaxAngle: 0.4 * Math.PI,
    });
});
// every axis limited: 0.1 m of play, and 10 degrees
const turn = (10 * Math.PI) / 180;
chain(20, 'six dof', spectrum[0], (a, b, z) => {
    const at: Vec3 = [20, HEIGHT, z];
    sixDOFConstraint.create(world, a, b, {
        space,
        pointA: at,
        pointB: at,
        axisXA: [0, 0, 1],
        axisXB: [0, 0, 1],
        axisYA: [1, 0, 0],
        axisYB: [1, 0, 0],
        limitMin: [-0.1, -0.1, -0.1, -turn, -turn, -turn],
        limitMax: [0.1, 0.1, 0.1, turn, turn, turn],
    });
});

const panel = createPanel('constraints');
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo('a chain of boxes for each kind of constraint, hanging from a static box and knocked sideways');

loop(
    stepTime,
    (deltaTime) => {
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
