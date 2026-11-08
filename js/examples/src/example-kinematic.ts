import { type Quat, quat, type Vec3 } from 'math';
import { mulberry32 } from 'math/random';
import {
    addBroadphaseLayer,
    addObjectLayer,
    type BodyId,
    box,
    convexHull,
    createWorld,
    createWorldSettings,
    enableCollision,
    init,
    MotionType,
    rigidBody,
    sphere,
    updateWorld,
} from 'tumble.js';
import maxwellUrl from '../../../samples/assets/maxwell.glb?url';
import { addBody, createBodies, drawBodies, readBodies, removeBody, snapBody } from './common/bodies';
import { createInfo } from './common/info';
import { loop } from './common/loop';
import { loadModel } from './common/model';
import { addOverlayControls, beginOverlay, createOverlay, endOverlay } from './common/overlay';
import { createPanel } from './common/panel';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';
import { palette } from './common/theme';

await init();
const renderer = await createRenderer([0, 5, 10], [0, 3, 0]);
const worldSettings = createWorldSettings();
const moving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
const notMoving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
enableCollision(worldSettings, moving, moving);
enableCollision(worldSettings, moving, notMoving);
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world);

const rng = mulberry32.create(7);
const random = () => mulberry32.sample(rng);

const ground = rigidBody.create(world, {
    shape: box.create({ halfExtents: [50, 0.5, 50] }),
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    position: [0, -0.5, 0],
    restitution: 0.2,
});
addBody(bodies, ground);

// maxwell the cat, as the hull of its body's vertices: the model's first 345, a tenth of their size
const BODY_VERTICES = 345;
const SCALE = 0.1;
const maxwell = await loadModel(maxwellUrl);
const catPoints = maxwell.positions.slice(0, BODY_VERTICES * 3).map((value) => value * SCALE);

// a kinematic body is moved by you, not by forces: moveKinematic sets the velocity that takes it to a pose in a step, so
// it pushes what it meets as a moving body would, and nothing pushes it back
const RANGE = 2.5;
const SPEED = 2;
const TURN_SPEED = Math.PI * 1.5;
const catPosition: Vec3 = [0, 3, 0];
const catQuaternion: Quat = [0, 0, 0, 1];
const cat = rigidBody.create(world, {
    shape: convexHull.create({ positions: catPoints }),
    objectLayer: moving,
    motionType: MotionType.KINEMATIC,
    position: catPosition,
    restitution: 0.5,
    friction: 0.3,
});
addBody(bodies, cat, palette.light);

// balls rain onto it, the oldest removed past the most
const settings = { stepsBetweenBalls: 3, maxBalls: 100 };
const ballShape = sphere.create({ radius: 0.15 });
const balls: BodyId[] = [];
const ballPosition: Vec3 = [0, 0, 0];

function dropFromAbove(body: BodyId): void {
    rigidBody.setPosition(world, body, [(random() - 0.5) * RANGE * 2, 12, 0], true);
    snapBody(bodies, body);
    rigidBody.setLinearVelocity(world, body, [0, 0, 0]);
    rigidBody.setAngularVelocity(world, body, [0, 0, 0]);
}

function spawnBall(): void {
    while (balls.length >= settings.maxBalls) {
        const oldest = balls.shift()!;
        removeBody(bodies, oldest);
        rigidBody.remove(world, oldest);
    }
    const body = rigidBody.create(world, {
        shape: ballShape,
        objectLayer: moving,
        motionType: MotionType.DYNAMIC,
        restitution: 0.6,
        friction: 0.4,
    });
    dropFromAbove(body);
    addBody(bodies, body);
    balls.push(body);
}

let time = 0;
let steps = 0;
function step(deltaTime: number): void {
    time += deltaTime;
    catPosition[0] = Math.sin(time * SPEED) * RANGE;
    quat.setAxisAngle(catQuaternion, [0, 1, 0], time * TURN_SPEED);
    rigidBody.moveKinematic(world, cat, catPosition, catQuaternion, deltaTime);

    if (++steps % settings.stepsBetweenBalls === 0) spawnBall();
    updateWorld(world, undefined, deltaTime);

    // a ball that rolls away is dropped again
    for (const body of balls) {
        rigidBody.getPosition(ballPosition, world, body);
        if (ballPosition[0] * ballPosition[0] + ballPosition[2] * ballPosition[2] > 15 * 15) dropFromAbove(body);
    }
}

const panel = createPanel('kinematic');
panel.add(settings, 'stepsBetweenBalls', { min: 1, max: 30, step: 1, label: 'steps a ball' });
panel.add(settings, 'maxBalls', { min: 10, max: 500, step: 10, label: 'most balls' });
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo('a kinematic cat moved with moveKinematic knocks away the balls that fall on it');

loop(
    stepTime,
    (deltaTime) => {
        step(deltaTime);
        readBodies(bodies);
    },
    (alpha) => {
        drawBodies(bodies, alpha);
        beginOverlay(overlay);
        endOverlay(overlay);
        render(renderer);
    },
);
