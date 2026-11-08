import { type Vec3, vec3 } from 'math';
import { mulberry32 } from 'math/random';
import {
    addBroadphaseLayer,
    addObjectLayer,
    type BodyId,
    createWorld,
    createWorldSettings,
    enableCollision,
    init,
    MotionType,
    rigidBody,
    sphere,
    updateWorld,
} from 'tumble.js';
import { addBody, createBodies, drawBodies, readBodies } from './common/bodies';
import { createInfo } from './common/info';
import { loop } from './common/loop';
import { addOverlayControls, beginOverlay, createOverlay, endOverlay } from './common/overlay';
import { createPanel } from './common/panel';
import { createPointerRay, pointerRay } from './common/pointer';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';
import { palette, spectrum } from './common/theme';

await init();
const renderer = await createRenderer([0, 0, 20], [0, 0, 0]);
// the pointer moves the pusher, so the camera stays put
renderer.controls.enabled = false;
const worldSettings = createWorldSettings();
const moving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
enableCollision(worldSettings, moving, moving);
worldSettings.gravity = [0, 0, 0];
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world);

const rng = mulberry32.create(11);
const between = (min: number, max: number) => min + (max - min) * mulberry32.sample(rng);

// balls scattered far out to both sides, pulled together each step towards a point: the middle, or the pointer while
// it is held down. their damping takes out what the pull puts in, so they settle into a cluster
const COUNT = 100;
const COLORS = [spectrum[2], spectrum[1], palette.light];
const ball = sphere.create({ radius: 1 });
const balls: BodyId[] = [];
for (let i = 0; i < COUNT; i++) {
    const side = mulberry32.sample(rng) > 0.5 ? 1 : -1;
    const body = rigidBody.create(world, {
        shape: ball,
        objectLayer: moving,
        motionType: MotionType.DYNAMIC,
        position: [side * between(50, 100), between(-50, 50), between(-10, 10)],
        linearDamping: 2,
        angularDamping: 4,
        friction: 0.1,
    });
    addBody(bodies, body, COLORS[i % COLORS.length]);
    balls.push(body);
}

// a big kinematic ball under the pointer, on the plane z = 0, pushing its way through
const pusherPosition: Vec3 = [0, -100, 0];
const pusher = rigidBody.create(world, {
    shape: sphere.create({ radius: 5 }),
    objectLayer: moving,
    motionType: MotionType.KINEMATIC,
    position: pusherPosition,
    restitution: 0.8,
});

const ray = createPointerRay();
let held = false;
renderer.canvas.addEventListener('pointermove', (event) => {
    pointerRay(ray, renderer, event.clientX, event.clientY);
    // where the ray crosses z = 0
    vec3.scaleAndAdd(pusherPosition, ray.origin, ray.direction, -ray.origin[2] / ray.direction[2]);
});
renderer.canvas.addEventListener('pointerdown', () => {
    held = true;
});
window.addEventListener('pointerup', () => {
    held = false;
});

const settings = { pull: 0.2, heldPull: 0.3 };
const pullTo: Vec3 = [0, 0, 0];
const position: Vec3 = [0, 0, 0];
const towards: Vec3 = [0, 0, 0];

function step(deltaTime: number): void {
    rigidBody.moveKinematic(world, pusher, pusherPosition, [0, 0, 0, 1], deltaTime);
    if (held) vec3.copy(pullTo, pusherPosition);
    else vec3.zero(pullTo);
    const pull = held ? settings.heldPull : settings.pull;
    for (const body of balls) {
        rigidBody.getPosition(position, world, body);
        vec3.subtract(towards, pullTo, position);
        rigidBody.addLinearVelocity(world, body, vec3.scale(towards, towards, pull));
    }
    updateWorld(world, undefined, deltaTime);
}

const panel = createPanel('cluster');
panel.add(settings, 'pull', { min: 0, max: 1, step: 0.01 });
panel.add(settings, 'heldPull', { min: 0, max: 1, step: 0.01, label: 'pull when held' });
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo('move the pointer to push through the balls, and hold it down to pull them towards it');

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
