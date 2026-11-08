import { mulberry32 } from 'math/random';
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
    plane,
    rigidBody,
    updateWorld,
} from 'tumble.js';
import { addBody, createBodies, drawBodies, readBodies, removeBody, snapBody } from './common/bodies';
import { createInfo } from './common/info';
import { loop } from './common/loop';
import { addOverlayControls, beginOverlay, createOverlay, endOverlay } from './common/overlay';
import { createPanel } from './common/panel';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';

await init();
const renderer = await createRenderer([0, 10, 10], [0, 5, 0]);
// strong gravity, for a lively heap
const worldSettings = createWorldSettings();
const moving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
const notMoving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
enableCollision(worldSettings, moving, moving);
enableCollision(worldSettings, moving, notMoving);
worldSettings.gravity = [0, -50, 0];
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world);

const rng = mulberry32.create(3);
const between = (min: number, max: number) => min + (max - min) * mulberry32.sample(rng);

const ground = rigidBody.create(world, {
    shape: plane.create({ normal: [0, 1, 0], constant: 0, halfExtent: 50 }),
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    restitution: 0,
});
addBody(bodies, ground);

const SPAWN_HEIGHT = 10;
const SPAWN_AREA = 2.5;
const cube = box.create({ halfExtents: [0.25, 0.25, 0.25], convexRadius: 0.05 });
const cubes: BodyId[] = [];
const settings = { cubes: 200 };

function matchCount(): void {
    while (cubes.length < settings.cubes) {
        const body = rigidBody.create(world, {
            shape: cube,
            objectLayer: moving,
            motionType: MotionType.DYNAMIC,
            position: [between(-SPAWN_AREA, SPAWN_AREA), between(0, SPAWN_HEIGHT), between(-SPAWN_AREA, SPAWN_AREA)],
            restitution: 0,
            mass: 1,
        });
        addBody(bodies, body);
        cubes.push(body);
    }
    while (cubes.length > settings.cubes) {
        const body = cubes.pop()!;
        removeBody(bodies, body);
        rigidBody.remove(world, body);
    }
}
matchCount();

// each step one cube is lifted back above the middle and dropped, so the heap never settles
function stir(): void {
    if (cubes.length === 0) return;
    const body = cubes[Math.floor(between(0, cubes.length))]!;
    rigidBody.setPosition(world, body, [0, between(0, SPAWN_HEIGHT), 0], true);
    snapBody(bodies, body);
    rigidBody.setLinearVelocity(world, body, [0, 0, 0]);
    rigidBody.setAngularVelocity(world, body, [0, 0, 0]);
}

const panel = createPanel('cube heap');
panel.add(settings, 'cubes', { min: 0, max: 2000, step: 10 }).onChange(matchCount);
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo('a heap of cubes under strong gravity, with one dropped back on top every step');

loop(
    stepTime,
    (deltaTime) => {
        stir();
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
