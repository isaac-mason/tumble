import { mulberry32 } from 'math/random';
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
    rigidBody,
    sphere,
    updateWorld,
} from 'tumble.js';
import { addBody, createBodies, drawBodies, readBodies, removeBody } from './common/bodies';
import { createInfo } from './common/info';
import { loop } from './common/loop';
import { addOverlayControls, beginOverlay, createOverlay, endOverlay } from './common/overlay';
import { createPanel } from './common/panel';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';
import { palette, spectrum } from './common/theme';

await init();
const renderer = await createRenderer([5, 5, 10], [0, 2, 0]);

// object layers, and which pairs of them collide: here debris meets the environment and itself, but not characters.
// the broadphase layers group object layers into trees of their own, what moves apart from what does not
const worldSettings = createWorldSettings();
const movingTree = addBroadphaseLayer(worldSettings);
const staticTree = addBroadphaseLayer(worldSettings);
const environment = addObjectLayer(worldSettings, staticTree);
const characters = addObjectLayer(worldSettings, movingTree);
const debris = addObjectLayer(worldSettings, movingTree);
enableCollision(worldSettings, environment, environment);
enableCollision(worldSettings, environment, characters);
enableCollision(worldSettings, environment, debris);
enableCollision(worldSettings, characters, characters);
enableCollision(worldSettings, debris, debris);
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world);

const rng = mulberry32.create(29);
const random = () => mulberry32.sample(rng);

const floor = rigidBody.create(world, {
    shape: box.create({ halfExtents: [5, 0.1, 5] }),
    objectLayer: environment,
    motionType: MotionType.STATIC,
});
addBody(bodies, floor);

const character = rigidBody.create(world, {
    shape: capsule.create({ halfHeightOfCylinder: 0.5, radius: 0.5 }),
    objectLayer: characters,
    motionType: MotionType.KINEMATIC,
    position: [0, 1.3, 0],
});
addBody(bodies, character, palette.light);

// debris rains over the character, passes through it, piles on the floor, and is gone five seconds later
const STEPS_BETWEEN = 6;
const LIFETIME_STEPS = 300;
const pebble = sphere.create({ radius: 0.05 });
const pebbles: { body: BodyId; born: number }[] = [];
let steps = 0;

function rain(): void {
    steps++;
    if (steps % STEPS_BETWEEN === 0) {
        const angle = random() * Math.PI * 2;
        const radius = random() * 1.5;
        const body = rigidBody.create(world, {
            shape: pebble,
            objectLayer: debris,
            motionType: MotionType.DYNAMIC,
            position: [Math.cos(angle) * radius, 5, Math.sin(angle) * radius],
            friction: 0.3,
            restitution: 0.2,
            allowedDegreesOfFreedom: Dof.TRANSLATION_ONLY,
        });
        addBody(bodies, body, spectrum[(steps / STEPS_BETWEEN) % spectrum.length]);
        pebbles.push({ body, born: steps });
    }
    while (pebbles.length > 0 && steps - pebbles[0]!.born > LIFETIME_STEPS) {
        const { body } = pebbles.shift()!;
        removeBody(bodies, body);
        rigidBody.remove(world, body);
    }
}

const panel = createPanel('object layer filtering');
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo('the debris collides with the floor and itself, but falls straight through the character');

loop(
    stepTime,
    (deltaTime) => {
        rain();
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
