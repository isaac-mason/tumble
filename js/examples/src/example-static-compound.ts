import { quat } from 'math';
import { mulberry32 } from 'math/random';
import {
    addBroadphaseLayer,
    addObjectLayer,
    type BodyId,
    box,
    type CompoundChild,
    createWorld,
    createWorldSettings,
    enableCollision,
    init,
    MotionType,
    rigidBody,
    sphere,
    staticCompound,
    updateWorld,
    type Vec3,
} from 'tumble.js';
import { addBody, createBodies, drawBodies, readBodies, removeBody } from './common/bodies';
import { createInfo } from './common/info';
import { loop } from './common/loop';
import { addOverlayControls, beginOverlay, createOverlay, endOverlay } from './common/overlay';
import { createPanel } from './common/panel';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';

await init();
const renderer = await createRenderer([0, 22, 30], [0, 8, 0]);
const worldSettings = createWorldSettings();
const moving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
const notMoving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
enableCollision(worldSettings, moving, moving);
enableCollision(worldSettings, moving, notMoving);
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world);

const rng = mulberry32.create(9);
const random = () => mulberry32.sample(rng);

// a bridge of beams as one static compound: a tree over its children finds the few a body touches
const LENGTH = 36;
const WIDTH = 3;
const HEIGHT = 8;
const BEAM = 0.12;
const DECK = 0.15;
const beam = (halfExtents: Vec3) => box.create({ halfExtents, convexRadius: 0.01 });
const children: CompoundChild[] = [];

const PLANKS = 12;
const plank = beam([LENGTH / PLANKS / 2 + 0.05, DECK / 2, WIDTH / 2]);
for (let i = 0; i < PLANKS; i++)
    children.push({ shape: plank, position: [-LENGTH / 2 + (i / (PLANKS - 1)) * LENGTH, HEIGHT, 0] });

const rail = beam([LENGTH / 2, BEAM, BEAM]);
const post = beam([BEAM, 0.6, BEAM]);
for (const side of [-1, 1]) {
    const z = side * (WIDTH / 2 + 0.2);
    children.push({ shape: rail, position: [0, HEIGHT + 0.3, z] }, { shape: rail, position: [0, HEIGHT + 1.2, z] });
    for (let i = 0; i < 9; i++) children.push({ shape: post, position: [-LENGTH / 2 + (i / 8) * LENGTH, HEIGHT + 0.75, z] });
}

const TRUSSES = 8;
const SUPPORT_HEIGHT = 3.5;
const support = beam([BEAM * 1.5, SUPPORT_HEIGHT / 2, BEAM * 1.5]);
for (let i = 0; i < TRUSSES; i++) {
    const x = -LENGTH / 2 + (i / (TRUSSES - 1)) * LENGTH;
    for (const side of [-1, 1])
        children.push({ shape: support, position: [x, HEIGHT - SUPPORT_HEIGHT / 2, side * (WIDTH / 2 - 0.3)] });
    if (i === TRUSSES - 1) continue;
    // an x of braces to the next truss, tilted down across the bridge
    const nextX = -LENGTH / 2 + ((i + 1) / (TRUSSES - 1)) * LENGTH;
    const span = nextX - x;
    const brace = beam([Math.sqrt(span ** 2 + WIDTH ** 2 + 1.5 ** 2) / 2, BEAM * 0.8, BEAM * 0.8]);
    for (const turn of [1, -1]) {
        const quaternion = quat.fromEuler([0, 0, 0, 1], [Math.atan2(1.5, span), turn * Math.atan2(WIDTH, span), 0, 'xyz']);
        children.push({ shape: brace, position: [(x + nextX) / 2, HEIGHT - 1.5, 0], quaternion });
    }
}

const keel = beam([LENGTH / 2, BEAM * 1.5, BEAM * 1.5]);
for (const side of [-1, 1]) children.push({ shape: keel, position: [0, HEIGHT - SUPPORT_HEIGHT, side * (WIDTH / 2 - 0.3)] });
const crossBeam = beam([BEAM, BEAM * 1.5, WIDTH / 2 - 0.3]);
for (let i = 0; i < 9; i++)
    children.push({ shape: crossBeam, position: [-LENGTH / 2 + (i / 8) * LENGTH, HEIGHT - SUPPORT_HEIGHT, 0] });

const bridge = rigidBody.create(world, {
    shape: staticCompound.create({ children }),
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    restitution: 0.3,
});
addBody(bodies, bridge);

// hollow boxes dropped onto it, each a static compound too: eight balls at a cube's corners
const corner = sphere.create({ radius: 0.12 });
const cornerBox = staticCompound.create({
    children: [-1, 1].flatMap((x) =>
        [-1, 1].flatMap((y) => [-1, 1].map((z) => ({ shape: corner, position: [x * 0.25, y * 0.25, z * 0.25] as Vec3 }))),
    ),
});

function spawn(): BodyId {
    const body = rigidBody.create(world, {
        shape: cornerBox,
        objectLayer: moving,
        motionType: MotionType.DYNAMIC,
        position: [(random() - 0.5) * (LENGTH - 4), HEIGHT + 5 + random() * 3, (random() - 0.5) * (WIDTH - 0.5)],
        restitution: 0.3,
    });
    addBody(bodies, body);
    return body;
}

function despawn(body: BodyId): void {
    removeBody(bodies, body);
    rigidBody.remove(world, body);
}

const settings = { boxes: 100, stepsBetweenRespawns: 12 };
const boxes: BodyId[] = [];
function matchCount(): void {
    while (boxes.length < settings.boxes) boxes.push(spawn());
    while (boxes.length > settings.boxes) despawn(boxes.pop()!);
}
matchCount();

// in turn, one box is taken away and a new one dropped, so they keep falling onto the bridge
let steps = 0;
let respawnIndex = 0;
function respawnInTurn(): void {
    if (boxes.length === 0 || ++steps % settings.stepsBetweenRespawns !== 0) return;
    const index = respawnIndex++ % boxes.length;
    despawn(boxes[index]!);
    boxes[index] = spawn();
}

const panel = createPanel('static compound');
panel.add(settings, 'boxes', { min: 0, max: 500, step: 1 }).onChange(matchCount);
panel.add(settings, 'stepsBetweenRespawns', { min: 3, max: 120, step: 3, label: 'steps a respawn' });
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo(`a bridge of ${children.length} beams in one static compound, with boxes of eight balls raining down on it`);

loop(
    stepTime,
    (deltaTime) => {
        updateWorld(world, undefined, deltaTime);
        readBodies(bodies);
        respawnInTurn();
    },
    (alpha) => {
        drawBodies(bodies, alpha);
        beginOverlay(overlay);
        endOverlay(overlay);
        render(renderer);
    },
);
