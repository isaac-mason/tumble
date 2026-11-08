import { type Quat, quat, type Vec3, vec3 } from 'math';
import { mulberry32 } from 'math/random';
import {
    addBroadphaseLayer,
    addObjectLayer,
    type BodyId,
    box,
    capsule,
    compound,
    convexHull,
    createWorld,
    createWorldSettings,
    cylinder,
    enableCollision,
    init,
    MotionType,
    plane,
    rigidBody,
    type Shape,
    sphere,
    staticCompound,
    transformed,
    updateWorld,
} from 'tumble.js';
import { addBody, createBodies, drawBodies, readBodies, removeBody } from './common/bodies';
import { createInfo } from './common/info';
import { loop } from './common/loop';
import { addOverlayControls, beginOverlay, createOverlay, endOverlay } from './common/overlay';
import { createPanel } from './common/panel';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';
import { spectrum } from './common/theme';

await init();
const renderer = await createRenderer([0, 10, 30], [0, 5, 0]);
const worldSettings = createWorldSettings();
const moving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
const notMoving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
enableCollision(worldSettings, moving, moving);
enableCollision(worldSettings, moving, notMoving);
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world);

const rng = mulberry32.create(5);
const random = () => mulberry32.sample(rng);

const ground = rigidBody.create(world, {
    shape: plane.create({ normal: [0, 1, 0], constant: 0, halfExtent: 50 }),
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    restitution: 0,
});
addBody(bodies, ground);

// each kind of shape made once, and shared by every body of it
const ball = sphere.create({ radius: 0.5 });
const rail = box.create({ halfExtents: [0.1, 1.5, 0.1], convexRadius: 0.05 });
const rung = box.create({ halfExtents: [0.6, 0.1, 0.1], convexRadius: 0.05 });
const kinds: { shape: Shape; color: string }[] = [
    { shape: ball, color: spectrum[1] },
    { shape: box.create({ halfExtents: [0.5, 0.5, 0.5], convexRadius: 0.05 }), color: spectrum[5] },
    { shape: capsule.create({ halfHeightOfCylinder: 1, radius: 1 }), color: spectrum[3] },
    { shape: cylinder.create({ halfHeight: 0.75, radius: 0.5 }), color: spectrum[6] },
    {
        // two balls on a bar
        shape: compound.create({
            children: [
                { shape: ball, position: [-1.1, 0, 0] },
                { shape: ball, position: [1.1, 0, 0] },
                {
                    shape: capsule.create({ halfHeightOfCylinder: 0.8, radius: 0.3 }),
                    quaternion: [0, 0, Math.SQRT1_2, Math.SQRT1_2],
                },
            ],
        }),
        color: spectrum[4],
    },
    {
        // a static compound suits many children that never change, and needs no body to be static
        shape: staticCompound.create({
            children: [
                { shape: rail, position: [-0.5, 0, 0] },
                { shape: rail, position: [0.5, 0, 0] },
                ...[-0.75, -0.25, 0.25, 0.75].map((y) => ({ shape: rung, position: [0, y, 0] as Vec3 })),
            ],
        }),
        color: spectrum[2],
    },
    {
        // a heavy head moved to the top of the handle by a transformed shape, inside a compound
        shape: compound.create({
            children: [
                { shape: box.create({ halfExtents: [0.2, 1.5, 0.2], convexRadius: 0.05 }) },
                {
                    shape: transformed.create({
                        shape: box.create({ halfExtents: [0.8, 0.3, 0.3], convexRadius: 0.05, density: 10000 }),
                        position: [0, 1.5, 0],
                    }),
                },
            ],
        }),
        color: spectrum[0],
    },
    {
        shape: convexHull.create({
            positions: [-0.5, 0, -0.5, 0.5, 0, -0.5, 0.5, 0, 0.5, -0.5, 0, 0.5, 0, 1, 0],
            convexRadius: 0.05,
        }),
        color: spectrum[6],
    },
];

const SPAWN_HEIGHT = 15;
const SPAWN_AREA = 10;
const axis: Vec3 = [0, 0, 0];

function spawn(): BodyId {
    const kind = kinds[Math.floor(random() * kinds.length)]!;
    vec3.normalize(axis, vec3.set(axis, random() - 0.5, random() - 0.5, random() - 0.5));
    const quaternion: Quat = quat.setAxisAngle([0, 0, 0, 1], axis, random() * Math.PI * 2);
    const body = rigidBody.create(world, {
        shape: kind.shape,
        objectLayer: moving,
        motionType: MotionType.DYNAMIC,
        position: [(random() * 2 - 1) * SPAWN_AREA, SPAWN_HEIGHT, (random() * 2 - 1) * SPAWN_AREA],
        quaternion,
        restitution: 0,
    });
    addBody(bodies, body, kind.color);
    return body;
}

function despawn(body: BodyId): void {
    removeBody(bodies, body);
    rigidBody.remove(world, body);
}

const settings = { bodies: 50, stepsBetweenRespawns: 30 };
const spawned: BodyId[] = [];

function matchCount(): void {
    while (spawned.length < settings.bodies) spawned.push(spawn());
    while (spawned.length > settings.bodies) despawn(spawned.pop()!);
}
matchCount();

// one body at a time, in turn, is taken away and a new one dropped in its place
let steps = 0;
let respawnIndex = 0;
function respawnInTurn(): void {
    if (spawned.length === 0 || ++steps % settings.stepsBetweenRespawns !== 0) return;
    const index = respawnIndex++ % spawned.length;
    despawn(spawned[index]!);
    spawned[index] = spawn();
}

const panel = createPanel('shapes');
panel.add(settings, 'bodies', { min: 0, max: 1000, step: 1 }).onChange(matchCount);
panel.add(settings, 'stepsBetweenRespawns', { min: 6, max: 300, step: 6, label: 'steps a respawn' });
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo('every shape type falling onto a plane, including compounds, a transformed shape and a convex hull');

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
