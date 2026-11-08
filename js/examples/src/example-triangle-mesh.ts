import { quat, type Vec3, vec3 } from 'math';
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
    rigidBody,
    type Shape,
    sphere,
    triangleMesh,
    updateWorld,
} from 'tumble.js';
import jungleUrl from '../../../samples/assets/low_poly_environment_jungle_scene.glb?url';
import { addBody, createBodies, drawBodies, readBodies, removeBody } from './common/bodies';
import { createInfo } from './common/info';
import { loop } from './common/loop';
import { addModelToScene, loadModel } from './common/model';
import { addOverlayControls, beginOverlay, createOverlay, endOverlay } from './common/overlay';
import { createPanel } from './common/panel';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';
import { spectrum } from './common/theme';

await init();
const renderer = await createRenderer([20, 15, -20], [0, 5, 0]);
const worldSettings = createWorldSettings();
const moving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
const notMoving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
enableCollision(worldSettings, moving, moving);
enableCollision(worldSettings, moving, notMoving);
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world);

const rng = mulberry32.create(21);
const random = () => mulberry32.sample(rng);

// a whole scene's triangles as one static triangle mesh, drawn in the scene's own colours
const jungle = await loadModel(jungleUrl);
rigidBody.create(world, {
    shape: triangleMesh.create({ positions: jungle.positions, indices: jungle.indices }),
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    restitution: 0.2,
    friction: 0.7,
});
addModelToScene(renderer, jungle);

const kinds: { shape: Shape; color: string }[] = [
    { shape: sphere.create({ radius: 0.5 }), color: spectrum[1] },
    { shape: box.create({ halfExtents: [0.5, 0.5, 0.5], convexRadius: 0.05 }), color: spectrum[6] },
];

const SPAWN_HEIGHT = 5;
const SPAWN_AREA = 5;
const axis: Vec3 = [0, 0, 0];

function spawn(): BodyId {
    const kind = kinds[Math.floor(random() * kinds.length)]!;
    vec3.normalize(axis, vec3.set(axis, random() - 0.5, random() - 0.5, random() - 0.5));
    const body = rigidBody.create(world, {
        shape: kind.shape,
        objectLayer: moving,
        motionType: MotionType.DYNAMIC,
        position: [(random() * 2 - 1) * SPAWN_AREA, SPAWN_HEIGHT, (random() * 2 - 1) * SPAWN_AREA],
        quaternion: quat.setAxisAngle([0, 0, 0, 1], axis, random() * Math.PI * 2),
        restitution: 0.4,
        friction: 0.6,
    });
    addBody(bodies, body, kind.color);
    return body;
}

function despawn(body: BodyId): void {
    removeBody(bodies, body);
    rigidBody.remove(world, body);
}

const settings = { bodies: 30, stepsBetweenRespawns: 30 };
const spawned: BodyId[] = [];
function matchCount(): void {
    while (spawned.length < settings.bodies) spawned.push(spawn());
    while (spawned.length > settings.bodies) despawn(spawned.pop()!);
}
matchCount();

let steps = 0;
let respawnIndex = 0;
function respawnInTurn(): void {
    if (spawned.length === 0 || ++steps % settings.stepsBetweenRespawns !== 0) return;
    const index = respawnIndex++ % spawned.length;
    despawn(spawned[index]!);
    spawned[index] = spawn();
}

const panel = createPanel('triangle mesh');
panel.add(settings, 'bodies', { min: 0, max: 200, step: 1 }).onChange(matchCount);
panel.add(settings, 'stepsBetweenRespawns', { min: 6, max: 300, step: 6, label: 'steps a respawn' });
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo(`balls and boxes dropped into a jungle scene, one static triangle mesh of ${jungle.indices.length / 3} triangles`);

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
