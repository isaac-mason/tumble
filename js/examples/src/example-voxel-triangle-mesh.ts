import { quat, type Vec3, vec3 } from 'math';
import { simplex2d } from 'math/noise';
import { mulberry32 } from 'math/random';
import {
    addBroadphaseLayer,
    addObjectLayer,
    type BodyId,
    BvhSplitStrategy,
    box,
    capsule,
    convexHull,
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
import { addBody, createBodies, drawBodies, readBodies, removeBody } from './common/bodies';
import { createInfo } from './common/info';
import { loop } from './common/loop';
import { addOverlayControls, beginOverlay, createOverlay, endOverlay } from './common/overlay';
import { createPanel } from './common/panel';
import { createRenderer, render } from './common/renderer';
import { addWorldStats } from './common/stats';
import { palette, spectrum } from './common/theme';

await init();
const renderer = await createRenderer([0, 32, 44], [0, 8, 0]);
const worldSettings = createWorldSettings();
const moving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
const notMoving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
enableCollision(worldSettings, moving, moving);
enableCollision(worldSettings, moving, notMoving);
const world = createWorld(worldSettings);
const bodies = createBodies(renderer, world);
const overlay = createOverlay(renderer, world);

const rng = mulberry32.create(42);
const random = () => mulberry32.sample(rng);

// voxel terrain: columns of unit blocks, as tall as layered noise says
const SIZE = 64;
const DEPTH = 24;
const solid = new Uint8Array(SIZE * DEPTH * SIZE);
// the grid's x and z run -SIZE / 2 to SIZE / 2
const indexOf = (x: number, y: number, z: number) => x + SIZE * (y + DEPTH * z);
const isSolid = (x: number, y: number, z: number) =>
    x >= 0 && x < SIZE && y >= 0 && y < DEPTH && z >= 0 && z < SIZE && solid[indexOf(x, y, z)] === 1;

const noise = simplex2d.create(42);
for (let x = 0; x < SIZE; x++) {
    for (let z = 0; z < SIZE; z++) {
        const wx = x - SIZE / 2;
        const wz = z - SIZE / 2;
        const layered =
            0.6 * (simplex2d.sample(noise, wx * 0.02, wz * 0.02) * 0.5 + 0.5) +
            0.3 * (simplex2d.sample(noise, wx * 0.08, wz * 0.08) * 0.5 + 0.5) +
            0.1 * (simplex2d.sample(noise, wx * 0.2, wz * 0.2) * 0.5 + 0.5);
        const height = Math.floor(4 + layered * 16);
        for (let y = 0; y < height; y++) solid[indexOf(x, y, z)] = 1;
    }
}

// each chunk of 16 x 16 x 16 blocks its own static triangle mesh, of the faces between a solid block and an empty one.
// a chunk is rebuilt alone when its blocks change, and the rest of the terrain is left as it is
const CHUNK = 16;
// along each axis, the two axes across its faces
const ACROSS: [number, number][] = [
    [1, 2],
    [2, 0],
    [0, 1],
];

function chunkMesh(cx: number, cy: number, cz: number): { positions: number[]; indices: number[] } {
    const positions: number[] = [];
    const indices: number[] = [];
    for (let x = cx * CHUNK; x < (cx + 1) * CHUNK; x++) {
        for (let y = cy * CHUNK; y < (cy + 1) * CHUNK; y++) {
            for (let z = cz * CHUNK; z < (cz + 1) * CHUNK; z++) {
                if (!isSolid(x, y, z)) continue;
                for (let axis = 0; axis < 3; axis++) {
                    for (const side of [1, -1]) {
                        const neighbour: Vec3 = [x, y, z];
                        neighbour[axis]! += side;
                        if (isSolid(neighbour[0], neighbour[1], neighbour[2])) continue;
                        // the face's corner, then along u and v, wound counter clockwise seen from outside
                        const corner: Vec3 = [x - SIZE / 2, y, z - SIZE / 2];
                        if (side > 0) corner[axis]! += 1;
                        const [first, second] = ACROSS[axis]!;
                        const u: Vec3 = [0, 0, 0];
                        const v: Vec3 = [0, 0, 0];
                        u[side > 0 ? first : second] = 1;
                        v[side > 0 ? second : first] = 1;
                        const base = positions.length / 3;
                        positions.push(...corner);
                        positions.push(corner[0] + u[0], corner[1] + u[1], corner[2] + u[2]);
                        positions.push(corner[0] + u[0] + v[0], corner[1] + u[1] + v[1], corner[2] + u[2] + v[2]);
                        positions.push(corner[0] + v[0], corner[1] + v[1], corner[2] + v[2]);
                        indices.push(base, base + 1, base + 2, base, base + 2, base + 3);
                    }
                }
            }
        }
    }
    return { positions, indices };
}

let triangles = 0;
for (let cx = 0; cx < SIZE / CHUNK; cx++) {
    for (let cy = 0; cy < Math.ceil(DEPTH / CHUNK); cy++) {
        for (let cz = 0; cz < SIZE / CHUNK; cz++) {
            const { positions, indices } = chunkMesh(cx, cy, cz);
            if (indices.length === 0) continue;
            triangles += indices.length / 3;
            const chunk = rigidBody.create(world, {
                // quick to build, for terrain that is rebuilt as it changes
                shape: triangleMesh.create({ positions, indices, bvhSplitStrategy: BvhSplitStrategy.CENTER }),
                objectLayer: notMoving,
                motionType: MotionType.STATIC,
                restitution: 0.2,
                friction: 0.7,
            });
            addBody(bodies, chunk, palette.dim);
        }
    }
}

const kinds: { shape: Shape; color: string }[] = [
    { shape: sphere.create({ radius: 0.5 }), color: spectrum[1] },
    { shape: box.create({ halfExtents: [0.5, 0.5, 0.5], convexRadius: 0.05 }), color: spectrum[5] },
    { shape: capsule.create({ halfHeightOfCylinder: 1, radius: 1 }), color: spectrum[3] },
    {
        shape: convexHull.create({
            positions: [-0.5, 0, -0.5, 0.5, 0, -0.5, 0.5, 0, 0.5, -0.5, 0, 0.5, 0, 1, 0],
            convexRadius: 0.05,
        }),
        color: spectrum[0],
    },
];

const SPAWN_HEIGHT = 25;
const SPAWN_AREA = 10;
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

let steps = 0;
let respawnIndex = 0;
function respawnInTurn(): void {
    if (spawned.length === 0 || ++steps % settings.stepsBetweenRespawns !== 0) return;
    const index = respawnIndex++ % spawned.length;
    despawn(spawned[index]!);
    spawned[index] = spawn();
}

const panel = createPanel('voxel triangle mesh');
panel.add(settings, 'bodies', { min: 0, max: 1000, step: 1 }).onChange(matchCount);
panel.add(settings, 'stepsBetweenRespawns', { min: 6, max: 300, step: 6, label: 'steps a respawn' });
addOverlayControls(overlay, panel);
const stepTime = addWorldStats(panel, world);
createInfo(`voxel terrain meshed one chunk at a time into static triangle meshes, ${triangles} triangles in all`);

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
