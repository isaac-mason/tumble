import {
    addBroadphaseLayer,
    addObjectLayer,
    type BodyId,
    box,
    createActiveBody,
    createWorld,
    createWorldSettings,
    DEBUG_VERTEX_STRIDE,
    debug,
    debugDraw,
    destroyWorld,
    enableCollision,
    getActiveBodies,
    getActiveBodyAt,
    getNumActiveBodies,
    init,
    type Listener,
    MotionType,
    type Quat,
    releaseShape,
    rigidBody,
    sphere,
    updateWorld,
    type Vec3,
} from 'tumble.js';

await init();
const settings = createWorldSettings();
const moving = addObjectLayer(settings, addBroadphaseLayer(settings));
const notMoving = addObjectLayer(settings, addBroadphaseLayer(settings));
enableCollision(settings, moving, moving);
enableCollision(settings, moving, notMoving);
const world = createWorld(settings);
const floorShape = box.create({ halfExtents: [20, 0.5, 20] });
rigidBody.create(world, { shape: floorShape, objectLayer: notMoving, motionType: MotionType.STATIC, position: [0, -0.5, 0] });
const ballShape = sphere.create({ radius: 0.3 });
const balls = Array.from({ length: 50 }, (_, i) =>
    rigidBody.create(world, {
        shape: ballShape,
        objectLayer: moving,
        motionType: MotionType.DYNAMIC,
        position: [(i % 10) - 5, 2 + Math.floor(i / 10), 0],
    }),
);
const check = (ok: boolean, what: string): void => {
    if (!ok) throw new Error(what);
};

/* SNIPPET_START: active-bodies */
// after a step, the bodies it moved are the active list: read it once, then copy each body's pose into its mesh,
// number by number, so nothing is allocated a frame. a body the step put to sleep has left the list, so the listener
// names it and its last pose is read once
const meshes = new Map<BodyId, { position: Vec3; quaternion: Quat }>();
for (const body of balls) meshes.set(body, { position: [0, 0, 0], quaternion: [0, 0, 0, 1] });

const activeBody = createActiveBody();
const listener: Listener = {
    onBodyDeactivated: (body) => {
        const mesh = meshes.get(body);
        if (mesh === undefined) return;
        rigidBody.getPosition(mesh.position, world, body);
        rigidBody.getQuaternion(mesh.quaternion, world, body);
    },
};

function step(): void {
    updateWorld(world, listener, 1 / 60);
    const count = getActiveBodies(world);
    for (let i = 0; i < count; i++) {
        getActiveBodyAt(activeBody, world, i);
        const mesh = meshes.get(activeBody.body);
        if (mesh === undefined) continue;
        mesh.position[0] = activeBody.position[0];
        mesh.position[1] = activeBody.position[1];
        mesh.position[2] = activeBody.position[2];
        mesh.quaternion[0] = activeBody.quaternion[0];
        mesh.quaternion[1] = activeBody.quaternion[1];
        mesh.quaternion[2] = activeBody.quaternion[2];
        mesh.quaternion[3] = activeBody.quaternion[3];
    }
}
/* SNIPPET_END: active-bodies */

for (let i = 0; i < 120; i++) step();
// each mesh at its body's pose
const at: Vec3 = [0, 0, 0];
rigidBody.getPosition(at, world, balls[42]!);
check(
    meshes.get(balls[42]!)!.position.every((value, axis) => value === at[axis]),
    'mesh at the body',
);
// asleep: the list is empty, and every mesh is where its body came to rest
for (let i = 0; i < 600 && getNumActiveBodies(world) > 0; i++) step();
check(getActiveBodies(world) === 0, 'nothing active');
for (const body of balls) {
    rigidBody.getPosition(at, world, body);
    check(
        meshes.get(body)!.position.every((value, axis) => value === at[axis]),
        'a sleeping mesh at its body',
    );
}

/* SNIPPET_START: debug */
// wireframes drawn into buffers kept in the wasm: clear, draw, read. the views are x y z r g b a vertex, two a line,
// three a triangle, straight onto the wasm's memory: copy them out before the next draw
const buffers = debugDraw.create();
let gpuLines = new Float32Array(1 << 16);
function drawDebug(): number {
    debugDraw.clear(buffers);
    debug.bodies(buffers, world, { showLinearVelocity: true });
    debug.contactConstraints(buffers, world);
    debug.joints(buffers, world);
    const { lines, lineCount } = debugDraw.read(buffers);
    if (lines.length > gpuLines.length) gpuLines = new Float32Array(2 * lines.length);
    gpuLines.set(lines);
    return lineCount;
}
/* SNIPPET_END: debug */
const lineCount = drawDebug();
const floats = DEBUG_VERTEX_STRIDE * 2 * lineCount;
check(
    lineCount > 50 && gpuLines.length >= floats && gpuLines[floats - 1] === debugDraw.read(buffers).lines[floats - 1],
    `drew ${lineCount} lines`,
);
// drawn again, the same scene takes no more memory
check(drawDebug() === lineCount, 'the same lines again');

debugDraw.destroy(buffers);
destroyWorld(world);
releaseShape(floorShape);
releaseShape(ballShape);
