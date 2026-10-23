import {
    Box,
    DEBUG_VERTEX_STRIDE,
    Debug,
    DebugBuffers,
    HingeConstraint,
    init,
    MotionType,
    Poses,
    RigidBody,
    Sphere,
    World,
    WorldSettings,
} from 'tumble.js';

await init();
const settings = WorldSettings.create();
const moving = WorldSettings.addObjectLayer(settings, WorldSettings.addBroadphaseLayer(settings));
const notMoving = WorldSettings.addObjectLayer(settings, WorldSettings.addBroadphaseLayer(settings));
WorldSettings.enableCollision(settings, moving, moving);
WorldSettings.enableCollision(settings, moving, notMoving);
const world = World.create(settings);
const floorShape = Box.create({ halfExtents: [20, 0.5, 20] });
RigidBody.create(world, { shape: floorShape, objectLayer: notMoving, motionType: MotionType.STATIC, position: [0, -0.5, 0] });
const ballShape = Sphere.create({ radius: 0.3 });
const balls = Array.from({ length: 50 }, (_, i) =>
    RigidBody.create(world, {
        shape: ballShape,
        objectLayer: moving,
        motionType: MotionType.DYNAMIC,
        position: [(i % 10) - 5, 2 + Math.floor(i / 10), 0],
    }),
);
HingeConstraint.create(world, balls[0]!, balls[1]!, { pointA: [-4.5, 2, 0], pointB: [-4.5, 2, 0] });
const check = (ok: boolean, what: string): void => {
    if (!ok) throw new Error(what);
};

/* SNIPPET_START: poses */
// a renderer's bodies, each in a slot of its own: one read a frame brings every slot to its body's pose, in arrays
// straight onto the wasm's memory, and lists the slots that changed, so only their meshes need updating. copied
// number by number into the mesh's own arrays, nothing is allocated a frame
const poses = Poses.create(world);
const meshes: { position: number[]; quaternion: number[] }[] = [];
for (const body of balls) meshes[Poses.add(poses, body)] = { position: [0, 0, 0], quaternion: [0, 0, 0, 1] };

function render(): void {
    const { positions, quaternions, changed } = Poses.read(poses);
    for (const slot of changed) {
        const mesh = meshes[slot]!;
        for (let k = 0; k < 3; k++) mesh.position[k] = positions[slot * 3 + k]!;
        for (let k = 0; k < 4; k++) mesh.quaternion[k] = quaternions[slot * 4 + k]!;
    }
}
/* SNIPPET_END: poses */

render();
for (let i = 0; i < 120; i++) World.step(world, undefined, 1 / 60);
render();
// each mesh at its body's pose
const at: [number, number, number] = [0, 0, 0];
RigidBody.getPosition(at, world, balls[42]!);
check(
    meshes[42]!.position.every((value, axis) => value === at[axis]),
    'mesh at the body',
);
// asleep, nothing changes
for (let i = 0; i < 120; i++) World.step(world, undefined, 1 / 60);
Poses.read(poses);
check(Poses.read(poses).changed.length === 0, 'nothing changed');

/* SNIPPET_START: debug */
// wireframes drawn into buffers kept in the wasm: clear, draw, read. the views are x y z r g b a vertex, two a line,
// three a triangle, straight onto the wasm's memory: copy them out before the next draw
const buffers = DebugBuffers.create();
let gpuLines = new Float32Array(1 << 16);
function drawDebug(): number {
    DebugBuffers.clear(buffers);
    Debug.bodies(buffers, world, { showLinearVelocity: true });
    Debug.contactConstraints(buffers, world);
    Debug.joints(buffers, world);
    const { lines, lineCount } = DebugBuffers.read(buffers);
    if (lines.length > gpuLines.length) gpuLines = new Float32Array(2 * lines.length);
    gpuLines.set(lines);
    return lineCount;
}
/* SNIPPET_END: debug */
const lineCount = drawDebug();
const floats = DEBUG_VERTEX_STRIDE * 2 * lineCount;
check(
    lineCount > 50 && gpuLines.length >= floats && gpuLines[floats - 1] === DebugBuffers.read(buffers).lines[floats - 1],
    `drew ${lineCount} lines`,
);
// drawn again, the same scene takes no more memory
check(drawDebug() === lineCount, 'the same lines again');

Poses.destroy(poses);
DebugBuffers.destroy(buffers);
World.destroy(world);
Box.destroy(floorShape);
Sphere.destroy(ballShape);
