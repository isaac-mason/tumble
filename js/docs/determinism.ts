import { Box, init, MotionType, type Quat, RigidBody, type Shape, Sphere, type Vec3, World, WorldSettings } from 'tumble';

await init();
const groundShape = Box.create({ halfExtents: [10, 0.5, 10] });
const ballShape = Sphere.create({ radius: 0.45 });

/** a pile of 20 balls on a floor, made in one order with exact offsets: the C++ docs' pile, the same bits */
function makePile(ground: Shape, ball: Shape): { world: World; bodies: number[] } {
    const settings = WorldSettings.create();
    const moving = WorldSettings.addObjectLayer(settings, WorldSettings.addBroadphaseLayer(settings));
    const notMoving = WorldSettings.addObjectLayer(settings, WorldSettings.addBroadphaseLayer(settings));
    WorldSettings.enableCollision(settings, moving, moving);
    WorldSettings.enableCollision(settings, moving, notMoving);
    const world = World.create(settings);
    const bodies = [
        RigidBody.create(world, { shape: ground, objectLayer: notMoving, motionType: MotionType.STATIC, position: [0, -0.5, 0] }),
    ];
    for (let i = 0; i < 20; i++) {
        const position: Vec3 = [0.0625 * (i % 3), 0.625 + 1.125 * i, 0.03125 * (i % 4)];
        bodies.push(RigidBody.create(world, { shape: ball, objectLayer: moving, motionType: MotionType.DYNAMIC, position }));
    }
    return { world, bodies };
}

/* SNIPPET_START: replay */
// the same scene, made in the same order and stepped by the same steps, is the same to the last bit: in another
// world, another run, another machine, or natively in C++
const first = makePile(groundShape, ballShape);
const second = makePile(groundShape, ballShape);
for (let i = 0; i < 300; i++) {
    World.step(first.world, undefined, 1 / 60);
    World.step(second.world, undefined, 1 / 60);
}
const a: Vec3 = [0, 0, 0];
const b: Vec3 = [0, 0, 0];
const same = first.bodies.every((body, i) => {
    RigidBody.getPosition(a, first.world, body);
    RigidBody.getPosition(b, second.world, second.bodies[i]!);
    return a.every((value, axis) => Object.is(value, b[axis]));
});
console.log(same ? 'bit for bit the same' : 'they differ');
/* SNIPPET_END: replay */
if (!same) throw new Error('the replay differs');

// the pile's poses hashed as docs/cpp/determinism.cpp hashes them, natively: the same constant
let hash = 2166136261;
const bits = new Uint32Array(new Float32Array(7).buffer);
const floats = new Float32Array(bits.buffer);
const quaternion: Quat = [0, 0, 0, 1];
for (const body of first.bodies) {
    RigidBody.getTransform(a, quaternion, first.world, body);
    floats.set([...a, ...quaternion]);
    for (const word of bits) {
        for (let k = 0; k < 4; k++) {
            hash ^= (word >>> (k * 8)) & 0xff;
            hash = Math.imul(hash, 16777619) >>> 0;
        }
    }
}
if (hash !== 0x2a2c339c) throw new Error(`the pile hashes to 0x${hash.toString(16)}, natively 0x2a2c339c`);

World.destroy(first.world);
World.destroy(second.world);
Box.destroy(groundShape);
Sphere.destroy(ballShape);
