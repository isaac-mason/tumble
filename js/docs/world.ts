import { Box, init, MotionType, RigidBody, type Vec3, World, WorldSettings } from 'tumble.js';

await init();

/* SNIPPET_START: layers */
// object layers decide what collides with what; each is in a broadphase layer, a tree of its own. keep what moves and
// what does not in trees of their own
const settings = WorldSettings.create();
const movingTree = WorldSettings.addBroadphaseLayer(settings);
const staticTree = WorldSettings.addBroadphaseLayer(settings);
const moving = WorldSettings.addObjectLayer(settings, movingTree);
const debris = WorldSettings.addObjectLayer(settings, movingTree);
const scenery = WorldSettings.addObjectLayer(settings, staticTree);
WorldSettings.enableCollision(settings, moving, moving);
WorldSettings.enableCollision(settings, moving, scenery);
WorldSettings.enableCollision(settings, debris, scenery);
/* SNIPPET_END: layers */

/* SNIPPET_START: settings */
// plain data, from the C++ defaults: change what you need. create copies them, so they can make another world
settings.gravity = [0, -20, 0];
settings.solver.velocityIterations = 12;
settings.sleeping.timeBeforeSleep = 1;
const world = World.create(settings);
/* SNIPPET_END: settings */

const floorShape = Box.create({ halfExtents: [20, 0.5, 20] });
RigidBody.create(world, { shape: floorShape, objectLayer: scenery, motionType: MotionType.STATIC, position: [0, -0.5, 0] });
const crateShape = Box.create({ halfExtents: [0.5, 0.5, 0.5] });
const crate = RigidBody.create(world, {
    shape: crateShape,
    objectLayer: debris,
    motionType: MotionType.DYNAMIC,
    position: [0, 3, 0],
});

/* SNIPPET_START: fixed-step */
// step by a fixed time, catching up a few steps when a frame runs long, so the simulation does not depend on the
// frame rate
const STEP = 1 / 60;
let pending = 0;
function frame(elapsed: number): void {
    pending = Math.min(pending + elapsed, 4 * STEP);
    while (pending >= STEP) {
        World.step(world, undefined, STEP);
        pending -= STEP;
    }
}
/* SNIPPET_END: fixed-step */

// three seconds of frames at an uneven rate
for (let i = 0; i < 180; i++) frame(i % 3 === 0 ? 1 / 30 : 1 / 90);
/* HIDE_START */
const at: Vec3 = [0, 0, 0];
RigidBody.getPosition(at, world, crate);
if (Math.abs(at[1] - 0.48) > 0.02) throw new Error(`the crate at y ${at[1]}`);
/* HIDE_END */

/* SNIPPET_START: materials */
// surfaces by index: a shape's or a mesh triangle's materialId picks one, which stands in for the body's friction
// and restitution on its side of a contact
World.setMaterials(world, [
    { friction: 0, restitution: 0 },
    { friction: 1, restitution: 0.8 },
]);
const iceShape = Box.create({ halfExtents: [5, 0.1, 5], materialId: 0 });
RigidBody.create(world, { shape: iceShape, objectLayer: scenery, motionType: MotionType.STATIC, position: [10, 0.1, 0] });
/* SNIPPET_END: materials */

// pushed at 4 m/s for a second, a crate glides across the frictionless ice, while one on the floor slows
const onIce = RigidBody.create(world, {
    shape: crateShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [7, 0.7, 0],
});
const onFloor = RigidBody.create(world, {
    shape: crateShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [0, 0.5, 6],
});
for (let i = 0; i < 10; i++) World.step(world, undefined, STEP);
RigidBody.setLinearVelocity(world, onIce, [4, 0, 0]);
RigidBody.setLinearVelocity(world, onFloor, [4, 0, 0]);
for (let i = 0; i < 60; i++) World.step(world, undefined, STEP);
/* HIDE_START */
const ice: Vec3 = [0, 0, 0];
const floor: Vec3 = [0, 0, 0];
RigidBody.getPosition(ice, world, onIce);
RigidBody.getPosition(floor, world, onFloor);
if (ice[0] - 7 < 3.8 || floor[0] > 2.5) throw new Error(`slid ${ice[0] - 7} on ice, ${floor[0]} on the floor`);
/* HIDE_END */

World.destroy(world);
Box.destroy(floorShape);
Box.destroy(crateShape);
Box.destroy(iceShape);
