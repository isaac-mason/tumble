import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    createWorld,
    createWorldSettings,
    destroyWorld,
    enableCollision,
    init,
    MotionType,
    releaseShape,
    rigidBody,
    setMaterials,
    updateWorld,
    type Vec3,
} from 'tumble.js';

await init();

/* SNIPPET_START: layers */
// object layers decide what collides with what. each object layer belongs to a broadphase layer, which has its own
// tree. put moving and still bodies in separate broadphase layers
const settings = createWorldSettings();
const movingTree = addBroadphaseLayer(settings);
const staticTree = addBroadphaseLayer(settings);
const moving = addObjectLayer(settings, movingTree);
const debris = addObjectLayer(settings, movingTree);
const scenery = addObjectLayer(settings, staticTree);
enableCollision(settings, moving, moving);
enableCollision(settings, moving, scenery);
enableCollision(settings, debris, scenery);
/* SNIPPET_END: layers */

/* SNIPPET_START: settings */
// settings are plain data with the defaults filled in. change what you need. createWorld copies them, so they can
// make another world
settings.gravity = [0, -20, 0];
settings.solver.velocityIterations = 12;
settings.sleeping.timeBeforeSleep = 1;
const world = createWorld(settings);
/* SNIPPET_END: settings */

const floorShape = box.create({ halfExtents: [20, 0.5, 20] });
rigidBody.create(world, { shape: floorShape, objectLayer: scenery, motionType: MotionType.STATIC, position: [0, -0.5, 0] });
const crateShape = box.create({ halfExtents: [0.5, 0.5, 0.5] });
const crate = rigidBody.create(world, {
    shape: crateShape,
    objectLayer: debris,
    motionType: MotionType.DYNAMIC,
    position: [0, 3, 0],
});

/* SNIPPET_START: fixed-step */
// step by a fixed time, and catch up a few steps when a frame runs long. then the simulation does not depend on the
// frame rate
const STEP = 1 / 60;
let pending = 0;
function frame(elapsed: number): void {
    pending = Math.min(pending + elapsed, 4 * STEP);
    while (pending >= STEP) {
        updateWorld(world, undefined, STEP);
        pending -= STEP;
    }
}
/* SNIPPET_END: fixed-step */

// three seconds of frames at an uneven rate
for (let i = 0; i < 180; i++) frame(i % 3 === 0 ? 1 / 30 : 1 / 90);
/* HIDE_START */
const at: Vec3 = [0, 0, 0];
rigidBody.getPosition(at, world, crate);
if (Math.abs(at[1] - 0.48) > 0.02) throw new Error(`the crate at y ${at[1]}`);
/* HIDE_END */

/* SNIPPET_START: materials */
// materials by index. a shape's or a mesh triangle's materialId picks one. it replaces the body's friction and
// restitution on its side of a contact
setMaterials(world, [
    { friction: 0, restitution: 0 },
    { friction: 1, restitution: 0.8 },
]);
const iceShape = box.create({ halfExtents: [5, 0.1, 5], materialId: 0 });
rigidBody.create(world, { shape: iceShape, objectLayer: scenery, motionType: MotionType.STATIC, position: [10, 0.1, 0] });
/* SNIPPET_END: materials */

// pushed at 4 m/s for a second, a crate glides across the frictionless ice, while one on the floor slows
const onIce = rigidBody.create(world, {
    shape: crateShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [7, 0.7, 0],
});
const onFloor = rigidBody.create(world, {
    shape: crateShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [0, 0.5, 6],
});
for (let i = 0; i < 10; i++) updateWorld(world, undefined, STEP);
rigidBody.setLinearVelocity(world, onIce, [4, 0, 0]);
rigidBody.setLinearVelocity(world, onFloor, [4, 0, 0]);
for (let i = 0; i < 60; i++) updateWorld(world, undefined, STEP);
/* HIDE_START */
const ice: Vec3 = [0, 0, 0];
const floor: Vec3 = [0, 0, 0];
rigidBody.getPosition(ice, world, onIce);
rigidBody.getPosition(floor, world, onFloor);
if (ice[0] - 7 < 3.8 || floor[0] > 2.5) throw new Error(`slid ${ice[0] - 7} on ice, ${floor[0]} on the floor`);
/* HIDE_END */

destroyWorld(world);
releaseShape(floorShape);
releaseShape(crateShape);
releaseShape(iceShape);
