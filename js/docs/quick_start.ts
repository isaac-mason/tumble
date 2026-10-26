/* SNIPPET_START: quick-start */
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
    sphere,
    updateWorld,
    type Vec3,
} from 'tumble.js';

// load the wasm once, before anything else
await init();

// one layer for what moves and one for what does not. moving bodies collide with both
const settings = createWorldSettings();
const moving = addObjectLayer(settings, addBroadphaseLayer(settings));
const notMoving = addObjectLayer(settings, addBroadphaseLayer(settings));
enableCollision(settings, moving, moving);
enableCollision(settings, moving, notMoving);
const world = createWorld(settings);

// you own the shapes, and any number of bodies can share them. a body is an id in its world
const floorShape = box.create({ halfExtents: [20, 0.5, 20] });
const boxShape = box.create({ halfExtents: [0.5, 0.5, 0.5] });
const ballShape = sphere.create({ radius: 0.5 });
rigidBody.create(world, { shape: floorShape, objectLayer: notMoving, motionType: MotionType.STATIC, position: [0, -0.5, 0] });
const boxBody = rigidBody.create(world, {
    shape: boxShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [0, 4, 0],
});
const ball = rigidBody.create(world, {
    shape: ballShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [2, 6, 0],
});

// two seconds at 60 Hz
for (let i = 0; i < 120; i++) updateWorld(world, undefined, 1 / 60);

// getters take the array they fill first, and allocate nothing
const position: Vec3 = [0, 0, 0];
rigidBody.getPosition(position, world, boxBody);
console.log(`the box rests at y ${position[1].toFixed(2)}`);
/* HIDE_START */
if (Math.abs(position[1] - 0.48) > 0.02) throw new Error(`the box at y ${position[1]}`);
rigidBody.getPosition(position, world, ball);
if (Math.abs(position[1] - 0.48) > 0.02) throw new Error(`the ball at y ${position[1]}`);
/* HIDE_END */

// free what you create. destroying the world frees its bodies
destroyWorld(world);
releaseShape(floorShape);
releaseShape(boxShape);
releaseShape(ballShape);
/* SNIPPET_END: quick-start */
