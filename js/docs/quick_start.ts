/* SNIPPET_START: quick-start */
import { Box, init, MotionType, RigidBody, Sphere, type Vec3, World, WorldSettings } from 'tumble';

// load the wasm, once, before anything else
await init();

// two layers, one for what moves and one for what does not: what moves collides with both
const settings = WorldSettings.create();
const moving = WorldSettings.addObjectLayer(settings, WorldSettings.addBroadphaseLayer(settings));
const notMoving = WorldSettings.addObjectLayer(settings, WorldSettings.addBroadphaseLayer(settings));
WorldSettings.enableCollision(settings, moving, moving);
WorldSettings.enableCollision(settings, moving, notMoving);
const world = World.create(settings);

// shapes are yours, shared by any number of bodies; a body is an id in its world
const floorShape = Box.create({ halfExtents: [20, 0.5, 20] });
const boxShape = Box.create({ halfExtents: [0.5, 0.5, 0.5] });
const ballShape = Sphere.create({ radius: 0.5 });
RigidBody.create(world, { shape: floorShape, objectLayer: notMoving, motionType: MotionType.STATIC, position: [0, -0.5, 0] });
const box = RigidBody.create(world, {
    shape: boxShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [0, 4, 0],
});
const ball = RigidBody.create(world, {
    shape: ballShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [2, 6, 0],
});

// two seconds at 60 Hz
for (let i = 0; i < 120; i++) World.step(world, undefined, 1 / 60);

// readers take what they fill first, and allocate nothing
const position: Vec3 = [0, 0, 0];
RigidBody.getPosition(position, world, box);
console.log(`the box rests at y ${position[1].toFixed(2)}`);
/* HIDE_START */
if (Math.abs(position[1] - 0.48) > 0.02) throw new Error(`the box at y ${position[1]}`);
RigidBody.getPosition(position, world, ball);
if (Math.abs(position[1] - 0.48) > 0.02) throw new Error(`the ball at y ${position[1]}`);
/* HIDE_END */

// what you create, you free: the world takes its bodies with it
World.destroy(world);
Box.destroy(floorShape);
Box.destroy(boxShape);
Sphere.destroy(ballShape);
/* SNIPPET_END: quick-start */
