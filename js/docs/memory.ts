import { readFile } from 'node:fs/promises';
import { Box, HingeConstraint, init, MotionType, RigidBody, World, WorldSettings } from 'tumble';

/* SNIPPET_START: init */
// the wasm given as its bytes, read here from the package in node. it loads once a realm: later calls wait for the same
const bytes = await readFile(new URL('../dist/tumble.wasm', import.meta.url));
await init({ wasm: bytes });
/* SNIPPET_END: init */

const settings = WorldSettings.create();
const moving = WorldSettings.addObjectLayer(settings, WorldSettings.addBroadphaseLayer(settings));
WorldSettings.enableCollision(settings, moving, moving);
const world = World.create(settings);
const crateShape = Box.create({ halfExtents: [0.5, 0.5, 0.5] });
const a = RigidBody.create(world, { shape: crateShape, objectLayer: moving, motionType: MotionType.DYNAMIC });
const b = RigidBody.create(world, {
    shape: crateShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [2, 0, 0],
});
let caught: string[] = [];

/* SNIPPET_START: errors */
// a call that cannot do what it was asked throws, having undone what it began: an id naming nothing, memory that
// cannot grow, or settings the C++ would refuse
RigidBody.remove(world, b);
try {
    HingeConstraint.create(world, a, b, {});
} catch (error) {
    (error as Error).message; // 'tumble: BAD_HANDLE'
    /* HIDE_START */
    caught.push((error as Error).message);
    /* HIDE_END */
}
try {
    HingeConstraint.create(world, a, a, { limitsMin: 1 });
} catch (error) {
    error instanceof RangeError; // true: a hinge's limitsMin is in [-pi, 0]
    /* HIDE_START */
    caught.push(error instanceof RangeError ? 'range' : String(error));
    /* HIDE_END */
}
// readers and setters with an id naming nothing return false instead, doing nothing
RigidBody.setLinearVelocity(world, b, [1, 0, 0]); // false
/* SNIPPET_END: errors */
if (caught.join() !== 'tumble: BAD_HANDLE,range') throw new Error(`caught ${caught}`);
caught = [];

/* SNIPPET_START: freeing */
// what you create, you free: worlds (their bodies and constraints go with them), shapes and their data, filters,
// policies, characters, poses and debug buffers. a body or constraint goes with RigidBody.remove or Constraint.remove too
World.destroy(world);
Box.destroy(crateShape);
/* SNIPPET_END: freeing */
