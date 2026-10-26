import { readFile } from 'node:fs/promises';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    createWorld,
    createWorldSettings,
    destroyWorld,
    enableCollision,
    hingeConstraint,
    init,
    MotionType,
    releaseShape,
    rigidBody,
} from 'tumble.js';

/* SNIPPET_START: init */
// the wasm given as its bytes, read here from the package in node. it loads once a realm: later calls wait for the same
const bytes = await readFile(new URL('../dist/tumble.wasm', import.meta.url));
await init({ wasm: bytes });
/* SNIPPET_END: init */

const settings = createWorldSettings();
const moving = addObjectLayer(settings, addBroadphaseLayer(settings));
enableCollision(settings, moving, moving);
const world = createWorld(settings);
const crateShape = box.create({ halfExtents: [0.5, 0.5, 0.5] });
const a = rigidBody.create(world, { shape: crateShape, objectLayer: moving, motionType: MotionType.DYNAMIC });
const b = rigidBody.create(world, {
    shape: crateShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [2, 0, 0],
});
let caught: string[] = [];

/* SNIPPET_START: errors */
// a call that cannot do what it was asked throws, having undone what it began: an id naming nothing, or settings the
// C++ would refuse
rigidBody.remove(world, b);
try {
    hingeConstraint.create(world, a, b, {});
} catch (error) {
    (error as Error).message; // 'tumble: BAD_HANDLE'
    /* HIDE_START */
    caught.push((error as Error).message);
    /* HIDE_END */
}
try {
    hingeConstraint.create(world, a, a, { limitsMin: 1 });
} catch (error) {
    error instanceof RangeError; // true: a hinge's limitsMin is in [-pi, 0]
    /* HIDE_START */
    caught.push(error instanceof RangeError ? 'range' : String(error));
    /* HIDE_END */
}
// readers and setters with an id naming nothing return false instead, doing nothing
rigidBody.setLinearVelocity(world, b, [1, 0, 0]); // false
/* SNIPPET_END: errors */
if (caught.join() !== 'tumble: BAD_HANDLE,range') throw new Error(`caught ${caught}`);
caught = [];

/* SNIPPET_START: freeing */
// what you create, you free: worlds (their bodies and constraints go with them), filters, policies and debug
// buffers. a body or constraint goes with rigidBody.remove or constraints.remove too. a shape or a character you made
// you release: whatever else holds it lets go of it itself
destroyWorld(world);
releaseShape(crateShape);
/* SNIPPET_END: freeing */
