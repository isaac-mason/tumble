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
// pass the wasm as bytes, here read from the package in node. it loads once per realm. later calls wait for that load
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
// a call that cannot do what you asked throws, and undoes anything it started. causes include a stale id or invalid
// settings
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
// getters and setters given a stale id return false and do nothing instead
rigidBody.setLinearVelocity(world, b, [1, 0, 0]); // false
/* SNIPPET_END: errors */
if (caught.join() !== 'tumble: BAD_HANDLE,range') throw new Error(`caught ${caught}`);
caught = [];

/* SNIPPET_START: freeing */
// free what you create: worlds, filters, policies and debug draws. a world frees its bodies and constraints. you can
// also remove them with rigidBody.remove and constraints.remove. release the shapes and characters you make. anything
// else that holds them lets go by itself
destroyWorld(world);
releaseShape(crateShape);
/* SNIPPET_END: freeing */
