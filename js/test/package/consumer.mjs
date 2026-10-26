// the package as an app imports it: by name, through its exports, from dist, the wasm from beside its modules
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
    updateWorld,
} from 'tumble.js';

await init();
const settings = createWorldSettings();
const moving = addObjectLayer(settings, addBroadphaseLayer(settings));
enableCollision(settings, moving, moving);
const world = createWorld(settings);
const crate = box.create({ halfExtents: [0.5, 0.5, 0.5] });
const body = rigidBody.create(world, { shape: crate, objectLayer: moving, motionType: MotionType.DYNAMIC, position: [0, 10, 0] });
for (let i = 0; i < 60; i++) updateWorld(world, undefined, 1 / 60);
const position = [0, 0, 0];
rigidBody.getPosition(position, world, body);
destroyWorld(world);
releaseShape(crate);
// a second of falling from 10 m: about 5 m down
if (Math.abs(position[1] - 5.1) > 0.2) throw new Error(`fell to ${position[1]}`);
console.log('the package works');
