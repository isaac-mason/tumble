// the package as an app imports it: by name, through its exports, from dist, the wasm from beside its modules
import { Box, init, MotionType, RigidBody, World, WorldSettings } from 'tumble';

await init();
const settings = WorldSettings.create();
const moving = WorldSettings.addObjectLayer(settings, WorldSettings.addBroadphaseLayer(settings));
WorldSettings.enableCollision(settings, moving, moving);
const world = World.create(settings);
const crate = Box.create({ halfExtents: [0.5, 0.5, 0.5] });
const body = RigidBody.create(world, { shape: crate, objectLayer: moving, motionType: MotionType.DYNAMIC, position: [0, 10, 0] });
for (let i = 0; i < 60; i++) World.step(world, undefined, 1 / 60);
const position = [0, 0, 0];
RigidBody.getPosition(position, world, body);
World.destroy(world);
Box.destroy(crate);
// a second of falling from 10 m: about 5 m down
if (Math.abs(position[1] - 5.1) > 0.2) throw new Error(`fell to ${position[1]}`);
console.log('the package works');
