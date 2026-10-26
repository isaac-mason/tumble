import GUI from 'lil-gui';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    capsule,
    createWorld,
    createWorldSettings,
    debug,
    debugDraw,
    emptyShape,
    enableCollision,
    hingeConstraint,
    MotionType,
    pointConstraint,
    rigidBody,
    sphere,
    updateWorld,
} from 'tumble.js';
import { caption, createDebugLines, loadTumble, loop, setup, showDebugLines } from './common';

await loadTumble();
const view = await setup([14, 10, 18], [0, 3, 0]);
caption("the library's own debug drawing: lines read from debugDraw each frame into gpucat line segments");

const settings = createWorldSettings();
const moving = addObjectLayer(settings, addBroadphaseLayer(settings));
const notMoving = addObjectLayer(settings, addBroadphaseLayer(settings));
enableCollision(settings, moving, moving);
enableCollision(settings, moving, notMoving);
const world = createWorld(settings);

rigidBody.create(world, {
    shape: box.create({ halfExtents: [12, 0.5, 12] }),
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    position: [0, -0.5, 0],
});

// a chain of planks hinged end to end from a fixed anchor, swinging down
const plank = box.create({ halfExtents: [0.5, 0.1, 0.25] });
const anchor = rigidBody.create(world, {
    shape: emptyShape.create(),
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    position: [-4, 8, 0],
});
let previous = anchor;
for (let i = 0; i < 8; i++) {
    const link = rigidBody.create(world, {
        shape: plank,
        objectLayer: moving,
        motionType: MotionType.DYNAMIC,
        position: [-3.5 + i, 8, 0],
    });
    const at: [number, number, number] = [-4 + i, 8, 0];
    hingeConstraint.create(world, previous, link, {
        pointA: at,
        pointB: at,
        hingeAxisA: [0, 0, 1],
        hingeAxisB: [0, 0, 1],
        normalAxisA: [1, 0, 0],
        normalAxisB: [1, 0, 0],
    });
    previous = link;
}

// a pendulum, and a pile for the contacts
const pivot = rigidBody.create(world, {
    shape: emptyShape.create(),
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    position: [5, 7, 0],
});
const bob = rigidBody.create(world, {
    shape: sphere.create({ radius: 0.6 }),
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [8, 7, 0],
});
pointConstraint.create(world, pivot, bob, { pointA: [5, 7, 0], pointB: [5, 7, 0] });
const crate = box.create({ halfExtents: [0.4, 0.4, 0.4] });
const pill = capsule.create({ halfHeightOfCylinder: 0.4, radius: 0.3 });
for (let i = 0; i < 24; i++) {
    rigidBody.create(world, {
        shape: i % 2 === 0 ? crate : pill,
        objectLayer: moving,
        motionType: MotionType.DYNAMIC,
        position: [2 + (i % 3) * 0.3, 1 + i * 0.9, 4 + (i % 4) * 0.25],
    });
}

const options = {
    bodies: true,
    velocities: false,
    contacts: true,
    joints: true,
    colorMode: debug.BodyColorMode.MOTION_TYPE as debug.BodyColorMode,
};
const gui = new GUI();
gui.add(options, 'bodies');
gui.add(options, 'velocities');
gui.add(options, 'contacts');
gui.add(options, 'joints');
gui.add(options, 'colorMode', {
    'motion type': debug.BodyColorMode.MOTION_TYPE,
    sleeping: debug.BodyColorMode.SLEEPING,
    island: debug.BodyColorMode.ISLAND,
});

const buffers = debugDraw.create();
const debugLines = createDebugLines(view.scene);

function draw(): void {
    debugDraw.clear(buffers);
    if (options.bodies)
        debug.bodies(buffers, world, {
            colorMode: options.colorMode,
            showLinearVelocity: options.velocities,
            showAngularVelocity: options.velocities,
        });
    if (options.contacts) debug.contactConstraints(buffers, world);
    if (options.joints) debug.joints(buffers, world);
    // views onto the wasm's memory: read them now, before anything draws again
    const { lines, lineCount } = debugDraw.read(buffers);
    showDebugLines(debugLines, lines, lineCount);
}

loop(
    (deltaTime) => updateWorld(world, undefined, deltaTime),
    () => {
        draw();
        view.render();
    },
);
