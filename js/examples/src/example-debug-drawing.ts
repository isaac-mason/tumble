import * as g from 'gpucat';
import GUI from 'lil-gui';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    capsule,
    createWorld,
    createWorldSettings,
    DEBUG_VERTEX_STRIDE,
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
import { caption, loop, setup } from './common';

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

/** a frame's lines of one colour */
type Bucket = { points: Float32Array; count: number; geometry: g.LineSegmentsGeometry; lines: g.LineSegments };
const buckets = new Map<number, Bucket>();

function bucketOf(r: number, gr: number, b: number): Bucket {
    const key = (Math.round(r * 255) << 16) | (Math.round(gr * 255) << 8) | Math.round(b * 255);
    let bucket = buckets.get(key);
    if (bucket === undefined) {
        const points = new Float32Array(6 * 1024);
        // it grows as an update needs
        const geometry = new g.LineSegmentsGeometry(new Float32Array(6), 2048);
        const lines = new g.LineSegments(geometry, new g.LineMaterial({ color: g.vec4f(r, gr, b, 1) as never, lineWidth: 1.5 }));
        view.scene.add(lines);
        bucket = { points, count: 0, geometry, lines };
        buckets.set(key, bucket);
    }
    return bucket;
}

const buffers = debugDraw.create();

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
    for (const bucket of buckets.values()) bucket.count = 0;
    for (let i = 0; i < lineCount; i++) {
        const a = i * 2 * DEBUG_VERTEX_STRIDE;
        const b = a + DEBUG_VERTEX_STRIDE;
        const bucket = bucketOf(lines[a + 3]!, lines[a + 4]!, lines[a + 5]!);
        if ((bucket.count + 2) * 3 > bucket.points.length) {
            const grown = new Float32Array(bucket.points.length * 2);
            grown.set(bucket.points);
            bucket.points = grown;
        }
        const points = bucket.points;
        const to = bucket.count * 3;
        points[to] = lines[a]!;
        points[to + 1] = lines[a + 1]!;
        points[to + 2] = lines[a + 2]!;
        points[to + 3] = lines[b]!;
        points[to + 4] = lines[b + 1]!;
        points[to + 5] = lines[b + 2]!;
        bucket.count += 2;
    }
    for (const bucket of buckets.values()) {
        bucket.lines.visible = bucket.count > 0;
        if (bucket.count > 0) bucket.geometry.update(bucket.points.subarray(0, bucket.count * 3));
    }
}

loop(
    (deltaTime) => updateWorld(world, undefined, deltaTime),
    () => {
        draw();
        view.render();
    },
);
