import { quat, vec3 } from 'math';
import {
    addBroadphaseLayer,
    addObjectLayer,
    type BodyId,
    box,
    capsule,
    createWorld,
    createWorldSettings,
    cylinder,
    DEBUG_VERTEX_STRIDE,
    debug,
    debugDraw,
    enableCollision,
    init,
    MotionType,
    type Quat,
    rigidBody,
    type Shape,
    sphere,
    updateWorld,
    type Vec3,
} from 'tumble.js';
import wasm from 'tumble.js/tumble.wasm?url';
import type { ShapeMesh } from '../examples/src/common';
import { type FrameRequest, KINDS, POSE_STRIDE, type WorkerMessage } from './shared';

const STEP = 1 / 60;
const BODY_COUNT = 300;

await init({ wasm });

const settings = createWorldSettings();
const moving = addObjectLayer(settings, addBroadphaseLayer(settings));
const notMoving = addObjectLayer(settings, addBroadphaseLayer(settings));
enableCollision(settings, moving, moving);
enableCollision(settings, moving, notMoving);
const world = createWorld(settings);

rigidBody.create(world, {
    shape: box.create({ halfExtents: [10, 0.5, 10] }),
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    position: [0, -0.5, 0],
});

function shapeOf(mesh: ShapeMesh): Shape {
    switch (mesh.kind) {
        case 'box':
            return box.create({ halfExtents: mesh.halfExtents });
        case 'sphere':
            return sphere.create({ radius: mesh.radius });
        case 'capsule':
            return capsule.create({ halfHeightOfCylinder: mesh.halfHeightOfCylinder, radius: mesh.radius });
        case 'cylinder':
            return cylinder.create({ halfHeight: mesh.halfHeight, radius: mesh.radius });
    }
}
const shapes = KINDS.map((kind) => shapeOf(kind.mesh));

let seed = 1;
const random = () => {
    seed = (seed * 1664525 + 1013904223) >>> 0;
    return seed / 4294967296;
};

const bodies: BodyId[] = [];
const kinds = new Uint8Array(BODY_COUNT);
for (let i = 0; i < BODY_COUNT; i++) {
    const kind = Math.floor(random() * KINDS.length);
    const axis = [random() - 0.5, random() - 0.5, random() - 0.5];
    const length = Math.hypot(axis[0]!, axis[1]!, axis[2]!);
    const angle = random() * Math.PI;
    const s = Math.sin(angle / 2) / length;
    kinds[i] = kind;
    bodies.push(
        rigidBody.create(world, {
            shape: shapes[kind]!,
            objectLayer: moving,
            motionType: MotionType.DYNAMIC,
            position: [(random() - 0.5) * 6, 2 + i * 0.12, (random() - 0.5) * 6],
            quaternion: [axis[0]! * s, axis[1]! * s, axis[2]! * s, Math.cos(angle / 2)],
            friction: 0.5,
        }),
    );
}

// every body's pose as the last step began, and as it left it
let before = new Float32Array(BODY_COUNT * POSE_STRIDE);
let now = new Float32Array(BODY_COUNT * POSE_STRIDE);
const position: Vec3 = [0, 0, 0];
const quaternion: Quat = [0, 0, 0, 1];

function readPoses(out: Float32Array): void {
    for (let i = 0; i < BODY_COUNT; i++) {
        rigidBody.getTransform(position, quaternion, world, bodies[i]!);
        out.set(position, i * POSE_STRIDE);
        out.set(quaternion, i * POSE_STRIDE + 3);
    }
}
readPoses(now);
before.set(now);

const beforePosition: Vec3 = [0, 0, 0];
const nowPosition: Vec3 = [0, 0, 0];
const beforeQuaternion: Quat = [0, 0, 0, 1];
const nowQuaternion: Quat = [0, 0, 0, 1];

function blendPoses(out: Float32Array, alpha: number): void {
    for (let i = 0; i < BODY_COUNT; i++) {
        const at = i * POSE_STRIDE;
        vec3.set(beforePosition, before[at]!, before[at + 1]!, before[at + 2]!);
        vec3.set(nowPosition, now[at]!, now[at + 1]!, now[at + 2]!);
        quat.set(beforeQuaternion, before[at + 3]!, before[at + 4]!, before[at + 5]!, before[at + 6]!);
        quat.set(nowQuaternion, now[at + 3]!, now[at + 4]!, now[at + 5]!, now[at + 6]!);
        vec3.lerp(position, beforePosition, nowPosition, alpha);
        quat.slerp(quaternion, beforeQuaternion, nowQuaternion, alpha);
        out.set(position, at);
        out.set(quaternion, at + 3);
    }
}

const debugBuffers = debugDraw.create();
let owed = 0;

self.onmessage = (event: MessageEvent<FrameRequest>) => {
    const { deltaTime, debug: showDebug, poses } = event.data;
    // at most 4 steps a frame, so a long pause does not stall the next frame
    owed += Math.min(deltaTime, 4 * STEP);
    while (owed >= STEP) {
        updateWorld(world, undefined, STEP);
        [before, now] = [now, before];
        readPoses(now);
        owed -= STEP;
    }
    blendPoses(poses, owed / STEP);

    let lines: Float32Array | null = null;
    let lineCount = 0;
    if (showDebug) {
        debugDraw.clear(debugBuffers);
        debug.bodies(debugBuffers, world, {});
        const read = debugDraw.read(debugBuffers);
        // the read is a view onto the wasm's memory, so it is copied out to be sent
        lineCount = read.lineCount;
        lines = read.lines.slice(0, lineCount * 2 * DEBUG_VERTEX_STRIDE);
    }

    const message: WorkerMessage = { type: 'frame', poses, lines, lineCount };
    self.postMessage(message, { transfer: lines ? [poses.buffer, lines.buffer] : [poses.buffer] });
};

const ready: WorkerMessage = { type: 'ready', kinds };
self.postMessage(ready);
