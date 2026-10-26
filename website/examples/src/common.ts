import * as g from 'gpucat';
import { type Quat, quat, type Vec3, vec3 } from 'math';
import {
    type ActiveBody,
    type BodyId,
    createActiveBody,
    DEBUG_VERTEX_STRIDE,
    getActiveBodies,
    getActiveBodyAt,
    init as initTumble,
    type Listener,
    rigidBody,
    type World,
} from 'tumble.js';
import wasm from 'tumble.js/tumble.wasm?url';

export type View = {
    renderer: Awaited<ReturnType<typeof g.init>>;
    scene: g.Scene;
    camera: g.PerspectiveCamera;
    controls: g.OrbitControls;
    render: () => void;
};

/** the wasm, from beside the page */
export function loadTumble(): Promise<void> {
    return initTumble({ wasm });
}

/** a webgpu renderer onto a full window canvas, a camera orbiting the target */
export async function setup(eye: [number, number, number], target: [number, number, number]): Promise<View> {
    const canvas = document.createElement('canvas');
    canvas.style.display = 'block';
    document.body.appendChild(canvas);
    const view = g.createCanvasTarget(canvas, { samples: 4 });
    view.setPixelRatio(devicePixelRatio);
    view.setSize(window.innerWidth, window.innerHeight);
    const renderer = await g.init(g.webgpu());

    const scene = new g.Scene();
    const camera = new g.PerspectiveCamera(Math.PI / 3, window.innerWidth / window.innerHeight, 0.1, 500);
    camera.position[0] = eye[0];
    camera.position[1] = eye[1];
    camera.position[2] = eye[2];
    scene.add(camera);
    const controls = new g.OrbitControls(camera, canvas);
    controls.target[0] = target[0];
    controls.target[1] = target[1];
    controls.target[2] = target[2];
    controls.update();

    window.addEventListener('resize', () => {
        view.setSize(window.innerWidth, window.innerHeight);
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
    });

    const composite = g.fullscreen(g.renderOutput(g.renderTexture(scene, camera).getTextureNode()));
    const render = () => {
        controls.update();
        scene.updateWorldMatrix();
        camera.updateViewMatrix();
        const frame = g.frame(renderer);
        const pass = frame.pass({ target: view });
        pass.draw(composite);
        pass.end();
        frame.submit();
    };
    return { renderer, scene, camera, controls, render };
}

const materials = new Map<string, g.Material>();

/** lit by a light from above, one material per colour */
export function lit(color: string): g.Material {
    const known = materials.get(color);
    if (known) return known;
    const position = g.attribute('position', g.d.vec3f);
    const normal = g.attribute('normal', g.d.vec3f);
    const world = g.mul(g.modelWorldMatrix, g.vec4(position, g.f32(1)));
    const clip = g.mul(g.cameraProjectionMatrix, g.mul(g.cameraViewMatrix, world));
    const worldNormal = g.varying(g.normalize(g.mul(g.modelNormalMatrix, normal)), 'v_normal');
    const light = g.vec3(g.f32(0.4), g.f32(1), g.f32(0.3)).normalize();
    const shade = g.f32(0.35).add(g.f32(0.65).mul(worldNormal.dot(light).max(g.f32(0))));
    const base = g.rgb(color);
    const material = g.createMaterial({ vertex: clip, fragment: g.vec4(g.vec3(base.x, base.y, base.z).mul(shade), g.f32(1)) });
    materials.set(color, material);
    return material;
}

/** the shapes the examples make, by their settings: a mesh in the shape's own space */
export type ShapeMesh =
    | { kind: 'box'; halfExtents: [number, number, number] }
    | { kind: 'sphere'; radius: number }
    | { kind: 'capsule'; halfHeightOfCylinder: number; radius: number }
    | { kind: 'cylinder'; halfHeight: number; radius: number };

export function meshOf(shape: ShapeMesh, color: string): g.Object3D {
    const material = lit(color);
    switch (shape.kind) {
        case 'box': {
            const [x, y, z] = shape.halfExtents;
            return new g.Mesh(g.createBoxGeometry(2 * x, 2 * y, 2 * z), material);
        }
        case 'sphere':
            return new g.Mesh(g.createSphereGeometry(shape.radius, 24, 16), material);
        case 'cylinder':
            return new g.Mesh(g.createCylinderGeometry(shape.radius, shape.radius, 2 * shape.halfHeight, 24), material);
        case 'capsule': {
            // a cylinder with a sphere on each end
            const group = new g.Object3D();
            group.add(
                new g.Mesh(g.createCylinderGeometry(shape.radius, shape.radius, 2 * shape.halfHeightOfCylinder, 24), material),
            );
            for (const end of [-1, 1]) {
                const cap = new g.Mesh(g.createSphereGeometry(shape.radius, 24, 16), material);
                cap.position[1] = end * shape.halfHeightOfCylinder;
                group.add(cap);
            }
            return group;
        }
    }
}

/**
 * steps at a fixed 60 a second, at most 4 a frame, then renders. frame gets how far it is past the last step, 0 to 1:
 * what it draws blends the state as that step began into the state it left by this, so motion is even at any frame rate
 */
export function loop(step: (deltaTime: number) => void, frame: (alpha: number) => void): void {
    const deltaTime = 1 / 60;
    let last = performance.now();
    let owed = 0;
    const tick = (now: number) => {
        owed += Math.min((now - last) / 1000, 4 * deltaTime);
        last = now;
        while (owed >= deltaTime) {
            step(deltaTime);
            owed -= deltaTime;
        }
        frame(owed / deltaTime);
        requestAnimationFrame(tick);
    };
    requestAnimationFrame(tick);
}

/** the poses of the bodies a renderer draws, read after every step: each as that step began and as the step left it */
export type StepPoses = {
    world: World;
    /** for updateWorld: names the bodies a step puts to sleep, whose last pose the active list no longer has */
    listener: Listener;
    slots: Map<BodyId, number>;
    /** by slot */
    bodies: BodyId[];
    beforePositions: Float32Array;
    beforeQuaternions: Float32Array;
    nowPositions: Float32Array;
    nowQuaternions: Float32Array;
    /** the slots of the bodies the last step put to sleep */
    slept: number[];
    activeBody: ActiveBody;
};

export function createStepPoses(world: World): StepPoses {
    const stepPoses: StepPoses = {
        world,
        listener: {
            onBodyDeactivated: (body) => {
                const slot = stepPoses.slots.get(body);
                if (slot !== undefined) stepPoses.slept.push(slot);
            },
        },
        slots: new Map(),
        bodies: [],
        beforePositions: new Float32Array(0),
        beforeQuaternions: new Float32Array(0),
        nowPositions: new Float32Array(0),
        nowQuaternions: new Float32Array(0),
        slept: [],
        activeBody: createActiveBody(),
    };
    return stepPoses;
}

function grown(array: Float32Array, length: number): Float32Array {
    if (array.length >= length) return array;
    const next = new Float32Array(Math.max(length, array.length * 2));
    next.set(array);
    return next;
}

const _position: Vec3 = [0, 0, 0];
const _quaternion: Quat = [0, 0, 0, 1];

/** the body's pose now, from the world, into the slot's now */
function readNow(stepPoses: StepPoses, body: BodyId, slot: number): void {
    rigidBody.getPosition(_position, stepPoses.world, body);
    rigidBody.getQuaternion(_quaternion, stepPoses.world, body);
    const p = slot * 3;
    const q = slot * 4;
    const { nowPositions, nowQuaternions } = stepPoses;
    nowPositions[p] = _position[0];
    nowPositions[p + 1] = _position[1];
    nowPositions[p + 2] = _position[2];
    nowQuaternions[q] = _quaternion[0];
    nowQuaternions[q + 1] = _quaternion[1];
    nowQuaternions[q + 2] = _quaternion[2];
    nowQuaternions[q + 3] = _quaternion[3];
}

/** a slot for the body, its pose read now and taken as before too, so it is drawn where it is until the next step */
export function addStepPose(stepPoses: StepPoses, body: BodyId): number {
    const slot = stepPoses.bodies.length;
    stepPoses.slots.set(body, slot);
    stepPoses.bodies.push(body);
    stepPoses.beforePositions = grown(stepPoses.beforePositions, (slot + 1) * 3);
    stepPoses.nowPositions = grown(stepPoses.nowPositions, (slot + 1) * 3);
    stepPoses.beforeQuaternions = grown(stepPoses.beforeQuaternions, (slot + 1) * 4);
    stepPoses.nowQuaternions = grown(stepPoses.nowQuaternions, (slot + 1) * 4);
    readNow(stepPoses, body, slot);
    const p = slot * 3;
    const q = slot * 4;
    const { beforePositions, beforeQuaternions, nowPositions, nowQuaternions } = stepPoses;
    beforePositions[p] = nowPositions[p]!;
    beforePositions[p + 1] = nowPositions[p + 1]!;
    beforePositions[p + 2] = nowPositions[p + 2]!;
    beforeQuaternions[q] = nowQuaternions[q]!;
    beforeQuaternions[q + 1] = nowQuaternions[q + 1]!;
    beforeQuaternions[q + 2] = nowQuaternions[q + 2]!;
    beforeQuaternions[q + 3] = nowQuaternions[q + 3]!;
    return slot;
}

/**
 * after a updateWorld given stepPoses.listener: what was now is as the step began, and the step's result is now. the
 * bodies it moved are the active list; the ones it put to sleep, their last pose read once
 */
export function readStepPoses(stepPoses: StepPoses): void {
    const { world, slots, bodies, slept, activeBody, beforePositions, beforeQuaternions, nowPositions, nowQuaternions } =
        stepPoses;
    beforePositions.set(nowPositions);
    beforeQuaternions.set(nowQuaternions);
    for (const slot of slept) readNow(stepPoses, bodies[slot]!, slot);
    slept.length = 0;
    const count = getActiveBodies(world);
    for (let i = 0; i < count; i++) {
        getActiveBodyAt(activeBody, world, i);
        const slot = slots.get(activeBody.body);
        if (slot === undefined) continue;
        const p = slot * 3;
        const q = slot * 4;
        nowPositions[p] = activeBody.position[0];
        nowPositions[p + 1] = activeBody.position[1];
        nowPositions[p + 2] = activeBody.position[2];
        nowQuaternions[q] = activeBody.quaternion[0];
        nowQuaternions[q + 1] = activeBody.quaternion[1];
        nowQuaternions[q + 2] = activeBody.quaternion[2];
        nowQuaternions[q + 3] = activeBody.quaternion[3];
    }
}

const _beforePosition: Vec3 = [0, 0, 0];
const _nowPosition: Vec3 = [0, 0, 0];
const _beforeQuaternion: Quat = [0, 0, 0, 1];
const _nowQuaternion: Quat = [0, 0, 0, 1];

/** the slot's pose between the last two steps, by alpha, into a mesh's position and quaternion */
export function blendStepPose(stepPoses: StepPoses, slot: number, alpha: number, position: Vec3, quaternion: Quat): void {
    const { beforePositions, beforeQuaternions, nowPositions, nowQuaternions } = stepPoses;
    const p = slot * 3;
    const q = slot * 4;
    _beforePosition[0] = beforePositions[p]!;
    _beforePosition[1] = beforePositions[p + 1]!;
    _beforePosition[2] = beforePositions[p + 2]!;
    _nowPosition[0] = nowPositions[p]!;
    _nowPosition[1] = nowPositions[p + 1]!;
    _nowPosition[2] = nowPositions[p + 2]!;
    _beforeQuaternion[0] = beforeQuaternions[q]!;
    _beforeQuaternion[1] = beforeQuaternions[q + 1]!;
    _beforeQuaternion[2] = beforeQuaternions[q + 2]!;
    _beforeQuaternion[3] = beforeQuaternions[q + 3]!;
    _nowQuaternion[0] = nowQuaternions[q]!;
    _nowQuaternion[1] = nowQuaternions[q + 1]!;
    _nowQuaternion[2] = nowQuaternions[q + 2]!;
    _nowQuaternion[3] = nowQuaternions[q + 3]!;
    vec3.lerp(position, _beforePosition, _nowPosition, alpha);
    quat.slerp(quaternion, _beforeQuaternion, _nowQuaternion, alpha);
}

/** a frame's lines of one colour */
type DebugBucket = { points: Float32Array; count: number; geometry: g.LineSegmentsGeometry; lines: g.LineSegments };

/** the library's debug lines, shown in the scene as line segments, one set per colour */
export type DebugLines = { scene: g.Scene; buckets: Map<number, DebugBucket> };

export function createDebugLines(scene: g.Scene): DebugLines {
    return { scene, buckets: new Map() };
}

function bucketOf(debugLines: DebugLines, r: number, gr: number, b: number): DebugBucket {
    const key = (Math.round(r * 255) << 16) | (Math.round(gr * 255) << 8) | Math.round(b * 255);
    let bucket = debugLines.buckets.get(key);
    if (bucket === undefined) {
        const points = new Float32Array(6 * 1024);
        // it grows as an update needs
        const geometry = new g.LineSegmentsGeometry(new Float32Array(6), 2048);
        const lines = new g.LineSegments(geometry, new g.LineMaterial({ color: g.vec4f(r, gr, b, 1) as never, lineWidth: 1.5 }));
        debugLines.scene.add(lines);
        bucket = { points, count: 0, geometry, lines };
        debugLines.buckets.set(key, bucket);
    }
    return bucket;
}

/** lines as debugDraw.read gives them, into the scene's line segments, in place of the last */
export function showDebugLines(debugLines: DebugLines, lines: Float32Array, lineCount: number): void {
    for (const bucket of debugLines.buckets.values()) bucket.count = 0;
    for (let i = 0; i < lineCount; i++) {
        const a = i * 2 * DEBUG_VERTEX_STRIDE;
        const b = a + DEBUG_VERTEX_STRIDE;
        const bucket = bucketOf(debugLines, lines[a + 3]!, lines[a + 4]!, lines[a + 5]!);
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
    for (const bucket of debugLines.buckets.values()) {
        bucket.lines.visible = bucket.count > 0;
        if (bucket.count > 0) bucket.geometry.update(bucket.points.subarray(0, bucket.count * 3));
    }
}

/** a line of help over the canvas */
export function caption(text: string): void {
    const element = document.createElement('div');
    element.className = 'caption';
    element.textContent = text;
    document.body.appendChild(element);
}
