import { type Mat4, mat4, type Quat, quat, type Vec3, vec3 } from 'math';
import { type BodyId, debug, debugDraw, init, rigidBody } from 'tumble.js';
import { createDemo, type Drawable, reachAt, STEP, stepDemo, steer, takeChanges } from './demo';
import type { PageMessage, WorkerMessage } from './messages';
import { addInstance, beginInstances, createDebugLines, createInstances, createSky, endInstances, showDebugLines } from './scene';
import { createView, renderView, resizeView } from './view';

// the landing page, all of it but the page's input: the world, stepped at a fixed rate on the page's frames, and drawn
// on the canvas the page handed over, each body between its last two steps, by a camera that follows the runner

// what the page sends while tumble and the renderer start is kept for them
const waiting: PageMessage[] = [];
let handle: ((message: PageMessage) => void) | null = null;
self.onmessage = (event: MessageEvent<PageMessage>) => {
    if (handle === null) waiting.push(event.data);
    else handle(event.data);
};

const post = (message: WorkerMessage) => self.postMessage(message);

await init();

function nextStart(): Promise<Extract<PageMessage, { type: 'start' }>> {
    const index = waiting.findIndex((message) => message.type === 'start');
    if (index >= 0) return Promise.resolve(waiting.splice(index, 1)[0] as Extract<PageMessage, { type: 'start' }>);
    return new Promise((resolve) => {
        handle = (message) => {
            if (message.type === 'start') resolve(message);
            else waiting.push(message);
        };
    });
}

/** the camera, behind the runner at about its shoulders, looking past it at what is coming */
const EYE_OFFSET: Vec3 = [0, 1.45, 4.6];
const LOOK_AHEAD: Vec3 = [0, 0.6, -8];
/** how much of the runner's sideways the camera follows, never nearer the walls where it is than this past the runner's reach */
const FOLLOW_ACROSS = 0.6;
const CAMERA_PAST_REACH = 0.3;
const FOV = (55 * Math.PI) / 180;

const start = await nextStart();
const demo = createDemo();
const target = vec3.clone(demo.focus);
const view = await createView(
    start.canvas,
    start.width,
    start.height,
    start.pixelRatio,
    vec3.add([0, 0, 0], target, EYE_OFFSET),
    vec3.add([0, 0, 0], target, LOOK_AHEAD),
    FOV,
);
const size = { width: start.width, height: start.height };
const sky = createSky(view.scene);
const instances = createInstances(view.scene);

/** a body's looks, each with its matrix in the body's frame, and its pose as the last step began and as it left it */
type Drawn = Drawable & {
    lookMatrices: Mat4[];
    beforePosition: Vec3;
    beforeQuaternion: Quat;
    nowPosition: Vec3;
    nowQuaternion: Quat;
};

const drawn = new Map<BodyId, Drawn>();

function show(drawable: Drawable): void {
    const entry: Drawn = {
        ...drawable,
        lookMatrices: drawable.looks.map((look) =>
            mat4.fromRotationTranslationScale(mat4.create(), look.rotation, look.offset, look.size),
        ),
        beforePosition: [0, 0, 0],
        beforeQuaternion: [0, 0, 0, 1],
        nowPosition: [0, 0, 0],
        nowQuaternion: [0, 0, 0, 1],
    };
    rigidBody.getTransform(entry.nowPosition, entry.nowQuaternion, demo.world, entry.body);
    vec3.copy(entry.beforePosition, entry.nowPosition);
    quat.copy(entry.beforeQuaternion, entry.nowQuaternion);
    drawn.set(drawable.body, entry);
}

/** the drawing caught up with the world: new bodies shown, gone ones taken away, and a rebase followed */
function catchUp(): void {
    for (const change of takeChanges(demo)) {
        if (change.type === 'add') show(change.drawable);
        else if (change.type === 'remove') drawn.delete(change.body);
        else {
            for (const entry of drawn.values()) {
                vec3.add(entry.beforePosition, entry.beforePosition, change.by);
                vec3.add(entry.nowPosition, entry.nowPosition, change.by);
            }
        }
    }
}
catchUp();

const debugLines = createDebugLines(view.scene);
const debugBuffers = debugDraw.create();
const noLines = new Float32Array(0);
let showDebug = false;

let last = -1;
let owed = 0;
const lookAt: Vec3 = [0, 0, 0];
const _position: Vec3 = [0, 0, 0];
const _quaternion: Quat = [0, 0, 0, 1];
const _body = mat4.create();
const _instance = mat4.create();

function frame(time: number): void {
    // at most 4 steps a frame, so a long pause does not stall the next
    const elapsed = last >= 0 ? Math.min((time - last) / 1000, 4 * STEP) : 0;
    owed += elapsed;
    last = time;
    while (owed >= STEP) {
        stepDemo(demo);
        catchUp();
        for (const entry of drawn.values()) {
            vec3.copy(entry.beforePosition, entry.nowPosition);
            quat.copy(entry.beforeQuaternion, entry.nowQuaternion);
            rigidBody.getTransform(entry.nowPosition, entry.nowQuaternion, demo.world, entry.body);
        }
        owed -= STEP;
    }

    const alpha = owed / STEP;
    beginInstances(instances);
    for (const entry of drawn.values()) {
        vec3.lerp(_position, entry.beforePosition, entry.nowPosition, alpha);
        quat.slerp(_quaternion, entry.beforeQuaternion, entry.nowQuaternion, alpha);
        mat4.fromRotationTranslation(_body, _quaternion, _position);
        if (entry.scale) mat4.scale(_body, _body, entry.scale);
        for (let i = 0; i < entry.looks.length; i++) {
            const look = entry.looks[i]!;
            addInstance(
                instances,
                look.primitive,
                mat4.multiply(_instance, _body, entry.lookMatrices[i]!),
                look.color,
                entry.fades,
            );
        }
    }
    endInstances(instances);

    // the camera keeps behind the runner as it is drawn, following some of the way across
    const pelvis = drawn.get(demo.runner.parts[0]!)!;
    vec3.lerp(target, pelvis.beforePosition, pelvis.nowPosition, alpha);
    const cameraReach = reachAt(demo, target[2] + EYE_OFFSET[2]) + CAMERA_PAST_REACH;
    target[0] = Math.max(-cameraReach, Math.min(cameraReach, target[0] * FOLLOW_ACROSS));
    target[1] = demo.focus[1];
    vec3.add(view.camera.position as Vec3, target, EYE_OFFSET);
    view.camera.lookAt(vec3.add(lookAt, target, LOOK_AHEAD));
    vec3.copy(sky.position as Vec3, view.camera.position as Vec3);

    if (showDebug) {
        debugDraw.clear(debugBuffers);
        debug.bodies(debugBuffers, demo.world, {});
        const lines = debugDraw.read(debugBuffers);
        showDebugLines(debugLines, lines.lines, lines.lineCount);
    } else showDebugLines(debugLines, noLines, 0);

    renderView(view);
}

handle = (message) => {
    switch (message.type) {
        case 'resize':
            size.width = message.width;
            size.height = message.height;
            resizeView(view, message.width, message.height, message.pixelRatio);
            break;
        case 'frame':
            frame(message.time);
            break;
        case 'debug':
            showDebug = message.show;
            break;
        case 'pointer':
            steer(demo, (message.x / size.width) * 2 - 1);
            break;
        case 'start':
            break;
    }
};
for (const message of waiting.splice(0)) handle(message);

renderView(view);
post({ type: 'ready' });
