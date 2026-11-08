import type * as g from 'gpucat';
import { type Quat, quat, type Vec3, vec3 } from 'math';
import { debug, debugDraw, init, rigidBody } from 'tumble.js';
import { createDemo, type Drawable, STEP, stepDemo } from './demo';
import type { PageMessage, WorkerMessage } from './messages';
import { createDebugLines, meshOf, showDebugLines } from './scene';
import { createView, renderView, resizeView } from './view';

// the landing page, all of it but the page's input: the world, stepped at a fixed rate on the page's frames, and drawn
// on the canvas the page handed over, each body between its last two steps

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

const start = await nextStart();
const view = await createView(start.canvas, start.width, start.height, start.pixelRatio, [6, 4.5, 7], [0, 0.5, 0]);
const demo = createDemo();

/** a body's mesh, and its pose as the last step began and as it left it */
type Drawn = Drawable & { object: g.Mesh; beforePosition: Vec3; beforeQuaternion: Quat; nowPosition: Vec3; nowQuaternion: Quat };

const drawn: Drawn[] = demo.bodies.map((drawable) => {
    const entry: Drawn = {
        ...drawable,
        object: meshOf(drawable.mesh, drawable.color),
        beforePosition: [0, 0, 0],
        beforeQuaternion: [0, 0, 0, 1],
        nowPosition: [0, 0, 0],
        nowQuaternion: [0, 0, 0, 1],
    };
    rigidBody.getTransform(entry.nowPosition, entry.nowQuaternion, demo.world, entry.body);
    vec3.copy(entry.beforePosition, entry.nowPosition);
    quat.copy(entry.beforeQuaternion, entry.nowQuaternion);
    view.scene.add(entry.object);
    return entry;
});

const debugLines = createDebugLines(view.scene);
const debugBuffers = debugDraw.create();
const noLines = new Float32Array(0);
let showDebug = false;

let last = -1;
let owed = 0;

function frame(time: number): void {
    // at most 4 steps a frame, so a long pause does not stall the next
    if (last >= 0) owed += Math.min((time - last) / 1000, 4 * STEP);
    last = time;
    while (owed >= STEP) {
        stepDemo(demo);
        for (const entry of drawn) {
            vec3.copy(entry.beforePosition, entry.nowPosition);
            quat.copy(entry.beforeQuaternion, entry.nowQuaternion);
            rigidBody.getTransform(entry.nowPosition, entry.nowQuaternion, demo.world, entry.body);
        }
        owed -= STEP;
    }

    const alpha = owed / STEP;
    for (const entry of drawn) {
        vec3.lerp(entry.object.position as Vec3, entry.beforePosition, entry.nowPosition, alpha);
        quat.slerp(entry.object.quaternion as Quat, entry.beforeQuaternion, entry.nowQuaternion, alpha);
    }

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
            resizeView(view, message.width, message.height, message.pixelRatio);
            break;
        case 'frame':
            frame(message.time);
            break;
        case 'debug':
            showDebug = message.show;
            break;
        case 'start':
            break;
    }
};
for (const message of waiting.splice(0)) handle(message);

renderView(view);
post({ type: 'ready' });
