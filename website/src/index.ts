import type * as gpucat from 'gpucat';
import { createDebugLines, meshOf, setup, showDebugLines } from '../examples/src/common';
import { type FrameRequest, KINDS, POSE_STRIDE, type WorkerMessage } from './shared';

const view = await setup([12, 9, 14], [0, 2, 0]);

const groundMesh = meshOf({ kind: 'box', halfExtents: [10, 0.5, 10] }, '#5a5f66');
groundMesh.position[1] = -0.5;
view.scene.add(groundMesh);

const debugLines = createDebugLines(view.scene);
let showDebug = false;
const debugToggle = document.getElementById('debug-toggle')!;
const toggleDebug = () => {
    showDebug = !showDebug;
    debugToggle.textContent = showDebug ? 'hide debug view [d]' : 'show debug view [d]';
};
debugToggle.addEventListener('click', toggleDebug);
window.addEventListener('keydown', (event) => {
    if (event.key === 'd' || event.key === 'D') toggleDebug();
});

// the world lives in the worker. each frame the page sends it the time passed and the poses buffer, and the worker steps
// and sends the buffer back filled. the buffer is transferred, never copied, so it is here only between the two
const worker = new Worker(new URL('./physics.worker.ts', import.meta.url), { type: 'module' });
const meshes: gpucat.Object3D[] = [];
const noLines = new Float32Array(0);
let poses: Float32Array | null = null;
let last = 0;

worker.onmessage = (event: MessageEvent<WorkerMessage>) => {
    const message = event.data;
    if (message.type === 'ready') {
        for (const kind of message.kinds) {
            const mesh = meshOf(KINDS[kind]!.mesh, KINDS[kind]!.color);
            meshes.push(mesh);
            view.scene.add(mesh);
        }
        poses = new Float32Array(meshes.length * POSE_STRIDE);
        last = performance.now();
        document.getElementById('loading')!.remove();
        requestAnimationFrame(frame);
        return;
    }
    poses = message.poses;
    for (let i = 0; i < meshes.length; i++) {
        const mesh = meshes[i]!;
        const at = i * POSE_STRIDE;
        mesh.position[0] = poses[at]!;
        mesh.position[1] = poses[at + 1]!;
        mesh.position[2] = poses[at + 2]!;
        mesh.quaternion[0] = poses[at + 3]!;
        mesh.quaternion[1] = poses[at + 4]!;
        mesh.quaternion[2] = poses[at + 5]!;
        mesh.quaternion[3] = poses[at + 6]!;
    }
    showDebugLines(debugLines, message.lines ?? noLines, message.lineCount);
};

function frame(time: number): void {
    // while the worker has the buffer, the time owed carries over to the next request
    if (poses !== null) {
        const request: FrameRequest = { deltaTime: (time - last) / 1000, debug: showDebug, poses };
        worker.postMessage(request, [poses.buffer]);
        poses = null;
        last = time;
    }
    view.render();
    requestAnimationFrame(frame);
}
