import type { PageMessage, WorkerMessage } from './messages';

// the page hands its canvas to the worker, which steps the world and draws it, and passes on what the visitor does

const canvas = document.createElement('canvas');
document.getElementById('root')!.appendChild(canvas);
const offscreen = canvas.transferControlToOffscreen();

const worker = new Worker(new URL('./demo.worker.ts', import.meta.url), { type: 'module' });
const send = (message: PageMessage, transfer: Transferable[] = []) => worker.postMessage(message, transfer);

const asked = new URLSearchParams(location.search).get('renderer');
const renderer = asked === 'webgpu' || asked === 'webgl' ? asked : null;
send(
    {
        type: 'start',
        canvas: offscreen,
        width: window.innerWidth,
        height: window.innerHeight,
        pixelRatio: devicePixelRatio,
        renderer,
    },
    [offscreen],
);

worker.onmessage = (event: MessageEvent<WorkerMessage>) => {
    if (event.data.type === 'ready') document.getElementById('loading')?.remove();
};

// the pointer anywhere on the page steers the runner
window.addEventListener('pointermove', (event) => send({ type: 'pointer', x: event.clientX, y: event.clientY }));

window.addEventListener('resize', () =>
    send({ type: 'resize', width: window.innerWidth, height: window.innerHeight, pixelRatio: devicePixelRatio }),
);

let showDebug = false;
const debugToggle = document.getElementById('debug-toggle')!;
const toggleDebug = () => {
    showDebug = !showDebug;
    debugToggle.textContent = showDebug ? 'hide debug view [d]' : 'show debug view [d]';
    send({ type: 'debug', show: showDebug });
};
debugToggle.addEventListener('click', toggleDebug);
window.addEventListener('keydown', (event) => {
    if (event.key === 'd' || event.key === 'D') toggleDebug();
});

// the worker steps and draws on the page's frames, which keep time with the display in every browser
const tick = (time: number) => {
    send({ type: 'frame', time });
    requestAnimationFrame(tick);
};
requestAnimationFrame(tick);
