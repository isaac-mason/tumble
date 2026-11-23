import type { RendererChoice } from './view';

// the page and the worker: the page hands over the canvas and passes on what the visitor does, and the worker does the
// rest. sizes and pointer positions are css pixels

export type PageMessage =
    /** renderer: the backend the page's `?renderer=webgpu` or `?renderer=webgl` asks for, else null for whichever there is */
    | { type: 'start'; canvas: OffscreenCanvas; width: number; height: number; pixelRatio: number; renderer: RendererChoice }
    | { type: 'resize'; width: number; height: number; pixelRatio: number }
    /** each of the page's animation frames, at its time in ms: the worker steps and draws on these */
    | { type: 'frame'; time: number }
    | { type: 'debug'; show: boolean }
    /** the pointer moving, wherever it is on the page */
    | { type: 'pointer'; x: number; y: number };

export type WorkerMessage =
    /** the first frame is drawn */
    { type: 'ready' };
