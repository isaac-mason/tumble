// the page and the worker: the page hands over the canvas and passes on what the visitor does, and the worker does the
// rest. sizes are css pixels

export type PageMessage =
    | { type: 'start'; canvas: OffscreenCanvas; width: number; height: number; pixelRatio: number }
    | { type: 'resize'; width: number; height: number; pixelRatio: number }
    /** each of the page's animation frames, at its time in ms: the worker steps and draws on these */
    | { type: 'frame'; time: number }
    | { type: 'debug'; show: boolean };

export type WorkerMessage =
    /** the first frame is drawn */
    { type: 'ready' };
