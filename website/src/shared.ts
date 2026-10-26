import type { ShapeMesh } from '../examples/src/common';

export type Kind = { mesh: ShapeMesh; color: string };

/** the kinds of shape in the heap: the worker makes their shapes, the page their meshes */
export const KINDS: Kind[] = [
    { mesh: { kind: 'box', halfExtents: [0.5, 0.5, 0.5] }, color: '#4ecdc4' },
    { mesh: { kind: 'sphere', radius: 0.5 }, color: '#ff6b6b' },
    { mesh: { kind: 'capsule', halfHeightOfCylinder: 0.5, radius: 0.35 }, color: '#dfe6e9' },
    { mesh: { kind: 'cylinder', halfHeight: 0.4, radius: 0.5 }, color: '#74b9ff' },
    { mesh: { kind: 'box', halfExtents: [1, 0.2, 0.4] }, color: '#f39c12' },
];

/** a body's pose in the poses buffer: position xyz, then quaternion xyzw */
export const POSE_STRIDE = 7;

/** each frame, the time since the last, and the poses buffer back to be filled */
export type FrameRequest = { deltaTime: number; debug: boolean; poses: Float32Array };

export type WorkerMessage =
    /** the world is made: each body's kind, in the order of the poses */
    | { type: 'ready'; kinds: Uint8Array }
    /** each body's pose, blended between the last two steps; and the debug lines, while asked for */
    | { type: 'frame'; poses: Float32Array; lines: Float32Array | null; lineCount: number };
