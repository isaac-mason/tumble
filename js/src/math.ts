import type { Vec3 } from './generated/tumble.js';

export type { Box3, Mat4, Quat, Vec2, Vec3 } from './generated/tumble.js';

/** out filled only when the read succeeded */
export function copied<T extends number[]>(out: T, read: T, ok: boolean): boolean {
    if (ok) for (let i = 0; i < out.length; i++) out[i] = read[i]!;
    return ok;
}

export function copyVec3(out: Vec3, from: Vec3): void {
    out[0] = from[0];
    out[1] = from[1];
    out[2] = from[2];
}
