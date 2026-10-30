import type { Vec3 } from './generated/tumble.js';

export type { Box3, Color, Mat4, Quat, Vec2, Vec3 } from './generated/tumble.js';

export function copyVec3(out: Vec3, from: Vec3): void {
    out[0] = from[0];
    out[1] = from[1];
    out[2] = from[2];
}
