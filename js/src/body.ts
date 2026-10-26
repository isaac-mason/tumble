import * as Low from './generated/tumble.js';

/** a body in a world: an index and a sequence in one number, so the id of a removed body names nothing */
export type BodyId = number;

/** the id naming no body */
export const INVALID_BODY_ID = 0xffffffff;

export const MotionType = Low.MotionType;
export type MotionType = Low.MotionType;
export const MotionQuality = Low.MotionQuality;
export type MotionQuality = Low.MotionQuality;
/** bits of allowedDegreesOfFreedom */
export const Dof = Low.Dof;
export const MaterialCombineMode = Low.MaterialCombineMode;
export type MaterialCombineMode = Low.MaterialCombineMode;
