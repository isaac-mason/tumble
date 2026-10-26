import * as Low from '../generated/tumble.js';
import type { Quat, Vec2, Vec3 } from '../math.js';
import { type Given, over } from '../settings.js';

/** a constraint in a world: its index, type and sequence in one number, so the id of a removed constraint names nothing */
export type ConstraintId = number;

/** the id naming no constraint */
export const INVALID_CONSTRAINT_ID = 0xffffffff;

export const ConstraintType = Low.ConstraintType;
export type ConstraintType = Low.ConstraintType;
export const ConstraintSpace = Low.ConstraintSpace;
export type ConstraintSpace = Low.ConstraintSpace;
export const SpringMode = Low.SpringMode;
export type SpringMode = Low.SpringMode;
export const MotorState = Low.MotorState;
export type MotorState = Low.MotorState;
export const SwingType = Low.SwingType;
export type SwingType = Low.SwingType;
export const SixDOFAxis = Low.SixDOFAxis;
export type SixDOFAxis = Low.SixDOFAxis;

/** a soft limit or a position motor's drive; hard (no spring) when frequencyOrStiffness is 0 or less */
export type SpringSettings = {
    mode: SpringMode;
    frequencyOrStiffness: number;
    /** ignored by a hard spring */
    damping: number;
};

/** a motor's drive: its spring drives a position motor, and its limits bound what it may apply */
export type MotorSettings = {
    spring: SpringSettings;
    /** N, for a linear motor; usually min is -max, unless it pushes but never pulls */
    minForceLimit: number;
    maxForceLimit: number;
    /** N m, for an angular motor */
    minTorqueLimit: number;
    maxTorqueLimit: number;
};

/** what every constraint's settings have */
export type Common = {
    /** WORLD (the default), or LOCAL: relative to each body's centre of mass and orientation */
    space?: ConstraintSpace;
    /** an integer in [0, 2^32), solved in ascending order, so the highest is solved last and is most likely to hold */
    constraintPriority?: number;
    /** 0 (the default) takes the world's iteration counts */
    numVelocityStepsOverride?: number;
    numPositionStepsOverride?: number;
};

/** float pi, the bound the C++ holds angles to */
export const PI = Math.fround(Math.PI);

export function expect(holds: boolean, what: string): void {
    if (!holds) throw new RangeError(`tumble: ${what}`);
}

export const motor = Low.MotorSettings.create();

/** a constraint's motor settings into out: false for an id naming no constraint of the type */
export function readMotor(out: MotorSettings, read: (scratch: Low.MotorSettings) => boolean): boolean {
    if (!read(motor)) return false;
    out.spring.mode = motor.spring.mode as SpringMode;
    out.spring.frequencyOrStiffness = motor.spring.frequencyOrStiffness;
    out.spring.damping = motor.spring.damping;
    out.minForceLimit = motor.minForceLimit;
    out.maxForceLimit = motor.maxForceLimit;
    out.minTorqueLimit = motor.minTorqueLimit;
    out.maxTorqueLimit = motor.maxTorqueLimit;
    return true;
}

/** the given fields over a constraint's current motor settings: false for an id naming no constraint of the type */
export function writeMotor(
    given: Given<MotorSettings>,
    read: (scratch: Low.MotorSettings) => boolean,
    write: (settings: Low.MotorSettings) => boolean,
): boolean {
    if (!read(motor)) return false;
    const settings = over(motor, given);
    expectMotor(settings);
    return write(settings);
}

/** what a motor is held to before it runs */
export function expectMotor(settings: Low.MotorSettings): void {
    const { spring } = settings;
    expect(
        spring.frequencyOrStiffness >= 0 && spring.damping >= 0,
        'a motor spring has no negative frequency, stiffness or damping',
    );
    expect(settings.minForceLimit <= settings.maxForceLimit, 'a motor force limit has min <= max');
    expect(settings.minTorqueLimit <= settings.maxTorqueLimit, 'a motor torque limit has min <= max');
}

/** limits equal to each other hold the bodies still, which takes a fixed constraint unless the limits are soft */
export function expectRange(min: number, max: number, spring: Low.SpringSettings): void {
    expect(
        Math.fround(min) !== Math.fround(max) || spring.frequencyOrStiffness > 0,
        'equal limits need a spring; a fixed constraint holds still',
    );
}

export function expectHingeLimits(min: number, max: number): void {
    expect(Math.fround(min) <= 0 && Math.fround(min) >= -PI, 'a hinge limitsMin is in [-pi, 0]');
    expect(Math.fround(max) >= 0 && Math.fround(max) <= PI, 'a hinge limitsMax is in [0, pi]');
}

export function expectSliderLimits(min: number, max: number): void {
    expect(min <= 0, 'a slider limitsMin is at most 0');
    expect(max >= 0, 'a slider limitsMax is at least 0');
}

export function expectAxis(axis: number, count: number): void {
    expect(Number.isInteger(axis) && axis >= 0 && axis < count, `an axis is an integer in [0, ${count})`);
}

/** a priority is an integer in [0, 2^32), as jolt's uint32 */
export function expectCommon(settings: Common): void {
    const priority = settings.constraintPriority;
    expect(
        priority === undefined || (Number.isInteger(priority) && priority >= 0 && priority <= 0xffffffff),
        'constraintPriority is an integer in [0, 2^32)',
    );
}

export function created(id: ConstraintId): ConstraintId {
    if (id === INVALID_CONSTRAINT_ID) Low.getWasm().check();
    return id;
}

export const vec2: Vec2 = [0, 0];
export const vec3: Vec3 = [0, 0, 0];
export const quat: Quat = [0, 0, 0, 1];

// each type: create(world, bodyA, bodyB, settings) gives the id, throwing when a body id names no body. its setters and
// readers return false, or 0 for one number, when the id names no constraint of the type. a change wakes neither body
