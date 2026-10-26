import type { BodyId } from '../body.js';
import * as Low from '../generated/tumble.js';
import type { Vec3 } from '../math.js';
import { defaultsOf, type Given, over } from '../settings.js';
import type { World } from '../world.js';
import {
    type Common,
    type ConstraintId,
    created,
    expectCommon,
    expectHingeLimits,
    expectMotor,
    expectRange,
    type MotorSettings,
    type MotorState,
    readMotor,
    type SpringSettings,
    writeMotor,
} from './common.js';

export type HingeConstraintSettings = {
    pointA?: Vec3;
    pointB?: Vec3;
    /** the axis it turns about; y by default, normalized */
    hingeAxisA?: Vec3;
    hingeAxisB?: Vec3;
    /** perpendicular to the hinge axis, where the angle is 0; x by default, normalized */
    normalAxisA?: Vec3;
    normalAxisB?: Vec3;
    /** radians, in [-pi, 0]; with limitsMax at pi, as by default, there are no limits */
    limitsMin?: number;
    /** radians, in [0, pi] */
    limitsMax?: number;
    /** soft limits */
    limitsSpringSettings?: Given<SpringSettings>;
    /** N m resisting the turn while the motor is off */
    maxFrictionTorque?: number;
    /** the drive for the motor, which starts off; its torque is unbounded by default, and an unbounded motor pushes through its limits */
    motorSettings?: Given<MotorSettings>;
} & Common;

const hingeDefaults = defaultsOf(Low.HingeConstraint.defaultSettings, Low.HingeConstraintSettings.create);

/** turning only about one axis, with optional angle limits, friction and a motor: a door, a wheel */
export function create(world: World, bodyA: BodyId, bodyB: BodyId, settings: HingeConstraintSettings = {}): ConstraintId {
    expectCommon(settings);
    const native = over(hingeDefaults(), settings);
    expectHingeLimits(native.limitsMin, native.limitsMax);
    expectRange(native.limitsMin, native.limitsMax, native.limitsSpringSettings);
    expectMotor(native.motorSettings);
    Low.errorClear();
    return created(Low.HingeConstraint.create(world, bodyA, bodyB, native));
}

/** radians, min in [-pi, 0] and max in [0, pi]; [-pi, pi] is no limits */
export function setLimits(world: World, constraint: ConstraintId, min: number, max: number): boolean {
    expectHingeLimits(min, max);
    return Low.HingeConstraint.setLimits(world, constraint, min, max);
}

export function setLimitsSpringSettings(world: World, constraint: ConstraintId, settings: SpringSettings): boolean {
    return Low.HingeConstraint.setLimitsSpringSettings(world, constraint, settings);
}

export function setMaxFrictionTorque(world: World, constraint: ConstraintId, torque: number): boolean {
    return Low.HingeConstraint.setMaxFrictionTorque(world, constraint, torque);
}

export function getMotorSettings(out: MotorSettings, world: World, constraint: ConstraintId): boolean {
    return readMotor(out, (scratch) => Low.HingeConstraint.getMotorSettings(scratch, world, constraint));
}

/** the given fields over the current ones */
export function setMotorSettings(world: World, constraint: ConstraintId, settings: Given<MotorSettings>): boolean {
    return writeMotor(
        settings,
        (scratch) => Low.HingeConstraint.getMotorSettings(scratch, world, constraint),
        (native) => Low.HingeConstraint.setMotorSettings(world, constraint, native),
    );
}

export function setMotorState(world: World, constraint: ConstraintId, state: MotorState): boolean {
    return Low.HingeConstraint.setMotorState(world, constraint, state);
}

/** rad/s, for a velocity motor */
export function setTargetAngularVelocity(world: World, constraint: ConstraintId, velocity: number): boolean {
    return Low.HingeConstraint.setTargetAngularVelocity(world, constraint, velocity);
}

/** radians, for a position motor; clamped to the limits */
export function setTargetAngle(world: World, constraint: ConstraintId, angle: number): boolean {
    return Low.HingeConstraint.setTargetAngle(world, constraint, angle);
}

/** radians, as of the last step */
export function getCurrentAngle(world: World, constraint: ConstraintId): number {
    return Low.HingeConstraint.getCurrentAngle(world, constraint);
}
