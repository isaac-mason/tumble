import type { BodyId } from '../body.js';
import * as Low from '../generated/tumble.js';
import { copied, type Quat, type Vec3 } from '../math.js';
import { defaultsOf, type Given, over } from '../settings.js';
import type { World } from '../world.js';
import {
    type Common,
    type ConstraintId,
    created,
    expectCommon,
    expectMotor,
    type MotorSettings,
    type MotorState,
    quat,
    readMotor,
    type SwingType,
    vec3,
    writeMotor,
} from './common.js';

export type SwingTwistConstraintSettings = {
    pointA?: Vec3;
    pointB?: Vec3;
    /** constraint space x, the axis it twists about; x by default, normalized */
    twistAxisA?: Vec3;
    /** constraint space z, perpendicular to the twist axis; z by default, normalized */
    planeAxisA?: Vec3;
    twistAxisB?: Vec3;
    planeAxisB?: Vec3;
    /** CONE (the default) or PYRAMID */
    swingType?: SwingType;
    /** radians, the swing about constraint space z */
    normalHalfConeAngle?: number;
    /** radians, the swing about constraint space y */
    planeHalfConeAngle?: number;
    /** radians in [-pi, pi] */
    twistMinAngle?: number;
    twistMaxAngle?: number;
    /** N m resisting the turn while the motors are off */
    maxFrictionTorque?: number;
    swingMotorSettings?: Given<MotorSettings>;
    twistMotorSettings?: Given<MotorSettings>;
} & Common;

const swingTwistDefaults = defaultsOf(Low.SwingTwistConstraint.defaultSettings, Low.SwingTwistConstraintSettings.create);

/** a point on each body held together, the turn limited apart in swing and twist, with optional friction and motors: a ragdoll's joint */
export function create(world: World, bodyA: BodyId, bodyB: BodyId, settings: SwingTwistConstraintSettings = {}): ConstraintId {
    expectCommon(settings);
    const native = over(swingTwistDefaults(), settings);
    expectMotor(native.swingMotorSettings);
    expectMotor(native.twistMotorSettings);
    Low.errorClear();
    return created(Low.SwingTwistConstraint.create(world, bodyA, bodyB, native));
}

export function setNormalHalfConeAngle(world: World, constraint: ConstraintId, angle: number): boolean {
    return Low.SwingTwistConstraint.setNormalHalfConeAngle(world, constraint, angle);
}

export function setPlaneHalfConeAngle(world: World, constraint: ConstraintId, angle: number): boolean {
    return Low.SwingTwistConstraint.setPlaneHalfConeAngle(world, constraint, angle);
}

export function setTwistMinAngle(world: World, constraint: ConstraintId, angle: number): boolean {
    return Low.SwingTwistConstraint.setTwistMinAngle(world, constraint, angle);
}

export function setTwistMaxAngle(world: World, constraint: ConstraintId, angle: number): boolean {
    return Low.SwingTwistConstraint.setTwistMaxAngle(world, constraint, angle);
}

export function setMaxFrictionTorque(world: World, constraint: ConstraintId, torque: number): boolean {
    return Low.SwingTwistConstraint.setMaxFrictionTorque(world, constraint, torque);
}

export function getSwingMotorSettings(out: MotorSettings, world: World, constraint: ConstraintId): boolean {
    return readMotor(out, (scratch) => Low.SwingTwistConstraint.getSwingMotorSettings(scratch, world, constraint));
}

export function getTwistMotorSettings(out: MotorSettings, world: World, constraint: ConstraintId): boolean {
    return readMotor(out, (scratch) => Low.SwingTwistConstraint.getTwistMotorSettings(scratch, world, constraint));
}

/** the given fields over the current ones */
export function setSwingMotorSettings(world: World, constraint: ConstraintId, settings: Given<MotorSettings>): boolean {
    return writeMotor(
        settings,
        (scratch) => Low.SwingTwistConstraint.getSwingMotorSettings(scratch, world, constraint),
        (native) => Low.SwingTwistConstraint.setSwingMotorSettings(world, constraint, native),
    );
}

/** the given fields over the current ones */
export function setTwistMotorSettings(world: World, constraint: ConstraintId, settings: Given<MotorSettings>): boolean {
    return writeMotor(
        settings,
        (scratch) => Low.SwingTwistConstraint.getTwistMotorSettings(scratch, world, constraint),
        (native) => Low.SwingTwistConstraint.setTwistMotorSettings(world, constraint, native),
    );
}

export function setSwingMotorState(world: World, constraint: ConstraintId, state: MotorState): boolean {
    return Low.SwingTwistConstraint.setSwingMotorState(world, constraint, state);
}

export function setTwistMotorState(world: World, constraint: ConstraintId, state: MotorState): boolean {
    return Low.SwingTwistConstraint.setTwistMotorState(world, constraint, state);
}

/** rad/s in body b's constraint space, for velocity motors */
export function setTargetAngularVelocityCS(world: World, constraint: ConstraintId, velocity: Vec3): boolean {
    return Low.SwingTwistConstraint.setTargetAngularVelocityCs(world, constraint, velocity);
}

/** in constraint space, for position motors; clamped to the limits */
export function setTargetOrientationCS(world: World, constraint: ConstraintId, orientation: Quat): boolean {
    return Low.SwingTwistConstraint.setTargetOrientationCs(world, constraint, orientation);
}

/** in body space: body b turned to body a's orientation times this */
export function setTargetOrientationBS(world: World, constraint: ConstraintId, orientation: Quat): boolean {
    return Low.SwingTwistConstraint.setTargetOrientationBs(world, constraint, orientation);
}

/** the turn between the bodies, in constraint space */
export function getRotationInConstraintSpace(out: Quat, world: World, constraint: ConstraintId): boolean {
    return copied(out, quat, Low.SwingTwistConstraint.getRotationInConstraintSpace(quat, world, constraint));
}

export function getTotalLambdaPosition(out: Vec3, world: World, constraint: ConstraintId): boolean {
    return copied(out, vec3, Low.SwingTwistConstraint.getTotalLambdaPosition(vec3, world, constraint));
}

export function getTotalLambdaTwist(world: World, constraint: ConstraintId): number {
    return Low.SwingTwistConstraint.getTotalLambdaTwist(world, constraint);
}

export function getTotalLambdaSwingY(world: World, constraint: ConstraintId): number {
    return Low.SwingTwistConstraint.getTotalLambdaSwingY(world, constraint);
}

export function getTotalLambdaSwingZ(world: World, constraint: ConstraintId): number {
    return Low.SwingTwistConstraint.getTotalLambdaSwingZ(world, constraint);
}

export function getTotalLambdaMotor(out: Vec3, world: World, constraint: ConstraintId): boolean {
    return copied(out, vec3, Low.SwingTwistConstraint.getTotalLambdaMotor(vec3, world, constraint));
}
