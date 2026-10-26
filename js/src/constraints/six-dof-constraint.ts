import type { BodyId } from '../body.js';
import * as Low from '../generated/tumble.js';
import type { Quat, Vec3 } from '../math.js';
import { defaultsOf, type Given, over } from '../settings.js';
import type { World } from '../world.js';
import {
    type Common,
    type ConstraintId,
    created,
    expect,
    expectAxis,
    expectCommon,
    expectMotor,
    type MotorSettings,
    type MotorState,
    readMotor,
    type SixDOFAxis,
    type SpringSettings,
    type SwingType,
    writeMotor,
} from './common.js';

export type SixDOFValues = [number, number, number, number, number, number];

export type SixDOFConstraintSettings = {
    pointA?: Vec3;
    pointB?: Vec3;
    /** body a's constraint space x and y axes; x and y by default, normalized */
    axisXA?: Vec3;
    axisYA?: Vec3;
    axisXB?: Vec3;
    axisYB?: Vec3;
    /** CONE (the default) or PYRAMID, for limited rotation axes */
    swingType?: SwingType;
    /**
     * per axis, m or radians: min equal to max fixes the axis there and min above max fixes it at 0, while an unbounded
     * translation or a [-pi, pi] rotation frees it; every axis is free by default. rotations are clamped to [-pi, pi],
     * and a CONE swing limits rotation y and z to +-max
     */
    limitMin?: SixDOFValues;
    limitMax?: SixDOFValues;
    /** per axis N or N m, 0 (the default) for none */
    maxFriction?: SixDOFValues;
    /** soft limits, for the translation axes */
    limitsSpringSettings?: readonly (Given<SpringSettings> | undefined)[];
    /** per axis: translation motors use the force limits, rotation motors the torque limits */
    motorSettings?: readonly (Given<MotorSettings> | undefined)[];
} & Common;

const sixDOFDefaults = defaultsOf(Low.SixDofConstraint.defaultSettings, Low.SixDOFConstraintSettings.create);

/** each of the six axes free, fixed or limited, with optional friction and a motor */
export function create(world: World, bodyA: BodyId, bodyB: BodyId, settings: SixDOFConstraintSettings = {}): ConstraintId {
    expectCommon(settings);
    expect((settings.limitsSpringSettings?.length ?? 0) <= 3, 'limitsSpringSettings has one per translation axis');
    expect((settings.motorSettings?.length ?? 0) <= 6, 'motorSettings has one per axis');
    const native = over(sixDOFDefaults(), settings);
    for (const motor of native.motorSettings) expectMotor(motor);
    Low.errorClear();
    return created(Low.SixDofConstraint.create(world, bodyA, bodyB, native));
}

export function setTranslationLimits(world: World, constraint: ConstraintId, limitMin: Vec3, limitMax: Vec3): boolean {
    return Low.SixDofConstraint.setTranslationLimits(world, constraint, limitMin, limitMax);
}

export function setRotationLimits(world: World, constraint: ConstraintId, limitMin: Vec3, limitMax: Vec3): boolean {
    return Low.SixDofConstraint.setRotationLimits(world, constraint, limitMin, limitMax);
}

/** a translation axis */
export function setLimitsSpringSettings(
    world: World,
    constraint: ConstraintId,
    axis: SixDOFAxis,
    settings: SpringSettings,
): boolean {
    expectAxis(axis, 3);
    return Low.SixDofConstraint.setLimitsSpringSettings(world, constraint, axis, settings);
}

export function getMotorSettings(out: MotorSettings, world: World, constraint: ConstraintId, axis: SixDOFAxis): boolean {
    expectAxis(axis, 6);
    return readMotor(out, (scratch) => Low.SixDofConstraint.getMotorSettings(scratch, world, constraint, axis));
}

/** the given fields over the axis's current ones */
export function setMotorSettings(
    world: World,
    constraint: ConstraintId,
    axis: SixDOFAxis,
    settings: Given<MotorSettings>,
): boolean {
    expectAxis(axis, 6);
    return writeMotor(
        settings,
        (scratch) => Low.SixDofConstraint.getMotorSettings(scratch, world, constraint, axis),
        (native) => Low.SixDofConstraint.setMotorSettings(world, constraint, axis, native),
    );
}

export function setMotorState(world: World, constraint: ConstraintId, axis: SixDOFAxis, state: MotorState): boolean {
    expectAxis(axis, 6);
    return Low.SixDofConstraint.setMotorState(world, constraint, axis, state);
}

/** m/s in constraint space, for the translation velocity motors */
export function setTargetVelocityCS(world: World, constraint: ConstraintId, velocity: Vec3): boolean {
    return Low.SixDofConstraint.setTargetVelocityCs(world, constraint, velocity);
}

/** rad/s in constraint space, for the rotation velocity motors */
export function setTargetAngularVelocityCS(world: World, constraint: ConstraintId, velocity: Vec3): boolean {
    return Low.SixDofConstraint.setTargetAngularVelocityCs(world, constraint, velocity);
}

/** m in constraint space, for the translation position motors */
export function setTargetPositionCS(world: World, constraint: ConstraintId, position: Vec3): boolean {
    return Low.SixDofConstraint.setTargetPositionCs(world, constraint, position);
}

/** in constraint space, for the rotation position motors; clamped to the rotation limits */
export function setTargetOrientationCS(world: World, constraint: ConstraintId, orientation: Quat): boolean {
    return Low.SixDofConstraint.setTargetOrientationCs(world, constraint, orientation);
}

export function setMaxFriction(world: World, constraint: ConstraintId, axis: SixDOFAxis, friction: number): boolean {
    expectAxis(axis, 6);
    return Low.SixDofConstraint.setMaxFriction(world, constraint, axis, friction);
}

export function makeFreeAxis(world: World, constraint: ConstraintId, axis: SixDOFAxis): boolean {
    expectAxis(axis, 6);
    return Low.SixDofConstraint.makeFreeAxis(world, constraint, axis);
}

/** locked at 0 */
export function makeFixedAxis(world: World, constraint: ConstraintId, axis: SixDOFAxis): boolean {
    expectAxis(axis, 6);
    return Low.SixDofConstraint.makeFixedAxis(world, constraint, axis);
}

/** max below min fixes the axis at 0 */
export function setLimitedAxis(world: World, constraint: ConstraintId, axis: SixDOFAxis, min: number, max: number): boolean {
    expectAxis(axis, 6);
    return Low.SixDofConstraint.setLimitedAxis(world, constraint, axis, min, max);
}
