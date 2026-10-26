import type { BodyId } from '../body.js';
import * as Low from '../generated/tumble.js';
import { copied, type Vec2, type Vec3 } from '../math.js';
import { defaultsOf, type Given, over } from '../settings.js';
import type { World } from '../world.js';
import {
    type Common,
    type ConstraintId,
    created,
    expectCommon,
    expectMotor,
    expectRange,
    expectSliderLimits,
    type MotorSettings,
    type MotorState,
    readMotor,
    type SpringSettings,
    vec2,
    vec3,
    writeMotor,
} from './common.js';

export type SliderConstraintSettings = {
    pointA?: Vec3;
    pointB?: Vec3;
    /** the axis it moves along; x by default, normalized */
    sliderAxisA?: Vec3;
    sliderAxisB?: Vec3;
    /** perpendicular to the slider axis; y by default, normalized */
    normalAxisA?: Vec3;
    normalAxisB?: Vec3;
    /** world space only: the points placed for you, on one body's centre of mass when the other is static, else between them weighted to the lighter */
    autoDetectPoint?: boolean;
    /** m, at most 0; with limitsMax, unbounded by default */
    limitsMin?: number;
    /** m, at least 0 */
    limitsMax?: number;
    /** soft limits */
    limitsSpringSettings?: Given<SpringSettings>;
    /** N resisting the slide while the motor is off */
    maxFrictionForce?: number;
    /** the drive for the motor, which starts off; its force is unbounded by default */
    motorSettings?: Given<MotorSettings>;
} & Common;

const sliderDefaults = defaultsOf(Low.SliderConstraint.defaultSettings, Low.SliderConstraintSettings.create);

/** moving only along one axis, with optional limits, friction and a motor: a piston, a lift, a drawer */
export function create(world: World, bodyA: BodyId, bodyB: BodyId, settings: SliderConstraintSettings = {}): ConstraintId {
    expectCommon(settings);
    const native = over(sliderDefaults(), settings);
    expectSliderLimits(native.limitsMin, native.limitsMax);
    expectRange(native.limitsMin, native.limitsMax, native.limitsSpringSettings);
    expectMotor(native.motorSettings);
    Low.errorClear();
    return created(Low.SliderConstraint.create(world, bodyA, bodyB, native));
}

/** m along the axis from its rest position, from the bodies as they are */
export function getCurrentPosition(world: World, constraint: ConstraintId): number {
    return Low.SliderConstraint.getCurrentPosition(world, constraint);
}

/** m, min at most 0 and max at least 0 */
export function setLimits(world: World, constraint: ConstraintId, min: number, max: number): boolean {
    expectSliderLimits(min, max);
    return Low.SliderConstraint.setLimits(world, constraint, min, max);
}

export function setLimitsSpringSettings(world: World, constraint: ConstraintId, settings: SpringSettings): boolean {
    return Low.SliderConstraint.setLimitsSpringSettings(world, constraint, settings);
}

export function setMaxFrictionForce(world: World, constraint: ConstraintId, force: number): boolean {
    return Low.SliderConstraint.setMaxFrictionForce(world, constraint, force);
}

export function getMaxFrictionForce(world: World, constraint: ConstraintId): number {
    return Low.SliderConstraint.getMaxFrictionForce(world, constraint);
}

export function getMotorSettings(out: MotorSettings, world: World, constraint: ConstraintId): boolean {
    return readMotor(out, (scratch) => Low.SliderConstraint.getMotorSettings(scratch, world, constraint));
}

/** the given fields over the current ones */
export function setMotorSettings(world: World, constraint: ConstraintId, settings: Given<MotorSettings>): boolean {
    return writeMotor(
        settings,
        (scratch) => Low.SliderConstraint.getMotorSettings(scratch, world, constraint),
        (native) => Low.SliderConstraint.setMotorSettings(world, constraint, native),
    );
}

export function setMotorState(world: World, constraint: ConstraintId, state: MotorState): boolean {
    return Low.SliderConstraint.setMotorState(world, constraint, state);
}

/** m/s, for a velocity motor */
export function setTargetVelocity(world: World, constraint: ConstraintId, velocity: number): boolean {
    return Low.SliderConstraint.setTargetVelocity(world, constraint, velocity);
}

/** m, for a position motor; clamped to the limits */
export function setTargetPosition(world: World, constraint: ConstraintId, position: number): boolean {
    return Low.SliderConstraint.setTargetPosition(world, constraint, position);
}

export function getTotalLambdaPosition(out: Vec2, world: World, constraint: ConstraintId): boolean {
    return copied(out, vec2, Low.SliderConstraint.getTotalLambdaPosition(vec2, world, constraint));
}

export function getTotalLambdaPositionLimits(world: World, constraint: ConstraintId): number {
    return Low.SliderConstraint.getTotalLambdaPositionLimits(world, constraint);
}

export function getTotalLambdaRotation(out: Vec3, world: World, constraint: ConstraintId): boolean {
    return copied(out, vec3, Low.SliderConstraint.getTotalLambdaRotation(vec3, world, constraint));
}

export function getTotalLambdaMotor(world: World, constraint: ConstraintId): number {
    return Low.SliderConstraint.getTotalLambdaMotor(world, constraint);
}
