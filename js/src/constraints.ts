import * as Low from './generated/tumble.js';
import { copied, type Quat, type Vec2, type Vec3 } from './math.js';
import type { BodyId } from './rigid-body.js';
import { defaultsOf, type Given, over } from './settings.js';
import type { World } from './world.js';

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
type Common = {
    /** WORLD (the default), or LOCAL: relative to each body's centre of mass and orientation */
    space?: ConstraintSpace;
    /** solved in ascending order, so the highest is solved last and is most likely to hold */
    constraintPriority?: number;
    /** 0 (the default) takes the world's iteration counts */
    numVelocityStepsOverride?: number;
    numPositionStepsOverride?: number;
};

/** float pi, the bound the C++ holds angles to */
const PI = Math.fround(Math.PI);

function expect(holds: boolean, what: string): void {
    if (!holds) throw new RangeError(`tumble: ${what}`);
}

/** what a motor is held to before it runs */
function expectMotor(settings: Low.MotorSettings): void {
    const { spring } = settings;
    expect(
        spring.frequencyOrStiffness >= 0 && spring.damping >= 0,
        'a motor spring has no negative frequency, stiffness or damping',
    );
    expect(settings.minForceLimit <= settings.maxForceLimit, 'a motor force limit has min <= max');
    expect(settings.minTorqueLimit <= settings.maxTorqueLimit, 'a motor torque limit has min <= max');
}

/** limits equal to each other hold the bodies still, which takes a fixed constraint unless the limits are soft */
function expectRange(min: number, max: number, spring: Low.SpringSettings): void {
    expect(
        Math.fround(min) !== Math.fround(max) || spring.frequencyOrStiffness > 0,
        'equal limits need a spring; a fixed constraint holds still',
    );
}

function expectHingeLimits(min: number, max: number): void {
    expect(Math.fround(min) <= 0 && Math.fround(min) >= -PI, 'a hinge limitsMin is in [-pi, 0]');
    expect(Math.fround(max) >= 0 && Math.fround(max) <= PI, 'a hinge limitsMax is in [0, pi]');
}

function expectSliderLimits(min: number, max: number): void {
    expect(min <= 0, 'a slider limitsMin is at most 0');
    expect(max >= 0, 'a slider limitsMax is at least 0');
}

function expectAxis(axis: number, count: number): void {
    expect(Number.isInteger(axis) && axis >= 0 && axis < count, `an axis is an integer in [0, ${count})`);
}

function created(id: ConstraintId): ConstraintId {
    if (id === INVALID_CONSTRAINT_ID) Low.getWasm().check();
    return id;
}

const vec2: Vec2 = [0, 0];
const vec3: Vec3 = [0, 0, 0];
const quat: Quat = [0, 0, 0, 1];

/** what every constraint has, whatever its type */
export const Constraint = {
    /** the type the id encodes, whether or not it names a live constraint */
    getType(constraint: ConstraintId): ConstraintType {
        return Low.Constraint.getType(constraint) as ConstraintType;
    },
    /** whether the id names a constraint in the world; one goes with either of its bodies */
    exists(world: World, constraint: ConstraintId): boolean {
        return Low.Constraint.exists(world, constraint);
    },
    /** a disabled constraint holds nothing and does not join its bodies' islands, so they may sleep apart; false when the id names no constraint */
    setEnabled(world: World, constraint: ConstraintId, enabled: boolean): boolean {
        return Low.Constraint.setEnabled(world, constraint, enabled);
    },
    isEnabled(world: World, constraint: ConstraintId): boolean {
        return Low.Constraint.isEnabled(world, constraint);
    },
};

// each type: create(world, bodyA, bodyB, settings) gives the id, throwing when a body id names no body. its setters and
// readers return false, or 0 for one number, when the id names no constraint of the type. a change wakes neither body

export type PointConstraintSettings = {
    /** the point on body a, held to the point on body b */
    pointA?: Vec3;
    pointB?: Vec3;
} & Common;

const pointDefaults = defaultsOf(Low.PointConstraint.defaultSettings, Low.PointConstraintSettings.create);

/** a point on each body held together: a ball and socket, free to turn */
export const PointConstraint = {
    create(world: World, bodyA: BodyId, bodyB: BodyId, settings: PointConstraintSettings = {}): ConstraintId {
        Low.errorClear();
        return created(Low.PointConstraint.create(world, bodyA, bodyB, over(pointDefaults(), settings)));
    },
    remove(world: World, constraint: ConstraintId): boolean {
        return Low.PointConstraint.remove(world, constraint);
    },
    /** the impulse it applied in the last step, N s: for breaking it by force */
    getTotalLambda(out: Vec3, world: World, constraint: ConstraintId): boolean {
        return copied(out, vec3, Low.PointConstraint.getTotalLambda(vec3, world, constraint));
    },
};

export type DistanceConstraintSettings = {
    /** the points kept within the distances; the same point on both is a cheap point constraint holding 1 degree of freedom, not 3 */
    pointA?: Vec3;
    pointB?: Vec3;
    /** -1 (the default) for the distance at creation */
    minDistance?: number;
    maxDistance?: number;
    /** soft limits */
    springSettings?: Given<SpringSettings>;
} & Common;

const distanceDefaults = defaultsOf(Low.DistanceConstraint.defaultSettings, Low.DistanceConstraintSettings.create);

/** the distance between a point on each body kept within [min, max]: a rope, a rod, a bungee */
export const DistanceConstraint = {
    create(world: World, bodyA: BodyId, bodyB: BodyId, settings: DistanceConstraintSettings = {}): ConstraintId {
        const native = over(distanceDefaults(), settings);
        if (native.minDistance >= 0 && native.maxDistance >= 0) {
            expect(native.minDistance <= native.maxDistance, 'minDistance is at most maxDistance');
        }
        Low.errorClear();
        return created(Low.DistanceConstraint.create(world, bodyA, bodyB, native));
    },
    remove(world: World, constraint: ConstraintId): boolean {
        return Low.DistanceConstraint.remove(world, constraint);
    },
    setDistance(world: World, constraint: ConstraintId, minDistance: number, maxDistance: number): boolean {
        expect(minDistance <= maxDistance, 'minDistance is at most maxDistance');
        return Low.DistanceConstraint.setDistance(world, constraint, minDistance, maxDistance);
    },
    setLimitsSpringSettings(world: World, constraint: ConstraintId, settings: SpringSettings): boolean {
        return Low.DistanceConstraint.setLimitsSpringSettings(world, constraint, settings);
    },
    getTotalLambda(world: World, constraint: ConstraintId): number {
        return Low.DistanceConstraint.getTotalLambda(world, constraint);
    },
};

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
export const HingeConstraint = {
    create(world: World, bodyA: BodyId, bodyB: BodyId, settings: HingeConstraintSettings = {}): ConstraintId {
        const native = over(hingeDefaults(), settings);
        expectHingeLimits(native.limitsMin, native.limitsMax);
        expectRange(native.limitsMin, native.limitsMax, native.limitsSpringSettings);
        expectMotor(native.motorSettings);
        Low.errorClear();
        return created(Low.HingeConstraint.create(world, bodyA, bodyB, native));
    },
    remove(world: World, constraint: ConstraintId): boolean {
        return Low.HingeConstraint.remove(world, constraint);
    },
    /** radians, min in [-pi, 0] and max in [0, pi]; [-pi, pi] is no limits */
    setLimits(world: World, constraint: ConstraintId, min: number, max: number): boolean {
        expectHingeLimits(min, max);
        return Low.HingeConstraint.setLimits(world, constraint, min, max);
    },
    setLimitsSpringSettings(world: World, constraint: ConstraintId, settings: SpringSettings): boolean {
        return Low.HingeConstraint.setLimitsSpringSettings(world, constraint, settings);
    },
    setMaxFrictionTorque(world: World, constraint: ConstraintId, torque: number): boolean {
        return Low.HingeConstraint.setMaxFrictionTorque(world, constraint, torque);
    },
    setMotorSettings(world: World, constraint: ConstraintId, settings: MotorSettings): boolean {
        expectMotor(settings);
        return Low.HingeConstraint.setMotorSettings(world, constraint, settings);
    },
    setMotorState(world: World, constraint: ConstraintId, state: MotorState): boolean {
        return Low.HingeConstraint.setMotorState(world, constraint, state);
    },
    /** rad/s, for a velocity motor */
    setTargetAngularVelocity(world: World, constraint: ConstraintId, velocity: number): boolean {
        return Low.HingeConstraint.setTargetAngularVelocity(world, constraint, velocity);
    },
    /** radians, for a position motor; clamped to the limits */
    setTargetAngle(world: World, constraint: ConstraintId, angle: number): boolean {
        return Low.HingeConstraint.setTargetAngle(world, constraint, angle);
    },
    /** radians, as of the last step */
    getCurrentAngle(world: World, constraint: ConstraintId): number {
        return Low.HingeConstraint.getCurrentAngle(world, constraint);
    },
};

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
export const SliderConstraint = {
    create(world: World, bodyA: BodyId, bodyB: BodyId, settings: SliderConstraintSettings = {}): ConstraintId {
        const native = over(sliderDefaults(), settings);
        expectSliderLimits(native.limitsMin, native.limitsMax);
        expectRange(native.limitsMin, native.limitsMax, native.limitsSpringSettings);
        expectMotor(native.motorSettings);
        Low.errorClear();
        return created(Low.SliderConstraint.create(world, bodyA, bodyB, native));
    },
    remove(world: World, constraint: ConstraintId): boolean {
        return Low.SliderConstraint.remove(world, constraint);
    },
    /** m along the axis from its rest position, from the bodies as they are */
    getCurrentPosition(world: World, constraint: ConstraintId): number {
        return Low.SliderConstraint.getCurrentPosition(world, constraint);
    },
    /** m, min at most 0 and max at least 0 */
    setLimits(world: World, constraint: ConstraintId, min: number, max: number): boolean {
        expectSliderLimits(min, max);
        return Low.SliderConstraint.setLimits(world, constraint, min, max);
    },
    setLimitsSpringSettings(world: World, constraint: ConstraintId, settings: SpringSettings): boolean {
        return Low.SliderConstraint.setLimitsSpringSettings(world, constraint, settings);
    },
    setMaxFrictionForce(world: World, constraint: ConstraintId, force: number): boolean {
        return Low.SliderConstraint.setMaxFrictionForce(world, constraint, force);
    },
    getMaxFrictionForce(world: World, constraint: ConstraintId): number {
        return Low.SliderConstraint.getMaxFrictionForce(world, constraint);
    },
    setMotorSettings(world: World, constraint: ConstraintId, settings: MotorSettings): boolean {
        expectMotor(settings);
        return Low.SliderConstraint.setMotorSettings(world, constraint, settings);
    },
    setMotorState(world: World, constraint: ConstraintId, state: MotorState): boolean {
        return Low.SliderConstraint.setMotorState(world, constraint, state);
    },
    /** m/s, for a velocity motor */
    setTargetVelocity(world: World, constraint: ConstraintId, velocity: number): boolean {
        return Low.SliderConstraint.setTargetVelocity(world, constraint, velocity);
    },
    /** m, for a position motor; clamped to the limits */
    setTargetPosition(world: World, constraint: ConstraintId, position: number): boolean {
        return Low.SliderConstraint.setTargetPosition(world, constraint, position);
    },
    getTotalLambdaPosition(out: Vec2, world: World, constraint: ConstraintId): boolean {
        return copied(out, vec2, Low.SliderConstraint.getTotalLambdaPosition(vec2, world, constraint));
    },
    getTotalLambdaPositionLimits(world: World, constraint: ConstraintId): number {
        return Low.SliderConstraint.getTotalLambdaPositionLimits(world, constraint);
    },
    getTotalLambdaRotation(out: Vec3, world: World, constraint: ConstraintId): boolean {
        return copied(out, vec3, Low.SliderConstraint.getTotalLambdaRotation(vec3, world, constraint));
    },
    getTotalLambdaMotor(world: World, constraint: ConstraintId): number {
        return Low.SliderConstraint.getTotalLambdaMotor(world, constraint);
    },
};

export type FixedConstraintSettings = {
    pointA?: Vec3;
    /** body a's reference axes, for its orientation; x and y by default */
    axisXA?: Vec3;
    axisYA?: Vec3;
    pointB?: Vec3;
    axisXB?: Vec3;
    axisYB?: Vec3;
    /** world space only: the point placed between the centres of mass, weighted to the lighter body */
    autoDetectPoint?: boolean;
} & Common;

const fixedDefaults = defaultsOf(Low.FixedConstraint.defaultSettings, Low.FixedConstraintSettings.create);

/** two bodies welded, moving as one */
export const FixedConstraint = {
    create(world: World, bodyA: BodyId, bodyB: BodyId, settings: FixedConstraintSettings = {}): ConstraintId {
        Low.errorClear();
        return created(Low.FixedConstraint.create(world, bodyA, bodyB, over(fixedDefaults(), settings)));
    },
    remove(world: World, constraint: ConstraintId): boolean {
        return Low.FixedConstraint.remove(world, constraint);
    },
    getTotalLambdaPosition(out: Vec3, world: World, constraint: ConstraintId): boolean {
        return copied(out, vec3, Low.FixedConstraint.getTotalLambdaPosition(vec3, world, constraint));
    },
    getTotalLambdaRotation(out: Vec3, world: World, constraint: ConstraintId): boolean {
        return copied(out, vec3, Low.FixedConstraint.getTotalLambdaRotation(vec3, world, constraint));
    },
};

export type ConeConstraintSettings = {
    pointA?: Vec3;
    /** x by default, normalized */
    twistAxisA?: Vec3;
    pointB?: Vec3;
    twistAxisB?: Vec3;
    /** radians in [0, pi], the most the twist axes may part; 0 (the default) keeps them aligned */
    halfConeAngle?: number;
} & Common;

const coneDefaults = defaultsOf(Low.ConeConstraint.defaultSettings, Low.ConeConstraintSettings.create);

function expectHalfConeAngle(angle: number): void {
    expect(Math.fround(angle) >= 0 && Math.fround(angle) <= PI, 'a half cone angle is in [0, pi]');
}

/** a point on each body held together, the twist axes kept within a cone; twisting about them stays free */
export const ConeConstraint = {
    create(world: World, bodyA: BodyId, bodyB: BodyId, settings: ConeConstraintSettings = {}): ConstraintId {
        const native = over(coneDefaults(), settings);
        expectHalfConeAngle(native.halfConeAngle);
        Low.errorClear();
        return created(Low.ConeConstraint.create(world, bodyA, bodyB, native));
    },
    remove(world: World, constraint: ConstraintId): boolean {
        return Low.ConeConstraint.remove(world, constraint);
    },
    setHalfConeAngle(world: World, constraint: ConstraintId, halfConeAngle: number): boolean {
        expectHalfConeAngle(halfConeAngle);
        return Low.ConeConstraint.setHalfConeAngle(world, constraint, halfConeAngle);
    },
    getHalfConeAngle(world: World, constraint: ConstraintId): number {
        return Low.ConeConstraint.getHalfConeAngle(world, constraint);
    },
    getCosHalfConeAngle(world: World, constraint: ConstraintId): number {
        return Low.ConeConstraint.getCosHalfConeAngle(world, constraint);
    },
    getTotalLambdaPosition(out: Vec3, world: World, constraint: ConstraintId): boolean {
        return copied(out, vec3, Low.ConeConstraint.getTotalLambdaPosition(vec3, world, constraint));
    },
    getTotalLambdaRotation(world: World, constraint: ConstraintId): number {
        return Low.ConeConstraint.getTotalLambdaRotation(world, constraint);
    },
};

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
export const SwingTwistConstraint = {
    create(world: World, bodyA: BodyId, bodyB: BodyId, settings: SwingTwistConstraintSettings = {}): ConstraintId {
        const native = over(swingTwistDefaults(), settings);
        expectMotor(native.swingMotorSettings);
        expectMotor(native.twistMotorSettings);
        Low.errorClear();
        return created(Low.SwingTwistConstraint.create(world, bodyA, bodyB, native));
    },
    remove(world: World, constraint: ConstraintId): boolean {
        return Low.SwingTwistConstraint.remove(world, constraint);
    },
    setNormalHalfConeAngle(world: World, constraint: ConstraintId, angle: number): boolean {
        return Low.SwingTwistConstraint.setNormalHalfConeAngle(world, constraint, angle);
    },
    setPlaneHalfConeAngle(world: World, constraint: ConstraintId, angle: number): boolean {
        return Low.SwingTwistConstraint.setPlaneHalfConeAngle(world, constraint, angle);
    },
    setTwistMinAngle(world: World, constraint: ConstraintId, angle: number): boolean {
        return Low.SwingTwistConstraint.setTwistMinAngle(world, constraint, angle);
    },
    setTwistMaxAngle(world: World, constraint: ConstraintId, angle: number): boolean {
        return Low.SwingTwistConstraint.setTwistMaxAngle(world, constraint, angle);
    },
    setMaxFrictionTorque(world: World, constraint: ConstraintId, torque: number): boolean {
        return Low.SwingTwistConstraint.setMaxFrictionTorque(world, constraint, torque);
    },
    setSwingMotorSettings(world: World, constraint: ConstraintId, settings: MotorSettings): boolean {
        expectMotor(settings);
        return Low.SwingTwistConstraint.setSwingMotorSettings(world, constraint, settings);
    },
    setTwistMotorSettings(world: World, constraint: ConstraintId, settings: MotorSettings): boolean {
        expectMotor(settings);
        return Low.SwingTwistConstraint.setTwistMotorSettings(world, constraint, settings);
    },
    setSwingMotorState(world: World, constraint: ConstraintId, state: MotorState): boolean {
        return Low.SwingTwistConstraint.setSwingMotorState(world, constraint, state);
    },
    setTwistMotorState(world: World, constraint: ConstraintId, state: MotorState): boolean {
        return Low.SwingTwistConstraint.setTwistMotorState(world, constraint, state);
    },
    /** rad/s in body b's constraint space, for velocity motors */
    setTargetAngularVelocityCS(world: World, constraint: ConstraintId, velocity: Vec3): boolean {
        return Low.SwingTwistConstraint.setTargetAngularVelocityCs(world, constraint, velocity);
    },
    /** in constraint space, for position motors; clamped to the limits */
    setTargetOrientationCS(world: World, constraint: ConstraintId, orientation: Quat): boolean {
        return Low.SwingTwistConstraint.setTargetOrientationCs(world, constraint, orientation);
    },
    /** in body space: body b turned to body a's orientation times this */
    setTargetOrientationBS(world: World, constraint: ConstraintId, orientation: Quat): boolean {
        return Low.SwingTwistConstraint.setTargetOrientationBs(world, constraint, orientation);
    },
    /** the turn between the bodies, in constraint space */
    getRotationInConstraintSpace(out: Quat, world: World, constraint: ConstraintId): boolean {
        return copied(out, quat, Low.SwingTwistConstraint.getRotationInConstraintSpace(quat, world, constraint));
    },
    getTotalLambdaPosition(out: Vec3, world: World, constraint: ConstraintId): boolean {
        return copied(out, vec3, Low.SwingTwistConstraint.getTotalLambdaPosition(vec3, world, constraint));
    },
    getTotalLambdaTwist(world: World, constraint: ConstraintId): number {
        return Low.SwingTwistConstraint.getTotalLambdaTwist(world, constraint);
    },
    getTotalLambdaSwingY(world: World, constraint: ConstraintId): number {
        return Low.SwingTwistConstraint.getTotalLambdaSwingY(world, constraint);
    },
    getTotalLambdaSwingZ(world: World, constraint: ConstraintId): number {
        return Low.SwingTwistConstraint.getTotalLambdaSwingZ(world, constraint);
    },
    getTotalLambdaMotor(out: Vec3, world: World, constraint: ConstraintId): boolean {
        return copied(out, vec3, Low.SwingTwistConstraint.getTotalLambdaMotor(vec3, world, constraint));
    },
};

/** one number per SixDOFAxis */
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
export const SixDOFConstraint = {
    create(world: World, bodyA: BodyId, bodyB: BodyId, settings: SixDOFConstraintSettings = {}): ConstraintId {
        expect((settings.limitsSpringSettings?.length ?? 0) <= 3, 'limitsSpringSettings has one per translation axis');
        expect((settings.motorSettings?.length ?? 0) <= 6, 'motorSettings has one per axis');
        const native = over(sixDOFDefaults(), settings);
        for (const motor of native.motorSettings) expectMotor(motor);
        Low.errorClear();
        return created(Low.SixDofConstraint.create(world, bodyA, bodyB, native));
    },
    remove(world: World, constraint: ConstraintId): boolean {
        return Low.SixDofConstraint.remove(world, constraint);
    },
    setTranslationLimits(world: World, constraint: ConstraintId, limitMin: Vec3, limitMax: Vec3): boolean {
        return Low.SixDofConstraint.setTranslationLimits(world, constraint, limitMin, limitMax);
    },
    setRotationLimits(world: World, constraint: ConstraintId, limitMin: Vec3, limitMax: Vec3): boolean {
        return Low.SixDofConstraint.setRotationLimits(world, constraint, limitMin, limitMax);
    },
    /** a translation axis */
    setLimitsSpringSettings(world: World, constraint: ConstraintId, axis: SixDOFAxis, settings: SpringSettings): boolean {
        expectAxis(axis, 3);
        return Low.SixDofConstraint.setLimitsSpringSettings(world, constraint, axis, settings);
    },
    setMotorSettings(world: World, constraint: ConstraintId, axis: SixDOFAxis, settings: MotorSettings): boolean {
        expectAxis(axis, 6);
        expectMotor(settings);
        return Low.SixDofConstraint.setMotorSettings(world, constraint, axis, settings);
    },
    setMotorState(world: World, constraint: ConstraintId, axis: SixDOFAxis, state: MotorState): boolean {
        expectAxis(axis, 6);
        return Low.SixDofConstraint.setMotorState(world, constraint, axis, state);
    },
    /** m/s in constraint space, for the translation velocity motors */
    setTargetVelocityCS(world: World, constraint: ConstraintId, velocity: Vec3): boolean {
        return Low.SixDofConstraint.setTargetVelocityCs(world, constraint, velocity);
    },
    /** rad/s in constraint space, for the rotation velocity motors */
    setTargetAngularVelocityCS(world: World, constraint: ConstraintId, velocity: Vec3): boolean {
        return Low.SixDofConstraint.setTargetAngularVelocityCs(world, constraint, velocity);
    },
    /** m in constraint space, for the translation position motors */
    setTargetPositionCS(world: World, constraint: ConstraintId, position: Vec3): boolean {
        return Low.SixDofConstraint.setTargetPositionCs(world, constraint, position);
    },
    /** in constraint space, for the rotation position motors; clamped to the rotation limits */
    setTargetOrientationCS(world: World, constraint: ConstraintId, orientation: Quat): boolean {
        return Low.SixDofConstraint.setTargetOrientationCs(world, constraint, orientation);
    },
    setMaxFriction(world: World, constraint: ConstraintId, axis: SixDOFAxis, friction: number): boolean {
        expectAxis(axis, 6);
        return Low.SixDofConstraint.setMaxFriction(world, constraint, axis, friction);
    },
    makeFreeAxis(world: World, constraint: ConstraintId, axis: SixDOFAxis): boolean {
        expectAxis(axis, 6);
        return Low.SixDofConstraint.makeFreeAxis(world, constraint, axis);
    },
    /** locked at 0 */
    makeFixedAxis(world: World, constraint: ConstraintId, axis: SixDOFAxis): boolean {
        expectAxis(axis, 6);
        return Low.SixDofConstraint.makeFixedAxis(world, constraint, axis);
    },
    /** max below min fixes the axis at 0 */
    setLimitedAxis(world: World, constraint: ConstraintId, axis: SixDOFAxis, min: number, max: number): boolean {
        expectAxis(axis, 6);
        return Low.SixDofConstraint.setLimitedAxis(world, constraint, axis, min, max);
    },
};
