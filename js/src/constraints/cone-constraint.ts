import type { BodyId } from '../body.js';
import * as Low from '../generated/tumble.js';
import { copied, type Vec3 } from '../math.js';
import { defaultsOf, over } from '../settings.js';
import type { World } from '../world.js';
import { type Common, type ConstraintId, created, expect, expectCommon, PI, vec3 } from './common.js';

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
export function create(world: World, bodyA: BodyId, bodyB: BodyId, settings: ConeConstraintSettings = {}): ConstraintId {
    expectCommon(settings);
    const native = over(coneDefaults(), settings);
    expectHalfConeAngle(native.halfConeAngle);
    Low.errorClear();
    return created(Low.ConeConstraint.create(world, bodyA, bodyB, native));
}

export function setHalfConeAngle(world: World, constraint: ConstraintId, halfConeAngle: number): boolean {
    expectHalfConeAngle(halfConeAngle);
    return Low.ConeConstraint.setHalfConeAngle(world, constraint, halfConeAngle);
}

export function getHalfConeAngle(world: World, constraint: ConstraintId): number {
    return Low.ConeConstraint.getHalfConeAngle(world, constraint);
}

export function getCosHalfConeAngle(world: World, constraint: ConstraintId): number {
    return Low.ConeConstraint.getCosHalfConeAngle(world, constraint);
}

export function getTotalLambdaPosition(out: Vec3, world: World, constraint: ConstraintId): boolean {
    return copied(out, vec3, Low.ConeConstraint.getTotalLambdaPosition(vec3, world, constraint));
}

export function getTotalLambdaRotation(world: World, constraint: ConstraintId): number {
    return Low.ConeConstraint.getTotalLambdaRotation(world, constraint);
}
