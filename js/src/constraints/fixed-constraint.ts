import type { BodyId } from '../body.js';
import * as Low from '../generated/tumble.js';
import { copied, type Vec3 } from '../math.js';
import { defaultsOf, over } from '../settings.js';
import type { World } from '../world.js';
import { type Common, type ConstraintId, created, expectCommon, vec3 } from './common.js';

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
export function create(world: World, bodyA: BodyId, bodyB: BodyId, settings: FixedConstraintSettings = {}): ConstraintId {
    expectCommon(settings);
    Low.errorClear();
    return created(Low.FixedConstraint.create(world, bodyA, bodyB, over(fixedDefaults(), settings)));
}

export function getTotalLambdaPosition(out: Vec3, world: World, constraint: ConstraintId): boolean {
    return copied(out, vec3, Low.FixedConstraint.getTotalLambdaPosition(vec3, world, constraint));
}

export function getTotalLambdaRotation(out: Vec3, world: World, constraint: ConstraintId): boolean {
    return copied(out, vec3, Low.FixedConstraint.getTotalLambdaRotation(vec3, world, constraint));
}
