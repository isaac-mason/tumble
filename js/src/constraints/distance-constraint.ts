import type { BodyId } from '../body.js';
import * as Low from '../generated/tumble.js';
import type { Vec3 } from '../math.js';
import { defaultsOf, type Given, over } from '../settings.js';
import type { World } from '../world.js';
import { type Common, type ConstraintId, created, expect, expectCommon, type SpringSettings } from './common.js';

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
export function create(world: World, bodyA: BodyId, bodyB: BodyId, settings: DistanceConstraintSettings = {}): ConstraintId {
    expectCommon(settings);
    const native = over(distanceDefaults(), settings);
    if (native.minDistance >= 0 && native.maxDistance >= 0) {
        expect(native.minDistance <= native.maxDistance, 'minDistance is at most maxDistance');
    }
    Low.errorClear();
    return created(Low.DistanceConstraint.create(world, bodyA, bodyB, native));
}

export function setDistance(world: World, constraint: ConstraintId, minDistance: number, maxDistance: number): boolean {
    expect(minDistance <= maxDistance, 'minDistance is at most maxDistance');
    return Low.DistanceConstraint.setDistance(world, constraint, minDistance, maxDistance);
}

export function setLimitsSpringSettings(world: World, constraint: ConstraintId, settings: SpringSettings): boolean {
    return Low.DistanceConstraint.setLimitsSpringSettings(world, constraint, settings);
}

export function getTotalLambda(world: World, constraint: ConstraintId): number {
    return Low.DistanceConstraint.getTotalLambda(world, constraint);
}
