import type { BodyId } from '../body.js';
import * as Low from '../generated/tumble.js';
import { copied, type Vec3 } from '../math.js';
import { defaultsOf, over } from '../settings.js';
import type { World } from '../world.js';
import { type Common, type ConstraintId, created, expectCommon, vec3 } from './common.js';

export type PointConstraintSettings = {
    /** the point on body a, held to the point on body b */
    pointA?: Vec3;
    pointB?: Vec3;
} & Common;

const pointDefaults = defaultsOf(Low.PointConstraint.defaultSettings, Low.PointConstraintSettings.create);

/** a point on each body held together: a ball and socket, free to turn */
export function create(world: World, bodyA: BodyId, bodyB: BodyId, settings: PointConstraintSettings = {}): ConstraintId {
    expectCommon(settings);
    Low.errorClear();
    return created(Low.PointConstraint.create(world, bodyA, bodyB, over(pointDefaults(), settings)));
}

/** the impulse it applied in the last step, N s: for breaking it by force */
export function getTotalLambda(out: Vec3, world: World, constraint: ConstraintId): boolean {
    return copied(out, vec3, Low.PointConstraint.getTotalLambda(vec3, world, constraint));
}
