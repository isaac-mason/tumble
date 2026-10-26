import * as Low from '../generated/tumble.js';
import type { World } from '../world.js';
import type { ConstraintId, ConstraintType } from './common.js';

// what every constraint has, whatever its type
/** the type the id encodes, whether or not it names a live constraint */
export function getType(constraint: ConstraintId): ConstraintType {
    return Low.Constraint.getType(constraint) as ConstraintType;
}

/** whether the id names a constraint in the world; one goes with either of its bodies */
export function exists(world: World, constraint: ConstraintId): boolean {
    return Low.Constraint.exists(world, constraint);
}

/** a disabled constraint holds nothing and does not join its bodies' islands, so they may sleep apart; false when the id names no constraint */
export function setEnabled(world: World, constraint: ConstraintId, enabled: boolean): boolean {
    return Low.Constraint.setEnabled(world, constraint, enabled);
}

export function isEnabled(world: World, constraint: ConstraintId): boolean {
    return Low.Constraint.isEnabled(world, constraint);
}

/** false when the id names no constraint */
export function remove(world: World, constraint: ConstraintId): boolean {
    return Low.Constraint.remove(world, constraint);
}
