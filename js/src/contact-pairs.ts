import * as Low from './generated/tumble.js';
import type { KCC } from './kcc.js';
import type { World } from './world.js';

/**
 * a pair of body sub shapes in contact, one side a body observing contacts; its fields are its last record's. handle is
 * stable while the pair lasts and reused the frame after it is removed; point is the first contact point, on a; normal
 * is from a to b; relativeVelocity is b's velocity less a's; added is not there the frame before
 */
export type ContactPair = Low.ContactPair;

/**
 * a frame of contact pairs: begin, record what moved (each world step, each character's update), end, then read. a
 * pair is added the frame it starts, persisted while it lasts, and removed the frame after it was last recorded. only
 * the pairs a body observing contacts is a side of are kept
 */
export function create(): ContactPair {
    return Low.ContactPair.create();
}

/** last frame's removed pairs freed, their handles reusable */
export function begin(world: World): void {
    Low.errorClear();
    Low.ContactPairs.begin(world);
    Low.getWasm().check();
}

/** the last step's contacts */
export function recordWorldContacts(world: World): void {
    Low.errorClear();
    Low.ContactPairs.recordWorldContacts(world, 0, 0);
    Low.getWasm().check();
}

/** the character's collisions from its last update or refresh, its inner body as side a; nothing without an inner body */
export function recordCharacterContacts(world: World, character: KCC): void {
    Low.errorClear();
    Low.ContactPairs.recordCharacterContacts(world, character.handle);
    Low.getWasm().check();
}

/** the live pairs not recorded this frame become its removed */
export function end(world: World): void {
    Low.errorClear();
    Low.ContactPairs.end(world);
    Low.getWasm().check();
}

/** this frame's pairs, added and persisted, in the order first recorded */
export function activeCount(world: World): number {
    return Low.ContactPairs.activeCount(world);
}

/** false past activeCount */
export function readActive(out: ContactPair, world: World, index: number): boolean {
    return Low.ContactPairs.readActive(out, world, index);
}

export function removedCount(world: World): number {
    return Low.ContactPairs.removedCount(world);
}

/** a pair live last frame, not this; its fields last frame's */
export function readRemoved(out: ContactPair, world: World, index: number): boolean {
    return Low.ContactPairs.readRemoved(out, world, index);
}
