import * as Low from './generated/tumble';
import type { KCC } from './kcc';
import type { Vec3 } from './math';
import type { BodyId } from './rigid-body';
import type { World } from './world';

/** a pair of body sub shapes in contact, one side a body observing contacts; its fields are its last record's */
export type ContactPair = {
    /** stable while the pair lasts; reused the frame after it is removed */
    handle: number;
    bodyA: BodyId;
    subShapeA: number;
    userDataA: number;
    sensorA: boolean;
    bodyB: BodyId;
    subShapeB: number;
    userDataB: number;
    sensorB: boolean;
    /** the first contact point, on a */
    point: Vec3;
    /** from a to b */
    normal: Vec3;
    penetrationDepth: number;
    /** b's velocity less a's */
    relativeVelocity: Vec3;
    /** not there the frame before */
    added: boolean;
};

const TWO_32 = 0x100000000;
const pair = Low.ContactPair.create();

function copyPair(out: ContactPair): ContactPair {
    out.handle = pair.handle;
    out.bodyA = pair.bodyA;
    out.subShapeA = pair.subShapeA;
    out.userDataA = pair.userDataAHigh * TWO_32 + pair.userDataALow;
    out.sensorA = pair.sensorA;
    out.bodyB = pair.bodyB;
    out.subShapeB = pair.subShapeB;
    out.userDataB = pair.userDataBHigh * TWO_32 + pair.userDataBLow;
    out.sensorB = pair.sensorB;
    for (let i = 0; i < 3; i++) {
        out.point[i] = pair.point[i]!;
        out.normal[i] = pair.normal[i]!;
        out.relativeVelocity[i] = pair.relativeVelocity[i]!;
    }
    out.penetrationDepth = pair.penetrationDepth;
    out.added = pair.added;
    return out;
}

/**
 * a frame of contact pairs: begin, record what moved (each world step, each character's update), end, then read. a
 * pair is added the frame it starts, persisted while it lasts, and removed the frame after it was last recorded. only
 * the pairs a body observing contacts is a side of are kept
 */
export const ContactPairs = {
    create(): ContactPair {
        return {
            handle: 0,
            bodyA: 0,
            subShapeA: 0,
            userDataA: 0,
            sensorA: false,
            bodyB: 0,
            subShapeB: 0,
            userDataB: 0,
            sensorB: false,
            point: [0, 0, 0],
            normal: [0, 0, 0],
            penetrationDepth: 0,
            relativeVelocity: [0, 0, 0],
            added: false,
        };
    },
    /** last frame's removed pairs freed, their handles reusable */
    begin(world: World): void {
        Low.errorClear();
        Low.ContactPairs.begin(world);
        Low.getWasm().check();
    },
    /** the last step's contacts */
    recordWorldContacts(world: World): void {
        Low.errorClear();
        Low.ContactPairs.recordWorldContacts(world, 0, 0);
        Low.getWasm().check();
    },
    /** the character's collisions from its last update or refresh, its inner body as side a; nothing without an inner body */
    recordCharacterContacts(world: World, character: KCC): void {
        Low.errorClear();
        Low.ContactPairs.recordCharacterContacts(world, character.handle);
        Low.getWasm().check();
    },
    /** the live pairs not recorded this frame become its removed */
    end(world: World): void {
        Low.errorClear();
        Low.ContactPairs.end(world);
        Low.getWasm().check();
    },
    /** this frame's pairs, added and persisted, in the order first recorded */
    activeCount(world: World): number {
        return Low.ContactPairs.activeCount(world);
    },
    /** false past activeCount */
    readActive(out: ContactPair, world: World, index: number): boolean {
        if (!Low.ContactPairs.readActive(pair, world, index)) return false;
        copyPair(out);
        return true;
    },
    removedCount(world: World): number {
        return Low.ContactPairs.removedCount(world);
    },
    /** a pair live last frame, not this; its fields last frame's */
    readRemoved(out: ContactPair, world: World, index: number): boolean {
        if (!Low.ContactPairs.readRemoved(pair, world, index)) return false;
        copyPair(out);
        return true;
    },
};
