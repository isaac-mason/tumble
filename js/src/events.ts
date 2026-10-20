import * as Low from './generated/tumble';
import type { Vec3 } from './math';
import type { BodyId } from './rigid-body';
import type { World } from './world';

/** how a contact is solved: its combined material, and the scales and surface velocities an override may change */
export type ContactSettings = Low.ContactSettings;

const TWO_32 = 0x100000000;

/** a pair of bodies whose bounds overlap, found by findPairs */
export type PairEvent = { bodyA: BodyId; bodyB: BodyId; userDataA: number; userDataB: number };

const pair = Low.PairEvent.create();

export const PairEvent = {
    create(): PairEvent {
        return { bodyA: 0, bodyB: 0, userDataA: 0, userDataB: 0 };
    },
    /** pairs entry `index` into out; false past World.pairs */
    read(out: PairEvent, world: World, index: number): boolean {
        if (!Low.Events.readPair(pair, world, index)) return false;
        out.bodyA = pair.bodyA;
        out.bodyB = pair.bodyB;
        out.userDataA = pair.userDataAHigh * TWO_32 + pair.userDataALow;
        out.userDataB = pair.userDataBHigh * TWO_32 + pair.userDataBLow;
        return true;
    },
};

/** a contact the narrowphase made or kept, with its manifold: up to four points on each body, the normal from a to b */
export type ContactEvent = {
    bodyA: BodyId;
    bodyB: BodyId;
    userDataA: number;
    userDataB: number;
    subShapeA: number;
    subShapeB: number;
    /** -1 for none */
    materialA: number;
    materialB: number;
    normal: Vec3;
    penetration: number;
    /** how many of pointsA and pointsB are the contact's */
    points: number;
    pointsA: Vec3[];
    pointsB: Vec3[];
    settings: ContactSettings;
};

const contact = Low.ContactEvent.create();

function copyContact(out: ContactEvent): void {
    out.bodyA = contact.bodyA;
    out.bodyB = contact.bodyB;
    out.userDataA = contact.userDataAHigh * TWO_32 + contact.userDataALow;
    out.userDataB = contact.userDataBHigh * TWO_32 + contact.userDataBLow;
    out.subShapeA = contact.subShapeA;
    out.subShapeB = contact.subShapeB;
    out.materialA = contact.materialA;
    out.materialB = contact.materialB;
    out.normal[0] = contact.normal[0];
    out.normal[1] = contact.normal[1];
    out.normal[2] = contact.normal[2];
    out.penetration = contact.penetration;
    out.points = contact.numPoints;
    for (let i = 0; i < contact.numPoints; i++) {
        const a = contact.pointsA[i]!;
        const b = contact.pointsB[i]!;
        const outA = out.pointsA[i]!;
        const outB = out.pointsB[i]!;
        outA[0] = a[0];
        outA[1] = a[1];
        outA[2] = a[2];
        outB[0] = b[0];
        outB[1] = b[1];
        outB[2] = b[2];
    }
    Object.assign(out.settings, contact.settings);
    out.settings.relativeLinearSurfaceVelocity = [...contact.settings.relativeLinearSurfaceVelocity];
    out.settings.relativeAngularSurfaceVelocity = [...contact.settings.relativeAngularSurfaceVelocity];
}

export const ContactEvent = {
    create(): ContactEvent {
        return {
            bodyA: 0,
            bodyB: 0,
            userDataA: 0,
            userDataB: 0,
            subShapeA: 0,
            subShapeB: 0,
            materialA: -1,
            materialB: -1,
            normal: [0, 0, 0],
            penetration: 0,
            points: 0,
            pointsA: [
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
            ],
            pointsB: [
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
            ],
            settings: Low.ContactSettings.create(),
        };
    },
    /** contacts added entry `index` into out; false past World.contactsAdded */
    readAdded(out: ContactEvent, world: World, index: number): boolean {
        if (!Low.Events.readContactAdded(contact, world, index)) return false;
        copyContact(out);
        return true;
    },
    /** contacts persisted entry `index` into out; false past World.contactsPersisted */
    readPersisted(out: ContactEvent, world: World, index: number): boolean {
        if (!Low.Events.readContactPersisted(contact, world, index)) return false;
        copyContact(out);
        return true;
    },
};

/** a contact that was there last step and is gone; its bodies may be gone too */
export type ContactRemovedEvent = { bodyA: BodyId; bodyB: BodyId; subShapeA: number; subShapeB: number };

const removed = Low.ContactRemovedEvent.create();

export const ContactRemovedEvent = {
    create(): ContactRemovedEvent {
        return { bodyA: 0, bodyB: 0, subShapeA: 0, subShapeB: 0 };
    },
    read(out: ContactRemovedEvent, world: World, index: number): boolean {
        if (!Low.Events.readContactRemoved(removed, world, index)) return false;
        Object.assign(out, removed);
        return true;
    },
};

/** answers to this step's events, by the index they were read at */
export const Answers = {
    /** no narrowphase for the pair, and its contacts go; after findPairs */
    rejectPair(world: World, index: number): boolean {
        return Low.Answers.rejectPair(world, index);
    },
    /**
     * contact `index` solved with these settings, after the last narrowphase. contacts are numbered as the narrowphase
     * makes them: the added first, then the persisted, so contacts persisted entry i is index contactsAdded + i
     */
    overrideContact(world: World, index: number, settings: ContactSettings): boolean {
        return Low.Answers.overrideContact(world, index, settings);
    },
    /** contact `index` dropped before the solve, numbered as overrideContact's */
    rejectContact(world: World, index: number): boolean {
        return Low.Answers.rejectContact(world, index);
    },
};
