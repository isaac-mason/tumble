import * as Low from './generated/tumble';
import type { Vec3 } from './math';
import type { BodyId } from './rigid-body';
import type { World } from './world';

/** how a contact is solved: its combined material, and the scales and surface velocities a hook may change */
export type ContactSettings = Low.ContactSettings;

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
};

/** a contact that was there last step and is gone; its bodies may be gone too */
export type ContactRemovedEvent = { bodyA: BodyId; bodyB: BodyId; subShapeA: number; subShapeB: number };

/**
 * what a step tells you, and asks you, as it runs. every hook is optional, and a step with none runs as one call. the
 * events a hook is given are reused: copy what you keep. the world is mid-step while the hooks run, so read it but do
 * not change it. a hook that throws ends the hooks for the step, which finishes and then throws it
 */
export type Listener = {
    /** two bodies whose bounds overlap, before the narrowphase: false and they do not collide this step. the cheapest place to filter a pair */
    onBodyPairValidate?: (bodyA: BodyId, bodyB: BodyId) => boolean;
    /** every contact, new or kept, each step before onContactAdded or onContactPersisted: false drops it from this step's solve */
    onContactValidate?: (contact: ContactEvent) => boolean;
    /** a contact new this step; change the settings to solve it differently */
    onContactAdded?: (contact: ContactEvent, settings: ContactSettings) => void;
    /** a contact kept from the last step; change the settings to solve it differently */
    onContactPersisted?: (contact: ContactEvent, settings: ContactSettings) => void;
    /** after the solve: a contact gone since the last step, a removed body's included */
    onContactRemoved?: (removed: ContactRemovedEvent) => void;
};

const TWO_32 = 0x100000000;

const pair = Low.PairEvent.create();
const contact = Low.ContactEvent.create();
const removed = Low.ContactRemovedEvent.create();

const contactEvent: ContactEvent = {
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
};
const removedEvent: ContactRemovedEvent = { bodyA: 0, bodyB: 0, subShapeA: 0, subShapeB: 0 };
/** the settings a hook changes, and what they were, to answer only a change */
const settings = Low.ContactSettings.create();
const settingsBefore = Low.ContactSettings.create();

function copyVec3(out: Vec3, from: Vec3): void {
    out[0] = from[0];
    out[1] = from[1];
    out[2] = from[2];
}

function copyContact(): void {
    contactEvent.bodyA = contact.bodyA;
    contactEvent.bodyB = contact.bodyB;
    contactEvent.userDataA = contact.userDataAHigh * TWO_32 + contact.userDataALow;
    contactEvent.userDataB = contact.userDataBHigh * TWO_32 + contact.userDataBLow;
    contactEvent.subShapeA = contact.subShapeA;
    contactEvent.subShapeB = contact.subShapeB;
    contactEvent.materialA = contact.materialA;
    contactEvent.materialB = contact.materialB;
    copyVec3(contactEvent.normal, contact.normal);
    contactEvent.penetration = contact.penetration;
    contactEvent.points = contact.numPoints;
    for (let i = 0; i < contact.numPoints; i++) {
        copyVec3(contactEvent.pointsA[i]!, contact.pointsA[i]!);
        copyVec3(contactEvent.pointsB[i]!, contact.pointsB[i]!);
    }
}

function copySettings(out: ContactSettings, from: ContactSettings): void {
    out.combinedFriction = from.combinedFriction;
    out.combinedRestitution = from.combinedRestitution;
    out.isSensor = from.isSensor;
    out.invMassScale1 = from.invMassScale1;
    out.invMassScale2 = from.invMassScale2;
    out.invInertiaScale1 = from.invInertiaScale1;
    out.invInertiaScale2 = from.invInertiaScale2;
    copyVec3(out.relativeLinearSurfaceVelocity, from.relativeLinearSurfaceVelocity);
    copyVec3(out.relativeAngularSurfaceVelocity, from.relativeAngularSurfaceVelocity);
}

function sameVec3(a: Vec3, b: Vec3): boolean {
    return a[0] === b[0] && a[1] === b[1] && a[2] === b[2];
}

function sameSettings(a: ContactSettings, b: ContactSettings): boolean {
    return (
        a.combinedFriction === b.combinedFriction &&
        a.combinedRestitution === b.combinedRestitution &&
        a.isSensor === b.isSensor &&
        a.invMassScale1 === b.invMassScale1 &&
        a.invMassScale2 === b.invMassScale2 &&
        a.invInertiaScale1 === b.invInertiaScale1 &&
        a.invInertiaScale2 === b.invInertiaScale2 &&
        sameVec3(a.relativeLinearSurfaceVelocity, b.relativeLinearSurfaceVelocity) &&
        sameVec3(a.relativeAngularSurfaceVelocity, b.relativeAngularSurfaceVelocity)
    );
}

let failed = false;
let failure: unknown;

function fail(error: unknown): void {
    if (failed) return;
    failed = true;
    failure = error;
}

/** contact `index` (the added, then the persisted) through a settings hook, answered when it changed them */
function answerSettings(world: World, index: number, hook: (contact: ContactEvent, settings: ContactSettings) => void): void {
    copySettings(settings, contact.settings);
    copySettings(settingsBefore, contact.settings);
    try {
        hook(contactEvent, settings);
    } catch (error) {
        fail(error);
        return;
    }
    if (!sameSettings(settings, settingsBefore)) Low.Answers.overrideContact(world, index, settings);
}

/** the contact just read, contact `index` (the added, then the persisted), through the validate hook and a settings hook */
function answerContact(
    world: World,
    index: number,
    validate: Listener['onContactValidate'],
    hook: ((contact: ContactEvent, settings: ContactSettings) => void) | undefined,
): void {
    copyContact();
    if (validate !== undefined) {
        let keep = true;
        try {
            keep = validate(contactEvent);
        } catch (error) {
            fail(error);
        }
        if (!keep) {
            Low.Answers.rejectContact(world, index);
            return;
        }
    }
    if (hook !== undefined && !failed) answerSettings(world, index, hook);
}

/** a step, its hooks run between its phases */
export function stepWith(world: World, listener: Listener | undefined, deltaTime: number): void {
    const { onBodyPairValidate, onContactValidate, onContactAdded, onContactPersisted, onContactRemoved } = listener ?? {};
    failed = false;
    failure = undefined;
    if (
        onBodyPairValidate === undefined &&
        onContactValidate === undefined &&
        onContactAdded === undefined &&
        onContactPersisted === undefined
    ) {
        Low.World.step(world, deltaTime);
    } else {
        Low.World.stepBegin(world, deltaTime);
        // each find appends the pairs of the bodies the last narrowphase woke
        let pairsRead = 0;
        do {
            Low.World.findPairs(world);
            if (onBodyPairValidate !== undefined) {
                for (const pairs = Low.Events.pairs(world); pairsRead < pairs && !failed; pairsRead++) {
                    Low.Events.readPair(pair, world, pairsRead);
                    let collide = true;
                    try {
                        collide = onBodyPairValidate(pair.bodyA, pair.bodyB);
                    } catch (error) {
                        fail(error);
                    }
                    if (!collide) Low.Answers.rejectPair(world, pairsRead);
                }
            }
            Low.World.narrowphase(world);
        } while (Low.World.woken(world) > 0);

        const added = Low.Events.contactsAdded(world);
        if (onContactValidate !== undefined || onContactAdded !== undefined) {
            for (let i = 0; i < added && !failed; i++) {
                Low.Events.readContactAdded(contact, world, i);
                answerContact(world, i, onContactValidate, onContactAdded);
            }
        }
        if (onContactValidate !== undefined || onContactPersisted !== undefined) {
            const persisted = Low.Events.contactsPersisted(world);
            for (let i = 0; i < persisted && !failed; i++) {
                Low.Events.readContactPersisted(contact, world, i);
                answerContact(world, added + i, onContactValidate, onContactPersisted);
            }
        }
        Low.World.solve(world);
        Low.World.stepEnd(world);
    }
    if (onContactRemoved !== undefined) {
        const count = Low.Events.contactsRemoved(world);
        for (let i = 0; i < count && !failed; i++) {
            Low.Events.readContactRemoved(removed, world, i);
            removedEvent.bodyA = removed.bodyA;
            removedEvent.bodyB = removed.bodyB;
            removedEvent.subShapeA = removed.subShapeA;
            removedEvent.subShapeB = removed.subShapeB;
            try {
                onContactRemoved(removedEvent);
            } catch (error) {
                fail(error);
            }
        }
    }
    if (failed) {
        const error = failure;
        failure = undefined;
        throw error;
    }
}
