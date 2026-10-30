import type { BodyId } from './body.js';
import * as Low from './generated/tumble.js';
import type { Vec3 } from './math.js';
import type { World } from './world.js';

/** how a contact is solved: its combined material, and the scales and surface velocities a hook may change */
export type ContactSettings = Low.ContactSettings;

/**
 * a contact the narrowphase made or kept, with its manifold: up to four points on each body (numPoints of them), the
 * normal from a to b. materialIdA and materialIdB are -1 for none
 */
export type ContactEvent = Omit<Low.ContactEvent, 'settings' | 'rejected' | 'settingsChanged'>;

/** a contact that was there last step and is gone; its bodies may be gone too */
export type ContactRemovedEvent = Low.ContactRemovedEvent;

/**
 * what a step tells you, and asks you, as it runs. every hook is optional, and a step with no pair or contact hook runs
 * as one call. the events a hook is given are reused: copy what you keep. the world is mid-step while the pair and
 * contact hooks run, so read it but do not change it. a hook that throws ends the hooks for the step, which finishes and
 * then throws it
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
    /**
     * after the step: a body joined the active list, woken by the step or by a call since the last step. heard in the
     * order it happened, with onBodyDeactivated; here the world can be changed, and what that wakes is heard too
     */
    onBodyActivated?: (body: BodyId) => void;
    /** after the step: a body left the active list, put to sleep, made static or removed, by the step or a call since */
    onBodyDeactivated?: (body: BodyId) => void;
};

const activation = Low.ActivationEvent.create();
const pair = Low.PairEvent.create();
/** read from the world's buffers where they lie; its settings are what a hook is given to change */
const contact = Low.ContactEvent.create();
const removed = Low.ContactRemovedEvent.create();
/** a contact's settings as the step has them, to tell whether a hook changed them */
const settingsBefore = Low.ContactSettings.create();

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

/** the contact just read from `address` through a settings hook, its settings written back when the hook changed them */
function changeSettings(address: number, hook: (contact: ContactEvent, settings: ContactSettings) => void): void {
    try {
        hook(contact, contact.settings);
    } catch (error) {
        fail(error);
        return;
    }
    if (sameSettings(contact.settings, Low.ContactEvent.readSettings(settingsBefore, address))) return;
    Low.ContactEvent.writeSettings(address, contact.settings);
    Low.ContactEvent.setSettingsChanged(address, true);
}

/** the step's added or persisted contacts, each read where it lies, through the validate hook and a settings hook */
function handleContacts(
    events: Low.WorldEvents,
    persisted: boolean,
    validate: Listener['onContactValidate'],
    hook: ((contact: ContactEvent, settings: ContactSettings) => void) | undefined,
): void {
    const count = persisted ? Low.WorldEvents.contactsPersistedLength(events) : Low.WorldEvents.contactsAddedLength(events);
    for (let i = 0; i < count && !failed; i++) {
        const address = persisted
            ? Low.WorldEvents.contactsPersistedAddress(events, i)
            : Low.WorldEvents.contactsAddedAddress(events, i);
        Low.ContactEvent.read(contact, address);
        if (validate !== undefined) {
            let keep = true;
            try {
                keep = validate(contact);
            } catch (error) {
                fail(error);
            }
            if (!keep) {
                Low.ContactEvent.setRejected(address, true);
                continue;
            }
        }
        if (hook !== undefined && !failed) changeSettings(address, hook);
    }
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
        const events = Low.World.events(world);
        // each find appends the pairs of the bodies the last narrowphase woke
        let pairsRead = 0;
        do {
            Low.World.findPairs(world);
            if (onBodyPairValidate !== undefined) {
                for (const count = Low.WorldEvents.pairsLength(events); pairsRead < count && !failed; pairsRead++) {
                    const address = Low.WorldEvents.pairsAddress(events, pairsRead);
                    Low.PairEvent.read(pair, address);
                    let collide = true;
                    try {
                        collide = onBodyPairValidate(pair.bodyIdA, pair.bodyIdB);
                    } catch (error) {
                        fail(error);
                    }
                    if (!collide) Low.PairEvent.setRejected(address, true);
                }
            }
            Low.World.narrowphase(world);
        } while (Low.WorldEvents.woken(events) > 0);

        if (onContactValidate !== undefined || onContactAdded !== undefined) {
            handleContacts(events, false, onContactValidate, onContactAdded);
        }
        if (onContactValidate !== undefined || onContactPersisted !== undefined) {
            handleContacts(events, true, onContactValidate, onContactPersisted);
        }
        Low.World.solve(world);
        Low.World.stepEnd(world);
    }
    if (onContactRemoved !== undefined) {
        const events = Low.World.events(world);
        const count = Low.WorldEvents.contactsRemovedLength(events);
        for (let i = 0; i < count && !failed; i++) {
            Low.WorldEvents.readContactsRemoved(removed, events, i);
            try {
                onContactRemoved(removed);
            } catch (error) {
                fail(error);
            }
        }
    }
    const { onBodyActivated, onBodyDeactivated } = listener ?? {};
    if (onBodyActivated !== undefined || onBodyDeactivated !== undefined) {
        // the count read each time round: a hook that wakes a body records another change
        for (let i = 0; i < Low.Activation.events(world) && !failed; i++) {
            Low.Activation.readEvent(activation, world, i);
            const hook = activation.activated !== 0 ? onBodyActivated : onBodyDeactivated;
            if (hook === undefined) continue;
            try {
                hook(activation.body);
            } catch (error) {
                fail(error);
            }
        }
    }
    Low.Activation.clear(world);
    if (failed) {
        const error = failure;
        failure = undefined;
        throw error;
    }
}
