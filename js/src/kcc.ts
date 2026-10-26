import { type BodyId, INVALID_BODY_ID, type MotionType } from './body.js';
import type { Filter } from './filter.js';
import * as Low from './generated/tumble.js';
import { type KCCPolicy, policyOf } from './kcc-policy.js';
import { copied, copyVec3, type Quat, type Vec3 } from './math.js';
import { defaultsOf, type Given, over } from './settings.js';
import type { Shape } from './shapes/shapes.js';
import type { World } from './world.js';

export const GroundState = Low.GroundState;
export type GroundState = Low.GroundState;
export const BackFaceMode = Low.BackFaceMode;
export type BackFaceMode = Low.BackFaceMode;
export const KCCContactEventKind = Low.KCCContactEventKind;
export type KCCContactEventKind = Low.KCCContactEventKind;

export type KCCSettings = {
    /** the character's shape, its origin at the character's position (the feet, say); borrowed */
    shape: Shape;
    /** from the character's position, in its frame */
    shapeOffset?: Vec3;
    /** kg, for its weight on what it stands on */
    mass?: number;
    /** N, the most it pushes with */
    maxStrength?: number;
    /** normalized */
    up?: Vec3;
    /** radians: steeper is too steep to walk */
    maxSlopeAngle?: number;
    /** contacts this far ahead are found before they are hit */
    predictiveContactDistance?: number;
    /** kept between the shape and what it touches */
    characterPadding?: number;
    maxCollisionIterations?: number;
    maxConstraintIterations?: number;
    /** the penetration accepted */
    collisionTolerance?: number;
    /** 0 to 1: how much of a penetration is undone each update */
    penetrationRecoverySpeed?: number;
    /** a move stops with less time than this left */
    minTimeRemaining?: number;
    /** contacts a collide keeps */
    maxNumHits?: number;
    /** contacts on one body within this cosine merge */
    hitReductionCosMaxAngle?: number;
    backFaceMode?: BackFaceMode;
    enhancedInternalEdgeRemoval?: boolean;
    /** what can support the character, in its own frame: normal xyz and constant w; without, anything below it */
    supportingVolumePlane?: [x: number, y: number, z: number, w: number];
    /** carried by the inner body, an integer up to 2^53 */
    userData?: number;
    /** carried by the inner body: its contacts become contact pairs */
    observesContacts?: boolean;
    /** the character's presence in the world, seen by bodies and queries, once added */
    innerRigidBody?: { shape: Shape; objectLayer: number };
};

/** what a character's settings can change after create, the fields KCCSettings gives at create */
export type KCCTuning = {
    shapeOffset: Vec3;
    mass: number;
    maxStrength: number;
    /** normalized when set */
    up: Vec3;
    maxSlopeAngle: number;
    predictiveContactDistance: number;
    characterPadding: number;
    maxCollisionIterations: number;
    maxConstraintIterations: number;
    collisionTolerance: number;
    penetrationRecoverySpeed: number;
    minTimeRemaining: number;
    maxNumHits: number;
    hitReductionCosMaxAngle: number;
    backFaceMode: BackFaceMode;
    enhancedInternalEdgeRemoval: boolean;
    /** normal xyz and constant w in the character's frame; without one given at create, up at -1e10 */
    supportingVolumePlane: [x: number, y: number, z: number, w: number];
};

const tuning = Low.KCCTuning.create();

export type KCCUpdateSettings = Low.KCCUpdateSettings;

export function createDefaultUpdateSettings(): KCCUpdateSettings {
    return Low.Kcc.defaultUpdateSettings(Low.KCCUpdateSettings.create());
}

export type GroundInfo = {
    state: GroundState;
    /** the average of the supporting normals */
    normal: Vec3;
    velocity: Vec3;
    /** INVALID_BODY_ID in the air */
    bodyId: BodyId;
    subShapeId: number;
    position: Vec3;
};

/** a contact added or persisted as the character first met it this frame, or removed at the frame's end */
export type KCCContactEvent = {
    kind: KCCContactEventKind;
    bodyId: BodyId;
    subShapeId: number;
    /** on the body; zero for a removal */
    position: Vec3;
    /** away from the character, into the body; zero for a removal */
    normal: Vec3;
};

/** a contact of the character's: its last call's, in character.contacts, or a sweep's (cast) */
export type KCCContact = {
    position: Vec3;
    /** the body's velocity at the point, the policy's surface velocity included; between collide and solve, yours to set */
    linearVelocity: Vec3;
    /** toward the character */
    contactNormal: Vec3;
    /** may differ from the contact normal at edges */
    surfaceNormal: Vec3;
    /** negative is penetration */
    distance: number;
    /** along the sweep, 0 to 1 */
    fraction: number;
    bodyId: BodyId;
    subShapeId: number;
    materialId: number;
    motionType: MotionType;
    isSensor: boolean;
    /** a real collision, not only a predictive contact */
    hadCollision: boolean;
    /** passed over by the solve, the sweep and the ground; between collide and solve, yours to set */
    wasDiscarded: boolean;
    canPushCharacter: boolean;
    canReceiveImpulses: boolean;
};

export function createContact(): KCCContact {
    return {
        position: [0, 0, 0],
        linearVelocity: [0, 0, 0],
        contactNormal: [0, 0, 0],
        surfaceNormal: [0, 0, 0],
        distance: 0,
        fraction: 0,
        bodyId: INVALID_BODY_ID,
        subShapeId: 0,
        materialId: -1,
        motionType: 0 as MotionType,
        isSensor: false,
        hadCollision: false,
        wasDiscarded: false,
        canPushCharacter: true,
        canReceiveImpulses: true,
    };
}

/** a solve's result: the velocity it left, the displacement and the time simulated */
export type KCCSolveResult = { velocity: Vec3; displacement: Vec3; timeSimulated: number };

export function createSolveResult(): KCCSolveResult {
    return { velocity: [0, 0, 0], displacement: [0, 0, 0], timeSimulated: 0 };
}

const STRIDE = Low.KCC_CONTACT_STRIDE;

/**
 * a character: a shape slid through the world by sweeps. its fields are the wasm's state as its last call left it,
 * copied out after each call; linearVelocity is yours to set before a move, and between collide and solve so are its
 * contacts' wasDiscarded and linearVelocity
 */
export type KCC = {
    /** the character in the wasm */
    handle: Low.KCC;
    position: Vec3;
    quaternion: Quat;
    linearVelocity: Vec3;
    up: Vec3;
    ground: GroundInfo;
    /** INVALID_BODY_ID without one, or before it is added */
    innerRigidBodyId: BodyId;
    shape: Shape;
    contacts: KCCContact[];
    /** since the frame began: update, refreshContacts and setShape begin one, and a move adds to it */
    contactEvents: KCCContactEvent[];
    /** the contacts and events made so far, reused from call to call */
    contactPool: KCCContact[];
    eventPool: KCCContactEvent[];
};

function readVec3(out: Vec3, f32: Float32Array, at: number): void {
    out[0] = f32[at]!;
    out[1] = f32[at + 1]!;
    out[2] = f32[at + 2]!;
}

/** a row of the wasm's contacts into a plain contact */
function readContact(out: KCCContact, row: number): void {
    const { f32, u32, i32, u8 } = Low.getWasm().mem;
    readVec3(out.position, f32, (row + Low.KCC_CONTACT_POSITION) >> 2);
    readVec3(out.linearVelocity, f32, (row + Low.KCC_CONTACT_LINEAR_VELOCITY) >> 2);
    readVec3(out.contactNormal, f32, (row + Low.KCC_CONTACT_CONTACT_NORMAL) >> 2);
    readVec3(out.surfaceNormal, f32, (row + Low.KCC_CONTACT_SURFACE_NORMAL) >> 2);
    out.distance = f32[(row + Low.KCC_CONTACT_DISTANCE) >> 2]!;
    out.fraction = f32[(row + Low.KCC_CONTACT_FRACTION) >> 2]!;
    out.bodyId = u32[(row + Low.KCC_CONTACT_BODY_ID) >> 2]!;
    out.subShapeId = u32[(row + Low.KCC_CONTACT_SUB_SHAPE_ID) >> 2]!;
    out.materialId = i32[(row + Low.KCC_CONTACT_MATERIAL_ID) >> 2]!;
    out.motionType = u32[(row + Low.KCC_CONTACT_MOTION_TYPE) >> 2]! as MotionType;
    out.isSensor = u8[row + Low.KCC_CONTACT_IS_SENSOR] !== 0;
    out.hadCollision = u8[row + Low.KCC_CONTACT_HAD_COLLISION] !== 0;
    out.wasDiscarded = u8[row + Low.KCC_CONTACT_WAS_DISCARDED] !== 0;
    out.canPushCharacter = u8[row + Low.KCC_CONTACT_CAN_PUSH_CHARACTER] !== 0;
    out.canReceiveImpulses = u8[row + Low.KCC_CONTACT_CAN_RECEIVE_IMPULSES] !== 0;
}

/** the state as the character's last call left it */
function pull(character: KCC): void {
    Low.Kcc.readState(state, character.handle);
    copyVec3(character.position, state.position);
    for (let i = 0; i < 4; i++) character.quaternion[i] = state.quaternion[i]!;
    copyVec3(character.linearVelocity, state.linearVelocity);
    copyVec3(character.up, state.up);
    const ground = character.ground;
    ground.state = state.groundState as GroundState;
    copyVec3(ground.normal, state.groundNormal);
    copyVec3(ground.velocity, state.groundVelocity);
    ground.bodyId = state.groundBodyId;
    ground.subShapeId = state.groundSubShapeId;
    copyVec3(ground.position, state.groundPosition);
    if (state.shape !== null) character.shape = state.shape;
    character.innerRigidBodyId = state.innerRigidBodyId;

    const contacts = character.contacts;
    contacts.length = 0;
    const base = state.contactCount > 0 ? Low.Kcc.contacts(character.handle) : 0;
    for (let i = 0; i < state.contactCount; i++) {
        let contact = character.contactPool[i];
        if (contact === undefined) {
            contact = createContact();
            character.contactPool.push(contact);
        }
        readContact(contact, base + i * STRIDE);
        contacts.push(contact);
    }

    const events = character.contactEvents;
    events.length = 0;
    for (let i = 0; i < state.contactEventCount; i++) {
        Low.Kcc.readContactEvent(event, character.handle, i);
        let out = character.eventPool[i];
        if (out === undefined) {
            out = {
                kind: KCCContactEventKind.ADDED,
                bodyId: INVALID_BODY_ID,
                subShapeId: 0,
                position: [0, 0, 0],
                normal: [0, 0, 0],
            };
            character.eventPool.push(out);
        }
        out.kind = event.kind as KCCContactEventKind;
        out.bodyId = event.bodyId;
        out.subShapeId = event.subShapeId;
        copyVec3(out.position, event.position);
        copyVec3(out.normal, event.normal);
        events.push(out);
    }
}

/** the velocity into the character, for a call that reads it */
function push(character: KCC): void {
    Low.Kcc.setLinearVelocity(character.handle, character.linearVelocity);
}

/** the contacts' wasDiscarded and linearVelocity back into the wasm's rows, for the phases that read them */
function pushContacts(character: KCC): void {
    const contacts = character.contacts;
    if (contacts.length === 0) return;
    const { f32, u8 } = Low.getWasm().mem;
    const base = Low.Kcc.contacts(character.handle);
    for (let i = 0; i < contacts.length; i++) {
        const contact = contacts[i]!;
        const row = base + i * STRIDE;
        u8[row + Low.KCC_CONTACT_WAS_DISCARDED] = contact.wasDiscarded ? 1 : 0;
        const at = (row + Low.KCC_CONTACT_LINEAR_VELOCITY) >> 2;
        f32[at] = contact.linearVelocity[0];
        f32[at + 1] = contact.linearVelocity[1];
        f32[at + 2] = contact.linearVelocity[2];
        // the fourth lane as a load of the three leaves it
        f32[at + 3] = contact.linearVelocity[2];
    }
}

const state = Low.KCCState.create();
const event = Low.KCCContactEvent.create();
const sweep = Low.KCCContact.create();
const solved = Low.KCCSolve.create();
const vec3: Vec3 = [0, 0, 0];
const quat: Quat = [0, 0, 0, 1];
const TWO_32 = 0x100000000;

const settingsDefaults = defaultsOf(Low.Kcc.defaultSettings, Low.KCCSettings.create);

/** the filter's layers, group and mask: a character's queries run in the wasm, past any body filter's reach */
function layersOf(filter: Filter): Low.Filter {
    if (filter.bodyFilter !== null)
        throw new RangeError("tumble: a character's queries cannot call a bodyFilter; set ignore in its policy");
    return filter.layers;
}

/** a call that moves the character: the velocity in, the call, the state out, any error thrown */
function moving<T>(character: KCC, call: () => T): T {
    push(character);
    Low.errorClear();
    const result = call();
    Low.getWasm().check();
    pull(character);
    return result;
}

/** a character at the pose; an inner body joins the world when added */
export function create(settings: KCCSettings, position: Vec3, quaternion: Quat): KCC {
    const native = over(settingsDefaults(), {
        shape: settings.shape,
        shapeOffset: settings.shapeOffset,
        mass: settings.mass,
        maxStrength: settings.maxStrength,
        up: settings.up,
        maxSlopeAngle: settings.maxSlopeAngle,
        predictiveContactDistance: settings.predictiveContactDistance,
        characterPadding: settings.characterPadding,
        maxCollisionIterations: settings.maxCollisionIterations,
        maxConstraintIterations: settings.maxConstraintIterations,
        collisionTolerance: settings.collisionTolerance,
        penetrationRecoverySpeed: settings.penetrationRecoverySpeed,
        minTimeRemaining: settings.minTimeRemaining,
        maxNumHits: settings.maxNumHits,
        hitReductionCosMaxAngle: settings.hitReductionCosMaxAngle,
        backFaceMode: settings.backFaceMode,
        enhancedInternalEdgeRemoval: settings.enhancedInternalEdgeRemoval,
        observesContacts: settings.observesContacts,
    });
    const plane = settings.supportingVolumePlane;
    native.hasSupportingVolumePlane = plane !== undefined;
    if (plane !== undefined) native.supportingVolumePlane = { normal: [plane[0], plane[1], plane[2]], constant: plane[3] };
    const userData = settings.userData ?? 0;
    native.userDataLow = userData >>> 0;
    native.userDataHigh = Math.floor(userData / TWO_32);
    native.innerRigidBodyShape = settings.innerRigidBody?.shape ?? null;
    native.innerRigidBodyObjectLayer = settings.innerRigidBody?.objectLayer ?? 0;
    Low.errorClear();
    const handle = Low.Kcc.create(native, position, quaternion);
    const character: KCC = {
        handle,
        position: [0, 0, 0],
        quaternion: [0, 0, 0, 1],
        linearVelocity: [0, 0, 0],
        up: [0, 1, 0],
        ground: {
            state: GroundState.IN_AIR,
            normal: [0, 1, 0],
            velocity: [0, 0, 0],
            bodyId: INVALID_BODY_ID,
            subShapeId: 0,
            position: [0, 0, 0],
        },
        innerRigidBodyId: INVALID_BODY_ID,
        shape: settings.shape,
        contacts: [],
        contactEvents: [],
        contactPool: [],
        eventPool: [],
    };
    pull(character);
    return character;
}

/**
 * lets go of the character made here; the last reference to it frees it, taking its inner body out of the world it
 * was added to, so a character still in a world is released before that world is destroyed
 */
export function release(character: KCC): void {
    Low.Kcc.release(character.handle);
}

/** the inner body into the world: seen by bodies, queries, contacts and sensors; nothing without one */
export function add(world: World, character: KCC): void {
    moving(character, () => Low.Kcc.add(world, character.handle));
}

export function remove(world: World, character: KCC): void {
    moving(character, () => Low.Kcc.remove(world, character.handle));
}

/** a frame: move with contact tracking, velocity toward steep slopes cancelled, floor sticking and stair walking */
export function update(
    world: World,
    character: KCC,
    deltaTime: number,
    gravity: Vec3,
    settings: KCCUpdateSettings,
    policy: KCCPolicy | undefined,
    filter: Filter,
): void {
    const layers = layersOf(filter);
    moving(character, () =>
        Low.Kcc.update(
            world,
            character.handle,
            deltaTime,
            gravity,
            settings,
            policyOf(policy),
            layers,
            filter.collisionGroups >>> 0,
            filter.collisionMask >>> 0,
        ),
    );
}

/** by the velocity with collision resolution, the ground and the inner body following; no tracking, stairs or sticking */
export function move(
    world: World,
    character: KCC,
    deltaTime: number,
    gravity: Vec3,
    policy: KCCPolicy | undefined,
    filter: Filter,
): void {
    const layers = layersOf(filter);
    moving(character, () =>
        Low.Kcc.move(
            world,
            character.handle,
            deltaTime,
            gravity,
            policyOf(policy),
            layers,
            filter.collisionGroups >>> 0,
            filter.collisionMask >>> 0,
        ),
    );
}

/** swept down by stepDown onto a floor: whether one was found */
export function stickToFloor(
    world: World,
    character: KCC,
    stepDown: Vec3,
    filter: Filter,
    policy: KCCPolicy | undefined,
): boolean {
    const layers = layersOf(filter);
    return moving(character, () =>
        Low.Kcc.stickToFloor(
            world,
            character.handle,
            stepDown,
            layers,
            filter.collisionGroups >>> 0,
            filter.collisionMask >>> 0,
            policyOf(policy),
        ),
    );
}

/** up, forward and down onto a walkable floor: whether the step succeeded */
export function walkStairs(
    world: World,
    character: KCC,
    deltaTime: number,
    stepUp: Vec3,
    stepForward: Vec3,
    stepForwardTest: Vec3,
    stepDownExtra: Vec3,
    filter: Filter,
    policy: KCCPolicy | undefined,
): boolean {
    const layers = layersOf(filter);
    return moving(character, () =>
        Low.Kcc.walkStairs(
            world,
            character.handle,
            deltaTime,
            stepUp,
            stepForward,
            stepForwardTest,
            stepDownExtra,
            layers,
            filter.collisionGroups >>> 0,
            filter.collisionMask >>> 0,
            policyOf(policy),
        ),
    );
}

/** the contacts and the ground where the character is now, a frame of tracking: for after a teleport */
export function refreshContacts(world: World, character: KCC, filter: Filter, policy: KCCPolicy | undefined): void {
    const layers = layersOf(filter);
    moving(character, () =>
        Low.Kcc.refreshContacts(
            world,
            character.handle,
            layers,
            filter.collisionGroups >>> 0,
            filter.collisionMask >>> 0,
            policyOf(policy),
        ),
    );
}

/** refused, false, when the new shape would sink deeper than maxPenetrationDepth; Infinity skips the test */
export function setShape(
    world: World,
    character: KCC,
    shape: Shape,
    filter: Filter,
    policy: KCCPolicy | undefined,
    maxPenetrationDepth: number,
): boolean {
    const layers = layersOf(filter);
    return moving(character, () =>
        Low.Kcc.setShape(
            world,
            character.handle,
            shape,
            layers,
            filter.collisionGroups >>> 0,
            filter.collisionMask >>> 0,
            policyOf(policy),
            Math.min(maxPenetrationDepth, 3.4028234663852886e38),
        ),
    );
}

/** a teleport; the inner body follows */
export function setPosition(world: World, character: KCC, position: Vec3): void {
    moving(character, () => Low.Kcc.setPosition(world, character.handle, position));
}

export function setQuaternion(world: World, character: KCC, quaternion: Quat): void {
    moving(character, () => Low.Kcc.setQuaternion(world, character.handle, quaternion));
}

/** from the ground body, its turning and the policy's surface velocity included */
export function updateGroundVelocity(world: World, character: KCC, policy: KCCPolicy | undefined): void {
    moving(character, () => Low.Kcc.updateGroundVelocity(world, character.handle, policyOf(policy)));
}

export function getTuning(out: KCCTuning, character: KCC): KCCTuning {
    Low.Kcc.getTuning(tuning, character.handle);
    copyVec3(out.shapeOffset, tuning.shapeOffset);
    out.mass = tuning.mass;
    out.maxStrength = tuning.maxStrength;
    copyVec3(out.up, tuning.up);
    out.maxSlopeAngle = tuning.maxSlopeAngle;
    out.predictiveContactDistance = tuning.predictiveContactDistance;
    out.characterPadding = tuning.characterPadding;
    out.maxCollisionIterations = tuning.maxCollisionIterations;
    out.maxConstraintIterations = tuning.maxConstraintIterations;
    out.collisionTolerance = tuning.collisionTolerance;
    out.penetrationRecoverySpeed = tuning.penetrationRecoverySpeed;
    out.minTimeRemaining = tuning.minTimeRemaining;
    out.maxNumHits = tuning.maxNumHits;
    out.hitReductionCosMaxAngle = tuning.hitReductionCosMaxAngle;
    out.backFaceMode = tuning.backFaceMode as BackFaceMode;
    out.enhancedInternalEdgeRemoval = tuning.enhancedInternalEdgeRemoval;
    for (let i = 0; i < 4; i++) out.supportingVolumePlane[i] = tuning.supportingVolumePlane[i]!;
    return out;
}

/** the given fields over the character's current ones, as create sets them: up normalized */
export function setTuning(character: KCC, given: Given<KCCTuning>): void {
    Low.Kcc.getTuning(tuning, character.handle);
    Low.Kcc.setTuning(character.handle, over(tuning, given));
    pull(character);
}

/** on ground or on steep ground */
export function isSupported(character: KCC): boolean {
    return character.ground.state === GroundState.ON_GROUND || character.ground.state === GroundState.ON_STEEP_GROUND;
}

export function isSlopeTooSteep(character: KCC, normal: Vec3): boolean {
    return Low.Kcc.isSlopeTooSteep(character.handle, normal);
}

/** in its last update */
export function hasCollidedWith(character: KCC, body: BodyId): boolean {
    return Low.Kcc.hasCollidedWith(character.handle, body);
}

/** supported and pushing into steep slopes it might walk up */
export function canWalkStairs(character: KCC, velocity: Vec3): boolean {
    return Low.Kcc.canWalkStairs(character.handle, velocity);
}

/** its shape's pose: the position plus the turned shape offset */
export function getCenterOfMassTransform(outPosition: Vec3, outQuaternion: Quat, character: KCC): void {
    Low.Kcc.getCenterOfMassTransform(vec3, quat, character.handle);
    copied(outPosition, vec3, true);
    copied(outQuaternion, quat, true);
}

// the toolkit: a move's phases, for a loop of your own. each collision iteration collide, edit the contacts, solve,
// cast, then advance the position by the cast's share of the displacement; after the iterations setPosition,
// updateGround and applyWeight. update adds beginContactTracking and cancelVelocityTowardsSteepSlopes before, and
// stickToFloor, walkStairs and endContactTracking after

/** the contacts at a position, into character.contacts: sorted, padded, the shallower of opposing pairs on a body discarded */
export function collide(
    world: World,
    character: KCC,
    position: Vec3,
    movementDirection: Vec3,
    filter: Filter,
    policy: KCCPolicy | undefined,
): void {
    const layers = layersOf(filter);
    moving(character, () =>
        Low.Kcc.collide(
            world,
            character.handle,
            position,
            movementDirection,
            layers,
            filter.collisionGroups >>> 0,
            filter.collisionMask >>> 0,
            policyOf(policy),
        ),
    );
}

/** the slide over the contacts for up to timeRemaining, each contact moved into met and pushed by the policy */
export function solve(
    out: KCCSolveResult,
    world: World,
    character: KCC,
    velocity: Vec3,
    deltaTime: number,
    timeRemaining: number,
    policy: KCCPolicy | undefined,
): KCCSolveResult {
    pushContacts(character);
    moving(character, () => Low.Kcc.solve(solved, world, character.handle, velocity, deltaTime, timeRemaining, policyOf(policy)));
    copyVec3(out.velocity, solved.velocity);
    copyVec3(out.displacement, solved.displacement);
    out.timeSimulated = solved.timeSimulated;
    return out;
}

/** the first contact sweeping by the displacement from a position, past the discarded rows: whether there is one */
export function cast(
    out: KCCContact,
    world: World,
    character: KCC,
    position: Vec3,
    displacement: Vec3,
    filter: Filter,
    policy: KCCPolicy | undefined,
): boolean {
    const layers = layersOf(filter);
    pushContacts(character);
    Low.errorClear();
    const hit = Low.Kcc.cast(
        sweep,
        world,
        character.handle,
        position,
        displacement,
        layers,
        filter.collisionGroups >>> 0,
        filter.collisionMask >>> 0,
        policyOf(policy),
    );
    Low.getWasm().check();
    if (!hit) return false;
    copyVec3(out.position, sweep.position);
    copyVec3(out.linearVelocity, sweep.linearVelocity);
    copyVec3(out.contactNormal, sweep.contactNormal);
    copyVec3(out.surfaceNormal, sweep.surfaceNormal);
    out.distance = sweep.distance;
    out.fraction = sweep.fraction;
    out.bodyId = sweep.bodyId;
    out.subShapeId = sweep.subShapeId;
    out.materialId = sweep.materialId;
    out.motionType = sweep.motionType as MotionType;
    out.isSensor = sweep.isSensor;
    out.hadCollision = false;
    out.wasDiscarded = false;
    out.canPushCharacter = true;
    out.canReceiveImpulses = true;
    return true;
}

/** the ground from the contacts, those near enough flagged touching unless moving away (with the velocity check) */
export function updateGround(
    world: World,
    character: KCC,
    deltaTime: number,
    skipContactVelocityCheck: boolean,
    policy: KCCPolicy | undefined,
): void {
    pushContacts(character);
    moving(character, () => Low.Kcc.updateGround(world, character.handle, deltaTime, skipContactVelocityCheck, policyOf(policy)));
}

/** the character's weight on a dynamic ground body */
export function applyWeight(world: World, character: KCC, gravity: Vec3, deltaTime: number): void {
    moving(character, () => Low.Kcc.applyWeight(world, character.handle, gravity, deltaTime));
}

/** a frame of contact tracking begun: contactEvents cleared, what touches now remembered */
export function beginContactTracking(character: KCC): void {
    moving(character, () => Low.Kcc.beginContactTracking(character.handle));
}

/** the frame ended: what was remembered and no longer touches reported removed */
export function endContactTracking(character: KCC): void {
    moving(character, () => Low.Kcc.endContactTracking(character.handle));
}

/** the velocity without its horizontal part into steep slopes the character touches, unless on ground or in the air */
export function cancelVelocityTowardsSteepSlopes(out: Vec3, character: KCC, velocity: Vec3): Vec3 {
    Low.Kcc.cancelVelocityTowardsSteepSlopes(vec3, character.handle, velocity);
    copyVec3(out, vec3);
    return out;
}
