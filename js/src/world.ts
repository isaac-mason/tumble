import { withCopies } from './arrays.js';
import { type BodyId, INVALID_BODY_ID } from './body.js';
import { type Listener, stepWith } from './events.js';
import * as Low from './generated/tumble.js';
import type { Quat, Vec3 } from './math.js';

export type World = Low.World;

export type ContactsSettings = Low.ContactsSettings;
export type NarrowphaseSettings = Low.NarrowphaseSettings;
export type SolverSettings = Low.SolverSettings;
export type SleepingSettings = Low.SleepingSettings;
export type CcdSettings = Low.CcdSettings;

/** what a world is made with: plain data, yours, read once by createWorld */
export type WorldSettings = {
    /** m/s2 */
    gravity: Vec3;
    gravityEnabled: boolean;
    contacts: ContactsSettings;
    narrowphase: NarrowphaseSettings;
    solver: SolverSettings;
    sleeping: SleepingSettings;
    ccd: CcdSettings;
    /** each object layer's broadphase layer, by object layer */
    objectLayers: number[];
    broadphaseLayers: number;
    /** the pairs of object layers that collide */
    collisions: [number, number][];
};

/** the defaults, and no layers */
export function createWorldSettings(): WorldSettings {
    const native = Low.WorldSettings.create();
    const settings: WorldSettings = {
        gravity: Low.WorldSettings.readGravity(Low.Vec3.create(), native),
        gravityEnabled: Low.WorldSettings.gravityEnabled(native),
        contacts: Low.WorldSettings.readContacts(Low.ContactsSettings.create(), native),
        narrowphase: Low.WorldSettings.readNarrowphase(Low.NarrowphaseSettings.create(), native),
        solver: Low.WorldSettings.readSolver(Low.SolverSettings.create(), native),
        sleeping: Low.WorldSettings.readSleeping(Low.SleepingSettings.create(), native),
        ccd: Low.WorldSettings.readCcd(Low.CcdSettings.create(), native),
        objectLayers: [],
        broadphaseLayers: 0,
        collisions: [],
    };
    Low.WorldSettings.destroy(native);
    return settings;
}

/** a new broadphase layer: a tree of its own, for the object layers in it */
export function addBroadphaseLayer(settings: WorldSettings): number {
    return settings.broadphaseLayers++;
}

/** a new object layer, in the broadphase layer */
export function addObjectLayer(settings: WorldSettings, broadphaseLayer: number): number {
    return settings.objectLayers.push(broadphaseLayer) - 1;
}

export function enableCollision(settings: WorldSettings, objectLayerA: number, objectLayerB: number): void {
    settings.collisions.push([objectLayerA, objectLayerB]);
}

/** a surface a shape's or a triangle's material id names */
export type Material = { friction: number; restitution: number };

/** a world over the settings, which it copies */
export function createWorld(settings: WorldSettings): World {
    Low.errorClear();
    const native = Low.WorldSettings.create();
    for (let i = 0; i < settings.broadphaseLayers; i++) Low.WorldSettings.addBroadphaseLayer(native);
    for (const broadphaseLayer of settings.objectLayers) Low.WorldSettings.addObjectLayer(native, broadphaseLayer);
    for (const [a, b] of settings.collisions) Low.WorldSettings.enableCollision(native, a, b);
    Low.WorldSettings.writeGravity(native, settings.gravity);
    Low.WorldSettings.setGravityEnabled(native, settings.gravityEnabled);
    Low.WorldSettings.writeContacts(native, settings.contacts);
    Low.WorldSettings.writeNarrowphase(native, settings.narrowphase);
    Low.WorldSettings.writeSolver(native, settings.solver);
    Low.WorldSettings.writeSleeping(native, settings.sleeping);
    Low.WorldSettings.writeCcd(native, settings.ccd);
    try {
        return Low.World.create(native);
    } finally {
        Low.WorldSettings.destroy(native);
    }
}

/** frees the world and every body and constraint in it */
export function destroyWorld(world: World): void {
    Low.World.destroy(world);
}

/** one step of deltaTime seconds; the listener hears its events and answers them as it runs */
export function updateWorld(world: World, listener: Listener | undefined, deltaTime: number): void {
    stepWith(world, listener, deltaTime);
}

/** the surfaces material ids name, replacing the table */
export function setMaterials(world: World, materials: Material[]): void {
    const values: number[] = [];
    for (const material of materials) values.push(material.friction, material.restitution);
    withCopies((copy) => Low.World.setMaterials(world, copy.f32(values), materials.length));
}

export function getGravity(out: Vec3, world: World): Vec3 {
    Low.World.getGravity(out, world);
    return out;
}

/** from the next step on */
export function setGravity(world: World, value: Vec3): void {
    Low.World.setGravity(world, value);
}

/** how many bodies are on the active list: the dynamic and kinematic bodies awake, which the next step simulates */
export function getNumActiveBodies(world: World): number {
    return Low.ActiveBodies.count(world);
}

/**
 * reads the active list for getActiveBodyAt, in body index order: how many. read after a step, it is every body the
 * step moved but those it put to sleep, which onBodyDeactivated names. valid until the next read
 */
export function getActiveBodies(world: World): number {
    Low.errorClear();
    const count = Low.ActiveBodies.read(world);
    if (count === 0 && Low.ActiveBodies.count(world) !== 0) Low.getWasm().check();
    rowsWorld = world;
    rowsAt = Low.ActiveBodies.rows(world) >> 2;
    return count;
}

/** the index-th body of the last getActiveBodies read of the world */
export function getActiveBodyAt(out: ActiveBody, world: World, index: number): ActiveBody {
    if (world !== rowsWorld) {
        rowsWorld = world;
        rowsAt = Low.ActiveBodies.rows(world) >> 2;
    }
    const { f32, u32 } = Low.getWasm().mem;
    const row = rowsAt + index * ROW_STRIDE;
    out.body = u32[row + ROW_BODY]!;
    let at = row + ROW_POSITION;
    out.position[0] = f32[at]!;
    out.position[1] = f32[at + 1]!;
    out.position[2] = f32[at + 2]!;
    at = row + ROW_QUATERNION;
    out.quaternion[0] = f32[at]!;
    out.quaternion[1] = f32[at + 1]!;
    out.quaternion[2] = f32[at + 2]!;
    out.quaternion[3] = f32[at + 3]!;
    at = row + ROW_CENTER_OF_MASS_POSITION;
    out.centerOfMassPosition[0] = f32[at]!;
    out.centerOfMassPosition[1] = f32[at + 1]!;
    out.centerOfMassPosition[2] = f32[at + 2]!;
    at = row + ROW_LINEAR_VELOCITY;
    out.linearVelocity[0] = f32[at]!;
    out.linearVelocity[1] = f32[at + 1]!;
    out.linearVelocity[2] = f32[at + 2]!;
    at = row + ROW_ANGULAR_VELOCITY;
    out.angularVelocity[0] = f32[at]!;
    out.angularVelocity[1] = f32[at + 1]!;
    out.angularVelocity[2] = f32[at + 2]!;
    return out;
}

/** an ActiveBody for getActiveBodyAt to fill, made once and reused */
export function createActiveBody(): ActiveBody {
    return {
        body: INVALID_BODY_ID,
        position: [0, 0, 0],
        quaternion: [0, 0, 0, 1],
        centerOfMassPosition: [0, 0, 0],
        linearVelocity: [0, 0, 0],
        angularVelocity: [0, 0, 0],
    };
}

/** a body of the active list, as getActiveBodyAt fills it */
export type ActiveBody = {
    body: BodyId;
    position: Vec3;
    quaternion: Quat;
    centerOfMassPosition: Vec3;
    linearVelocity: Vec3;
    angularVelocity: Vec3;
};

const ROW_STRIDE = Low.ACTIVE_BODY_ROW_STRIDE >> 2;
const ROW_BODY = Low.ACTIVE_BODY_ROW_BODY >> 2;
const ROW_POSITION = Low.ACTIVE_BODY_ROW_POSITION >> 2;
const ROW_QUATERNION = Low.ACTIVE_BODY_ROW_QUATERNION >> 2;
const ROW_CENTER_OF_MASS_POSITION = Low.ACTIVE_BODY_ROW_CENTER_OF_MASS_POSITION >> 2;
const ROW_LINEAR_VELOCITY = Low.ACTIVE_BODY_ROW_LINEAR_VELOCITY >> 2;
const ROW_ANGULAR_VELOCITY = Low.ACTIVE_BODY_ROW_ANGULAR_VELOCITY >> 2;

/** where the last getActiveBodies read wrote, so getActiveBodyAt needs no call into the wasm */
let rowsWorld: World | undefined;
let rowsAt = 0;
