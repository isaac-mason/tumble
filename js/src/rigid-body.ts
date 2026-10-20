import * as Low from './generated/tumble';
import type { Quat, Vec3 } from './math';
import type { Shape } from './shapes';
import type { World } from './world';

/** a body in a world: an index and a sequence in one number, so an id of a removed body names nothing */
export type BodyId = number;

export const MotionType = {
    /** never moves */
    STATIC: 0,
    /** moves by its velocity, pushing dynamic bodies aside */
    KINEMATIC: 1,
    /** moved by gravity, forces and contacts */
    DYNAMIC: 2,
} as const;
export type MotionType = (typeof MotionType)[keyof typeof MotionType];

/** what a body is made with; everything but the shape, the layer and the motion type has a default */
export type RigidBodySettings = {
    shape: Shape;
    objectLayer: number;
    motionType: MotionType;
    /** your own value for the body, an integer up to 2^53, reported back with it in events */
    userData?: number;
    position?: Vec3;
    quaternion?: Quat;
    friction?: number;
    restitution?: number;
    linearDamping?: number;
    angularDamping?: number;
    gravityFactor?: number;
    /** the mass with the shape's inertia scaled to it; without it, from the shape's volume and density */
    mass?: number;
    allowSleeping?: boolean;
    sensor?: boolean;
    observesContacts?: boolean;
};

let defaults: Low.BodySettings | null = null;
const scratch: Low.BodySettings = Low.BodySettings.create();
const vec3: Vec3 = [0, 0, 0];
const quat: Quat = [0, 0, 0, 1];

/** out filled only when the read succeeded */
function copied<T extends number[]>(out: T, read: T, ok: boolean): boolean {
    if (ok) for (let i = 0; i < out.length; i++) out[i] = read[i]!;
    return ok;
}

export const RigidBody = {
    /** a new body in the world, found by the next step; the shape is borrowed and outlives it */
    create(world: World, settings: RigidBodySettings): BodyId {
        if (defaults === null) defaults = Low.RigidBody.defaultSettings(Low.BodySettings.create());
        Object.assign(scratch, defaults);
        scratch.position = [...(settings.position ?? defaults.position)];
        scratch.quaternion = [...(settings.quaternion ?? defaults.quaternion)];
        scratch.shape = settings.shape;
        scratch.objectLayer = settings.objectLayer;
        scratch.motionType = settings.motionType;
        const userData = settings.userData ?? 0;
        scratch.userDataLow = userData >>> 0;
        scratch.userDataHigh = Math.floor(userData / 0x100000000);
        if (settings.friction !== undefined) scratch.friction = settings.friction;
        if (settings.restitution !== undefined) scratch.restitution = settings.restitution;
        if (settings.linearDamping !== undefined) scratch.linearDamping = settings.linearDamping;
        if (settings.angularDamping !== undefined) scratch.angularDamping = settings.angularDamping;
        if (settings.gravityFactor !== undefined) scratch.gravityFactor = settings.gravityFactor;
        if (settings.mass !== undefined) scratch.mass = settings.mass;
        if (settings.allowSleeping !== undefined) scratch.allowSleeping = settings.allowSleeping;
        if (settings.sensor !== undefined) scratch.sensor = settings.sensor;
        if (settings.observesContacts !== undefined) scratch.observesContacts = settings.observesContacts;
        Low.errorClear();
        const id = Low.RigidBody.create(world, scratch);
        if (id === INVALID_BODY_ID) Low.getWasm().check();
        return id;
    },
    /** out of the world; false when the id names no body */
    remove(world: World, body: BodyId): boolean {
        return Low.RigidBody.remove(world, body);
    },
    /** the shape's origin, world space; false (out unchanged) when the id names no body */
    getPosition(out: Vec3, world: World, body: BodyId): boolean {
        return copied(out, vec3, Low.RigidBody.getPosition(vec3, world, body));
    },
    getQuaternion(out: Quat, world: World, body: BodyId): boolean {
        return copied(out, quat, Low.RigidBody.getQuaternion(quat, world, body));
    },
    getLinearVelocity(out: Vec3, world: World, body: BodyId): boolean {
        return copied(out, vec3, Low.RigidBody.getLinearVelocity(vec3, world, body));
    },
    /** wakes the body */
    setLinearVelocity(world: World, body: BodyId, velocity: Vec3): boolean {
        return Low.RigidBody.setLinearVelocity(world, body, velocity);
    },
};

/** the id naming no body */
export const INVALID_BODY_ID = 0xffffffff;
