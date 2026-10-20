import * as Low from './generated/tumble';
import type { Box3, Quat, Vec3 } from './math';
import type { MassProperties, Shape } from './shapes';
import type { World } from './world';

/** a body in a world: an index and a sequence in one number, so the id of a removed body names nothing */
export type BodyId = number;

/** the id naming no body */
export const INVALID_BODY_ID = 0xffffffff;

export const MotionType = Low.MotionType;
export type MotionType = Low.MotionType;
export const MotionQuality = Low.MotionQuality;
export type MotionQuality = Low.MotionQuality;
/** bits of allowedDegreesOfFreedom */
export const Dof = Low.Dof;
export const MaterialCombineMode = Low.MaterialCombineMode;
export type MaterialCombineMode = Low.MaterialCombineMode;

/** what a body is made with; everything but the shape, the layer and the motion type has a default */
export type RigidBodySettings = {
    /** borrowed: it outlives the body */
    shape: Shape;
    objectLayer: number;
    motionType: MotionType;
    /** your own value for the body, an integer up to 2^53, reported back with it in events */
    userData?: number;
    position?: Vec3;
    quaternion?: Quat;
    motionQuality?: MotionQuality;
    /** Dof bits */
    allowedDegreesOfFreedom?: number;
    gravityFactor?: number;
    linearDamping?: number;
    angularDamping?: number;
    maxLinearVelocity?: number;
    maxAngularVelocity?: number;
    /** for a dynamic body in contact; 0 takes the world's */
    numVelocityStepsOverride?: number;
    numPositionStepsOverride?: number;
    friction?: number;
    restitution?: number;
    frictionCombineMode?: MaterialCombineMode;
    restitutionCombineMode?: MaterialCombineMode;
    /** the groups it is in */
    collisionGroups?: number;
    /** the groups it collides with */
    collisionMask?: number;
    useManifoldReduction?: boolean;
    allowSleeping?: boolean;
    /** reports its contacts, pushes nothing */
    sensor?: boolean;
    enhancedInternalEdgeRemoval?: boolean;
    /** a kinematic body that also collides with static and kinematic ones */
    collideKinematicVsNonDynamic?: boolean;
    /** its contact pairs are recorded (see ContactPairs) */
    observesContacts?: boolean;
    /** the mass with the shape's inertia scaled to it; without it, from the shape's volume and density */
    mass?: number;
    /** mass and inertia outright, for a shape they cannot be derived from (a triangle mesh) */
    massPropertiesOverride?: MassProperties;
};

const TWO_32 = 0x100000000;

let defaults: Low.BodySettings | null = null;
const native: Low.BodySettings = Low.BodySettings.create();
const vec3: Vec3 = [0, 0, 0];
const quat: Quat = [0, 0, 0, 1];
const box3: Box3 = { min: [0, 0, 0], max: [0, 0, 0] };

/** out filled only when the read succeeded */
function copied<T extends number[]>(out: T, read: T, ok: boolean): boolean {
    if (ok) for (let i = 0; i < out.length; i++) out[i] = read[i]!;
    return ok;
}

function nativeSettings(settings: RigidBodySettings): Low.BodySettings {
    if (defaults === null) defaults = Low.RigidBody.defaultSettings(Low.BodySettings.create());
    Object.assign(native, defaults);
    native.position = [...(settings.position ?? defaults.position)];
    native.quaternion = [...(settings.quaternion ?? defaults.quaternion)];
    const target = native as unknown as Record<string, unknown>;
    for (const [key, value] of Object.entries(settings)) {
        if (
            value === undefined ||
            key === 'userData' ||
            key === 'massPropertiesOverride' ||
            key === 'position' ||
            key === 'quaternion'
        ) {
            continue;
        }
        target[key] = value;
    }
    const userData = settings.userData ?? 0;
    native.userDataLow = userData >>> 0;
    native.userDataHigh = Math.floor(userData / TWO_32);
    const override = settings.massPropertiesOverride;
    native.hasMassProperties = override !== undefined;
    if (override !== undefined) native.massProperties = override;
    return native;
}

export const RigidBody = {
    /** a new body in the world, found by the next step */
    create(world: World, settings: RigidBodySettings): BodyId {
        const native = nativeSettings(settings);
        Low.errorClear();
        const id = Low.RigidBody.create(world, native);
        if (id === INVALID_BODY_ID) Low.getWasm().check();
        return id;
    },
    /** out of the world, with its constraints; its contacts' removal events come with the next step */
    remove(world: World, body: BodyId): boolean {
        return Low.RigidBody.remove(world, body);
    },
    /** whether the id names a body in the world */
    exists(world: World, body: BodyId): boolean {
        return Low.RigidBody.exists(world, body);
    },

    // readers: out-first, false (out as it was) when the id names no body

    /** the shape's origin, world space */
    getPosition(out: Vec3, world: World, body: BodyId): boolean {
        return copied(out, vec3, Low.RigidBody.getPosition(vec3, world, body));
    },
    getQuaternion(out: Quat, world: World, body: BodyId): boolean {
        return copied(out, quat, Low.RigidBody.getQuaternion(quat, world, body));
    },
    getTransform(outPosition: Vec3, outQuaternion: Quat, world: World, body: BodyId): boolean {
        if (!Low.RigidBody.getTransform(vec3, quat, world, body)) return false;
        copied(outPosition, vec3, true);
        copied(outQuaternion, quat, true);
        return true;
    },
    getCenterOfMassPosition(out: Vec3, world: World, body: BodyId): boolean {
        return copied(out, vec3, Low.RigidBody.getCenterOfMassPosition(vec3, world, body));
    },
    getLinearVelocity(out: Vec3, world: World, body: BodyId): boolean {
        return copied(out, vec3, Low.RigidBody.getLinearVelocity(vec3, world, body));
    },
    getAngularVelocity(out: Vec3, world: World, body: BodyId): boolean {
        return copied(out, vec3, Low.RigidBody.getAngularVelocity(vec3, world, body));
    },
    /** the velocity of a world point carried with the body */
    getVelocityAtPoint(out: Vec3, world: World, body: BodyId, point: Vec3): boolean {
        return copied(out, vec3, Low.RigidBody.getVelocityAtPoint(vec3, world, body, point));
    },
    /** world space bounds */
    getBounds(out: Box3, world: World, body: BodyId): boolean {
        if (!Low.RigidBody.getAabb(box3, world, body)) return false;
        copied(out.min, box3.min, true);
        copied(out.max, box3.max, true);
        return true;
    },
    /** kg; 0 for a static or kinematic body, or an id naming no body */
    getMass(world: World, body: BodyId): number {
        return Low.RigidBody.getMass(world, body);
    },
    getMotionType(world: World, body: BodyId): MotionType {
        return Low.RigidBody.getMotionType(world, body) as MotionType;
    },
    getObjectLayer(world: World, body: BodyId): number {
        return Low.RigidBody.getObjectLayer(world, body);
    },
    getUserData(world: World, body: BodyId): number {
        return Low.RigidBody.getUserDataHigh(world, body) * TWO_32 + Low.RigidBody.getUserDataLow(world, body);
    },
    /** null for an id naming no body */
    getShape(world: World, body: BodyId): Shape | null {
        return Low.RigidBody.getShape(world, body);
    },
    isSleeping(world: World, body: BodyId): boolean {
        return Low.RigidBody.isSleeping(world, body);
    },

    // writers: false when the id names no body

    /** a teleport: the broadphase and the pairs follow; nothing between here and there stops it */
    setPosition(world: World, body: BodyId, position: Vec3, wake = true): boolean {
        return Low.RigidBody.setPosition(world, body, position, wake);
    },
    setQuaternion(world: World, body: BodyId, quaternion: Quat, wake = true): boolean {
        return Low.RigidBody.setQuaternion(world, body, quaternion, wake);
    },
    setTransform(world: World, body: BodyId, position: Vec3, quaternion: Quat, wake = true): boolean {
        return Low.RigidBody.setTransform(world, body, position, quaternion, wake);
    },
    /** m/s, world space; wakes the body */
    setLinearVelocity(world: World, body: BodyId, velocity: Vec3): boolean {
        return Low.RigidBody.setLinearVelocity(world, body, velocity);
    },
    /** rad/s, world space; wakes the body */
    setAngularVelocity(world: World, body: BodyId, velocity: Vec3): boolean {
        return Low.RigidBody.setAngularVelocity(world, body, velocity);
    },
    addLinearVelocity(world: World, body: BodyId, velocity: Vec3): boolean {
        return Low.RigidBody.addLinearVelocity(world, body, velocity);
    },
    addAngularVelocity(world: World, body: BodyId, velocity: Vec3): boolean {
        return Low.RigidBody.addAngularVelocity(world, body, velocity);
    },
    /** N at the centre of mass over the next step, then cleared; dynamic bodies only */
    addForce(world: World, body: BodyId, force: Vec3, wake = true): boolean {
        return Low.RigidBody.addForce(world, body, force, wake);
    },
    addTorque(world: World, body: BodyId, torque: Vec3, wake = true): boolean {
        return Low.RigidBody.addTorque(world, body, torque, wake);
    },
    /** the force, and the torque it makes about the centre of mass */
    addForceAtPosition(world: World, body: BodyId, force: Vec3, position: Vec3, wake = true): boolean {
        return Low.RigidBody.addForceAtPosition(world, body, force, position, wake);
    },
    /** N s at the centre of mass, the velocity changed at once; dynamic bodies only, always wakes */
    addImpulse(world: World, body: BodyId, impulse: Vec3): boolean {
        return Low.RigidBody.addImpulse(world, body, impulse);
    },
    addImpulseAtPosition(world: World, body: BodyId, impulse: Vec3, position: Vec3): boolean {
        return Low.RigidBody.addImpulseAtPosition(world, body, impulse, position);
    },
    addAngularImpulse(world: World, body: BodyId, impulse: Vec3): boolean {
        return Low.RigidBody.addAngularImpulse(world, body, impulse);
    },
    /** the active list follows; `wake` wakes one that keeps moving */
    setMotionType(world: World, body: BodyId, motionType: MotionType, wake = true): boolean {
        return Low.RigidBody.setMotionType(world, body, motionType, wake);
    },
    /** into another object layer's broadphase tree, finding its pairs again */
    setObjectLayer(world: World, body: BodyId, objectLayer: number): boolean {
        return Low.RigidBody.setObjectLayer(world, body, objectLayer);
    },
    /** mass and bounds follow, and the constraints keep their anchors where they were on the body */
    setShape(world: World, body: BodyId, shape: Shape): boolean {
        return Low.RigidBody.setShape(world, body, shape);
    },
    /** the velocities that bring a kinematic body to the target in deltaTime, waking it */
    moveKinematic(world: World, body: BodyId, position: Vec3, quaternion: Quat, deltaTime: number): boolean {
        return Low.RigidBody.moveKinematic(world, body, position, quaternion, deltaTime);
    },
    sleep(world: World, body: BodyId): boolean {
        return Low.RigidBody.sleep(world, body);
    },
    wake(world: World, body: BodyId): boolean {
        return Low.RigidBody.wake(world, body);
    },
    setUserData(world: World, body: BodyId, userData: number): boolean {
        return Low.RigidBody.setUserData(world, body, userData >>> 0, Math.floor(userData / TWO_32));
    },
    setFriction(world: World, body: BodyId, friction: number): boolean {
        return Low.RigidBody.setFriction(world, body, friction);
    },
    setRestitution(world: World, body: BodyId, restitution: number): boolean {
        return Low.RigidBody.setRestitution(world, body, restitution);
    },
};
