import {
    Box,
    type ContactEvent,
    init,
    type Listener,
    MotionType,
    RigidBody,
    Sphere,
    type Vec3,
    World,
    WorldSettings,
} from 'tumble.js';

await init();
const settings = WorldSettings.create();
const moving = WorldSettings.addObjectLayer(settings, WorldSettings.addBroadphaseLayer(settings));
const notMoving = WorldSettings.addObjectLayer(settings, WorldSettings.addBroadphaseLayer(settings));
WorldSettings.enableCollision(settings, moving, moving);
WorldSettings.enableCollision(settings, moving, notMoving);
const world = World.create(settings);
const floorShape = Box.create({ halfExtents: [50, 0.5, 50] });
RigidBody.create(world, { shape: floorShape, objectLayer: notMoving, motionType: MotionType.STATIC, position: [0, -0.5, 0] });
const slabShape = Box.create({ halfExtents: [1.5, 0.1, 1.5] });
const ballShape = Sphere.create({ radius: 0.25 });
const parcelShape = Box.create({ halfExtents: [0.25, 0.25, 0.25] });
const at: Vec3 = [0, 0, 0];
const check = (ok: boolean, what: string): void => {
    if (!ok) throw new Error(what);
};
const body = (shape: typeof ballShape, motionType: MotionType, position: Vec3, userData = 0): number =>
    RigidBody.create(world, {
        shape,
        objectLayer: motionType === MotionType.STATIC ? notMoving : moving,
        motionType,
        position,
        userData,
    });

const PLAYER = 1;
const platform = body(slabShape, MotionType.STATIC, [0, 2, 0]);
const pad = body(slabShape, MotionType.STATIC, [6, 0.1, 0]);
const belt = body(slabShape, MotionType.STATIC, [12, 0.1, 0]);
const ghostA = body(ballShape, MotionType.DYNAMIC, [18, 1, 0], PLAYER);
const ghostB = body(ballShape, MotionType.DYNAMIC, [18, 3, 0], PLAYER);

/** the other body of the contact, when one side is `body` */
const other = (contact: ContactEvent, of: number): number | null =>
    contact.bodyA === of ? contact.bodyB : contact.bodyB === of ? contact.bodyA : null;

/* SNIPPET_START: listener */
// the step's events, and your answers to them, as it runs
const listener: Listener = {
    // two bodies whose bounds overlap, before the narrowphase: false and they do not collide this step. the cheapest
    // place to filter a pair: here a player's parts pass through each other
    onBodyPairValidate(bodyA, bodyB) {
        return !(RigidBody.getUserData(world, bodyA) === PLAYER && RigidBody.getUserData(world, bodyB) === PLAYER);
    },
    // every contact, each step, before it is solved: false drops it this step. a one-way platform: solid only from above
    onContactValidate(contact) {
        if (other(contact, platform) === null) return true;
        // the normal is from a to b: from the platform up into what lands on it is up
        const up = contact.bodyA === platform ? contact.normal[1] : -contact.normal[1];
        return up > 0.5;
    },
    // a contact new this step, its settings yours to change: a pad that bounces whatever lands on it
    onContactAdded(contact, settings) {
        if (other(contact, pad) !== null) settings.combinedRestitution = 1;
    },
    // a contact kept from the last step: a belt whose surface moves 2 m/s along x, carrying what rests on it. the
    // surface velocity is b's less a's
    onContactPersisted(contact, settings) {
        if (other(contact, belt) === null) return;
        const carry = contact.bodyA === belt ? -2 : 2;
        settings.relativeLinearSurfaceVelocity = [carry, 0, 0];
    },
    // after the solve: a contact gone, a removed body's included
    onContactRemoved(removed) {
        removed.bodyA;
    },
};
World.step(world, listener, 1 / 60);
/* SNIPPET_END: listener */

// up through the platform from below, landing on it
const jumper = body(ballShape, MotionType.DYNAMIC, [0, 0.25, 0]);
RigidBody.setLinearVelocity(world, jumper, [0, 9, 0]);
const bouncer = body(ballShape, MotionType.DYNAMIC, [6, 3, 0]);
const parcel = body(parcelShape, MotionType.DYNAMIC, [11, 0.5, 0]);
let highestBounce = 0;
for (let i = 0; i < 150; i++) {
    World.step(world, listener, 1 / 60);
    if (i > 40) {
        RigidBody.getPosition(at, world, bouncer);
        highestBounce = Math.max(highestBounce, at[1]);
    }
}
RigidBody.getPosition(at, world, jumper);
check(Math.abs(at[1] - 2.33) < 0.03, `the jumper on the platform at y ${at[1]}`);
check(highestBounce > 2.5, `bounced to ${highestBounce}`);
RigidBody.getPosition(at, world, parcel);
// two and a half seconds at the belt's 2 m/s, off its end at x 13.5
check(at[0] > 13.5, `carried along x to ${at[0]}`);
RigidBody.getPosition(at, world, ghostA);
const a = at[1];
RigidBody.getPosition(at, world, ghostB);
check(Math.abs(at[1] - a) < 0.1, "a player's parts fell through each other");

/* SNIPPET_START: rules */
// a hook's events are reused: copy what you keep. the world is mid step while hooks run, so read it but do not change
// it. a hook that throws ends the step's hooks: the step finishes, then throws it
const kept: { bodyA: number; normal: Vec3 }[] = [];
World.step(
    world,
    {
        onContactAdded(contact) {
            kept.push({ bodyA: contact.bodyA, normal: [...contact.normal] });
        },
    },
    1 / 60,
);
/* SNIPPET_END: rules */

World.destroy(world);
Box.destroy(floorShape);
Box.destroy(slabShape);
Box.destroy(parcelShape);
Sphere.destroy(ballShape);
