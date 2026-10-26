import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    type ContactEvent,
    createWorld,
    createWorldSettings,
    destroyWorld,
    enableCollision,
    init,
    type Listener,
    MotionType,
    releaseShape,
    rigidBody,
    sphere,
    updateWorld,
    type Vec3,
} from 'tumble.js';

await init();
const settings = createWorldSettings();
const moving = addObjectLayer(settings, addBroadphaseLayer(settings));
const notMoving = addObjectLayer(settings, addBroadphaseLayer(settings));
enableCollision(settings, moving, moving);
enableCollision(settings, moving, notMoving);
const world = createWorld(settings);
const floorShape = box.create({ halfExtents: [50, 0.5, 50] });
rigidBody.create(world, { shape: floorShape, objectLayer: notMoving, motionType: MotionType.STATIC, position: [0, -0.5, 0] });
const slabShape = box.create({ halfExtents: [1.5, 0.1, 1.5] });
const ballShape = sphere.create({ radius: 0.25 });
const parcelShape = box.create({ halfExtents: [0.25, 0.25, 0.25] });
const at: Vec3 = [0, 0, 0];
const check = (ok: boolean, what: string): void => {
    if (!ok) throw new Error(what);
};
const body = (shape: typeof ballShape, motionType: MotionType, position: Vec3, userData = 0): number =>
    rigidBody.create(world, {
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
// the listener hears the step's events, and can reject or change them, while it runs
const listener: Listener = {
    // called for two bodies whose bounds overlap, before the narrowphase. return false and they do not collide this step.
    // this is the cheapest place to filter a pair. here a player's parts pass through each other
    onBodyPairValidate(bodyA, bodyB) {
        return !(rigidBody.getUserData(world, bodyA) === PLAYER && rigidBody.getUserData(world, bodyB) === PLAYER);
    },
    // called for every contact each step, before it is solved. return false to drop it this step. here a one-way platform
    // is solid only from above
    onContactValidate(contact) {
        if (other(contact, platform) === null) return true;
        // the normal points from a to b. from the platform to a body landing on it, that is up
        const up = contact.bodyA === platform ? contact.normal[1] : -contact.normal[1];
        return up > 0.5;
    },
    // called for a new contact. you can change its settings. here a pad bounces whatever lands on it
    onContactAdded(contact, settings) {
        if (other(contact, pad) !== null) settings.combinedRestitution = 1;
    },
    // called for a contact kept from the last step. here a belt's surface moves 2 m/s along x and carries what rests on
    // it. the surface velocity is b's minus a's
    onContactPersisted(contact, settings) {
        if (other(contact, belt) === null) return;
        const carry = contact.bodyA === belt ? -2 : 2;
        settings.relativeLinearSurfaceVelocity = [carry, 0, 0];
    },
    // called after the solve for each contact that ended, including those of removed bodies
    onContactRemoved(removed) {
        removed.bodyA;
    },
};
updateWorld(world, listener, 1 / 60);
/* SNIPPET_END: listener */

// up through the platform from below, landing on it
const jumper = body(ballShape, MotionType.DYNAMIC, [0, 0.25, 0]);
rigidBody.setLinearVelocity(world, jumper, [0, 9, 0]);
const bouncer = body(ballShape, MotionType.DYNAMIC, [6, 3, 0]);
const parcel = body(parcelShape, MotionType.DYNAMIC, [11, 0.5, 0]);
let highestBounce = 0;
for (let i = 0; i < 150; i++) {
    updateWorld(world, listener, 1 / 60);
    if (i > 40) {
        rigidBody.getPosition(at, world, bouncer);
        highestBounce = Math.max(highestBounce, at[1]);
    }
}
rigidBody.getPosition(at, world, jumper);
check(Math.abs(at[1] - 2.33) < 0.03, `the jumper on the platform at y ${at[1]}`);
check(highestBounce > 2.5, `bounced to ${highestBounce}`);
rigidBody.getPosition(at, world, parcel);
// two and a half seconds at the belt's 2 m/s, off its end at x 13.5
check(at[0] > 13.5, `carried along x to ${at[0]}`);
rigidBody.getPosition(at, world, ghostA);
const a = at[1];
rigidBody.getPosition(at, world, ghostB);
check(Math.abs(at[1] - a) < 0.1, "a player's parts fell through each other");

/* SNIPPET_START: rules */
// hook events are reused, so copy what you keep. the world is mid step while hooks run. read it, but do not change it.
// if a hook throws, no more hooks run. the step finishes, then throws the error
const kept: { bodyA: number; normal: Vec3 }[] = [];
updateWorld(
    world,
    {
        onContactAdded(contact) {
            kept.push({ bodyA: contact.bodyA, normal: [...contact.normal] });
        },
    },
    1 / 60,
);
/* SNIPPET_END: rules */

destroyWorld(world);
releaseShape(floorShape);
releaseShape(slabShape);
releaseShape(parcelShape);
releaseShape(ballShape);
