import {
    addBroadphaseLayer,
    addObjectLayer,
    type Box3,
    box,
    createWorld,
    createWorldSettings,
    Dof,
    destroyWorld,
    enableCollision,
    init,
    type Listener,
    MotionQuality,
    MotionType,
    type Quat,
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
const crateShape = box.create({ halfExtents: [0.5, 0.5, 0.5] });
const ballShape = sphere.create({ radius: 0.25 });
const steps = (count: number): void => {
    for (let i = 0; i < count; i++) updateWorld(world, undefined, 1 / 60);
};
const at: Vec3 = [0, 0, 0];
const check = (ok: boolean, what: string): void => {
    if (!ok) throw new Error(what);
};

/* SNIPPET_START: create */
// a body is an id. it holds an index and a sequence, so a removed body's id names nothing, even after its slot is reused
const crate = rigidBody.create(world, {
    shape: crateShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [0, 3, 0],
    quaternion: [0, 0, 0, 1],
});
const doomed = rigidBody.create(world, { shape: ballShape, objectLayer: moving, motionType: MotionType.DYNAMIC });
rigidBody.remove(world, doomed);
rigidBody.exists(world, doomed); // false
// a call with a stale id does nothing. a getter returns false and leaves its out unchanged
rigidBody.getPosition(at, world, doomed); // false
/* SNIPPET_END: create */
check(rigidBody.exists(world, crate) && !rigidBody.exists(world, doomed), 'ids');

/* SNIPPET_START: motion-types */
// a static body never moves. a kinematic body moves by its velocity and pushes dynamic bodies aside. a dynamic body is
// moved by gravity, forces and contacts. the type can change later
const platform = rigidBody.create(world, {
    shape: crateShape,
    objectLayer: moving,
    motionType: MotionType.KINEMATIC,
    position: [5, 1, 0],
});
rigidBody.setMotionType(world, platform, MotionType.DYNAMIC, true);
/* SNIPPET_END: motion-types */
check(rigidBody.getMotionType(world, platform) === MotionType.DYNAMIC, 'motion type');

/* SNIPPET_START: transform */
const position: Vec3 = [0, 0, 0];
const quaternion: Quat = [0, 0, 0, 1];
rigidBody.getTransform(position, quaternion, world, crate);
// this teleports the body, so nothing in between stops it. the last argument wakes it
rigidBody.setTransform(world, crate, [0, 6, 0], [0, Math.SQRT1_2, 0, Math.SQRT1_2], true);
/* SNIPPET_END: transform */
rigidBody.getPosition(at, world, crate);
check(at[1] === 6, 'teleported');

/* SNIPPET_START: velocity */
rigidBody.setLinearVelocity(world, crate, [2, 0, 0]);
rigidBody.setAngularVelocity(world, crate, [0, Math.PI, 0]);
const velocity: Vec3 = [0, 0, 0];
rigidBody.getLinearVelocity(velocity, world, crate);
// the velocity of a point that moves with the body, here a corner
rigidBody.getVelocityAtPoint(velocity, world, crate, [0.5, 6.5, 0.5]);
/* SNIPPET_END: velocity */
check(velocity[0] !== 2, 'a corner turning moves faster than the centre');

/* SNIPPET_START: forces */
// a force acts over the next step, then is cleared. an impulse changes the velocity at once. both move only dynamic
// bodies. a crate's mass comes from its shape, at 1000 kg/m3 by default
const mass = rigidBody.getMass(world, crate);
rigidBody.addForce(world, crate, [0, 20 * mass, 0], true);
rigidBody.addImpulseAtPosition(world, crate, [0, 0, 5 * mass], [0.5, 6, 0]);
rigidBody.addAngularImpulse(world, crate, [0, 0, 0]);
/* SNIPPET_END: forces */
steps(1);
rigidBody.getLinearVelocity(velocity, world, crate);
check(velocity[2] > 4.9 && velocity[1] > 0, 'pushed');

/* SNIPPET_START: mass */
// set a mass and the inertia scales to it, or set the mass properties directly. a hollow mesh has no volume to give a mass
const heavy = rigidBody.create(world, {
    shape: ballShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [-5, 1, 0],
    mass: 80,
});
rigidBody.getMass(world, heavy); // 80
/* SNIPPET_END: mass */
check(Math.abs(rigidBody.getMass(world, heavy) - 80) < 1e-3, 'mass');

/* SNIPPET_START: tuning */
// damping slows the body each step. the limits cap how fast it moves and turns. the degrees of freedom pick its free axes.
// here it is a puck that slides on x and z and turns only about y
const puck = rigidBody.create(world, {
    shape: ballShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [-8, 0.25, 0],
    linearDamping: 0.1,
    angularDamping: 0.5,
    maxLinearVelocity: 30,
    gravityFactor: 0,
    allowedDegreesOfFreedom: Dof.TRANSLATION_X | Dof.TRANSLATION_Z | Dof.ROTATION_Y,
});
/* SNIPPET_END: tuning */
rigidBody.addImpulse(world, puck, [0, 50, 0]);
steps(10);
rigidBody.getPosition(at, world, puck);
check(at[1] === 0.25, 'held to its plane');

/* SNIPPET_START: sleeping */
// a body at rest sleeps. it costs nothing until something touches it or you wake it
steps(120);
rigidBody.isSleeping(world, heavy); // true
rigidBody.wake(world, heavy);
rigidBody.sleep(world, heavy);
// set allowSleeping to false for a body that should never sleep
/* SNIPPET_END: sleeping */
check(rigidBody.isSleeping(world, heavy), 'asleep');

/* SNIPPET_START: kinematic */
// a lift. moveKinematic sets the velocity that reaches the target in deltaTime. it pushes what it meets and wakes the body
const lift = rigidBody.create(world, {
    shape: crateShape,
    objectLayer: moving,
    motionType: MotionType.KINEMATIC,
    position: [10, 0.5, 0],
});
for (let i = 1; i <= 60; i++) {
    rigidBody.moveKinematic(world, lift, [10, 0.5 + i / 30, 0], [0, 0, 0, 1], 1 / 60);
    updateWorld(world, undefined, 1 / 60);
}
/* SNIPPET_END: kinematic */
rigidBody.getPosition(at, world, lift);
check(Math.abs(at[1] - 2.5) < 1e-4, 'lifted');

/* SNIPPET_START: ccd */
// a fast body can jump over a thin wall between two steps. this one moves 5 m a step. linear cast stops that
const wallShape = box.create({ halfExtents: [0.05, 2, 2] });
rigidBody.create(world, { shape: wallShape, objectLayer: notMoving, motionType: MotionType.STATIC, position: [20, 2, 0] });
const bullet = rigidBody.create(world, {
    shape: ballShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [14.3, 2, 0],
    motionQuality: MotionQuality.LINEAR_CAST,
    gravityFactor: 0,
});
rigidBody.setLinearVelocity(world, bullet, [300, 0, 0]);
/* SNIPPET_END: ccd */
const tunneller = rigidBody.create(world, {
    shape: ballShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [14.3, 2, 1],
    gravityFactor: 0,
});
rigidBody.setLinearVelocity(world, tunneller, [300, 0, 0]);
steps(3);
rigidBody.getPosition(at, world, bullet);
check(at[0] < 20, 'the linear cast stopped at the wall');
rigidBody.getPosition(at, world, tunneller);
check(at[0] > 20, 'the discrete body went through');

/* SNIPPET_START: user-data */
// your own value for a body, such as an entity index. it is an integer up to 2^53, and events report it with the body
rigidBody.setUserData(world, crate, 2 ** 40 + 7);
rigidBody.getUserData(world, crate); // 1099511627783
/* SNIPPET_END: user-data */
check(rigidBody.getUserData(world, crate) === 2 ** 40 + 7, 'user data');

/* SNIPPET_START: groups */
// two bodies in colliding layers collide when each one's group is in the other's mask. every body is in group 1 by
// default. this ghost is in group 2, and its mask leaves out group 1, so it passes through everything else
const ghost = rigidBody.create(world, {
    shape: ballShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [30, 3, 0],
    collisionGroups: 2,
    collisionMask: ~1 >>> 0,
});
/* SNIPPET_END: groups */
const groupedFloor = box.create({ halfExtents: [2, 0.5, 2] });
rigidBody.create(world, {
    shape: groupedFloor,
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    position: [30, 1, 0],
    collisionGroups: 1,
});
steps(60);
rigidBody.getPosition(at, world, ghost);
check(at[1] < 1, 'through the group 1 floor');

/* SNIPPET_START: sensors */
// a sensor reports contacts but pushes nothing. the listener sees them (see Events), and their settings mark them as
// sensor contacts
const zoneShape = box.create({ halfExtents: [1, 1, 1] });
const zone = rigidBody.create(world, {
    shape: zoneShape,
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    position: [40, 1, 0],
    sensor: true,
});
const entered: number[] = [];
const sensing: Listener = {
    onContactAdded(contact) {
        if (contact.bodyA === zone) entered.push(contact.bodyB);
        else if (contact.bodyB === zone) entered.push(contact.bodyA);
    },
};
/* SNIPPET_END: sensors */
const visitor = rigidBody.create(world, {
    shape: ballShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [40, 4, 0],
});
for (let i = 0; i < 90; i++) updateWorld(world, sensing, 1 / 60);
rigidBody.getPosition(at, world, visitor);
check(entered.includes(visitor) && at[1] < 0.3, 'fell through the zone, seen entering');

/* SNIPPET_START: shape */
// a new shape updates the mass, centre of mass and bounds. constraints keep their anchors on the body
const lump = sphere.create({ radius: 0.3 });
rigidBody.setShape(world, crate, lump, true);
// after changing a shape in place, pass it to setShape for each body that uses it
sphere.update(lump, { radius: 0.5 });
rigidBody.setShape(world, crate, lump, true);
/* SNIPPET_END: shape */
const reshaped: Box3 = { min: [0, 0, 0], max: [0, 0, 0] };
rigidBody.getBounds(reshaped, world, crate);
check(rigidBody.getShape(world, crate) === lump && Math.abs(reshaped.max[1] - reshaped.min[1] - 1) < 1e-4, 'reshaped');

/* SNIPPET_START: layer */
rigidBody.setObjectLayer(world, crate, notMoving);
/* SNIPPET_END: layer */
check(rigidBody.getObjectLayer(world, crate) === notMoving, 'relayered');

destroyWorld(world);
for (const shape of [floorShape, crateShape, wallShape, groupedFloor, zoneShape]) releaseShape(shape);
releaseShape(ballShape);
releaseShape(lump);
