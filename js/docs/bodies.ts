import {
    Box,
    Dof,
    init,
    type Listener,
    MotionQuality,
    MotionType,
    type Quat,
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
const crateShape = Box.create({ halfExtents: [0.5, 0.5, 0.5] });
const ballShape = Sphere.create({ radius: 0.25 });
const steps = (count: number): void => {
    for (let i = 0; i < count; i++) World.step(world, undefined, 1 / 60);
};
const at: Vec3 = [0, 0, 0];
const check = (ok: boolean, what: string): void => {
    if (!ok) throw new Error(what);
};

/* SNIPPET_START: create */
// a body is an id: its index and a sequence, so the id of a removed body names nothing, even once its slot is reused
const crate = RigidBody.create(world, {
    shape: crateShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [0, 3, 0],
    quaternion: [0, 0, 0, 1],
});
const doomed = RigidBody.create(world, { shape: ballShape, objectLayer: moving, motionType: MotionType.DYNAMIC });
RigidBody.remove(world, doomed);
RigidBody.exists(world, doomed); // false
// a call with an id naming no body does nothing: a reader returns false and leaves its out as it was
RigidBody.getPosition(at, world, doomed); // false
/* SNIPPET_END: create */
check(RigidBody.exists(world, crate) && !RigidBody.exists(world, doomed), 'ids');

/* SNIPPET_START: motion-types */
// static never moves; kinematic moves by its velocity and pushes dynamic bodies aside; dynamic is moved by gravity,
// forces and contacts. the type can change later
const platform = RigidBody.create(world, {
    shape: crateShape,
    objectLayer: moving,
    motionType: MotionType.KINEMATIC,
    position: [5, 1, 0],
});
RigidBody.setMotionType(world, platform, MotionType.DYNAMIC, true);
/* SNIPPET_END: motion-types */
check(RigidBody.getMotionType(world, platform) === MotionType.DYNAMIC, 'motion type');

/* SNIPPET_START: transform */
const position: Vec3 = [0, 0, 0];
const quaternion: Quat = [0, 0, 0, 1];
RigidBody.getTransform(position, quaternion, world, crate);
// a teleport: nothing between here and there stops it. the last argument wakes it
RigidBody.setTransform(world, crate, [0, 6, 0], [0, Math.SQRT1_2, 0, Math.SQRT1_2], true);
/* SNIPPET_END: transform */
RigidBody.getPosition(at, world, crate);
check(at[1] === 6, 'teleported');

/* SNIPPET_START: velocity */
RigidBody.setLinearVelocity(world, crate, [2, 0, 0]);
RigidBody.setAngularVelocity(world, crate, [0, Math.PI, 0]);
const velocity: Vec3 = [0, 0, 0];
RigidBody.getLinearVelocity(velocity, world, crate);
// the velocity of a point carried with the body, a corner here
RigidBody.getVelocityAtPoint(velocity, world, crate, [0.5, 6.5, 0.5]);
/* SNIPPET_END: velocity */
check(velocity[0] !== 2, 'a corner turning moves faster than the centre');

/* SNIPPET_START: forces */
// a force acts over the next step, then is cleared; an impulse changes the velocity at once. both move only dynamic
// bodies. a crate's mass is from its shape: 1000 kg/m3 by default
const mass = RigidBody.getMass(world, crate);
RigidBody.addForce(world, crate, [0, 20 * mass, 0], true);
RigidBody.addImpulseAtPosition(world, crate, [0, 0, 5 * mass], [0.5, 6, 0]);
RigidBody.addAngularImpulse(world, crate, [0, 0, 0]);
/* SNIPPET_END: forces */
steps(1);
RigidBody.getLinearVelocity(velocity, world, crate);
check(velocity[2] > 4.9 && velocity[1] > 0, 'pushed');

/* SNIPPET_START: mass */
// a mass the shape's inertia is scaled to, or mass properties outright: a hollow mesh has no volume to take one from
const heavy = RigidBody.create(world, {
    shape: ballShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [-5, 1, 0],
    mass: 80,
});
RigidBody.getMass(world, heavy); // 80
/* SNIPPET_END: mass */
check(Math.abs(RigidBody.getMass(world, heavy) - 80) < 1e-3, 'mass');

/* SNIPPET_START: tuning */
// damping slows it each step; the most it may move and turn; the axes it is free on: here a puck, sliding on x and z
// and turning only about y
const puck = RigidBody.create(world, {
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
RigidBody.addImpulse(world, puck, [0, 50, 0]);
steps(10);
RigidBody.getPosition(at, world, puck);
check(at[1] === 0.25, 'held to its plane');

/* SNIPPET_START: sleeping */
// a body at rest sleeps, costing nothing until something touches it or you wake it
steps(120);
RigidBody.isSleeping(world, heavy); // true
RigidBody.wake(world, heavy);
RigidBody.sleep(world, heavy);
// one that should never sleep: allowSleeping false
/* SNIPPET_END: sleeping */
check(RigidBody.isSleeping(world, heavy), 'asleep');

/* SNIPPET_START: kinematic */
// a lift: its velocity brings it to the target in deltaTime, pushing what it meets, and wakes it
const lift = RigidBody.create(world, {
    shape: crateShape,
    objectLayer: moving,
    motionType: MotionType.KINEMATIC,
    position: [10, 0.5, 0],
});
for (let i = 1; i <= 60; i++) {
    RigidBody.moveKinematic(world, lift, [10, 0.5 + i / 30, 0], [0, 0, 0, 1], 1 / 60);
    World.step(world, undefined, 1 / 60);
}
/* SNIPPET_END: kinematic */
RigidBody.getPosition(at, world, lift);
check(Math.abs(at[1] - 2.5) < 1e-4, 'lifted');

/* SNIPPET_START: ccd */
// fast and thin: a discrete body can step clean over a wall between one step and the next (5 m a step here); a
// linear cast cannot
const wallShape = Box.create({ halfExtents: [0.05, 2, 2] });
RigidBody.create(world, { shape: wallShape, objectLayer: notMoving, motionType: MotionType.STATIC, position: [20, 2, 0] });
const bullet = RigidBody.create(world, {
    shape: ballShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [14.3, 2, 0],
    motionQuality: MotionQuality.LINEAR_CAST,
    gravityFactor: 0,
});
RigidBody.setLinearVelocity(world, bullet, [300, 0, 0]);
/* SNIPPET_END: ccd */
const tunneller = RigidBody.create(world, {
    shape: ballShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [14.3, 2, 1],
    gravityFactor: 0,
});
RigidBody.setLinearVelocity(world, tunneller, [300, 0, 0]);
steps(3);
RigidBody.getPosition(at, world, bullet);
check(at[0] < 20, 'the linear cast stopped at the wall');
RigidBody.getPosition(at, world, tunneller);
check(at[0] > 20, 'the discrete body went through');

/* SNIPPET_START: user-data */
// your own value for a body, an integer up to 2^53: an entity's index, say. it comes back with the body in events
RigidBody.setUserData(world, crate, 2 ** 40 + 7);
RigidBody.getUserData(world, crate); // 1099511627783
/* SNIPPET_END: user-data */
check(RigidBody.getUserData(world, crate) === 2 ** 40 + 7, 'user data');

/* SNIPPET_START: groups */
// within layers that collide, two bodies collide when each is in a group of the other's mask: a ghost in group 2, its
// mask without group 1, passes through everything in group 1 (every body's group by default)
const ghost = RigidBody.create(world, {
    shape: ballShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [30, 3, 0],
    collisionGroups: 2,
    collisionMask: ~1 >>> 0,
});
/* SNIPPET_END: groups */
const groupedFloor = Box.create({ halfExtents: [2, 0.5, 2] });
RigidBody.create(world, {
    shape: groupedFloor,
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    position: [30, 1, 0],
    collisionGroups: 1,
});
steps(60);
RigidBody.getPosition(at, world, ghost);
check(at[1] < 1, 'through the group 1 floor');

/* SNIPPET_START: sensors */
// a sensor reports contacts and pushes nothing: through the listener (see Events), its contacts' settings say so
const zoneShape = Box.create({ halfExtents: [1, 1, 1] });
const zone = RigidBody.create(world, {
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
const visitor = RigidBody.create(world, {
    shape: ballShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [40, 4, 0],
});
for (let i = 0; i < 90; i++) World.step(world, sensing, 1 / 60);
RigidBody.getPosition(at, world, visitor);
check(entered.includes(visitor) && at[1] < 0.3, 'fell through the zone, seen entering');

/* SNIPPET_START: shape */
// a new shape: the mass, centre of mass and bounds follow, and constraints keep their anchors on the body
RigidBody.setShape(world, crate, ballShape);
/* SNIPPET_END: shape */
check(RigidBody.getShape(world, crate) === ballShape, 'reshaped');

/* SNIPPET_START: layer */
RigidBody.setObjectLayer(world, crate, notMoving);
/* SNIPPET_END: layer */
check(RigidBody.getObjectLayer(world, crate) === notMoving, 'relayered');

World.destroy(world);
for (const shape of [floorShape, crateShape, wallShape, groupedFloor, zoneShape]) Box.destroy(shape);
Sphere.destroy(ballShape);
