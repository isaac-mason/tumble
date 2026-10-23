import {
    Box,
    Capsule,
    ContactPairs,
    createContact,
    createDefaultUpdateSettings,
    createSolveResult,
    Filter,
    GroundState,
    init,
    KCC,
    type KCCContact,
    KCCContactEventKind,
    KCCPolicy,
    MotionType,
    RigidBody,
    Transformed,
    type Vec3,
    World,
    WorldSettings,
} from 'tumble.js';

await init();
const worldSettings = WorldSettings.create();
const moving = WorldSettings.addObjectLayer(worldSettings, WorldSettings.addBroadphaseLayer(worldSettings));
const notMoving = WorldSettings.addObjectLayer(worldSettings, WorldSettings.addBroadphaseLayer(worldSettings));
WorldSettings.enableCollision(worldSettings, moving, moving);
WorldSettings.enableCollision(worldSettings, moving, notMoving);
const world = World.create(worldSettings);
const groundShape = Box.create({ halfExtents: [50, 0.5, 50] });
const stepShape = Box.create({ halfExtents: [1, 0.15, 2] });
const crateShape = Box.create({ halfExtents: [0.4, 0.4, 0.4], density: 20 });
RigidBody.create(world, { shape: groundShape, objectLayer: notMoving, motionType: MotionType.STATIC, position: [0, -0.5, 0] });
// a 30 cm step 4 m ahead, and a crate beside the way
RigidBody.create(world, { shape: stepShape, objectLayer: notMoving, motionType: MotionType.STATIC, position: [5, 0.15, 0] });
const crate = RigidBody.create(world, {
    shape: crateShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [2, 0.4, 3],
});
const check = (ok: boolean, what: string): void => {
    if (!ok) throw new Error(what);
};

/* SNIPPET_START: create */
// a capsule standing on its feet: the character's position is its feet, the capsule raised 0.8 m to stand on them.
// the inner body is its presence in the world, which bodies and queries see and contact pairs name it by
const standingCapsule = Capsule.create({ halfHeightOfCylinder: 0.5, radius: 0.3 });
const standing = Transformed.create({ shape: standingCapsule, position: [0, 0.8, 0] });
const character = KCC.create(
    {
        shape: standing,
        maxSlopeAngle: (45 * Math.PI) / 180,
        innerRigidBody: { shape: standing, objectLayer: moving },
        observesContacts: true,
    },
    [0, 0, 0],
    [0, 0, 0, 1],
);
KCC.add(world, character);
// what the character collides with: everything its inner body does
const filter = Filter.create(world);
Filter.setFromBody(filter, world, character.innerRigidBodyId);
/* SNIPPET_END: create */

/* SNIPPET_START: policy */
// how characters treat the bodies they meet, by body, the rest at the defaults; one policy serves any number of
// characters. here the crate may be pushed (until the character crouches, below)
const policy = KCCPolicy.create();
KCCPolicy.set(policy, crate, { canReceiveImpulses: true });
/* SNIPPET_END: policy */

const gravity: Vec3 = [0, -9.81, 0];
/* SNIPPET_START: update */
// a frame of a game: the velocity from the ground's when standing, else falling on, plus the input; then the character
// moves (sliding along walls, up stairs, sticking to floors going down), then the world steps
const updateSettings = createDefaultUpdateSettings();
function frame(input: Vec3, deltaTime: number): void {
    KCC.updateGroundVelocity(world, character, policy);
    const { up, ground } = character;
    const v = character.linearVelocity;
    const along = v[0] * up[0] + v[1] * up[1] + v[2] * up[2];
    const base =
        ground.state === GroundState.ON_GROUND ? ground.velocity : ([up[0] * along, up[1] * along, up[2] * along] as Vec3);
    character.linearVelocity = [
        base[0] + gravity[0] * deltaTime + input[0],
        base[1] + gravity[1] * deltaTime + input[1],
        base[2] + gravity[2] * deltaTime + input[2],
    ];
    KCC.update(world, character, deltaTime, gravity, updateSettings, policy, filter);
    World.step(world, undefined, deltaTime);
}
for (let i = 0; i < 180; i++) frame([3, 0, 0], 1 / 60);
character.position; // up the step, along it and down again
/* SNIPPET_END: update */
// three seconds at 3 m/s: past the step and onto its top, then off it again, on the ground throughout
check(character.position[0] > 8 && character.ground.state === GroundState.ON_GROUND, `at ${character.position}`);

/* SNIPPET_START: events */
// what the character met in its last update (or refreshContacts, or setShape): each contact added or persisted as it
// was first touched, and each gone by the end
for (const event of character.contactEvents) {
    if (event.kind === KCCContactEventKind.ADDED) event.bodyId; // footsteps, say
}
/* SNIPPET_END: events */
check(character.contactEvents.length > 0, 'events');

/* SNIPPET_START: crouch */
// a shorter shape on the same feet, where it fits: refused, false, if it would sink deeper than the given depth
const crouchingCapsule = Capsule.create({ halfHeightOfCylinder: 0.2, radius: 0.3 });
const crouching = Transformed.create({ shape: crouchingCapsule, position: [0, 0.5, 0] });
if (KCC.setShape(world, character, crouching, filter, policy, 0.01)) KCCPolicy.set(policy, crate, { canReceiveImpulses: false });
/* SNIPPET_END: crouch */
check(character.shape === crouching, 'crouched');

/* SNIPPET_START: tuning */
// its settings, tuned after create: here a lower point of support while crouched
KCC.setTuning(character, { supportingVolumePlane: [0, 1, 0, -0.3] });
/* SNIPPET_END: tuning */

/* SNIPPET_START: contacts */
// its contacts as data, through its inner body, in a contact pairs frame
ContactPairs.begin(world);
KCC.refreshContacts(world, character, filter, policy);
ContactPairs.recordCharacterContacts(world, character);
ContactPairs.end(world);
/* SNIPPET_END: contacts */
const pair = ContactPairs.create();
check(
    ContactPairs.readActive(pair, world, 0) && pair.bodyA === character.innerRigidBodyId && pair.normal[1] < -0.99,
    'its floor pair',
);

/* SNIPPET_START: toolkit */
// update is the whole move in one call; the toolkit is its phases, for a loop of your own that edits the contacts
// between collide and solve. here a one-way platform overhead, jumped up through: from below, its contacts and the
// sweep's hit on it are passed over
const platformShape = Box.create({ halfExtents: [1.5, 0.1, 1.5] });
const platform = RigidBody.create(world, {
    shape: platformShape,
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    position: [character.position[0], 1.8, 0],
});
const fromBelow = (bodyId: number, normal: Vec3): boolean => bodyId === platform && normal[1] < 0.5;
const solved = createSolveResult();
const hit: KCCContact = createContact();
character.linearVelocity = [0, 7, 0];
for (let i = 0; i < 90; i++) {
    const deltaTime = 1 / 60;
    const v = character.linearVelocity;
    character.linearVelocity = [v[0], v[1] + gravity[1] * deltaTime, v[2]];
    const position: Vec3 = [...character.position];
    const velocity: Vec3 = [...character.linearVelocity];
    let timeRemaining = deltaTime;
    for (let iteration = 0; iteration < 5 && timeRemaining > 1e-4; iteration++) {
        const speed = Math.hypot(...velocity);
        const direction: Vec3 = speed > 0 ? [velocity[0] / speed, velocity[1] / speed, velocity[2] / speed] : [0, 0, 0];
        KCC.collide(world, character, position, direction, filter, policy);
        for (const contact of character.contacts) {
            if (fromBelow(contact.bodyId, contact.contactNormal)) contact.wasDiscarded = true;
        }
        KCC.solve(solved, world, character, velocity, deltaTime, timeRemaining, policy);
        velocity.splice(0, 3, ...solved.velocity);
        let share = 1;
        if (
            KCC.cast(hit, world, character, position, solved.displacement, filter, policy) &&
            !fromBelow(hit.bodyId, hit.contactNormal)
        ) {
            share = hit.fraction;
        }
        for (let axis = 0; axis < 3; axis++) position[axis]! += solved.displacement[axis]! * share;
        timeRemaining -= solved.timeSimulated * share;
    }
    KCC.setPosition(world, character, position);
    KCC.updateGround(world, character, deltaTime, false, policy);
    // landed, coming down
    if (character.ground.state === GroundState.ON_GROUND && character.linearVelocity[1] <= 0)
        character.linearVelocity = [0, 0, 0];
    World.step(world, undefined, deltaTime);
}
/* SNIPPET_END: toolkit */
check(
    character.ground.bodyId === platform && Math.abs(character.position[1] - 1.9) < 0.01,
    `on the platform at ${character.position}`,
);

/* SNIPPET_START: remove */
KCC.remove(world, character);
KCC.destroy(character);
KCCPolicy.destroy(policy);
/* SNIPPET_END: remove */

Filter.destroy(filter);
World.destroy(world);
for (const shape of [standing, crouching]) Transformed.destroy(shape);
for (const shape of [standingCapsule, crouchingCapsule]) Capsule.destroy(shape);
for (const shape of [groundShape, stepShape, crateShape, platformShape]) Box.destroy(shape);
