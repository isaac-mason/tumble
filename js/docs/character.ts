import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    capsule,
    contactPairs,
    createWorld,
    createWorldSettings,
    destroyWorld,
    enableCollision,
    filter,
    init,
    type KCCContact,
    kcc,
    kccPolicy,
    MotionType,
    releaseShape,
    rigidBody,
    transformed,
    updateWorld,
    type Vec3,
} from 'tumble.js';

await init();
const worldSettings = createWorldSettings();
const moving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
const notMoving = addObjectLayer(worldSettings, addBroadphaseLayer(worldSettings));
enableCollision(worldSettings, moving, moving);
enableCollision(worldSettings, moving, notMoving);
const world = createWorld(worldSettings);
const groundShape = box.create({ halfExtents: [50, 0.5, 50] });
const stepShape = box.create({ halfExtents: [1, 0.15, 2] });
const crateShape = box.create({ halfExtents: [0.4, 0.4, 0.4], density: 20 });
rigidBody.create(world, { shape: groundShape, objectLayer: notMoving, motionType: MotionType.STATIC, position: [0, -0.5, 0] });
// a 30 cm step 4 m ahead, and a crate beside the way
rigidBody.create(world, { shape: stepShape, objectLayer: notMoving, motionType: MotionType.STATIC, position: [5, 0.15, 0] });
const crate = rigidBody.create(world, {
    shape: crateShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [2, 0.4, 3],
});
const check = (ok: boolean, what: string): void => {
    if (!ok) throw new Error(what);
};

/* SNIPPET_START: create */
// a capsule standing on its feet. the character's position is at its feet, so the capsule is raised 0.8 m. the inner
// body represents the character in the world. bodies and queries see it, and contact pairs use it
const standingCapsule = capsule.create({ halfHeightOfCylinder: 0.5, radius: 0.3 });
const standing = transformed.create({ shape: standingCapsule, position: [0, 0.8, 0] });
const character = kcc.create(
    {
        shape: standing,
        maxSlopeAngle: (45 * Math.PI) / 180,
        innerRigidBody: { shape: standing, objectLayer: moving },
        observesContacts: true,
    },
    [0, 0, 0],
    [0, 0, 0, 1],
);
kcc.add(world, character);
// the character collides with everything its inner body collides with
const queryFilter = filter.create(world);
filter.setFromBody(queryFilter, world, character.innerRigidBodyId);
/* SNIPPET_END: create */

/* SNIPPET_START: policy */
// a policy sets how characters treat each body. bodies without their own entry use the defaults. one policy can serve
// any number of characters. here the crate can be pushed, until the character crouches below
const policy = kccPolicy.create();
kccPolicy.set(policy, crate, { canReceiveImpulses: true });
/* SNIPPET_END: policy */

const gravity: Vec3 = [0, -9.81, 0];
/* SNIPPET_START: update */
// one game frame. the velocity starts from the ground's when standing, or keeps falling when not, and adds the input.
// then the character moves. it slides along walls, climbs stairs and sticks to floors going down. then the world steps
const updateSettings = kcc.createDefaultUpdateSettings();
function frame(input: Vec3, deltaTime: number): void {
    kcc.updateGroundVelocity(world, character, policy);
    const { up, ground } = character;
    const v = character.linearVelocity;
    const along = v[0] * up[0] + v[1] * up[1] + v[2] * up[2];
    const base =
        ground.state === kcc.GroundState.ON_GROUND ? ground.velocity : ([up[0] * along, up[1] * along, up[2] * along] as Vec3);
    character.linearVelocity = [
        base[0] + gravity[0] * deltaTime + input[0],
        base[1] + gravity[1] * deltaTime + input[1],
        base[2] + gravity[2] * deltaTime + input[2],
    ];
    kcc.update(world, character, deltaTime, gravity, updateSettings, policy, queryFilter);
    updateWorld(world, undefined, deltaTime);
}
for (let i = 0; i < 180; i++) frame([3, 0, 0], 1 / 60);
character.position; // up the step, along it and down again
/* SNIPPET_END: update */
// three seconds at 3 m/s: past the step and onto its top, then off it again, on the ground throughout
check(character.position[0] > 8 && character.ground.state === kcc.GroundState.ON_GROUND, `at ${character.position}`);

/* SNIPPET_START: events */
// the contacts from the last update, refreshContacts or setShape. each one is added, persisted or removed
for (const event of character.contactEvents) {
    if (event.kind === kcc.KCCContactEventKind.ADDED) event.bodyId; // footsteps, say
}
/* SNIPPET_END: events */
check(character.contactEvents.length > 0, 'events');

/* SNIPPET_START: crouch */
// a shorter shape on the same feet. setShape returns false and keeps the old shape if the new one would sink deeper
// than the given depth
const crouchingCapsule = capsule.create({ halfHeightOfCylinder: 0.2, radius: 0.3 });
const crouching = transformed.create({ shape: crouchingCapsule, position: [0, 0.5, 0] });
if (kcc.setShape(world, character, crouching, queryFilter, policy, 0.01))
    kccPolicy.set(policy, crate, { canReceiveImpulses: false });
/* SNIPPET_END: crouch */
check(character.shape === crouching, 'crouched');

/* SNIPPET_START: tuning */
// settings can change after create. here the supporting point is lower while crouched
kcc.setTuning(character, { supportingVolumePlane: [0, 1, 0, -0.3] });
/* SNIPPET_END: tuning */

/* SNIPPET_START: contacts */
// read the character's contacts as data, through its inner body, in a contact pairs frame
contactPairs.begin(world);
kcc.refreshContacts(world, character, queryFilter, policy);
contactPairs.recordCharacterContacts(world, character);
contactPairs.end(world);
/* SNIPPET_END: contacts */
const pair = contactPairs.create();
check(
    contactPairs.readActive(pair, world, 0) && pair.bodyA === character.innerRigidBodyId && pair.normal[1] < -0.99,
    'its floor pair',
);

/* SNIPPET_START: toolkit */
// update does the whole move in one call. the toolkit splits it into phases, so your own loop can edit the contacts
// between collide and solve. here the character jumps up through a one-way platform. from below, the loop discards the
// platform's contacts and the sweep's hit on it
const platformShape = box.create({ halfExtents: [1.5, 0.1, 1.5] });
const platform = rigidBody.create(world, {
    shape: platformShape,
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    position: [character.position[0], 1.8, 0],
});
const fromBelow = (bodyId: number, normal: Vec3): boolean => bodyId === platform && normal[1] < 0.5;
const solved = kcc.createSolveResult();
const hit: KCCContact = kcc.createContact();
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
        kcc.collide(world, character, position, direction, queryFilter, policy);
        for (const contact of character.contacts) {
            if (fromBelow(contact.bodyId, contact.contactNormal)) contact.wasDiscarded = true;
        }
        kcc.solve(solved, world, character, velocity, deltaTime, timeRemaining, policy);
        velocity.splice(0, 3, ...solved.velocity);
        let share = 1;
        if (
            kcc.cast(hit, world, character, position, solved.displacement, queryFilter, policy) &&
            !fromBelow(hit.bodyId, hit.contactNormal)
        ) {
            share = hit.fraction;
        }
        for (let axis = 0; axis < 3; axis++) position[axis]! += solved.displacement[axis]! * share;
        timeRemaining -= solved.timeSimulated * share;
    }
    kcc.setPosition(world, character, position);
    kcc.updateGround(world, character, deltaTime, false, policy);
    // it lands on the platform on the way down
    if (character.ground.state === kcc.GroundState.ON_GROUND && character.linearVelocity[1] <= 0)
        character.linearVelocity = [0, 0, 0];
    updateWorld(world, undefined, deltaTime);
}
/* SNIPPET_END: toolkit */
check(
    character.ground.bodyId === platform && Math.abs(character.position[1] - 1.9) < 0.01,
    `on the platform at ${character.position}`,
);

/* SNIPPET_START: remove */
kcc.remove(world, character);
kcc.release(character);
kccPolicy.destroy(policy);
/* SNIPPET_END: remove */

filter.destroy(queryFilter);
destroyWorld(world);
for (const shape of [standing, crouching]) releaseShape(shape);
for (const shape of [standingCapsule, crouchingCapsule]) releaseShape(shape);
for (const shape of [groundShape, stepShape, crateShape, platformShape]) releaseShape(shape);
