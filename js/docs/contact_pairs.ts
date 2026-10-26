import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    contactPairs,
    createWorld,
    createWorldSettings,
    destroyWorld,
    enableCollision,
    init,
    MotionType,
    releaseShape,
    rigidBody,
    sphere,
    updateWorld,
} from 'tumble.js';

await init();
const settings = createWorldSettings();
const moving = addObjectLayer(settings, addBroadphaseLayer(settings));
const notMoving = addObjectLayer(settings, addBroadphaseLayer(settings));
enableCollision(settings, moving, moving);
enableCollision(settings, moving, notMoving);
const world = createWorld(settings);
const floorShape = box.create({ halfExtents: [20, 0.5, 20] });
rigidBody.create(world, { shape: floorShape, objectLayer: notMoving, motionType: MotionType.STATIC, position: [0, -0.5, 0] });
const ballShape = sphere.create({ radius: 0.5 });

/* SNIPPET_START: observe */
// only the pairs a body observing contacts is a side of are kept: the few a game reacts to, not every contact
const ball = rigidBody.create(world, {
    shape: ballShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [0, 2, 0],
    restitution: 0.6,
    observesContacts: true,
});
/* SNIPPET_END: observe */

/* SNIPPET_START: frame */
// a frame of contact pairs: begin, record what moved (each world step, each character's update), end. a pair is
// added the frame it starts, persisted while it lasts, and removed the frame after it was last recorded
function frame(steps: number): void {
    contactPairs.begin(world);
    for (let i = 0; i < steps; i++) {
        updateWorld(world, undefined, 1 / 60 / steps);
        contactPairs.recordWorldContacts(world);
    }
    contactPairs.end(world);
}
/* SNIPPET_END: frame */

let added = 0;
let removed = 0;
const lines: string[] = [];
/* SNIPPET_START: read */
const pair = contactPairs.create();
function read(): void {
    for (let i = 0; i < contactPairs.activeCount(world); i++) {
        contactPairs.readActive(pair, world, i);
        lines.push(
            `${pair.added ? 'added' : 'persisted'}: ${pair.bodyA} touches ${pair.bodyB}, ${pair.relativeVelocity[1].toFixed(2)} m/s apart`,
        );
        /* HIDE_START */
        if (pair.added) added++;
        /* HIDE_END */
    }
    for (let i = 0; i < contactPairs.removedCount(world); i++) {
        contactPairs.readRemoved(pair, world, i);
        lines.push(`removed: ${pair.bodyA} and ${pair.bodyB}`);
        /* HIDE_START */
        removed++;
        /* HIDE_END */
    }
}
/* SNIPPET_END: read */

for (let i = 0; i < 120; i++) {
    frame(2);
    read();
}
// the ball bounced: its pair with the floor added, removed in the air, and added again
if (added < 2 || removed < 1 || !lines.some((line) => line.includes(String(ball))))
    throw new Error(`added ${added}, removed ${removed}`);

destroyWorld(world);
releaseShape(floorShape);
releaseShape(ballShape);
