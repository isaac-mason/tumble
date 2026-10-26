import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    CastRayStatus,
    CastShapeStatus,
    castRay,
    castShape,
    collidePoint,
    collideShape,
    createAllCastRayCollector,
    createAllCollidePointCollector,
    createAnyCastRayCollector,
    createClosestCastRayCollector,
    createClosestCastShapeCollector,
    createClosestCollideShapeCollector,
    createDefaultCastRaySettings,
    createDefaultCastShapeSettings,
    createDefaultCollidePointSettings,
    createDefaultCollideShapeSettings,
    createWorld,
    createWorldSettings,
    destroyWorld,
    enableCollision,
    filter,
    init,
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
settings.gravity = [0, 0, 0];
const world = createWorld(settings);
const crateShape = box.create({ halfExtents: [0.5, 0.5, 0.5] });
const floorShape = box.create({ halfExtents: [20, 0.5, 20] });
const floor = rigidBody.create(world, {
    shape: floorShape,
    objectLayer: notMoving,
    motionType: MotionType.STATIC,
    position: [0, -0.5, 0],
});
// two crates stacked over the floor: y 0.5 to 1.5, and 2.5 to 3.5
const lower = rigidBody.create(world, {
    shape: crateShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [0, 1, 0],
});
const upper = rigidBody.create(world, {
    shape: crateShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [0, 3, 0],
    userData: 9,
});
updateWorld(world, undefined, 1 / 60);
const check = (ok: boolean, what: string): void => {
    if (!ok) throw new Error(what);
};

/* SNIPPET_START: cast-ray */
const raySettings = createDefaultCastRaySettings();
const origin: Vec3 = [0, 10, 0];
const down: Vec3 = [0, -1, 0];

// the nearest hit, its fraction along the ray's 20 m
const closest = createClosestCastRayCollector();
castRay(world, closest, raySettings, origin, down, 20);
if (closest.hit.status === CastRayStatus.COLLIDING) {
    const distance = closest.hit.fraction * 20;
    const point: Vec3 = [origin[0], origin[1] - distance, origin[2]];
    console.log(`body ${closest.hit.bodyIdB} hit at ${point}`); // the upper crate, its top at y 3.5
}

// the first hit found, for line of sight; every hit, nearest first after sort
const any = createAnyCastRayCollector();
castRay(world, any, raySettings, origin, down, 20);
const all = createAllCastRayCollector();
castRay(world, all, raySettings, origin, down, 20);
all.sort();
all.hits.length; // 3: the crates and the floor

// a collector keeps what it had, across queries, until reset
closest.reset();
/* SNIPPET_END: cast-ray */
check(any.hit.status === CastRayStatus.COLLIDING, 'any');
check(all.hits.length === 3 && all.hits[0]!.bodyIdB === upper && all.hits[2]!.bodyIdB === floor, 'all, nearest first');

/* SNIPPET_START: collide-point */
// the bodies whose shapes hold a point
const inside = createAllCollidePointCollector();
collidePoint(world, inside, createDefaultCollidePointSettings(), [0, 1, 0]);
inside.hits[0]?.bodyIdB; // the lower crate
/* SNIPPET_END: collide-point */
check(inside.hits.length === 1 && inside.hits[0]!.bodyIdB === lower, 'the point in the lower crate');

/* SNIPPET_START: collide-shape */
// what a shape overlaps, posed and scaled: placing something, or a blast's reach. the closest is the deepest
const probe = sphere.create({ radius: 0.8 });
const deepest = createClosestCollideShapeCollector();
collideShape(world, deepest, createDefaultCollideShapeSettings(), probe, [0, 2, 0], [0, 0, 0, 1], [1, 1, 1]);
deepest.hit?.penetration; // into a crate by 0.3
/* SNIPPET_END: collide-shape */
check(deepest.hit !== null && Math.abs(deepest.hit.penetration - 0.3) < 0.01, 'overlapping a crate by 0.3');

/* SNIPPET_START: cast-shape */
// a shape swept: where a falling ball first meets something, for placing it or for a projectile's path
const sweep = createClosestCastShapeCollector();
const ball = sphere.create({ radius: 0.5 });
castShape(world, sweep, createDefaultCastShapeSettings(), ball, [0, 10, 0], [0, 0, 0, 1], [1, 1, 1], [0, -20, 0]);
if (sweep.hit.status === CastShapeStatus.COLLIDING) {
    sweep.hit.fraction; // 0.3: its centre half a metre over the upper crate's top
    sweep.hit.normal; // up, from the crate towards the ball
}
/* SNIPPET_END: cast-shape */
check(Math.abs(sweep.hit.fraction - 0.3) < 0.01 && sweep.hit.normal[1] > 0.99, 'swept onto the upper crate');

/* SNIPPET_START: filters */
// a query without a filter hits everything. a filter, held in the wasm and freed when done, starts with every layer
// of the world's enabled; then layers, groups and masks, and a body filter must all pass
const queryFilter = filter.create(world);
filter.disableObjectLayer(queryFilter, world, notMoving); // the floor's layer: crates only
queryFilter.collisionMask = ~2 >>> 0; // no body in group 2
filter.setAllEnabled(queryFilter, world); // back to everything
// what a body collides with: its layer's layers, and its groups and mask
filter.setFromBody(queryFilter, world, upper);
/* SNIPPET_END: filters */
filter.setAllEnabled(queryFilter, world);

/* SNIPPET_START: body-filter */
// your own test, for each body a query reaches: the closest it passes, not the closest there is. such a query collects
// every hit first, losing its early out, so prefer layers and masks where they do
queryFilter.bodyFilter = (body) => rigidBody.getUserData(world, body) !== 9;
closest.reset();
castRay(world, closest, raySettings, origin, down, 20, queryFilter);
closest.hit.bodyIdB; // the lower crate: the upper is filtered out
queryFilter.bodyFilter = null;
/* SNIPPET_END: body-filter */
check(closest.hit.bodyIdB === lower, 'past the filtered crate');

filter.destroy(queryFilter);
destroyWorld(world);
releaseShape(crateShape);
releaseShape(floorShape);
releaseShape(probe);
releaseShape(ball);
