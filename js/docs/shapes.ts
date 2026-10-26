import {
    addBroadphaseLayer,
    addObjectLayer,
    type Box3,
    box,
    ConvexHullBuildError,
    ConvexHullBuildResult,
    capsule,
    compound,
    convexHull,
    createWorld,
    createWorldSettings,
    cylinder,
    destroyWorld,
    emptyShape,
    enableCollision,
    getShapeLocalBounds,
    getShapeVolume,
    init,
    MotionType,
    offsetCenterOfMass,
    plane,
    releaseShape,
    rigidBody,
    scaled,
    sphere,
    staticCompound,
    transformed,
    triangleMesh,
    updateWorld,
} from 'tumble.js';

await init();
const settings = createWorldSettings();
const moving = addObjectLayer(settings, addBroadphaseLayer(settings));
const notMoving = addObjectLayer(settings, addBroadphaseLayer(settings));
enableCollision(settings, moving, moving);
enableCollision(settings, moving, notMoving);
const world = createWorld(settings);
const check = (ok: boolean, what: string): void => {
    if (!ok) throw new Error(what);
};
const near = (a: number, b: number): boolean => Math.abs(a - b) < 1e-3;

/* SNIPPET_START: convex */
// each shape is centred on its origin. its density in kg/m3 gives the mass, and defaults to 1000
const crate = box.create({ halfExtents: [0.5, 0.5, 0.5], convexRadius: 0.05, density: 200 });
const ball = sphere.create({ radius: 0.5 });
// along y. a cylinder with half height 0.5 between two hemispheres
const pill = capsule.create({ halfHeightOfCylinder: 0.5, radius: 0.3 });
const barrel = cylinder.create({ halfHeight: 0.6, radius: 0.4 });
getShapeVolume(ball); // 4/3 pi r^3
/* SNIPPET_END: convex */
check(near(getShapeVolume(ball), (4 / 3) * Math.PI * 0.125), 'volume');

/* SNIPPET_START: convex-hull */
// the convex hull around the points. the shape builds it and frees it with itself. points inside the hull are
// dropped. any number of bodies can share the shape
const points = [-1, -1, -1, 1, -1, -1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 0, 0, 0];
const rock = convexHull.create({ positions: points, density: 2500 });
// points that make no hull throw an error that says why
try {
    convexHull.create({ positions: [0, 0, 0, 1, 0, 0] });
} catch (error) {
    if (error instanceof ConvexHullBuildError) error.result; // ConvexHullBuildResult.TOO_FEW_POINTS
}
/* SNIPPET_END: convex-hull */
check(near(getShapeVolume(rock), 8), 'a cube hull');
try {
    convexHull.create({ positions: [0, 0, 0, 1, 0, 0] });
    check(false, 'too few points');
} catch (error) {
    check(error instanceof ConvexHullBuildError && error.result === ConvexHullBuildResult.TOO_FEW_POINTS, 'why');
}

/* SNIPPET_START: plane */
// everything below y 0 is solid. halfExtent bounds it for the broadphase
const ground = plane.create({ normal: [0, 1, 0], constant: 0, halfExtent: 1000 });
rigidBody.create(world, { shape: ground, objectLayer: notMoving, motionType: MotionType.STATIC });
/* SNIPPET_END: plane */

/* SNIPPET_START: triangle-mesh */
// the mesh is welded, gets a bvh, and marks its edges so things slide over the joins. each triangle can pick a
// material from setMaterials
const terrain = triangleMesh.create({
    positions: [-10, 0, -10, 10, 0, -10, 10, 0, 10, -10, 0, 10],
    indices: [0, 2, 1, 0, 3, 2],
    materialIds: [0, 1],
});
rigidBody.create(world, { shape: terrain, objectLayer: notMoving, motionType: MotionType.STATIC, position: [0, 5, 0] });
/* SNIPPET_END: triangle-mesh */
const bounds: Box3 = { min: [0, 0, 0], max: [0, 0, 0] };
getShapeLocalBounds(bounds, terrain);
check(bounds.max[0] === 10 && bounds.min[2] === -10, 'mesh bounds');

/* SNIPPET_START: compound */
// several shapes placed as one, here a table top and four legs. a static compound suits static bodies with many
// children, and builds a tree over them
const leg = box.create({ halfExtents: [0.05, 0.4, 0.05] });
const top = box.create({ halfExtents: [0.6, 0.05, 0.4] });
const table = compound.create({
    children: [
        { shape: top, position: [0, 0.45, 0] },
        { shape: leg, position: [-0.5, 0, -0.3] },
        { shape: leg, position: [0.5, 0, -0.3] },
        { shape: leg, position: [-0.5, 0, 0.3] },
        { shape: leg, position: [0.5, 0, 0.3] },
    ],
});
const fence = staticCompound.create({
    children: Array.from({ length: 20 }, (_, i) => ({ shape: leg, position: [i * 0.5, 0.4, 0] as [number, number, number] })),
});
/* SNIPPET_END: compound */
const placed = rigidBody.create(world, {
    shape: table,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [0, 1, 0],
});
for (let i = 0; i < 90; i++) updateWorld(world, undefined, 1 / 60);
const at: [number, number, number] = [0, 0, 0];
rigidBody.getPosition(at, world, placed);
// on its legs, 0.4 tall, sunk the solver's 2 cm of slop
check(Math.abs(at[1] - 0.38) < 0.002, `the table at y ${at[1]}`);

/* SNIPPET_START: decorators */
// a shape moved, scaled, or with its centre of mass offset, such as a weighted toy that rights itself
const raised = transformed.create({ shape: crate, position: [0, 1, 0], quaternion: [0, 0, 0, 1] });
const stretched = scaled.create({ shape: crate, scale: [2, 1, 1] });
const weighted = offsetCenterOfMass.create({ shape: pill, offset: [0, -0.4, 0] });
/* SNIPPET_END: decorators */
check(near(getShapeVolume(stretched), 2 * getShapeVolume(crate)), 'scaled');

/* SNIPPET_START: empty */
// no volume and no collisions. use it as a static constraint anchor, or for a body whose shape comes later
const nothing = emptyShape.create();
/* SNIPPET_END: empty */

/* SNIPPET_START: freeing */
// a shape built from another holds it, so you can release shapes in any order, before or after the world
for (const shape of [crate, leg, top, ball, pill, barrel, rock, terrain, ground, nothing]) releaseShape(shape);
for (const shape of [raised, stretched, weighted, table, fence]) releaseShape(shape);
destroyWorld(world);
/* SNIPPET_END: freeing */
