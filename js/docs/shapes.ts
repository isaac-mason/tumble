import {
    Box,
    type Box3,
    Capsule,
    Compound,
    ConvexHull,
    ConvexHullBuildError,
    ConvexHullBuildResult,
    ConvexHullData,
    Cylinder,
    Empty,
    init,
    MotionType,
    OffsetCenterOfMass,
    Plane,
    RigidBody,
    Scaled,
    Shape,
    Sphere,
    StaticCompound,
    Transformed,
    TriangleMesh,
    TriangleMeshData,
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
const check = (ok: boolean, what: string): void => {
    if (!ok) throw new Error(what);
};
const near = (a: number, b: number): boolean => Math.abs(a - b) < 1e-3;

/* SNIPPET_START: convex */
// each centred on its origin; density in kg/m3 (1000 by default) gives the mass
const crate = Box.create({ halfExtents: [0.5, 0.5, 0.5], convexRadius: 0.05, density: 200 });
const ball = Sphere.create({ radius: 0.5 });
// along y: a cylinder of half height 0.5 between two hemispheres
const pill = Capsule.create({ halfHeightOfCylinder: 0.5, radius: 0.3 });
const barrel = Cylinder.create({ halfHeight: 0.6, radius: 0.4 });
Shape.getVolume(ball); // 4/3 pi r^3
/* SNIPPET_END: convex */
check(near(Shape.getVolume(ball), (4 / 3) * Math.PI * 0.125), 'volume');

/* SNIPPET_START: convex-hull */
// the hull around the points, built for this shape alone and freed with it; points inside it are dropped
const points = [-1, -1, -1, 1, -1, -1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 0, 0, 0];
const rock = ConvexHull.create({ positions: points, density: 2500 });
// or built once, as data, and shared by any number of shapes
const cubeData = ConvexHullData.build({ positions: points });
const pebble = ConvexHull.create({ data: cubeData, density: 2500 });
const boulder = ConvexHull.create({ data: cubeData, density: 3000 });
// points that make no hull throw, saying why
try {
    ConvexHullData.build({ positions: [0, 0, 0, 1, 0, 0] });
} catch (error) {
    if (error instanceof ConvexHullBuildError) error.result; // ConvexHullBuildResult.TOO_FEW_POINTS
}
/* SNIPPET_END: convex-hull */
check(near(Shape.getVolume(rock), 8) && near(Shape.getVolume(pebble), 8), 'a cube hull');
try {
    ConvexHullData.build({ positions: [0, 0, 0, 1, 0, 0] });
    check(false, 'too few points');
} catch (error) {
    check(error instanceof ConvexHullBuildError && error.result === ConvexHullBuildResult.TOO_FEW_POINTS, 'why');
}

/* SNIPPET_START: plane */
// everything below y 0 is solid; halfExtent bounds it for the broadphase
const ground = Plane.create({ normal: [0, 1, 0], constant: 0, halfExtent: 1000 });
RigidBody.create(world, { shape: ground, objectLayer: notMoving, motionType: MotionType.STATIC });
/* SNIPPET_END: plane */

/* SNIPPET_START: triangle-mesh */
// welded, its bvh made and its edges marked so things slide over the joins. a material per triangle, picking a surface
// from World.setMaterials
const terrain = TriangleMesh.create({
    positions: [-10, 0, -10, 10, 0, -10, 10, 0, 10, -10, 0, 10],
    indices: [0, 2, 1, 0, 3, 2],
    materialIds: [0, 1],
});
RigidBody.create(world, { shape: terrain, objectLayer: notMoving, motionType: MotionType.STATIC, position: [0, 5, 0] });
// as with hulls, mesh data built once is shared by the shapes over it
const tileData = TriangleMeshData.build({ positions: [0, 0, 0, 1, 0, 0, 0, 0, 1], indices: [0, 2, 1] });
const tile = TriangleMesh.create({ data: tileData });
/* SNIPPET_END: triangle-mesh */
const bounds: Box3 = { min: [0, 0, 0], max: [0, 0, 0] };
Shape.getLocalBounds(bounds, terrain);
check(bounds.max[0] === 10 && bounds.min[2] === -10, 'mesh bounds');

/* SNIPPET_START: compound */
// shapes placed in one: a table, its top and four legs. a static compound is for static bodies, with a tree over its
// children for many of them
const leg = Box.create({ halfExtents: [0.05, 0.4, 0.05] });
const top = Box.create({ halfExtents: [0.6, 0.05, 0.4] });
const table = Compound.create({
    children: [
        { shape: top, position: [0, 0.45, 0] },
        { shape: leg, position: [-0.5, 0, -0.3] },
        { shape: leg, position: [0.5, 0, -0.3] },
        { shape: leg, position: [-0.5, 0, 0.3] },
        { shape: leg, position: [0.5, 0, 0.3] },
    ],
});
const fence = StaticCompound.create({
    children: Array.from({ length: 20 }, (_, i) => ({ shape: leg, position: [i * 0.5, 0.4, 0] as [number, number, number] })),
});
/* SNIPPET_END: compound */
const placed = RigidBody.create(world, {
    shape: table,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [0, 1, 0],
});
for (let i = 0; i < 90; i++) World.step(world, undefined, 1 / 60);
const at: [number, number, number] = [0, 0, 0];
RigidBody.getPosition(at, world, placed);
// on its legs, 0.4 tall, sunk the solver's 2 cm of slop
check(Math.abs(at[1] - 0.38) < 0.002, `the table at y ${at[1]}`);

/* SNIPPET_START: decorators */
// a shape moved, scaled, or with its centre of mass offset (a weighted toy that rights itself)
const raised = Transformed.create({ shape: crate, position: [0, 1, 0], quaternion: [0, 0, 0, 1] });
const stretched = Scaled.create({ shape: crate, scale: [2, 1, 1] });
const weighted = OffsetCenterOfMass.create({ shape: pill, offset: [0, -0.4, 0] });
/* SNIPPET_END: decorators */
check(near(Shape.getVolume(stretched), 2 * Shape.getVolume(crate)), 'scaled');

/* SNIPPET_START: empty */
// no volume and no collisions: a static anchor for a constraint, or a body whose shape comes later
const nothing = Empty.create();
/* SNIPPET_END: empty */

/* SNIPPET_START: freeing */
// bodies borrow their shapes, and shapes over other shapes borrow those: free the world (or the bodies) first, then
// the shapes over other shapes, then theirs. a hull or mesh made from points frees its data; shared data goes last
World.destroy(world);
for (const shape of [raised, stretched, weighted, table, fence]) Shape.destroy(shape);
for (const shape of [crate, leg, top, ball, pill, barrel, rock, terrain, ground, nothing]) Shape.destroy(shape);
for (const shape of [pebble, boulder]) Shape.destroy(shape);
ConvexHullData.release(cubeData);
Shape.destroy(tile);
TriangleMeshData.release(tileData);
/* SNIPPET_END: freeing */
