<!-- rendered by docs/build.mjs from docs/js.template.md: edit that, not this -->

# tumble for JavaScript

tumble is a 3D rigid body physics engine. The JS package is the C++ library compiled to WebAssembly, with a TypeScript
API over it, and it simulates the same bits as the native library does.

Every code block here is part of a program in `js/docs/` that the tests run, and each program checks what it shows.

- [Getting tumble](#getting-tumble)
- [Quick start](#quick-start)
- [Conventions](#conventions)
- [Memory and errors](#memory-and-errors)
- [The world](#the-world)
  - [Layers](#layers)
  - [Settings](#settings)
  - [Stepping](#stepping)
  - [Materials](#materials)
- [Rigid bodies](#rigid-bodies)
  - [Creating and removing](#creating-and-removing)
  - [Motion types](#motion-types)
  - [Position and rotation](#position-and-rotation)
  - [Velocity](#velocity)
  - [Forces and impulses](#forces-and-impulses)
  - [Mass](#mass)
  - [Damping, limits and degrees of freedom](#damping-limits-and-degrees-of-freedom)
  - [Sleeping](#sleeping)
  - [Kinematic bodies](#kinematic-bodies)
  - [Continuous collision detection](#continuous-collision-detection)
  - [User data](#user-data)
  - [Collision groups and masks](#collision-groups-and-masks)
  - [Sensors](#sensors)
  - [Changing a body's shape or layer](#changing-a-bodys-shape-or-layer)
- [Shapes](#shapes)
  - [Convex shapes](#convex-shapes)
  - [Convex hulls](#convex-hulls)
  - [Planes](#planes)
  - [Triangle meshes](#triangle-meshes)
  - [Compounds](#compounds)
  - [Decorators](#decorators)
  - [Empty](#empty)
  - [Freeing shapes](#freeing-shapes)
- [Events](#events)
- [Contact pairs](#contact-pairs)
- [Queries](#queries)
  - [Rays](#rays)
  - [Points](#points)
  - [Overlaps](#overlaps)
  - [Sweeps](#sweeps)
  - [Filters](#filters)
- [Constraints](#constraints)
  - [Hinge](#hinge)
  - [Point](#point)
  - [Distance](#distance)
  - [Slider](#slider)
  - [Fixed](#fixed)
  - [Swing-twist and cone](#swing-twist-and-cone)
  - [Six degrees of freedom](#six-degrees-of-freedom)
  - [Motors and springs](#motors-and-springs)
  - [Breaking](#breaking)
- [Character controller](#character-controller)
  - [Policy](#policy)
  - [Changing shape](#changing-shape)
  - [Your own loop](#your-own-loop)
  - [Contacts](#contacts)
- [Rendering](#rendering)
  - [Poses](#poses)
  - [Debug drawing](#debug-drawing)
- [Determinism](#determinism)

## Getting tumble

The package is `tumble`: ES modules with their types, and `tumble.wasm` beside them. Load the wasm once, before anything
else, with `await init()`. With no options it loads the wasm from beside the package's modules, which suits Node and
bundlers that copy the file. Otherwise give it the wasm as a URL, its bytes or a compiled `WebAssembly.Module`, as here
with its bytes:

```ts
// the wasm given as its bytes, read here from the package in node. it loads once a realm: later calls wait for the same
const bytes = await readFile(new URL('../dist/tumble.wasm', import.meta.url));
await init({ wasm: bytes });
```

The package exports the file as `tumble/tumble.wasm` for a bundler to resolve: with Vite, import it with `?url` and pass
that URL to `init`.

## Quick start

A world, a floor, and a box and a ball dropped onto it:

```ts
import { Box, init, MotionType, RigidBody, Sphere, type Vec3, World, WorldSettings } from 'tumble';

// load the wasm, once, before anything else
await init();

// two layers, one for what moves and one for what does not: what moves collides with both
const settings = WorldSettings.create();
const moving = WorldSettings.addObjectLayer(settings, WorldSettings.addBroadphaseLayer(settings));
const notMoving = WorldSettings.addObjectLayer(settings, WorldSettings.addBroadphaseLayer(settings));
WorldSettings.enableCollision(settings, moving, moving);
WorldSettings.enableCollision(settings, moving, notMoving);
const world = World.create(settings);

// shapes are yours, shared by any number of bodies; a body is an id in its world
const floorShape = Box.create({ halfExtents: [20, 0.5, 20] });
const boxShape = Box.create({ halfExtents: [0.5, 0.5, 0.5] });
const ballShape = Sphere.create({ radius: 0.5 });
RigidBody.create(world, { shape: floorShape, objectLayer: notMoving, motionType: MotionType.STATIC, position: [0, -0.5, 0] });
const box = RigidBody.create(world, {
    shape: boxShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [0, 4, 0],
});
const ball = RigidBody.create(world, {
    shape: ballShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [2, 6, 0],
});

// two seconds at 60 Hz
for (let i = 0; i < 120; i++) World.step(world, undefined, 1 / 60);

// readers take what they fill first, and allocate nothing
const position: Vec3 = [0, 0, 0];
RigidBody.getPosition(position, world, box);
console.log(`the box rests at y ${position[1].toFixed(2)}`);

// what you create, you free: the world takes its bodies with it
World.destroy(world);
Box.destroy(floorShape);
Box.destroy(boxShape);
Sphere.destroy(ballShape);
```

The rest of this guide leaves out the imports and the world's setup.

## Conventions

- Each subject is a namespace of functions: `RigidBody.create`, `Box.create`, `HingeConstraint.setMotorState`.
- A call that reads or changes a world takes the world first, after any out: `RigidBody.setPosition(world, body, ...)`.
- A reader takes what it fills first, and returns it or whether it could: `RigidBody.getPosition(out, world, body)`. It
  allocates nothing, so keep your arrays and reuse them.
- Vectors are arrays: a `Vec3` is `[x, y, z]` and a `Quat` is `[x, y, z, w]`, the identity `[0, 0, 0, 1]`.
- Bodies and constraints are ids, plain numbers, checked on every call: a removed one's id names nothing, even once its
  slot is reused.
- Settings are object literals. Leave out what you do not need: the C++ defaults fill the rest, at every depth, so
  `motorSettings: { maxTorqueLimit: 100 }` keeps the motor's default spring.
- The simulation is in 32-bit floats: what you read back is a float32 value.
- Units are metres, kilograms, seconds and radians.

## Memory and errors

The wasm's memory is not collected. What you create, you free:

```ts
// what you create, you free: worlds (their bodies and constraints go with them), shapes and their data, filters,
// policies, characters, poses and debug buffers. a body or constraint goes with RigidBody.remove or its remove too
World.destroy(world);
Box.destroy(crateShape);
```

A call that cannot do what it was asked throws, and backs out of what it began:

```ts
// a call that cannot do what it was asked throws, having undone what it began: an id naming nothing, memory that
// cannot grow, or settings the C++ would refuse
RigidBody.remove(world, b);
try {
    HingeConstraint.create(world, a, b, {});
} catch (error) {
    (error as Error).message; // 'tumble: BAD_HANDLE'
}
try {
    HingeConstraint.create(world, a, a, { limitsMin: 1 });
} catch (error) {
    error instanceof RangeError; // true: a hinge's limitsMin is in [-pi, 0]
}
// readers and setters with an id naming nothing return false instead, doing nothing
RigidBody.setLinearVelocity(world, b, [1, 0, 0]); // false
```

A view onto the wasm's memory (from `Poses.read` or `DebugBuffers.read`) is detached when the memory grows. Take views
from their read each time rather than keeping them.

## The world

A world holds bodies, constraints and the simulation's state. Several worlds can live side by side.

### Layers

```ts
// object layers decide what collides with what; each is in a broadphase layer, a tree of its own. keep what moves and
// what does not in trees of their own
const settings = WorldSettings.create();
const movingTree = WorldSettings.addBroadphaseLayer(settings);
const staticTree = WorldSettings.addBroadphaseLayer(settings);
const moving = WorldSettings.addObjectLayer(settings, movingTree);
const debris = WorldSettings.addObjectLayer(settings, movingTree);
const scenery = WorldSettings.addObjectLayer(settings, staticTree);
WorldSettings.enableCollision(settings, moving, moving);
WorldSettings.enableCollision(settings, moving, scenery);
WorldSettings.enableCollision(settings, debris, scenery);
```

### Settings

```ts
// plain data, from the C++ defaults: change what you need. create copies them, so they can make another world
settings.gravity = [0, -20, 0];
settings.solver.velocityIterations = 12;
settings.sleeping.timeBeforeSleep = 1;
const world = World.create(settings);
```

### Stepping

`World.step(world, listener, deltaTime)` advances the world one step, its events going to the listener (see
[Events](#events)), or nowhere with `undefined`. Step by a fixed time:

```ts
// step by a fixed time, catching up a few steps when a frame runs long, so the simulation does not depend on the
// frame rate
const STEP = 1 / 60;
let pending = 0;
function frame(elapsed: number): void {
    pending = Math.min(pending + elapsed, 4 * STEP);
    while (pending >= STEP) {
        World.step(world, undefined, STEP);
        pending -= STEP;
    }
}
```

Bodies resting on others sink up to `settings.solver.penetrationSlop` (2 cm) into them: the solver allows it to keep
stacks from jittering.

### Materials

```ts
// surfaces by index: a shape's or a mesh triangle's materialId picks one, which stands in for the body's friction
// and restitution on its side of a contact
World.setMaterials(world, [
    { friction: 0, restitution: 0 },
    { friction: 1, restitution: 0.8 },
]);
const iceShape = Box.create({ halfExtents: [5, 0.1, 5], materialId: 0 });
RigidBody.create(world, { shape: iceShape, objectLayer: scenery, motionType: MotionType.STATIC, position: [10, 0.1, 0] });
```

## Rigid bodies

### Creating and removing

```ts
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
```

### Motion types

```ts
// static never moves; kinematic moves by its velocity and pushes dynamic bodies aside; dynamic is moved by gravity,
// forces and contacts. the type can change later
const platform = RigidBody.create(world, {
    shape: crateShape,
    objectLayer: moving,
    motionType: MotionType.KINEMATIC,
    position: [5, 1, 0],
});
RigidBody.setMotionType(world, platform, MotionType.DYNAMIC, true);
```

### Position and rotation

```ts
const position: Vec3 = [0, 0, 0];
const quaternion: Quat = [0, 0, 0, 1];
RigidBody.getTransform(position, quaternion, world, crate);
// a teleport: nothing between here and there stops it. the last argument wakes it
RigidBody.setTransform(world, crate, [0, 6, 0], [0, Math.SQRT1_2, 0, Math.SQRT1_2], true);
```

Teleporting a body skips the simulation in between. Move a kinematic body by velocity instead (see
[Kinematic bodies](#kinematic-bodies)).

### Velocity

```ts
RigidBody.setLinearVelocity(world, crate, [2, 0, 0]);
RigidBody.setAngularVelocity(world, crate, [0, Math.PI, 0]);
const velocity: Vec3 = [0, 0, 0];
RigidBody.getLinearVelocity(velocity, world, crate);
// the velocity of a point carried with the body, a corner here
RigidBody.getVelocityAtPoint(velocity, world, crate, [0.5, 6.5, 0.5]);
```

### Forces and impulses

```ts
// a force acts over the next step, then is cleared; an impulse changes the velocity at once. both move only dynamic
// bodies. a crate's mass is from its shape: 1000 kg/m3 by default
const mass = RigidBody.getMass(world, crate);
RigidBody.addForce(world, crate, [0, 20 * mass, 0], true);
RigidBody.addImpulseAtPosition(world, crate, [0, 0, 5 * mass], [0.5, 6, 0]);
RigidBody.addAngularImpulse(world, crate, [0, 0, 0]);
```

### Mass

```ts
// a mass the shape's inertia is scaled to, or mass properties outright: a hollow mesh has no volume to take one from
const heavy = RigidBody.create(world, {
    shape: ballShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [-5, 1, 0],
    mass: 80,
});
RigidBody.getMass(world, heavy); // 80
```

A triangle mesh has no volume to take a mass from: a dynamic body with one needs `massPropertiesOverride`.

### Damping, limits and degrees of freedom

```ts
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
```

### Sleeping

```ts
// a body at rest sleeps, costing nothing until something touches it or you wake it
steps(120);
RigidBody.isSleeping(world, heavy); // true
RigidBody.wake(world, heavy);
RigidBody.sleep(world, heavy);
// one that should never sleep: allowSleeping false
```

### Kinematic bodies

```ts
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
```

### Continuous collision detection

```ts
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
```

### User data

```ts
// your own value for a body, an integer up to 2^53: an entity's index, say. it comes back with the body in events
RigidBody.setUserData(world, crate, 2 ** 40 + 7);
RigidBody.getUserData(world, crate); // 1099511627783
```

### Collision groups and masks

```ts
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
```

### Sensors

```ts
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
```

### Changing a body's shape or layer

```ts
// a new shape: the mass, centre of mass and bounds follow, and constraints keep their anchors on the body
RigidBody.setShape(world, crate, ballShape);
```

```ts
RigidBody.setObjectLayer(world, crate, notMoving);
```

## Shapes

Shapes are created by each type's `create`, shared by any number of bodies, and freed by its `destroy` once nothing uses
them. A body's pose places its shape: a shape's own geometry is centred on its origin.

### Convex shapes

```ts
// each centred on its origin; density in kg/m3 (1000 by default) gives the mass
const crate = Box.create({ halfExtents: [0.5, 0.5, 0.5], convexRadius: 0.05, density: 200 });
const ball = Sphere.create({ radius: 0.5 });
// along y: a cylinder of half height 0.5 between two hemispheres
const pill = Capsule.create({ halfHeightOfCylinder: 0.5, radius: 0.3 });
const barrel = Cylinder.create({ halfHeight: 0.6, radius: 0.4 });
Shape.getVolume(ball); // 4/3 pi r^3
```

### Convex hulls

```ts
// a hull is built once, as data, and shared by any number of shapes; points inside it are dropped
const points = [-1, -1, -1, 1, -1, -1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 0, 0, 0];
const cubeData = ConvexHullData.build({ positions: points });
const rock = ConvexHull.create({ data: cubeData, density: 2500 });
// points that make no hull throw, saying why
try {
    ConvexHullData.build({ positions: [0, 0, 0, 1, 0, 0] });
} catch (error) {
    if (error instanceof ConvexHullBuildError) error.result; // ConvexHullBuildResult.TOO_FEW_POINTS
}
```

### Planes

```ts
// everything below y 0 is solid; halfExtent bounds it for the broadphase
const ground = Plane.create({ normal: [0, 1, 0], constant: 0, halfExtent: 1000 });
RigidBody.create(world, { shape: ground, objectLayer: notMoving, motionType: MotionType.STATIC });
```

### Triangle meshes

```ts
// a mesh is built once, as data: welded, its bvh made and its edges marked so things slide over the joins. a material
// per triangle, picking a surface from World.setMaterials
const terrainData = TriangleMeshData.build({
    positions: [-10, 0, -10, 10, 0, -10, 10, 0, 10, -10, 0, 10],
    indices: [0, 2, 1, 0, 3, 2],
    materialIds: [0, 1],
});
const terrain = TriangleMesh.create({ data: terrainData });
RigidBody.create(world, { shape: terrain, objectLayer: notMoving, motionType: MotionType.STATIC, position: [0, 5, 0] });
```

Meshes are for static bodies: they are hollow, with no volume, no inside and no mass.

### Compounds

```ts
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
```

### Decorators

```ts
// a shape moved, scaled, or with its centre of mass offset (a weighted toy that rights itself)
const raised = Transformed.create({ shape: crate, position: [0, 1, 0], quaternion: [0, 0, 0, 1] });
const stretched = Scaled.create({ shape: crate, scale: [2, 1, 1] });
const weighted = OffsetCenterOfMass.create({ shape: pill, offset: [0, -0.4, 0] });
```

### Empty

```ts
// no volume and no collisions: a static anchor for a constraint, or a body whose shape comes later
const nothing = Empty.create();
```

### Freeing shapes

```ts
// bodies borrow their shapes, and shapes their data: free the world (or the bodies) first, then the shapes over
// other shapes, then theirs, then the data
World.destroy(world);
Transformed.destroy(raised);
Scaled.destroy(stretched);
OffsetCenterOfMass.destroy(weighted);
Compound.destroy(table);
StaticCompound.destroy(fence);
for (const box of [crate, leg, top]) Box.destroy(box);
Sphere.destroy(ball);
Capsule.destroy(pill);
Cylinder.destroy(barrel);
ConvexHull.destroy(rock);
ConvexHullData.release(cubeData);
TriangleMesh.destroy(terrain);
TriangleMeshData.release(terrainData);
Plane.destroy(ground);
Empty.destroy(nothing);
```

## Events

A step's listener hears its events and answers them as it runs. The wasm never calls into JS: `World.step` runs the
step's phases and calls the hooks between them, so a step with no hooks is one call into the wasm.

```ts
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
```

```ts
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
```

## Contact pairs

The listener hears every contact of every step. Contact pairs keep the few you care about, between frames: each pair of
body parts in contact, with a stable handle while the pair lasts, added, persisted or removed each frame. A frame can
span several world steps and the characters' updates between them.

```ts
// only the pairs a body observing contacts is a side of are kept: the few a game reacts to, not every contact
const ball = RigidBody.create(world, {
    shape: ballShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [0, 2, 0],
    restitution: 0.6,
    observesContacts: true,
});
```

```ts
// a frame of contact pairs: begin, record what moved (each world step, each character's update), end. a pair is
// added the frame it starts, persisted while it lasts, and removed the frame after it was last recorded
function frame(steps: number): void {
    ContactPairs.begin(world);
    for (let i = 0; i < steps; i++) {
        World.step(world, undefined, 1 / 60 / steps);
        ContactPairs.recordWorldContacts(world);
    }
    ContactPairs.end(world);
}
```

```ts
const pair = ContactPairs.create();
function read(): void {
    for (let i = 0; i < ContactPairs.activeCount(world); i++) {
        ContactPairs.readActive(pair, world, i);
        lines.push(
            `${pair.added ? 'added' : 'persisted'}: ${pair.bodyA} touches ${pair.bodyB}, ${pair.relativeVelocity[1].toFixed(2)} m/s apart`,
        );
    }
    for (let i = 0; i < ContactPairs.removedCount(world); i++) {
        ContactPairs.readRemoved(pair, world, i);
        lines.push(`removed: ${pair.bodyA} and ${pair.bodyB}`);
    }
}
```

## Queries

Each query walks the world under a filter and reports to a collector: the closest hit, any hit (stopping at the first),
or every hit.

### Rays

```ts
// what a query may hit: every layer of the world's, until told otherwise. free it when done
const filter = Filter.create(world);
const raySettings = createDefaultCastRaySettings();
const origin: Vec3 = [0, 10, 0];
const down: Vec3 = [0, -1, 0];

// the nearest hit, its fraction along the ray's 20 m
const closest = createClosestCastRayCollector();
castRay(world, closest, raySettings, origin, down, 20, filter);
if (closest.hit.status === CastRayStatus.COLLIDING) {
    const distance = closest.hit.fraction * 20;
    const point: Vec3 = [origin[0], origin[1] - distance, origin[2]];
    console.log(`body ${closest.hit.bodyIdB} hit at ${point}`); // the upper crate, its top at y 3.5
}

// the first hit found, for line of sight; every hit, nearest first after sort
const any = createAnyCastRayCollector();
castRay(world, any, raySettings, origin, down, 20, filter);
const all = createAllCastRayCollector();
castRay(world, all, raySettings, origin, down, 20, filter);
all.sort();
all.hits.length; // 3: the crates and the floor

// a collector keeps what it had, across queries, until reset
closest.reset();
```

### Points

```ts
// the bodies whose shapes hold a point
const inside = createAllCollidePointCollector();
collidePoint(world, inside, createDefaultCollidePointSettings(), [0, 1, 0], filter);
inside.hits[0]?.bodyIdB; // the lower crate
```

### Overlaps

```ts
// what a shape overlaps, posed and scaled: placing something, or a blast's reach. the closest is the deepest
const probe = Sphere.create({ radius: 0.8 });
const deepest = createClosestCollideShapeCollector();
collideShape(world, deepest, createDefaultCollideShapeSettings(), probe, [0, 2, 0], [0, 0, 0, 1], [1, 1, 1], filter);
deepest.hit?.penetration; // into a crate by 0.3
```

### Sweeps

```ts
// a shape swept: where a falling ball first meets something, for placing it or for a projectile's path
const sweep = createClosestCastShapeCollector();
const ball = Sphere.create({ radius: 0.5 });
castShape(world, sweep, createDefaultCastShapeSettings(), ball, [0, 10, 0], [0, 0, 0, 1], [1, 1, 1], [0, -20, 0], filter);
if (sweep.hit.status === CastShapeStatus.COLLIDING) {
    sweep.hit.fraction; // 0.3: its centre half a metre over the upper crate's top
    sweep.hit.normal; // up, from the crate towards the ball
}
```

### Filters

```ts
// layers first, then groups and masks, then a body filter: all must pass
Filter.disableObjectLayer(filter, world, notMoving); // the floor's layer: crates only
filter.collisionMask = ~2 >>> 0; // no body in group 2
Filter.setAllEnabled(filter, world); // back to everything
// what a body collides with: its layer's layers, and its groups and mask
Filter.setFromBody(filter, world, upper);
```

```ts
// your own test, for each body a query reaches: the closest it passes, not the closest there is. such a query collects
// every hit first, losing its early out, so prefer layers and masks where they do
filter.bodyFilter = (body) => RigidBody.getUserData(world, body) !== 9;
closest.reset();
castRay(world, closest, raySettings, origin, down, 20, filter);
closest.hit.bodyIdB; // the lower crate: the upper is filtered out
filter.bodyFilter = null;
```

## Constraints

A constraint holds two bodies relative to each other. Each type's `create(world, bodyA, bodyB, settings)` returns its
id, which its functions take. A constraint goes with either of its bodies, or with its type's `remove`.

Points and axes are in world space by default. With `space: ConstraintSpace.LOCAL` they are relative to each body.

### Hinge

```ts
// a door on a hinge along y at its left edge, opening a quarter turn at most, driven open by its motor. a motor's
// torque is unbounded unless limited, and an unbounded motor driving into a limit pushes past it: give it a limit
const frame = anchor([0, 1, 0]);
const door = dynamic(doorShape, [0.5, 1, 0]);
const hinge = HingeConstraint.create(world, frame, door, {
    pointA: [0, 1, 0],
    pointB: [0, 1, 0],
    hingeAxisA: [0, 1, 0],
    hingeAxisB: [0, 1, 0],
    normalAxisA: [1, 0, 0],
    normalAxisB: [1, 0, 0],
    limitsMin: 0,
    limitsMax: 0.5 * Math.PI,
    motorSettings: { minTorqueLimit: -100, maxTorqueLimit: 100 },
});
HingeConstraint.setMotorState(world, hinge, MotorState.VELOCITY);
HingeConstraint.setTargetAngularVelocity(world, hinge, 2);
```

```ts
// gone, or with either body: a stale id names nothing
HingeConstraint.remove(world, hinge);
Constraint.exists(world, hinge); // false
```

### Point

```ts
// a pendulum: a ball held 2 m from the point it swings about
const pivot = anchor([10, 5, 0]);
const bob = dynamic(ballShape, [12, 5, 0]);
PointConstraint.create(world, pivot, bob, { pointA: [10, 5, 0], pointB: [10, 5, 0] });
```

### Distance

```ts
// a bungee: no further than 3 m, pulled back by a spring; minDistance and maxDistance of -1 keep where it starts
const jumper = dynamic(ballShape, [14, 5, 0]);
DistanceConstraint.create(world, pivot, jumper, {
    pointA: [10, 5, 0],
    pointB: [14, 5, 0],
    minDistance: 0,
    maxDistance: 3,
    springSettings: { mode: SpringMode.FREQUENCY_AND_DAMPING, frequencyOrStiffness: 2, damping: 0.5 },
});
```

### Slider

```ts
// a lift: a platform that slides along y only, its position motor holding it 2 m up the rail. a position motor drives
// through a spring, which a load sags: this one is stiff enough (10 Hz) to sag by millimetres
const rail = anchor([20, 0, 0]);
const platform = dynamic(blockShape, [20, 0, 0]);
const lift = SliderConstraint.create(world, rail, platform, {
    sliderAxisA: [0, 1, 0],
    sliderAxisB: [0, 1, 0],
    normalAxisA: [1, 0, 0],
    normalAxisB: [1, 0, 0],
    autoDetectPoint: true,
    limitsMin: 0,
    limitsMax: 5,
    motorSettings: { spring: { frequencyOrStiffness: 10, damping: 1 } },
});
SliderConstraint.setMotorState(world, lift, MotorState.POSITION);
SliderConstraint.setTargetPosition(world, lift, 2);
```

### Fixed

```ts
// a weld: two bodies held as they are, moving as one
const left = dynamic(blockShape, [30, 3, 0]);
const right = dynamic(blockShape, [31, 3, 0]);
FixedConstraint.create(world, left, right, { autoDetectPoint: true });
```

### Swing-twist and cone

```ts
// a shoulder: the arm swings within a 45 degree cone about the shoulder's twist axis (x here), twisting up to 30
// degrees either way. a cone constraint is the same without the twist limits
const torso = anchor([40, 3, 0]);
const arm = dynamic(ballShape, [40.5, 3, 0]);
SwingTwistConstraint.create(world, torso, arm, {
    pointA: [40, 3, 0],
    pointB: [40, 3, 0],
    twistAxisA: [1, 0, 0],
    planeAxisA: [0, 0, 1],
    twistAxisB: [1, 0, 0],
    planeAxisB: [0, 0, 1],
    normalHalfConeAngle: 0.25 * Math.PI,
    planeHalfConeAngle: 0.25 * Math.PI,
    twistMinAngle: -Math.PI / 6,
    twistMaxAngle: Math.PI / 6,
});
```

```ts
ConeConstraint.create(world, coneAnchor, coneArm, {
    pointA: [40, 6, 0],
    twistAxisA: [1, 0, 0],
    pointB: [40, 6, 0],
    twistAxisB: [1, 0, 0],
    halfConeAngle: 0.25 * Math.PI,
});
```

### Six degrees of freedom

```ts
// every axis its own: here free to slide along x within a metre either way and to turn about y, the rest locked
const base = anchor([50, 3, 0]);
const cart = dynamic(blockShape, [50, 3, 0]);
const limitMin: SixDOFValues = [0, 0, 0, 0, 0, 0];
const limitMax: SixDOFValues = [0, 0, 0, 0, 0, 0];
limitMin[SixDOFAxis.TRANSLATION_X] = -1;
limitMax[SixDOFAxis.TRANSLATION_X] = 1;
limitMin[SixDOFAxis.ROTATION_Y] = -Math.PI;
limitMax[SixDOFAxis.ROTATION_Y] = Math.PI;
SixDOFConstraint.create(world, base, cart, { pointA: [50, 3, 0], pointB: [50, 3, 0], limitMin, limitMax });
RigidBody.setLinearVelocity(world, cart, [5, 0, 0]);
```

### Motors and springs

A motor drives a hinge, slider, swing-twist or six-dof axis. `MotorState.VELOCITY` drives to a target velocity;
`POSITION` drives to a target position through the motor's spring. A motor's torque and force are unbounded unless its
`motorSettings` limit them. Limit one that drives into a limit, or it pushes past the limit. A spring's
`frequencyOrStiffness` above 0 makes a limit or a distance soft. Settings the C++ would refuse, such as a hinge's
limits outside [-pi, pi], throw a `RangeError`.

### Breaking

```ts
// a joint that breaks: read the impulse it applied each step, and disable it past a strength. a disabled constraint
// holds nothing and joins nothing, until enabled again
const hook = anchor([60, 5, 0]);
const weight = RigidBody.create(world, {
    shape: blockShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [60, 4, 0],
    mass: 50,
});
const rope = PointConstraint.create(world, hook, weight, { pointA: [60, 5, 0], pointB: [60, 5, 0] });
const lambda: Vec3 = [0, 0, 0];
const STRENGTH = 15; // N s a step: 50 kg hangs at about 8
for (let i = 0; i < 120; i++) {
    if (i === 60) RigidBody.addImpulse(world, weight, [0, -2000, 0]);
    World.step(world, undefined, 1 / 60);
    PointConstraint.getTotalLambda(lambda, world, rope);
    if (Math.hypot(...lambda) > STRENGTH) Constraint.setEnabled(world, rope, false);
}
```

## Character controller

`KCC` moves a character through the world by sweeps: it walks up slopes and stairs, slides along walls, sticks to the
floor going down, and stands on moving bodies. It is not a rigid body. Its optional inner body, a kinematic body that
follows it, is its presence in the world for bodies, queries and contact pairs. The character is an object: its
`position`, `quaternion`, `ground` and `contacts` are as its last call left them, and `linearVelocity` is yours to set.

```ts
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
```

```ts
// how characters treat the bodies they meet, by body, the rest at the defaults; one policy serves any number of
// characters. here the crate may be pushed (until the character crouches, below)
const policy = KCCPolicy.create();
KCCPolicy.set(policy, crate, { canReceiveImpulses: true });
```

```ts
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
```

`character.ground` says what it stands on: its `state` (`ON_GROUND`, `ON_STEEP_GROUND`, `NOT_SUPPORTED`, `IN_AIR`), its
`bodyId`, the `normal` and the ground's `velocity`.

### Policy

A body's policy: `ignore` passes through it, `canPushCharacter` false blocks without moving the character,
`canReceiveImpulses` false keeps the character from pushing it, `slideOnWalkable` false stops the character on a still,
walkable surface of it, and `surfaceLinearVelocity` and `surfaceAngularVelocity` move the character as a conveyor's
belt would.

### Changing shape

```ts
// a shorter shape on the same feet, where it fits: refused, false, if it would sink deeper than the given depth
const crouchingCapsule = Capsule.create({ halfHeightOfCylinder: 0.2, radius: 0.3 });
const crouching = Transformed.create({ shape: crouchingCapsule, position: [0, 0.5, 0] });
if (KCC.setShape(world, character, crouching, filter, policy, 0.01)) KCCPolicy.set(policy, crate, { canReceiveImpulses: false });
```

Give every shape the character's feet as its origin, and swap them freely.

```ts
// its settings, tuned after create: here a lower point of support while crouched
KCC.setTuning(character, { supportingVolumePlane: [0, 1, 0, -0.3] });
```

### Your own loop

```ts
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
const hit: KCCSweepContact = createSweepContact();
const normal: Vec3 = [0, 0, 0];
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
        const rows = character.contacts;
        for (let row = 0; row < rows.count; row++) {
            if (fromBelow(rows.bodyId(row), rows.contactNormal(normal, row))) rows.setDiscarded(row, true);
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
```

`collide`, `solve` and `cast` are a move's phases, as `move` runs them each collision iteration. Between `collide` and
`solve`, a row of `character.contacts` is yours to discard (`setDiscarded`) or give another velocity
(`setLinearVelocity`): the solve, the sweep and the ground pass over a discarded one. `update` is the same phases with
contact tracking (`beginContactTracking`, `endContactTracking`), `cancelVelocityTowardsSteepSlopes` before, and
`stickToFloor` and `walkStairs` after. A character's queries run in the wasm, so a filter's `bodyFilter` cannot reach
them: set `ignore` in its policy instead.

### Contacts

```ts
// what the character met in its last update (or refreshContacts, or setShape): each contact added or persisted as it
// was first touched, and each gone by the end
for (const event of character.contactEvents) {
    if (event.kind === KCCContactEventKind.ADDED) event.bodyId; // footsteps, say
}
```

```ts
// its contacts as data, through its inner body, in a contact pairs frame
ContactPairs.begin(world);
KCC.refreshContacts(world, character, filter, policy);
ContactPairs.recordCharacterContacts(world, character);
ContactPairs.end(world);
```

```ts
KCC.remove(world, character);
KCC.destroy(character);
KCCPolicy.destroy(policy);
```

## Rendering

### Poses

```ts
// a renderer's bodies, each in a slot of its own: one read a frame brings every slot to its body's pose, in arrays
// straight onto the wasm's memory, and lists the slots that changed, so only their meshes need updating
const poses = Poses.create(world);
const meshOf = new Map<number, { position: number[]; quaternion: number[] }>();
for (const body of balls) meshOf.set(Poses.add(poses, body), { position: [0, 0, 0], quaternion: [0, 0, 0, 1] });

function render(): void {
    const { positions, quaternions, changed } = Poses.read(poses);
    for (const slot of changed) {
        const mesh = meshOf.get(slot)!;
        mesh.position = Array.from(positions.subarray(slot * 3, slot * 3 + 3));
        mesh.quaternion = Array.from(quaternions.subarray(slot * 4, slot * 4 + 4));
    }
}
```

### Debug drawing

```ts
// wireframes drawn into buffers kept in the wasm: clear, draw, read. the views are x y z r g b a vertex, two a line,
// three a triangle, straight onto the wasm's memory: copy them out before the next draw
const buffers = DebugBuffers.create();
let gpuLines = new Float32Array(1 << 16);
function drawDebug(): number {
    DebugBuffers.clear(buffers);
    Debug.bodies(buffers, world, { showLinearVelocity: true });
    Debug.contactConstraints(buffers, world);
    Debug.joints(buffers, world);
    const { lines, lineCount } = DebugBuffers.read(buffers);
    if (lines.length > gpuLines.length) gpuLines = new Float32Array(2 * lines.length);
    gpuLines.set(lines);
    return lineCount;
}
```

`Debug.characterContacts` draws a character's contacts, `Debug.shape` a shape at a pose, and `Debug.line` a line.

## Determinism

```ts
// the same scene, made in the same order and stepped by the same steps, is the same to the last bit: in another
// world, another run, another machine, or natively in C++
const first = makePile(groundShape, ballShape);
const second = makePile(groundShape, ballShape);
for (let i = 0; i < 300; i++) {
    World.step(first.world, undefined, 1 / 60);
    World.step(second.world, undefined, 1 / 60);
}
const a: Vec3 = [0, 0, 0];
const b: Vec3 = [0, 0, 0];
const same = first.bodies.every((body, i) => {
    RigidBody.getPosition(a, first.world, body);
    RigidBody.getPosition(b, second.world, second.bodies[i]!);
    return a.every((value, axis) => Object.is(value, b[axis]));
});
console.log(same ? 'bit for bit the same' : 'they differ');
```

The wasm is built as the native library is: no fused multiply-add, no fast math, and its own trigonometry. So the same
steps give the same bits in the browser, in Node and natively. The tests hold the JS package to the native library's
hash of a simulated pile. To keep it, create bodies and constraints, and step, in the same order with the same values:
a number crosses into the wasm as a float32, so a value computed in JS (in 64 bits) may round differently from the same
sum in C++.
