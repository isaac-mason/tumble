<!-- rendered by docs/build.mjs from docs/js.template.md: edit that, not this -->

[![Version](https://img.shields.io/npm/v/tumble.js?style=for-the-badge)](https://www.npmjs.com/package/tumble.js)
![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/isaac-mason/tumble/release.yml?style=for-the-badge)

```bash
> npm install tumble.js
```

# tumble.js

tumble.js is a 3D physics engine for JavaScript, built for games, simulations and creative websites. It runs in
WebAssembly, with a TypeScript API.

**Features**

- rigid bodies, with sleeping, sensors and continuous collision detection
- spheres, boxes, capsules, cylinders, convex hulls, triangle meshes, planes and compounds
- constraints with motors and springs: hinge, slider, distance, point, fixed, cone, swing-twist and six-dof
- a character controller that walks up slopes and stairs, slides along walls and rides moving bodies
- ray, point, overlap and sweep queries, with collision filtering
- hooks that read and change contacts during a step
- contact pairs that track only the contacts you care about
- deterministic: the same steps give the same results in the browser, in Node and in the native library
- getters fill arrays you keep, so reading state allocates nothing
- works with three.js, babylon.js, playcanvas or your own engine

**Examples**

- [Falling shapes](https://isaac-mason.github.io/tumble/example-falling-shapes.html)
- [Character](https://isaac-mason.github.io/tumble/example-character.html)
- [Debug drawing](https://isaac-mason.github.io/tumble/example-debug-drawing.html)

**Documentation**

This README is the guide. Every code block comes from a program in `js/docs/`, and the tests run each one.

tumble is also a C++ library, with its own guide at
[github.com/isaac-mason/tumble](https://github.com/isaac-mason/tumble#readme).

- [Quick start](#quick-start)
- [Loading the wasm](#loading-the-wasm)
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
  - [Active bodies](#active-bodies)
  - [Debug drawing](#debug-drawing)
- [Determinism](#determinism)
- [From crashcat](#from-crashcat)

## Quick start

This creates a world and a floor, then drops a box and a ball onto it:

```ts
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
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
    type Vec3,
} from 'tumble.js';

// load the wasm once, before anything else
await init();

// one layer for what moves and one for what does not. moving bodies collide with both
const settings = createWorldSettings();
const moving = addObjectLayer(settings, addBroadphaseLayer(settings));
const notMoving = addObjectLayer(settings, addBroadphaseLayer(settings));
enableCollision(settings, moving, moving);
enableCollision(settings, moving, notMoving);
const world = createWorld(settings);

// you own the shapes, and any number of bodies can share them. a body is an id in its world
const floorShape = box.create({ halfExtents: [20, 0.5, 20] });
const boxShape = box.create({ halfExtents: [0.5, 0.5, 0.5] });
const ballShape = sphere.create({ radius: 0.5 });
rigidBody.create(world, { shape: floorShape, objectLayer: notMoving, motionType: MotionType.STATIC, position: [0, -0.5, 0] });
const boxBody = rigidBody.create(world, {
    shape: boxShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [0, 4, 0],
});
const ball = rigidBody.create(world, {
    shape: ballShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [2, 6, 0],
});

// two seconds at 60 Hz
for (let i = 0; i < 120; i++) updateWorld(world, undefined, 1 / 60);

// getters take the array they fill first, and allocate nothing
const position: Vec3 = [0, 0, 0];
rigidBody.getPosition(position, world, boxBody);
console.log(`the box rests at y ${position[1].toFixed(2)}`);

// free what you create. destroying the world frees its bodies
destroyWorld(world);
releaseShape(floorShape);
releaseShape(boxShape);
releaseShape(ballShape);
```

The rest of this guide leaves out the imports and the world setup.

## Loading the wasm

The package holds ES modules, their types, and `tumble.wasm`.

Load the wasm once, before anything else, with `await init()`. With no options, `init` loads the wasm from next to the
package's modules. This works in Node and in bundlers that copy the file. You can also pass the wasm as a URL, as bytes
or as a compiled `WebAssembly.Module`. This example passes bytes:

```ts
// pass the wasm as bytes, here read from the package in node. it loads once per realm. later calls wait for that load
const bytes = await readFile(new URL('../dist/tumble.wasm', import.meta.url));
await init({ wasm: bytes });
```

The package exports the wasm as `tumble.js/tumble.wasm`. With Vite, import it with `?url` and pass the URL to `init`.

## Conventions

- Each subject is a namespace of functions, such as `rigidBody.create`, `box.create` and
  `hingeConstraint.setMotorState`.
- World functions and queries are not in a namespace. Examples are `createWorld`, `updateWorld`, `setGravity`,
  `castRay` and `collideShape`.
- A function that reads or changes a world takes the world first, after any out parameter:
  `rigidBody.setPosition(world, body, ...)`.
- A getter takes the object it fills first, and returns it or a success flag: `rigidBody.getPosition(out, world, body)`.
  Getters allocate nothing, so keep your arrays and reuse them.
- Vectors are arrays. A `Vec3` is `[x, y, z]`. A `Quat` is `[x, y, z, w]`, and the identity is `[0, 0, 0, 1]`.
- Bodies and constraints are ids. An id is a plain number, and every call checks it. The id of a removed body names
  nothing, even after its slot is reused.
- Settings are object literals. Leave out what you do not need, and the defaults fill in the rest at every depth. For
  example, `motorSettings: { maxTorqueLimit: 100 }` keeps the motor's default spring.
- The simulation uses 32-bit floats. Values you read back are float32 values.
- Units are metres, kilograms, seconds and radians.

## Memory and errors

The garbage collector does not free wasm memory. Free what you create:

```ts
// free what you create: worlds, filters, policies and debug draws. a world frees its bodies and constraints. you can
// also remove them with rigidBody.remove and constraints.remove. release the shapes and characters you make. anything
// else that holds them lets go by itself
destroyWorld(world);
releaseShape(crateShape);
```

A call that cannot do what you asked throws, and undoes anything it started:

```ts
// a call that cannot do what you asked throws, and undoes anything it started. causes include a stale id or invalid
// settings
rigidBody.remove(world, b);
try {
    hingeConstraint.create(world, a, b, {});
} catch (error) {
    (error as Error).message; // 'tumble: BAD_HANDLE'
}
try {
    hingeConstraint.create(world, a, a, { limitsMin: 1 });
} catch (error) {
    error instanceof RangeError; // true: a hinge's limitsMin is in [-pi, 0]
}
// getters and setters given a stale id return false and do nothing instead
rigidBody.setLinearVelocity(world, b, [1, 0, 0]); // false
```

Running out of memory is different. The wasm stops, the call throws a `WebAssembly.RuntimeError`, and the instance
cannot be used again.

A view into wasm memory, such as one from `debugDraw.read`, breaks when the memory grows. Read new views each time
instead of keeping them.

## The world

A world holds bodies, constraints and the simulation state. You can run several worlds at once.

### Layers

```ts
// object layers decide what collides with what. each object layer belongs to a broadphase layer, which has its own
// tree. put moving and still bodies in separate broadphase layers
const settings = createWorldSettings();
const movingTree = addBroadphaseLayer(settings);
const staticTree = addBroadphaseLayer(settings);
const moving = addObjectLayer(settings, movingTree);
const debris = addObjectLayer(settings, movingTree);
const scenery = addObjectLayer(settings, staticTree);
enableCollision(settings, moving, moving);
enableCollision(settings, moving, scenery);
enableCollision(settings, debris, scenery);
```

### Settings

```ts
// settings are plain data with the defaults filled in. change what you need. createWorld copies them, so they can
// make another world
settings.gravity = [0, -20, 0];
settings.solver.velocityIterations = 12;
settings.sleeping.timeBeforeSleep = 1;
const world = createWorld(settings);
```

### Stepping

`updateWorld(world, listener, deltaTime)` advances the world by one step. It sends the step's events to the listener
(see [Events](#events)). Pass `undefined` to ignore them. Step by a fixed time:

```ts
// step by a fixed time, and catch up a few steps when a frame runs long. then the simulation does not depend on the
// frame rate
const STEP = 1 / 60;
let pending = 0;
function frame(elapsed: number): void {
    pending = Math.min(pending + elapsed, 4 * STEP);
    while (pending >= STEP) {
        updateWorld(world, undefined, STEP);
        pending -= STEP;
    }
}
```

A resting body can sink into the body below by up to `settings.solver.penetrationSlop`, which is 2 cm. This keeps
stacks from jittering.

### Materials

```ts
// materials by index. a shape's or a mesh triangle's materialId picks one. it replaces the body's friction and
// restitution on its side of a contact
setMaterials(world, [
    { friction: 0, restitution: 0 },
    { friction: 1, restitution: 0.8 },
]);
const iceShape = box.create({ halfExtents: [5, 0.1, 5], materialId: 0 });
rigidBody.create(world, { shape: iceShape, objectLayer: scenery, motionType: MotionType.STATIC, position: [10, 0.1, 0] });
```

A query hit names the part it hit with a sub shape id. `rigidBody.getMaterial(world, hit.bodyIdB, hit.subShapeId)`
returns that part's material id.

## Rigid bodies

### Creating and removing

```ts
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
```

### Motion types

```ts
// a static body never moves. a kinematic body moves by its velocity and pushes dynamic bodies aside. a dynamic body is
// moved by gravity, forces and contacts. the type can change later
const platform = rigidBody.create(world, {
    shape: crateShape,
    objectLayer: moving,
    motionType: MotionType.KINEMATIC,
    position: [5, 1, 0],
});
rigidBody.setMotionType(world, platform, MotionType.DYNAMIC, true);
```

### Position and rotation

```ts
const position: Vec3 = [0, 0, 0];
const quaternion: Quat = [0, 0, 0, 1];
rigidBody.getTransform(position, quaternion, world, crate);
// this teleports the body, so nothing in between stops it. the last argument wakes it
rigidBody.setTransform(world, crate, [0, 6, 0], [0, Math.SQRT1_2, 0, Math.SQRT1_2], true);
```

Setting a position teleports the body and skips the simulation in between. To move a kinematic body, set its velocity
instead (see [Kinematic bodies](#kinematic-bodies)).

### Velocity

```ts
rigidBody.setLinearVelocity(world, crate, [2, 0, 0]);
rigidBody.setAngularVelocity(world, crate, [0, Math.PI, 0]);
const velocity: Vec3 = [0, 0, 0];
rigidBody.getLinearVelocity(velocity, world, crate);
// the velocity of a point that moves with the body, here a corner
rigidBody.getVelocityAtPoint(velocity, world, crate, [0.5, 6.5, 0.5]);
```

### Forces and impulses

```ts
// a force acts over the next step, then is cleared. an impulse changes the velocity at once. both move only dynamic
// bodies. a crate's mass comes from its shape, at 1000 kg/m3 by default
const mass = rigidBody.getMass(world, crate);
rigidBody.addForce(world, crate, [0, 20 * mass, 0], true);
rigidBody.addImpulseAtPosition(world, crate, [0, 0, 5 * mass], [0.5, 6, 0]);
rigidBody.addAngularImpulse(world, crate, [0, 0, 0]);
```

### Mass

```ts
// set a mass and the inertia scales to it, or set the mass properties directly. a hollow mesh has no volume to give a mass
const heavy = rigidBody.create(world, {
    shape: ballShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [-5, 1, 0],
    mass: 80,
});
rigidBody.getMass(world, heavy); // 80
```

A triangle mesh has no volume, so it has no mass. A dynamic body with a mesh shape needs `massPropertiesOverride`.

### Damping, limits and degrees of freedom

```ts
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
```

### Sleeping

```ts
// a body at rest sleeps. it costs nothing until something touches it or you wake it
steps(120);
rigidBody.isSleeping(world, heavy); // true
rigidBody.wake(world, heavy);
rigidBody.sleep(world, heavy);
// set allowSleeping to false for a body that should never sleep
```

### Kinematic bodies

```ts
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
```

### Continuous collision detection

```ts
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
```

### User data

```ts
// your own value for a body, such as an entity index. it is an integer up to 2^53, and events report it with the body
rigidBody.setUserData(world, crate, 2 ** 40 + 7);
rigidBody.getUserData(world, crate); // 1099511627783
```

### Collision groups and masks

```ts
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
```

### Sensors

```ts
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
```

### Changing a body's shape or layer

```ts
// a new shape updates the mass, centre of mass and bounds. constraints keep their anchors on the body
const lump = sphere.create({ radius: 0.3 });
rigidBody.setShape(world, crate, lump, true);
// after changing a shape in place, pass it to setShape for each body that uses it
sphere.update(lump, { radius: 0.5 });
rigidBody.setShape(world, crate, lump, true);
```

```ts
rigidBody.setObjectLayer(world, crate, notMoving);
```

## Shapes

Each shape type has a `create` function. Any number of bodies can share a shape.

Shapes are reference counted. A body, a character, or a shape built from another shape holds the shape it uses. The
shape you create is held for you until you call `releaseShape`. The last holder to let go frees it. So release a shape
when you stop making things with it, in any order. `getShapeRefCount` returns how many holders a shape has.

A shape's geometry is centred on its origin. The body's pose places it in the world.

### Convex shapes

```ts
// each shape is centred on its origin. its density in kg/m3 gives the mass, and defaults to 1000
const crate = box.create({ halfExtents: [0.5, 0.5, 0.5], convexRadius: 0.05, density: 200 });
const ball = sphere.create({ radius: 0.5 });
// along y. a cylinder with half height 0.5 between two hemispheres
const pill = capsule.create({ halfHeightOfCylinder: 0.5, radius: 0.3 });
const barrel = cylinder.create({ halfHeight: 0.6, radius: 0.4 });
getShapeVolume(ball); // 4/3 pi r^3
```

### Convex hulls

```ts
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
```

### Planes

```ts
// everything below y 0 is solid. halfExtent bounds it for the broadphase
const ground = plane.create({ normal: [0, 1, 0], constant: 0, halfExtent: 1000 });
rigidBody.create(world, { shape: ground, objectLayer: notMoving, motionType: MotionType.STATIC });
```

### Triangle meshes

```ts
// the mesh is welded, gets a bvh, and marks its edges so things slide over the joins. each triangle can pick a
// material from setMaterials
const terrain = triangleMesh.create({
    positions: [-10, 0, -10, 10, 0, -10, 10, 0, 10, -10, 0, 10],
    indices: [0, 2, 1, 0, 3, 2],
    materialIds: [0, 1],
});
rigidBody.create(world, { shape: terrain, objectLayer: notMoving, motionType: MotionType.STATIC, position: [0, 5, 0] });
```

Use meshes for static bodies. A mesh is hollow. It has no volume, no inside and no mass.

### Compounds

```ts
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
```

### Decorators

```ts
// a shape moved, scaled, or with its centre of mass offset, such as a weighted toy that rights itself
const raised = transformed.create({ shape: crate, position: [0, 1, 0], quaternion: [0, 0, 0, 1] });
const stretched = scaled.create({ shape: crate, scale: [2, 1, 1] });
const weighted = offsetCenterOfMass.create({ shape: pill, offset: [0, -0.4, 0] });
```

### Empty

```ts
// no volume and no collisions. use it as a static constraint anchor, or for a body whose shape comes later
const nothing = emptyShape.create();
```

### Freeing shapes

```ts
// a shape built from another holds it, so you can release shapes in any order, before or after the world
for (const shape of [crate, leg, top, ball, pill, barrel, rock, terrain, ground, nothing]) releaseShape(shape);
for (const shape of [raised, stretched, weighted, table, fence]) releaseShape(shape);
destroyWorld(world);
```

## Events

A listener receives a step's events and answers them during the step. The wasm never calls into JS. Instead,
`updateWorld` runs the step in phases and calls your hooks between them. A step with no hooks is a single call into the
wasm.

```ts
// the listener hears the step's events and answers them while it runs
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
```

```ts
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
```

## Contact pairs

A listener hears every contact in every step. Contact pairs track only the contacts you care about, across frames.

Each pair is two body parts in contact. It has a stable handle while the contact lasts, and is marked added, persisted
or removed each frame. A frame can cover several world steps and the character updates between them.

```ts
// only pairs that include a body observing contacts are kept. these are the few a game reacts to
const ball = rigidBody.create(world, {
    shape: ballShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [0, 2, 0],
    restitution: 0.6,
    observesContacts: true,
});
```

```ts
// a contact pairs frame. begin it, record each world step and character update, then end it. a pair is added in the
// frame it starts, persisted while it lasts, and removed in the frame after it was last recorded
function frame(steps: number): void {
    contactPairs.begin(world);
    for (let i = 0; i < steps; i++) {
        updateWorld(world, undefined, 1 / 60 / steps);
        contactPairs.recordWorldContacts(world);
    }
    contactPairs.end(world);
}
```

```ts
const pair = contactPairs.create();
function read(): void {
    for (let i = 0; i < contactPairs.activeCount(world); i++) {
        contactPairs.readActive(pair, world, i);
        lines.push(
            `${pair.added ? 'added' : 'persisted'}: ${pair.bodyA} touches ${pair.bodyB}, ${pair.relativeVelocity[1].toFixed(2)} m/s apart`,
        );
    }
    for (let i = 0; i < contactPairs.removedCount(world); i++) {
        contactPairs.readRemoved(pair, world, i);
        lines.push(`removed: ${pair.bodyA} and ${pair.bodyB}`);
    }
}
```

## Queries

A query searches the world and reports hits to a collector. An optional filter decides what it can hit. A collector
keeps the closest hit, any one hit, or every hit. An any-hit collector stops at the first hit.

### Rays

```ts
const raySettings = createDefaultCastRaySettings();
const origin: Vec3 = [0, 10, 0];
const down: Vec3 = [0, -1, 0];

// the nearest hit, and its fraction along the 20 m ray
const closest = createClosestCastRayCollector();
castRay(world, closest, raySettings, origin, down, 20);
if (closest.hit.status === CastRayStatus.COLLIDING) {
    const distance = closest.hit.fraction * 20;
    const point: Vec3 = [origin[0], origin[1] - distance, origin[2]];
    console.log(`body ${closest.hit.bodyIdB} hit at ${point}`); // the upper crate, its top at y 3.5
}

// the first hit found, for line of sight. then every hit, nearest first after sorting
const any = createAnyCastRayCollector();
castRay(world, any, raySettings, origin, down, 20);
const all = createAllCastRayCollector();
castRay(world, all, raySettings, origin, down, 20);
all.sort();
all.hits.length; // 3: the crates and the floor

// a collector keeps its hits across queries until you reset it
closest.reset();
```

### Points

```ts
// the bodies whose shapes contain a point
const inside = createAllCollidePointCollector();
collidePoint(world, inside, createDefaultCollidePointSettings(), [0, 1, 0]);
inside.hits[0]?.bodyIdB; // the lower crate
```

### Overlaps

```ts
// what a posed, scaled shape overlaps. use it to place things or find a blast's reach. the closest hit is the deepest
const probe = sphere.create({ radius: 0.8 });
const deepest = createClosestCollideShapeCollector();
collideShape(world, deepest, createDefaultCollideShapeSettings(), probe, [0, 2, 0], [0, 0, 0, 1], [1, 1, 1]);
deepest.hit?.penetration; // into a crate by 0.3
```

### Sweeps

```ts
// a swept shape. it finds where a falling ball first hits something, for placing it or for a projectile's path
const sweep = createClosestCastShapeCollector();
const ball = sphere.create({ radius: 0.5 });
castShape(world, sweep, createDefaultCastShapeSettings(), ball, [0, 10, 0], [0, 0, 0, 1], [1, 1, 1], [0, -20, 0]);
if (sweep.hit.status === CastShapeStatus.COLLIDING) {
    sweep.hit.fraction; // 0.3: its centre half a metre over the upper crate's top
    sweep.hit.normal; // up, from the crate towards the ball
}
```

### Filters

```ts
// a query without a filter hits everything. a filter lives in the wasm, so free it when done. it starts with every
// layer enabled. a body must pass its layers, groups and mask, and its body filter
const queryFilter = filter.create(world);
filter.disableObjectLayer(queryFilter, world, notMoving); // the floor's layer: crates only
queryFilter.collisionMask = ~2 >>> 0; // no body in group 2
filter.setAllEnabled(queryFilter, world); // back to everything
// set a filter from a body, so it hits what that body collides with
filter.setFromBody(queryFilter, world, upper);
```

```ts
// your own test for each body a query reaches. the result is the closest body that passes. such a query collects
// every hit first and loses its early out, so prefer layers and masks when they are enough
queryFilter.bodyFilter = (body) => rigidBody.getUserData(world, body) !== 9;
closest.reset();
castRay(world, closest, raySettings, origin, down, 20, queryFilter);
closest.hit.bodyIdB; // the lower crate: the upper is filtered out
queryFilter.bodyFilter = null;
```

## Constraints

A constraint holds two bodies relative to each other. Each type's `create(world, bodyA, bodyB, settings)` returns an id.
The type's other functions take that id. Removing either body removes the constraint. You can also remove it with
`constraints.remove`.

Points and axes are in world space by default. With `space: ConstraintSpace.LOCAL`, they are relative to each body.

### Hinge

```ts
// a door hinged along y at its left edge. it opens a quarter turn at most, and its motor drives it open. a motor's
// torque is unbounded unless you limit it. an unbounded motor pushes past the hinge limit, so this one has a limit
const frame = anchor([0, 1, 0]);
const door = dynamic(doorShape, [0.5, 1, 0]);
const hinge = hingeConstraint.create(world, frame, door, {
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
hingeConstraint.setMotorState(world, hinge, MotorState.VELOCITY);
hingeConstraint.setTargetAngularVelocity(world, hinge, 2);
```

```ts
// remove it, or remove either body. after that, the id names nothing
constraints.remove(world, hinge);
constraints.exists(world, hinge); // false
```

### Point

```ts
// a pendulum. the ball stays 2 m from the point it swings about
const pivot = anchor([10, 5, 0]);
const bob = dynamic(ballShape, [12, 5, 0]);
pointConstraint.create(world, pivot, bob, { pointA: [10, 5, 0], pointB: [10, 5, 0] });
```

### Distance

```ts
// a bungee. a spring pulls it back past 3 m. a minDistance or maxDistance of -1 keeps the starting distance
const jumper = dynamic(ballShape, [14, 5, 0]);
distanceConstraint.create(world, pivot, jumper, {
    pointA: [10, 5, 0],
    pointB: [14, 5, 0],
    minDistance: 0,
    maxDistance: 3,
    springSettings: { mode: SpringMode.FREQUENCY_AND_DAMPING, frequencyOrStiffness: 2, damping: 0.5 },
});
```

### Slider

```ts
// a lift. the platform slides along y only, and its position motor holds it 2 m up the rail. a position motor drives
// through a spring, so a load makes it sag. at 10 Hz this spring sags by only millimetres
const rail = anchor([20, 0, 0]);
const platform = dynamic(blockShape, [20, 0, 0]);
const lift = sliderConstraint.create(world, rail, platform, {
    sliderAxisA: [0, 1, 0],
    sliderAxisB: [0, 1, 0],
    normalAxisA: [1, 0, 0],
    normalAxisB: [1, 0, 0],
    autoDetectPoint: true,
    limitsMin: 0,
    limitsMax: 5,
    motorSettings: { spring: { frequencyOrStiffness: 10, damping: 1 } },
});
sliderConstraint.setMotorState(world, lift, MotorState.POSITION);
sliderConstraint.setTargetPosition(world, lift, 2);
```

### Fixed

```ts
// a weld. the two bodies keep their relative pose and move as one
const left = dynamic(blockShape, [30, 3, 0]);
const right = dynamic(blockShape, [31, 3, 0]);
fixedConstraint.create(world, left, right, { autoDetectPoint: true });
```

### Swing-twist and cone

```ts
// a shoulder. the arm swings within a 45 degree cone around the twist axis, here x. it twists up to 30 degrees either
// way. a cone constraint is the same without the twist limits
const torso = anchor([40, 3, 0]);
const arm = dynamic(ballShape, [40.5, 3, 0]);
swingTwistConstraint.create(world, torso, arm, {
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
coneConstraint.create(world, coneAnchor, coneArm, {
    pointA: [40, 6, 0],
    twistAxisA: [1, 0, 0],
    pointB: [40, 6, 0],
    twistAxisB: [1, 0, 0],
    halfConeAngle: 0.25 * Math.PI,
});
```

### Six degrees of freedom

```ts
// each axis has its own setting. here the body slides along x up to a metre either way and turns about y. the other
// axes are locked
const base = anchor([50, 3, 0]);
const cart = dynamic(blockShape, [50, 3, 0]);
const limitMin: SixDOFValues = [0, 0, 0, 0, 0, 0];
const limitMax: SixDOFValues = [0, 0, 0, 0, 0, 0];
limitMin[SixDOFAxis.TRANSLATION_X] = -1;
limitMax[SixDOFAxis.TRANSLATION_X] = 1;
limitMin[SixDOFAxis.ROTATION_Y] = -Math.PI;
limitMax[SixDOFAxis.ROTATION_Y] = Math.PI;
sixDOFConstraint.create(world, base, cart, { pointA: [50, 3, 0], pointB: [50, 3, 0], limitMin, limitMax });
rigidBody.setLinearVelocity(world, cart, [5, 0, 0]);
```

### Motors and springs

A motor drives a hinge, slider, swing-twist or six-dof axis.

- `MotorState.VELOCITY` drives to a target velocity.
- `MotorState.POSITION` drives to a target position, using the motor's spring.

A motor's torque and force are unbounded unless its `motorSettings` limit them. Set a limit on a motor that drives
against a constraint limit. Otherwise it pushes past the constraint limit.

A spring with `frequencyOrStiffness` above 0 makes a limit or a distance soft.

Invalid settings throw a `RangeError`. One example is hinge limits outside [-pi, pi].

### Breaking

```ts
// a breakable joint. read the impulse it applied each step, and disable it above a strength. a disabled constraint
// holds nothing and links nothing until you enable it again
const hook = anchor([60, 5, 0]);
const weight = rigidBody.create(world, {
    shape: blockShape,
    objectLayer: moving,
    motionType: MotionType.DYNAMIC,
    position: [60, 4, 0],
    mass: 50,
});
const rope = pointConstraint.create(world, hook, weight, { pointA: [60, 5, 0], pointB: [60, 5, 0] });
const lambda: Vec3 = [0, 0, 0];
const STRENGTH = 15; // N s a step: 50 kg hangs at about 8
for (let i = 0; i < 120; i++) {
    if (i === 60) rigidBody.addImpulse(world, weight, [0, -2000, 0]);
    updateWorld(world, undefined, 1 / 60);
    pointConstraint.getTotalLambda(lambda, world, rope);
    if (Math.hypot(...lambda) > STRENGTH) constraints.setEnabled(world, rope, false);
}
```

## Character controller

`KCC` moves a character through the world with sweeps. It walks up slopes and stairs, slides along walls, sticks to the
floor going down, and stands on moving bodies.

A character is not a rigid body. It can have an optional inner body. This is a kinematic body that follows the
character, so other bodies, queries and contact pairs can see it.

The character is a plain object. Its `position`, `quaternion`, `ground` and `contacts` show the result of its last call.
You set its `linearVelocity`.

```ts
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
```

```ts
// a policy sets how characters treat each body. bodies without their own entry use the defaults. one policy can serve
// any number of characters. here the crate can be pushed, until the character crouches below
const policy = kccPolicy.create();
kccPolicy.set(policy, crate, { canReceiveImpulses: true });
```

```ts
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
```

`character.ground` describes what the character stands on. It has a `state`, which is one of `ON_GROUND`,
`ON_STEEP_GROUND`, `NOT_SUPPORTED` and `IN_AIR`. It also has a `bodyId`, a `normal`, and the ground's `velocity`.

### Policy

A policy sets how a character treats each body it meets:

- `ignore`: the character passes through the body.
- `canPushCharacter: false`: the body blocks the character but does not move it.
- `canReceiveImpulses: false`: the character does not push the body.
- `slideOnWalkable: false`: the character does not slide on a still, walkable surface of the body.
- `surfaceLinearVelocity` and `surfaceAngularVelocity`: the body moves the character like a conveyor belt.

### Changing shape

```ts
// a shorter shape on the same feet. setShape returns false and keeps the old shape if the new one would sink deeper
// than the given depth
const crouchingCapsule = capsule.create({ halfHeightOfCylinder: 0.2, radius: 0.3 });
const crouching = transformed.create({ shape: crouchingCapsule, position: [0, 0.5, 0] });
if (kcc.setShape(world, character, crouching, queryFilter, policy, 0.01))
    kccPolicy.set(policy, crate, { canReceiveImpulses: false });
```

Put the origin of every shape at the character's feet. Then you can swap shapes freely.

```ts
// settings can change after create. here the supporting point is lower while crouched
kcc.setTuning(character, { supportingVolumePlane: [0, 1, 0, -0.3] });
```

### Your own loop

```ts
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
```

`collide`, `solve` and `cast` are the phases of a move. `move` runs them once per collision iteration.

Between `collide` and `solve`, each entry in `character.contacts` is a plain object you can edit. Set `wasDiscarded`
to skip a contact, or give it a different `linearVelocity`. The solve, the sweep and the ground check skip discarded
contacts.

`update` runs the same phases, plus a few more:

- Contact tracking, with `beginContactTracking` and `endContactTracking`.
- `cancelVelocityTowardsSteepSlopes` before the move.
- `stickToFloor` and `walkStairs` after it.

A character's queries run in the wasm, so a filter's `bodyFilter` does not apply to them. Set `ignore` in the
character's policy instead.

### Contacts

```ts
// the contacts from the last update, refreshContacts or setShape. each one is added, persisted or removed
for (const event of character.contactEvents) {
    if (event.kind === kcc.KCCContactEventKind.ADDED) event.bodyId; // footsteps, say
}
```

```ts
// read the character's contacts as data, through its inner body, in a contact pairs frame
contactPairs.begin(world);
kcc.refreshContacts(world, character, queryFilter, policy);
contactPairs.recordCharacterContacts(world, character);
contactPairs.end(world);
```

```ts
kcc.remove(world, character);
kcc.release(character);
kccPolicy.destroy(policy);
```

## Rendering

### Active bodies

```ts
// after a step, the active list holds the bodies it moved. read the list once and copy each pose into its mesh,
// number by number, so nothing is allocated per frame. a body that fell asleep leaves the list. the listener names it,
// so its last pose is read once
const meshes = new Map<BodyId, { position: Vec3; quaternion: Quat }>();
for (const body of balls) meshes.set(body, { position: [0, 0, 0], quaternion: [0, 0, 0, 1] });

const activeBody = createActiveBody();
const listener: Listener = {
    onBodyDeactivated: (body) => {
        const mesh = meshes.get(body);
        if (mesh === undefined) return;
        rigidBody.getPosition(mesh.position, world, body);
        rigidBody.getQuaternion(mesh.quaternion, world, body);
    },
};

function step(): void {
    updateWorld(world, listener, 1 / 60);
    const count = getActiveBodies(world);
    for (let i = 0; i < count; i++) {
        getActiveBodyAt(activeBody, world, i);
        const mesh = meshes.get(activeBody.body);
        if (mesh === undefined) continue;
        mesh.position[0] = activeBody.position[0];
        mesh.position[1] = activeBody.position[1];
        mesh.position[2] = activeBody.position[2];
        mesh.quaternion[0] = activeBody.quaternion[0];
        mesh.quaternion[1] = activeBody.quaternion[1];
        mesh.quaternion[2] = activeBody.quaternion[2];
        mesh.quaternion[3] = activeBody.quaternion[3];
    }
}
```

`onBodyActivated` runs when a body joins the active list. `onBodyDeactivated` runs when one leaves it. Both run after
the step, in order. They report every change to the list, whether the step made it or a call you made since the last
step.

### Debug drawing

```ts
// wireframes drawn into buffers in the wasm. clear, draw, then read. each vertex is x y z r g b. a line has two
// vertices and a triangle three. the views point into wasm memory, so copy them out before the next draw
const buffers = debugDraw.create();
let gpuLines = new Float32Array(1 << 16);
function drawDebug(): number {
    debugDraw.clear(buffers);
    debug.bodies(buffers, world, { showLinearVelocity: true });
    debug.contactConstraints(buffers, world);
    debug.joints(buffers, world);
    const { lines, lineCount } = debugDraw.read(buffers);
    if (lines.length > gpuLines.length) gpuLines = new Float32Array(2 * lines.length);
    gpuLines.set(lines);
    return lineCount;
}
```

`debug.characterContacts` draws a character's contacts. `debug.shape` draws a shape at a pose. `debug.line` draws a
line.

## Determinism

```ts
// the same scene, built in the same order and stepped the same way, gives exactly the same result. this holds in
// another world, another run, another machine, or natively in C++
const first = makePile(groundShape, ballShape);
const second = makePile(groundShape, ballShape);
for (let i = 0; i < 300; i++) {
    updateWorld(first.world, undefined, 1 / 60);
    updateWorld(second.world, undefined, 1 / 60);
}
const a: Vec3 = [0, 0, 0];
const b: Vec3 = [0, 0, 0];
const same = first.bodies.every((body, i) => {
    rigidBody.getPosition(a, first.world, body);
    rigidBody.getPosition(b, second.world, second.bodies[i]!);
    return a.every((value, axis) => Object.is(value, b[axis]));
});
console.log(same ? 'bit for bit the same' : 'they differ');
```

The wasm and the native library build the same way. Neither uses fused multiply-add or fast math, and both use their
own trigonometry. So the same steps give exactly the same results in the browser, in Node and natively. The tests check
that the JS package matches the native library's hash of a simulated pile.

To get the same results, create bodies and constraints and step in the same order, with the same values. A number
becomes a float32 when it crosses into the wasm. So a value computed in JS, with 64-bit floats, may round differently
from the same sum computed in C++.

## From crashcat

The names match crashcat's, so most crashcat code reads the same. These are the differences:

- A body is an id, not an object. You read its state with getters, such as `rigidBody.getPosition(out, world, body)`,
  and change it with setters. `getActiveBodies` lists the bodies that moved in a step.
- The world and everything in it live in the wasm. You free them yourself with `destroyWorld`, `releaseShape`,
  `filter.destroy` and `kcc.release`.
- A shape changes in place with its type's `update`. Then pass it to `rigidBody.setShape` for each body that uses it.
  There is no `updateShape`, and `body.shape` is not writable.
