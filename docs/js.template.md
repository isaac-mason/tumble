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

<TOC />

## Quick start

This creates a world and a floor, then drops a box and a ball onto it:

<Snippet source="../js/docs/quick_start.ts" select="quick-start" />

The rest of this guide leaves out the imports and the world setup.

## Loading the wasm

The package holds ES modules, their types, and `tumble.wasm`.

Load the wasm once, before anything else, with `await init()`. With no options, `init` loads the wasm from next to the
package's modules. This works in Node and in bundlers that copy the file. You can also pass the wasm as a URL, as bytes
or as a compiled `WebAssembly.Module`. This example passes bytes:

<Snippet source="../js/docs/memory.ts" select="init" />

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

<Snippet source="../js/docs/memory.ts" select="freeing" />

A call that cannot do what you asked throws, and undoes anything it started:

<Snippet source="../js/docs/memory.ts" select="errors" />

Running out of memory is different. The wasm stops, the call throws a `WebAssembly.RuntimeError`, and the instance
cannot be used again.

A view into wasm memory, such as one from `debugDraw.read`, breaks when the memory grows. Read new views each time
instead of keeping them.

## The world

A world holds bodies, constraints and the simulation state. You can run several worlds at once.

### Layers

<Snippet source="../js/docs/world.ts" select="layers" />

### Settings

<Snippet source="../js/docs/world.ts" select="settings" />

### Stepping

`updateWorld(world, listener, deltaTime)` advances the world by one step. It sends the step's events to the listener
(see [Events](#events)). Pass `undefined` to ignore them. Step by a fixed time:

<Snippet source="../js/docs/world.ts" select="fixed-step" />

A resting body can sink into the body below by up to `settings.solver.penetrationSlop`, which is 2 cm. This keeps
stacks from jittering.

### Materials

<Snippet source="../js/docs/world.ts" select="materials" />

A query hit names the part it hit with a sub shape id. `rigidBody.getMaterial(world, hit.bodyIdB, hit.subShapeId)`
returns that part's material id.

## Rigid bodies

### Creating and removing

<Snippet source="../js/docs/bodies.ts" select="create" />

### Motion types

<Snippet source="../js/docs/bodies.ts" select="motion-types" />

### Position and rotation

<Snippet source="../js/docs/bodies.ts" select="transform" />

Setting a position teleports the body and skips the simulation in between. To move a kinematic body, set its velocity
instead (see [Kinematic bodies](#kinematic-bodies)).

### Velocity

<Snippet source="../js/docs/bodies.ts" select="velocity" />

### Forces and impulses

<Snippet source="../js/docs/bodies.ts" select="forces" />

### Mass

<Snippet source="../js/docs/bodies.ts" select="mass" />

A triangle mesh has no volume, so it has no mass. A dynamic body with a mesh shape needs `massPropertiesOverride`.

### Damping, limits and degrees of freedom

<Snippet source="../js/docs/bodies.ts" select="tuning" />

### Sleeping

<Snippet source="../js/docs/bodies.ts" select="sleeping" />

### Kinematic bodies

<Snippet source="../js/docs/bodies.ts" select="kinematic" />

### Continuous collision detection

<Snippet source="../js/docs/bodies.ts" select="ccd" />

### User data

<Snippet source="../js/docs/bodies.ts" select="user-data" />

### Collision groups and masks

<Snippet source="../js/docs/bodies.ts" select="groups" />

### Sensors

<Snippet source="../js/docs/bodies.ts" select="sensors" />

### Changing a body's shape or layer

<Snippet source="../js/docs/bodies.ts" select="shape" />

<Snippet source="../js/docs/bodies.ts" select="layer" />

## Shapes

Each shape type has a `create` function. Any number of bodies can share a shape.

Shapes are reference counted. A body, a character, or a shape built from another shape holds the shape it uses. The
shape you create is held for you until you call `releaseShape`. The last holder to let go frees it. So release a shape
when you stop making things with it, in any order. `getShapeRefCount` returns how many holders a shape has.

A shape's geometry is centred on its origin. The body's pose places it in the world.

### Convex shapes

<Snippet source="../js/docs/shapes.ts" select="convex" />

### Convex hulls

<Snippet source="../js/docs/shapes.ts" select="convex-hull" />

### Planes

<Snippet source="../js/docs/shapes.ts" select="plane" />

### Triangle meshes

<Snippet source="../js/docs/shapes.ts" select="triangle-mesh" />

Use meshes for static bodies. A mesh is hollow. It has no volume, no inside and no mass.

### Compounds

<Snippet source="../js/docs/shapes.ts" select="compound" />

### Decorators

<Snippet source="../js/docs/shapes.ts" select="decorators" />

### Empty

<Snippet source="../js/docs/shapes.ts" select="empty" />

### Freeing shapes

<Snippet source="../js/docs/shapes.ts" select="freeing" />

## Events

A listener receives a step's events and answers them during the step. The wasm never calls into JS. Instead,
`updateWorld` runs the step in phases and calls your hooks between them. A step with no hooks is a single call into the
wasm.

<Snippet source="../js/docs/events.ts" select="listener" />

<Snippet source="../js/docs/events.ts" select="rules" />

## Contact pairs

A listener hears every contact in every step. Contact pairs track only the contacts you care about, across frames.

Each pair is two body parts in contact. It has a stable handle while the contact lasts, and is marked added, persisted
or removed each frame. A frame can cover several world steps and the character updates between them.

<Snippet source="../js/docs/contact_pairs.ts" select="observe" />

<Snippet source="../js/docs/contact_pairs.ts" select="frame" />

<Snippet source="../js/docs/contact_pairs.ts" select="read" />

## Queries

A query searches the world and reports hits to a collector. An optional filter decides what it can hit. A collector
keeps the closest hit, any one hit, or every hit. An any-hit collector stops at the first hit.

### Rays

<Snippet source="../js/docs/queries.ts" select="cast-ray" />

### Points

<Snippet source="../js/docs/queries.ts" select="collide-point" />

### Overlaps

<Snippet source="../js/docs/queries.ts" select="collide-shape" />

### Sweeps

<Snippet source="../js/docs/queries.ts" select="cast-shape" />

### Filters

<Snippet source="../js/docs/queries.ts" select="filters" />

<Snippet source="../js/docs/queries.ts" select="body-filter" />

## Constraints

A constraint holds two bodies relative to each other. Each type's `create(world, bodyA, bodyB, settings)` returns an id.
The type's other functions take that id. Removing either body removes the constraint. You can also remove it with
`constraints.remove`.

Points and axes are in world space by default. With `space: ConstraintSpace.LOCAL`, they are relative to each body.

### Hinge

<Snippet source="../js/docs/constraints.ts" select="hinge" />

<Snippet source="../js/docs/constraints.ts" select="remove" />

### Point

<Snippet source="../js/docs/constraints.ts" select="point" />

### Distance

<Snippet source="../js/docs/constraints.ts" select="distance" />

### Slider

<Snippet source="../js/docs/constraints.ts" select="slider" />

### Fixed

<Snippet source="../js/docs/constraints.ts" select="fixed" />

### Swing-twist and cone

<Snippet source="../js/docs/constraints.ts" select="ragdoll" />

<Snippet source="../js/docs/constraints.ts" select="cone" />

### Six degrees of freedom

<Snippet source="../js/docs/constraints.ts" select="six-dof" />

### Motors and springs

A motor drives a hinge, slider, swing-twist or six-dof axis.

- `MotorState.VELOCITY` drives to a target velocity.
- `MotorState.POSITION` drives to a target position, using the motor's spring.

A motor's torque and force are unbounded unless its `motorSettings` limit them. Set a limit on a motor that drives
against a constraint limit. Otherwise it pushes past the constraint limit.

A spring with `frequencyOrStiffness` above 0 makes a limit or a distance soft.

Invalid settings throw a `RangeError`. One example is hinge limits outside [-pi, pi].

### Breaking

<Snippet source="../js/docs/constraints.ts" select="breaking" />

## Character controller

`KCC` moves a character through the world with sweeps. It walks up slopes and stairs, slides along walls, sticks to the
floor going down, and stands on moving bodies.

A character is not a rigid body. It can have an optional inner body. This is a kinematic body that follows the
character, so other bodies, queries and contact pairs can see it.

The character is a plain object. Its `position`, `quaternion`, `ground` and `contacts` show the result of its last call.
You set its `linearVelocity`.

<Snippet source="../js/docs/character.ts" select="create" />

<Snippet source="../js/docs/character.ts" select="policy" />

<Snippet source="../js/docs/character.ts" select="update" />

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

<Snippet source="../js/docs/character.ts" select="crouch" />

Put the origin of every shape at the character's feet. Then you can swap shapes freely.

<Snippet source="../js/docs/character.ts" select="tuning" />

### Your own loop

<Snippet source="../js/docs/character.ts" select="toolkit" />

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

<Snippet source="../js/docs/character.ts" select="events" />

<Snippet source="../js/docs/character.ts" select="contacts" />

<Snippet source="../js/docs/character.ts" select="remove" />

## Rendering

### Active bodies

<Snippet source="../js/docs/rendering.ts" select="active-bodies" />

`onBodyActivated` runs when a body joins the active list. `onBodyDeactivated` runs when one leaves it. Both run after
the step, in order. They report every change to the list, whether the step made it or a call you made since the last
step.

### Debug drawing

<Snippet source="../js/docs/rendering.ts" select="debug" />

`debug.characterContacts` draws a character's contacts. `debug.shape` draws a shape at a pose. `debug.line` draws a
line.

## Determinism

<Snippet source="../js/docs/determinism.ts" select="replay" />

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
