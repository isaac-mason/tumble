# tumble for JavaScript

tumble is a 3D rigid body physics engine. The JS package is the C++ library compiled to WebAssembly, with a TypeScript
API over it, and it simulates the same bits as the native library does.

Every code block here is part of a program in `js/docs/` that the tests run, and each program checks what it shows.

<TOC />

## Getting tumble

The package is `tumble`: ES modules with their types, and `tumble.wasm` beside them. Load the wasm once, before anything
else, with `await init()`. With no options it loads the wasm from beside the package's modules, which suits Node and
bundlers that copy the file. Otherwise give it the wasm as a URL, its bytes or a compiled `WebAssembly.Module`, as here
with its bytes:

<Snippet source="../js/docs/memory.ts" select="init" />

The package exports the file as `tumble/tumble.wasm` for a bundler to resolve: with Vite, import it with `?url` and pass
that URL to `init`.

## Quick start

A world, a floor, and a box and a ball dropped onto it:

<Snippet source="../js/docs/quick_start.ts" select="quick-start" />

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

<Snippet source="../js/docs/memory.ts" select="freeing" />

A call that cannot do what it was asked throws, and backs out of what it began:

<Snippet source="../js/docs/memory.ts" select="errors" />

A view onto the wasm's memory (from `Poses.read` or `DebugBuffers.read`) is detached when the memory grows. Take views
from their read each time rather than keeping them.

## The world

A world holds bodies, constraints and the simulation's state. Several worlds can live side by side.

### Layers

<Snippet source="../js/docs/world.ts" select="layers" />

### Settings

<Snippet source="../js/docs/world.ts" select="settings" />

### Stepping

`World.step(world, listener, deltaTime)` advances the world one step, its events going to the listener (see
[Events](#events)), or nowhere with `undefined`. Step by a fixed time:

<Snippet source="../js/docs/world.ts" select="fixed-step" />

Bodies resting on others sink up to `settings.solver.penetrationSlop` (2 cm) into them: the solver allows it to keep
stacks from jittering.

### Materials

<Snippet source="../js/docs/world.ts" select="materials" />

A query's hit names the part it met by sub shape id: `RigidBody.getMaterial(world, hit.bodyIdB, hit.subShapeId)` is that
part's material id.

## Rigid bodies

### Creating and removing

<Snippet source="../js/docs/bodies.ts" select="create" />

### Motion types

<Snippet source="../js/docs/bodies.ts" select="motion-types" />

### Position and rotation

<Snippet source="../js/docs/bodies.ts" select="transform" />

Teleporting a body skips the simulation in between. Move a kinematic body by velocity instead (see
[Kinematic bodies](#kinematic-bodies)).

### Velocity

<Snippet source="../js/docs/bodies.ts" select="velocity" />

### Forces and impulses

<Snippet source="../js/docs/bodies.ts" select="forces" />

### Mass

<Snippet source="../js/docs/bodies.ts" select="mass" />

A triangle mesh has no volume to take a mass from: a dynamic body with one needs `massPropertiesOverride`.

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

Shapes are created by each type's `create`, shared by any number of bodies, and freed by `Shape.destroy` once nothing
uses them. A body's pose places its shape: a shape's own geometry is centred on its origin.

### Convex shapes

<Snippet source="../js/docs/shapes.ts" select="convex" />

### Convex hulls

<Snippet source="../js/docs/shapes.ts" select="convex-hull" />

### Planes

<Snippet source="../js/docs/shapes.ts" select="plane" />

### Triangle meshes

<Snippet source="../js/docs/shapes.ts" select="triangle-mesh" />

Meshes are for static bodies: they are hollow, with no volume, no inside and no mass.

### Compounds

<Snippet source="../js/docs/shapes.ts" select="compound" />

### Decorators

<Snippet source="../js/docs/shapes.ts" select="decorators" />

### Empty

<Snippet source="../js/docs/shapes.ts" select="empty" />

### Freeing shapes

<Snippet source="../js/docs/shapes.ts" select="freeing" />

## Events

A step's listener hears its events and answers them as it runs. The wasm never calls into JS: `World.step` runs the
step's phases and calls the hooks between them, so a step with no hooks is one call into the wasm.

<Snippet source="../js/docs/events.ts" select="listener" />

<Snippet source="../js/docs/events.ts" select="rules" />

## Contact pairs

The listener hears every contact of every step. Contact pairs keep the few you care about, between frames: each pair of
body parts in contact, with a stable handle while the pair lasts, added, persisted or removed each frame. A frame can
span several world steps and the characters' updates between them.

<Snippet source="../js/docs/contact_pairs.ts" select="observe" />

<Snippet source="../js/docs/contact_pairs.ts" select="frame" />

<Snippet source="../js/docs/contact_pairs.ts" select="read" />

## Queries

Each query walks the world, under a filter when given one, and reports to a collector: the closest hit, any hit
(stopping at the first), or every hit.

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

A constraint holds two bodies relative to each other. Each type's `create(world, bodyA, bodyB, settings)` returns its
id, which its functions take. A constraint goes with either of its bodies, or with `Constraint.remove`.

Points and axes are in world space by default. With `space: ConstraintSpace.LOCAL` they are relative to each body.

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

A motor drives a hinge, slider, swing-twist or six-dof axis. `MotorState.VELOCITY` drives to a target velocity;
`POSITION` drives to a target position through the motor's spring. A motor's torque and force are unbounded unless its
`motorSettings` limit them. Limit one that drives into a limit, or it pushes past the limit. A spring's
`frequencyOrStiffness` above 0 makes a limit or a distance soft. Settings the C++ would refuse, such as a hinge's
limits outside [-pi, pi], throw a `RangeError`.

### Breaking

<Snippet source="../js/docs/constraints.ts" select="breaking" />

## Character controller

`KCC` moves a character through the world by sweeps: it walks up slopes and stairs, slides along walls, sticks to the
floor going down, and stands on moving bodies. It is not a rigid body. Its optional inner body, a kinematic body that
follows it, is its presence in the world for bodies, queries and contact pairs. The character is a plain object: its
`position`, `quaternion`, `ground` and `contacts` are as its last call left them, and `linearVelocity` is yours to set.

<Snippet source="../js/docs/character.ts" select="create" />

<Snippet source="../js/docs/character.ts" select="policy" />

<Snippet source="../js/docs/character.ts" select="update" />

`character.ground` says what it stands on: its `state` (`ON_GROUND`, `ON_STEEP_GROUND`, `NOT_SUPPORTED`, `IN_AIR`), its
`bodyId`, the `normal` and the ground's `velocity`.

### Policy

A body's policy: `ignore` passes through it, `canPushCharacter` false blocks without moving the character,
`canReceiveImpulses` false keeps the character from pushing it, `slideOnWalkable` false stops the character on a still,
walkable surface of it, and `surfaceLinearVelocity` and `surfaceAngularVelocity` move the character as a conveyor's
belt would.

### Changing shape

<Snippet source="../js/docs/character.ts" select="crouch" />

Give every shape the character's feet as its origin, and swap them freely.

<Snippet source="../js/docs/character.ts" select="tuning" />

### Your own loop

<Snippet source="../js/docs/character.ts" select="toolkit" />

`collide`, `solve` and `cast` are a move's phases, as `move` runs them each collision iteration. Between `collide` and
`solve`, each of `character.contacts` is a plain object yours to edit: set its `wasDiscarded`, or give it another
`linearVelocity`. The solve, the sweep and the ground pass over a discarded one. `update` is the same phases with
contact tracking (`beginContactTracking`, `endContactTracking`), `cancelVelocityTowardsSteepSlopes` before, and
`stickToFloor` and `walkStairs` after. A character's queries run in the wasm, so a filter's `bodyFilter` cannot reach
them: set `ignore` in its policy instead.

### Contacts

<Snippet source="../js/docs/character.ts" select="events" />

<Snippet source="../js/docs/character.ts" select="contacts" />

<Snippet source="../js/docs/character.ts" select="remove" />

## Rendering

### Poses

<Snippet source="../js/docs/rendering.ts" select="poses" />

### Debug drawing

<Snippet source="../js/docs/rendering.ts" select="debug" />

`Debug.characterContacts` draws a character's contacts, `Debug.shape` a shape at a pose, and `Debug.line` a line.

## Determinism

<Snippet source="../js/docs/determinism.ts" select="replay" />

The wasm is built as the native library is: no fused multiply-add, no fast math, and its own trigonometry. So the same
steps give the same bits in the browser, in Node and natively. The tests hold the JS package to the native library's
hash of a simulated pile. To keep it, create bodies and constraints, and step, in the same order with the same values:
a number crosses into the wasm as a float32, so a value computed in JS (in 64 bits) may round differently from the same
sum in C++.
