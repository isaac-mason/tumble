# tumble for C++

tumble is a 3D rigid body physics engine. It is C++20 with no dependencies: no C++ standard library, no exceptions, no
RTTI. The same sources build to WebAssembly for the JS package, and simulate the same bits there as natively.

Every code block here is part of a program in `docs/cpp/` that the tests build and run, and each program checks what
it shows.

<TOC />

## Getting tumble

### CMake

tumble's `CMakeLists.txt` makes the `tumble` library and its alias `tumble::tumble`. Fetch it, or add a checkout (a git
submodule, say) with `add_subdirectory(path/to/tumble)`, then link the target:

<Snippet source="./cmake/fetch_content/CMakeLists.txt" select="fetch-content" />

The tests build only when tumble is the top-level project. Its options:

- `TUMBLE_DEBUG`: checks the library's asserts.
- `TUMBLE_NO_SIMD`: the scalar backend where SIMD is available. Both backends compute the same bits.

### Without CMake

Compile every `.cpp` under `tumble/` with the repository root on the include path (headers are included as
`"tumble/physics/world.h"`), as C++20, with `-ffp-contract=off` (MSVC: `/fp:precise`) and without fast math.

## Quick start

A world, a floor, and a box and a ball dropped onto it:

<Snippet source="./cpp/quick_start.cpp" select="quick-start" />

`dispatch::init()` registers the shape types and the collision function for every pair of them. Call it once, before
anything collides, and again only to change the shape types (see [Custom shapes](#custom-shapes)).

The rest of this guide leaves out the includes, `using namespace tumble;` and the world's setup.

## Conventions

- Everything is in `namespace tumble`, in a namespace per subject named after its header: `rigid_body::create` is in
  `tumble/physics/body/rigid_body.h`.
- A call that changes a world takes the world first: `rigid_body::set_position(world, body, ...)`.
- Settings structs hold storage types (`Float3`, `Float4`, plain floats) and work with designated initializers, which
  C++20 requires in declaration order. Function arguments take the register types (`Vec3f`, `Quatf`), made with
  `vec3f::make`, `quatf::set_axis_angle` and friends, loaded from storage with `vec3f::load`.
- Units are metres, kilograms, seconds and radians.
- What you create, you free: shapes, filters, collectors, debug buffers and worlds. Bodies and constraints belong to
  their world.

## Memory

tumble takes all its memory through one allocator. A hosted build starts with aligned `malloc` and `free`. Set your own
before anything allocates. Every block comes back with the size it was asked for, so your allocator needs no headers of
its own:

<Snippet source="./cpp/memory.cpp" select="arena" />

<Snippet source="./cpp/memory.cpp" select="set-allocator" />

`util::allocated_bytes` and `util::allocated_blocks` count what tumble holds:

<Snippet source="./cpp/memory.cpp" select="allocated" />

### Errors

A call that cannot do what it was asked sets the error word and backs out, freeing anything it took. The first error
stands until you clear it:

<Snippet source="./cpp/memory.cpp" select="errors" />

## The world

A world holds bodies, constraints and the simulation's state. Several worlds can live side by side.

### Layers

Object layers decide what collides with what. Each object layer belongs to a broadphase layer, and each broadphase
layer is a tree of its own: a query or the pair search visits only the trees whose layers can collide. Keep moving and
static bodies in separate broadphase layers.

<Snippet source="./cpp/world.cpp" select="layers" />

### Settings

`world_settings::WorldSettings` holds the layers, gravity, and the solver, narrowphase, sleeping and CCD settings, with
defaults that suit most games. `world::create` copies them, so you can release them right away, or reuse them for
another world:

<Snippet source="./cpp/world.cpp" select="settings" />

### Stepping

`world::step(world, delta_time)` advances the world one step. Step by a fixed time and catch up a few steps when a frame
runs long, so the simulation does not depend on the frame rate:

<Snippet source="./cpp/world.cpp" select="fixed-step" />

Bodies resting on others sink up to `settings.solver.penetration_slop` (2 cm) into them: the solver allows it to keep
stacks from jittering.

### Events and answers

There are no callbacks. A step is a series of phases. Between phases you read what the last phase found from
`world.events` and write your answers to `world.answers`, which the next phase reads. `world::step` runs the phases and
answers nothing. Run them yourself to answer:

<Snippet source="./cpp/world.cpp" select="phases" />

- `events.pairs`, after `find_pairs`: the pairs of bodies whose bounds overlap. `events::reject_pair` drops one before
  the narrowphase.
- `events.contacts_added` and `events.contacts_persisted`, after `narrowphase`: the contacts it made, each with its
  manifold and the combined material. `events::override_contact` changes how one is solved, or rejects it.
- `events.woken`, after `narrowphase`: the bodies its contacts woke. While it is above zero, find their pairs and run
  the narrowphase again.

After a step, the events say what it did:

<Snippet source="./cpp/world.cpp" select="events" />

`contacts_removed` also holds the contacts of bodies removed since the step before, so a removal's events come with the
next step.

## Rigid bodies

### Creating and removing

<Snippet source="./cpp/bodies.cpp" select="create" />

A body pointer stays valid while the body lives. Its id outlives it safely: `rigid_body::get` returns null for a removed
body's id, even once its slot is reused.

### Motion types

<Snippet source="./cpp/bodies.cpp" select="motion-types" />

### Position and rotation

<Snippet source="./cpp/bodies.cpp" select="transform" />

Teleporting a body skips the simulation in between: nothing it passes through stops it. Move a kinematic body by
velocity instead (see [Kinematic bodies](#kinematic-bodies)).

### Velocity

<Snippet source="./cpp/bodies.cpp" select="velocity" />

### Forces and impulses

<Snippet source="./cpp/bodies.cpp" select="forces" />

Forces and impulses only move dynamic bodies.

### Mass

<Snippet source="./cpp/bodies.cpp" select="mass" />

A triangle mesh has no volume to take a mass from: a dynamic body with one needs `mass_properties_override`.

### Damping, limits and degrees of freedom

<Snippet source="./cpp/bodies.cpp" select="tuning" />

### Sleeping

<Snippet source="./cpp/bodies.cpp" select="sleeping" />

### Kinematic bodies

<Snippet source="./cpp/bodies.cpp" select="kinematic" />

`move_kinematic` wakes the body, so a lift that waited at rest moves when told.

### Continuous collision detection

<Snippet source="./cpp/bodies.cpp" select="ccd" />

### User data

<Snippet source="./cpp/bodies.cpp" select="user-data" />

### Collision groups and masks

<Snippet source="./cpp/bodies.cpp" select="groups" />

### Materials

<Snippet source="./cpp/bodies.cpp" select="materials" />

`world::set_materials` gives the world a table of surfaces. A shape's or a triangle's `material_id` picks a surface,
which then stands in for the body's friction and restitution on that side of a contact.

### Sensors

<Snippet source="./cpp/bodies.cpp" select="sensors" />

### Changing a body's shape

<Snippet source="./cpp/bodies.cpp" select="shape" />

### Changing a body's layer

<Snippet source="./cpp/bodies.cpp" select="layer" />

## Shapes

Shapes are created by each type's `create`, shared by any number of bodies, and freed by its `destroy` once nothing uses
them. Each body's pose places its shape: a shape's own geometry is centred on its origin.

### Convex shapes

<Snippet source="./cpp/shapes.cpp" select="convex" />

A box's and a cylinder's `convex_radius` rounds their edges a little, which makes collisions faster and steadier.

### Convex hulls

<Snippet source="./cpp/shapes.cpp" select="convex-hull" />

### Planes

<Snippet source="./cpp/shapes.cpp" select="plane" />

### Triangle meshes

<Snippet source="./cpp/shapes.cpp" select="triangle-mesh" />

Meshes are for static bodies: they are hollow, with no volume, no inside and no mass.

### Compounds

<Snippet source="./cpp/shapes.cpp" select="compound" />

### Decorators

<Snippet source="./cpp/shapes.cpp" select="decorators" />

### Empty

<Snippet source="./cpp/shapes.cpp" select="empty" />

### Freeing shapes

<Snippet source="./cpp/shapes.cpp" select="freeing" />

## Contact pairs

Events report every contact of every step. Contact pairs keep the few you care about, between frames: each pair of body
parts in contact, with a stable handle while the pair lasts, added, persisted or removed each frame. A frame can span
several world steps and the characters' moves between them.

<Snippet source="./cpp/contacts.cpp" select="observe" />

<Snippet source="./cpp/contacts.cpp" select="frame" />

<Snippet source="./cpp/contacts.cpp" select="read" />

A removed pair's handle is reused after the next frame begins.

## Queries

Each query walks the broadphase under a filter and runs against every body it reaches, reporting to a collector:

- `collector::COLLECT_CLOSEST`: the nearest hit.
- `collector::COLLECT_ANY`: the first hit found, then stops.
- `collector::COLLECT_ALL`: every hit; `sort` orders them nearest first.

### Rays

<Snippet source="./cpp/queries.cpp" select="cast-ray" />

### Points

<Snippet source="./cpp/queries.cpp" select="collide-point" />

### Overlaps

<Snippet source="./cpp/queries.cpp" select="collide-shape" />

### Sweeps

<Snippet source="./cpp/queries.cpp" select="cast-shape" />

### Bounds only

`broadphase_query` finds bodies by their bounds alone, without testing their shapes, as Jolt's `BroadPhaseQuery`: a
box, a sphere, a point or an oriented box for the bodies it overlaps, and a ray or a moving box for the bodies it enters
and where. They take the same filter. The overlaps' collector takes `COLLECT_ALL`, `COLLECT_ANY` or `COLLECT_CUSTOM`, as
an overlap has no nearest; the casts' takes `COLLECT_CLOSEST` too.

<Snippet source="./cpp/queries.cpp" select="broadphase" />

### Filters

<Snippet source="./cpp/queries.cpp" select="filters" />

<Snippet source="./cpp/queries.cpp" select="body-filter" />

## Constraints

A constraint holds two bodies relative to each other. `<type>_constraint::create(world, body_a, body_b, settings)`
returns its id. `get(world, id)` returns its record for changing it (limits, motors), valid until the next create of
that type, so get it again rather than keeping it. A constraint goes with either of its bodies, or with
`remove(world, id)`.

Points and axes are in world space by default. With `.space = constraints::SPACE_LOCAL` they are relative to each body.
The two sides of a constraint are always `_a` and `_b`, as its bodies are.

### Hinge

<Snippet source="./cpp/constraints.cpp" select="hinge" />

<Snippet source="./cpp/constraints.cpp" select="remove" />

### Point

<Snippet source="./cpp/constraints.cpp" select="point" />

### Distance

<Snippet source="./cpp/constraints.cpp" select="distance" />

### Slider

<Snippet source="./cpp/constraints.cpp" select="slider" />

### Fixed

<Snippet source="./cpp/constraints.cpp" select="fixed" />

### Swing-twist and cone

<Snippet source="./cpp/constraints.cpp" select="ragdoll" />

<Snippet source="./cpp/constraints.cpp" select="cone" />

### Six degrees of freedom

<Snippet source="./cpp/constraints.cpp" select="six-dof" />

### Motors and springs

A motor drives a hinge, slider, swing-twist or six-dof axis. `motor_settings::VELOCITY` drives to a target velocity;
`POSITION` drives to a target position through the motor's spring. A motor's torque and force are unbounded unless
limited by its `MotorSettings`. Limit one that drives into a limit, or it pushes past the limit. A spring's
`frequency_or_stiffness` above 0 makes a limit or a distance soft.

## Character controller

`kcc` moves a character through the world by sweeps: it walks up slopes and stairs, slides along walls, sticks to the
floor going down, and stands on moving bodies. It is not a rigid body. Its optional inner body, a kinematic body that
follows it, is its presence in the world for bodies, queries and contact pairs.

<Snippet source="./cpp/character.cpp" select="create" />

<Snippet source="./cpp/character.cpp" select="update" />

`character.ground` says what it stands on: its `state` (`ON_GROUND`, `ON_STEEP_GROUND`, `NOT_SUPPORTED`, `IN_AIR`), its
`body_id`, the `normal` and the ground's `velocity`.

### Changing shape

<Snippet source="./cpp/character.cpp" select="crouch" />

Give every shape the character's feet as its origin, and swap them freely.

### Listeners and policies

How the character treats the bodies it meets is a `CharacterListener`'s: crashcat's and Jolt's callbacks, each
optional. `on_adjust_body_velocity` changes a body's velocity as the character sees it (a conveyor's belt),
`on_contact_validate` lets the character pass through a contact, `on_contact_added` and `on_contact_persisted` set a
contact's push settings, `on_contact_solve` changes the character's velocity against a contact, and
`on_contact_removed` reports a contact gone. Pass the listener to each call, or null for none.

<Snippet source="./cpp/character.cpp" select="listener" />

A policy is per body data that `kcc_policy::listener` makes into a listener:

<Snippet source="./cpp/character.cpp" select="policy" />

A body's policy: `ignore` passes through it, `can_push_character` false blocks without moving the character,
`can_receive_impulses` false keeps the character from pushing it, `slide_on_walkable` false stops the character on a
still, walkable surface of it, and `surface_linear_velocity` and `surface_angular_velocity` move the character as a
conveyor's belt would. The JS package's characters take policies: a listener's callbacks would have to call from the
wasm into JS.

### Your own loop

<Snippet source="./cpp/character.cpp" select="toolkit" />

`collide`, `solve` and `cast` are a move's phases, as `move` runs them each collision iteration. Between `collide` and
`solve` a contact's `was_discarded` and `linear_velocity` are yours: the solve, the sweep and the ground pass over a
discarded one. `update` is the same phases with contact tracking (`begin_contact_tracking`, `end_contact_tracking`),
`cancel_velocity_towards_steep_slopes` before, and `stick_to_floor` and `walk_stairs` after.

### Contacts

<Snippet source="./cpp/character.cpp" select="events" />

<Snippet source="./cpp/character.cpp" select="contacts" />

<Snippet source="./cpp/character.cpp" select="remove" />

## Custom shapes

A shape type of your own takes one of eight user slots, `shapes::SHAPE_USER_1` to `SHAPE_USER_8`. Its def gives its
functions, its shapes carry your payload in `shape->user.data`, and `dispatch::init` puts it beside the built-ins. This
one is an ellipsoid:

<Snippet source="./cpp/custom_shapes.cpp" select="data" />

A convex type gives a support: gjk and epa need nothing else to collide it with every other convex shape.

<Snippet source="./cpp/custom_shapes.cpp" select="support" />

<Snippet source="./cpp/custom_shapes.cpp" select="functions" />

<Snippet source="./cpp/custom_shapes.cpp" select="def" />

<Snippet source="./cpp/custom_shapes.cpp" select="register" />

<Snippet source="./cpp/custom_shapes.cpp" select="use" />

A type that is not convex leaves `set_support` null and gives its own pairs with `shapes::set_collide_shape_fn` and
`shapes::set_cast_shape_fn` after `dispatch::init`.

A type whose sub shape ids name state you keep between steps, such as a pool of hits you clear before each step, sets
`skips_pair_cache`. A pair that barely moved normally reuses last step's contacts, which would name state you have
since cleared. With it set, the type's pairs always collide anew.

## Debug drawing

<Snippet source="./cpp/debug_draw.cpp" select="draw" />

<Snippet source="./cpp/debug_draw.cpp" select="read" />

`debug::character_contacts` draws a character's contacts. A custom shape draws through its def's `draw`, with
`debug::line`, or as its bounds without one.

## Determinism

<Snippet source="./cpp/determinism.cpp" select="replay" />

tumble uses no fused multiply-add, no fast math and none of the platform's math library: its trigonometry is its own.
So the same steps give the same bits natively and in WebAssembly, with SIMD and without; its tests hold every build to
one digest of a simulated pile. To keep them:

- Build tumble with its own flags, as its CMake does: `-ffp-contract=off` (MSVC: `/fp:precise`), no fast math.
- Build your code that calls tumble's math headers with `-ffp-contract=off` too. The math is inline, so it is compiled
  with your flags.
- Create bodies and constraints, and step, in the same order with the same values.
