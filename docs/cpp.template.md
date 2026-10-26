![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/isaac-mason/tumble/release.yml?style=for-the-badge)

# tumble

tumble is a 3D physics engine in C++, built for games and simulations. It comes with JavaScript bindings,
[tumble.js](https://github.com/isaac-mason/tumble/tree/main/js#readme).

**Features**

- rigid bodies, with sleeping, sensors and continuous collision detection
- spheres, boxes, capsules, cylinders, convex hulls, triangle meshes, planes and compounds, plus your own shape types
- constraints with motors and springs: hinge, slider, distance, point, fixed, cone, swing-twist and six-dof
- a character controller that walks up slopes and stairs, slides along walls and rides moving bodies
- ray, point, overlap, sweep and bounds queries, with collision filtering
- a step you can run phase by phase, reading and answering its contacts in between
- contact pairs that track only the contacts you care about
- deterministic: the same steps give the same results natively and in WebAssembly, with or without SIMD
- C++20 with no dependencies: no standard library, no exceptions and no RTTI
- all memory through one allocator that you can replace
- the whole API from one include, `tumble/tumble.h`

**Examples**

- [Samples](https://isaac-mason.github.io/tumble/samples/), the C++ samples built for the web
- [tumble.js examples](https://isaac-mason.github.io/tumble/examples/)

**Documentation**

This README is the guide. Every code block comes from a program in `docs/cpp/`, and the tests build and run each one.

<TOC />

## Getting tumble

### CMake

tumble's `CMakeLists.txt` defines the `tumble` library and its alias `tumble::tumble`. Fetch it with `FetchContent`, or
add a checkout such as a git submodule with `add_subdirectory(path/to/tumble)`. Then link the target:

<Snippet source="./cmake/fetch_content/CMakeLists.txt" select="fetch-content" />

The tests only build when tumble is the top-level project. It has two options:

- `TUMBLE_DEBUG` enables the library's asserts.
- `TUMBLE_NO_SIMD` uses the scalar backend where SIMD is available. Both backends give the same results.

### Without CMake

Compile every `.cpp` under `tumble/` as C++20, with the repository root on the include path. Headers are included as
`"tumble/physics/world.h"`. Use `-ffp-contract=off`, or `/fp:precise` on MSVC, and do not use fast math.

## Quick start

This creates a world and a floor, then drops a box and a ball onto it:

<Snippet source="./cpp/quick_start.cpp" select="quick-start" />

`dispatch::init()` registers the shape types and the collision function for each pair of them. Call it once, before you
make any shape. Call it again only to change the shape types (see [Custom shapes](#custom-shapes)).

The rest of this guide leaves out the includes, `using namespace tumble;` and the world setup.

## Conventions

- Everything is in `namespace tumble`.
- Types are directly in `tumble`, such as `RigidBody`, `Shape` and `BoxShapeSettings`. A type that is part of another
  is nested in it, such as `KCC::ContactEvent` and `ConvexHullShape::BuildResult`.
- Functions and constants are in a namespace per subject, named after its header. For example, `rigid_body::create` is
  in `tumble/physics/body/rigid_body.h`.
- Enums are scoped, such as `MotionType::DYNAMIC` and `ShapeType::BOX`.
- A function that changes a world takes the world first: `rigid_body::set_position(world, body, ...)`.
- Settings structs hold storage types: `Float3`, `Float4` and plain floats. They work with designated initializers,
  which C++20 requires in declaration order.
- Functions take register types such as `Vec3f` and `Quatf`. Make them with functions like `vec3f::make` and
  `quatf::set_axis_angle`, or load them from storage with `vec3f::load`.
- Units are metres, kilograms, seconds and radians.
- Values free their memory when they go out of scope. This covers settings, filters, collectors and debug draws.
- `world::create` makes a world and `world::destroy` frees it, with its bodies and constraints.
- Shapes and characters are reference counted. Hold one in a `Ref<Shape>` or `Ref<KCC>`. The last `Ref` to let go
  frees it.

### Public API

`tumble/tumble.h` includes the whole API, as the quick start does. You can also include only the headers you use.

These headers are tumble's API. The rest are internals. They can change without notice, and you never need them:

- **The world:** `world.h`, `world_settings.h`, `layers.h`, `filter.h`, `update.h`, `events.h`, `contact_pairs.h` and
  `debug.h`. `update.h` has the phases of a step, which you can answer between.
- **Bodies:** `body/rigid_body.h`, `body/body_id.h`, `body/motion_type.h`, `body/dof.h`, `body/mass_properties.h` and
  `body/sub_shape.h`. You can read a body's fields. Change them only through `rigid_body` functions, which keep the
  broadphase, the pairs and the sleep state up to date.
- **Shapes:** each shape's header under `shapes/`, and `shapes/dispatch.h`. The builders and data in `shapes/utils/`
  are `convex_hull_builder.h`, `triangle_mesh_builder.h` and `triangle_mesh_data.h`.
- **Queries:** `query.h` and `broadphase_query.h`. The collectors, settings and hits are in `collision/collector.h`,
  `collision/cast_ray_vs_shape.h`, `collision/cast_shape_vs_shape.h`, `collision/collide_point_vs_shape.h` and
  `collision/collide_shape_vs_shape.h`. Also `collision/estimate_collision_response.h`.
- **Constraints:** each constraint's header under `constraints/`, plus `constraints/constraint_id.h`,
  `constraints/contact_settings.h`, `constraints/constraint_part/motor_settings.h` and
  `constraints/constraint_part/spring_settings.h`. `world::set_materials` takes `WorldMaterial` from
  `contact_constraints.h`. The rest of that header is internal.
- **Characters:** `character/kcc.h` and `character/kcc_policy.h`. Add `character/character_constraints.h` to write
  your own controller.
- **Custom shapes:** `shapes/shapes.h` has `ShapeDef`, `allocate`, and the collide and cast tables. Also
  `shapes/convex.h`, `collision/narrowphase.h` to collide or cast the shapes inside yours, `collision/support.h` for a
  convex type's support, and `utils/face.h`.
- **Maths and memory:** `tumble/math/`, `util/alloc.h`, `util/error.h`, `util/collections.h` and `util/ref.h`.

## Memory

tumble gets all its memory from one allocator. A hosted build starts with aligned `malloc` and `free`. Set your own
before anything allocates. tumble passes the size back when it frees a block, so your allocator does not need to store
it:

<Snippet source="./cpp/memory.cpp" select="arena" />

<Snippet source="./cpp/memory.cpp" select="set-allocator" />

`util::allocated_bytes` and `util::allocated_blocks` count the memory tumble holds:

<Snippet source="./cpp/memory.cpp" select="allocated" />

### Errors

Running out of memory stops the program. When the allocator returns null, tumble sets `Error::OUT_OF_MEMORY` in the
error word and traps. In a browser, the wasm instance cannot be used after that, and the error word says why. Nothing
else in tumble can fail for lack of memory, so nothing checks for it.

The only other error is a full pool. `rigid_body::create` and the constraint `create` functions then set
`Error::QUOTA` and return null or an invalid id. The first error stays set until you call `util::error_clear()`.

## The world

A world holds bodies, constraints and the simulation state. You can run several worlds at once.

### Layers

Object layers decide what collides with what. Each object layer belongs to a broadphase layer, and each broadphase
layer has its own tree. A query or the pair search only visits trees whose layers can collide. Put moving and static
bodies in separate broadphase layers.

<Snippet source="./cpp/world.cpp" select="layers" />

### Settings

`WorldSettings` holds the layers, gravity, and the solver, narrowphase, sleeping and CCD settings. The defaults suit
most games. `world::create` copies the settings, so you can free them right after, or use them for another world:

<Snippet source="./cpp/world.cpp" select="settings" />

### Stepping

`world::step(world, delta_time)` advances the world by one step. Step by a fixed time, and catch up a few steps when a
frame runs long. Then the simulation does not depend on the frame rate:

<Snippet source="./cpp/world.cpp" select="fixed-step" />

A resting body can sink into the body below by up to `settings.solver.penetration_slop`, which is 2 cm. This keeps
stacks from jittering.

### Events and answers

There are no callbacks. A step runs in phases. Between two phases, you read what the last phase found in
`world.events`, and write your answers to `world.answers`. The next phase reads them. `world::step` runs all the phases
and answers nothing. To answer, run the phases yourself:

<Snippet source="./cpp/world.cpp" select="phases" />

- After `find_pairs`, `events.pairs` holds the pairs of bodies whose bounds overlap. `events::reject_pair` drops a pair
  before the narrowphase.
- After `narrowphase`, `events.contacts_added` and `events.contacts_persisted` hold its contacts, each with its manifold
  and combined material. `events::override_contact` changes how a contact is solved, or rejects it.
- After `narrowphase`, `events.woken` counts the bodies its contacts woke. While it is above zero, find their pairs and
  run the narrowphase again.

After a step, the events describe what it did:

<Snippet source="./cpp/world.cpp" select="events" />

`contacts_removed` also holds the contacts of bodies removed since the last step. So a removal's events arrive with the
next step.

### Active bodies

The active list holds the dynamic and kinematic bodies that are awake. Read it with `world::get_num_active_bodies` and
`world::get_active_bodies`. Entries are indices into `bodies.pool`, valid until the list next changes. After a step, the
list names every body the step moved, except those it put to sleep.

An activation listener hears each body join or leave the list. It hears changes from the step, and from calls such as
`rigid_body::wake`, as they happen:

<Snippet source="./cpp/world.cpp" select="activation" />

<Snippet source="./cpp/world.cpp" select="activation-flush" />

## Rigid bodies

### Creating and removing

<Snippet source="./cpp/bodies.cpp" select="create" />

A body pointer stays valid while the body exists. Its id is safe to keep after that. `rigid_body::get` returns null for
a removed body's id, even after its slot is reused.

### Motion types

<Snippet source="./cpp/bodies.cpp" select="motion-types" />

### Position and rotation

<Snippet source="./cpp/bodies.cpp" select="transform" />

Setting a position teleports the body and skips the simulation in between. Nothing it passes through stops it. To move
a kinematic body, set its velocity instead (see [Kinematic bodies](#kinematic-bodies)).

### Velocity

<Snippet source="./cpp/bodies.cpp" select="velocity" />

### Forces and impulses

<Snippet source="./cpp/bodies.cpp" select="forces" />

Forces and impulses only move dynamic bodies.

### Mass

<Snippet source="./cpp/bodies.cpp" select="mass" />

A triangle mesh has no volume, so it has no mass. A dynamic body with a mesh shape needs `mass_properties_override`.

### Damping, limits and degrees of freedom

<Snippet source="./cpp/bodies.cpp" select="tuning" />

### Sleeping

<Snippet source="./cpp/bodies.cpp" select="sleeping" />

### Kinematic bodies

<Snippet source="./cpp/bodies.cpp" select="kinematic" />

`move_kinematic` wakes the body, so a lift waiting at rest starts moving.

### Continuous collision detection

<Snippet source="./cpp/bodies.cpp" select="ccd" />

### User data

<Snippet source="./cpp/bodies.cpp" select="user-data" />

### Collision groups and masks

<Snippet source="./cpp/bodies.cpp" select="groups" />

### Materials

<Snippet source="./cpp/bodies.cpp" select="materials" />

`world::set_materials` gives the world a table of materials. A shape's or a triangle's `material_id` picks one. On that
side of a contact, the material replaces the body's friction and restitution.

A query hit names the part it hit with a sub shape id. `rigid_body::get_material(body, hit.sub_shape_id)` returns that
part's material id.

### Sensors

<Snippet source="./cpp/bodies.cpp" select="sensors" />

### Changing a body's shape

<Snippet source="./cpp/bodies.cpp" select="shape" />

### Changing a body's layer

<Snippet source="./cpp/bodies.cpp" select="layer" />

## Shapes

Each shape type has a `create` function. Any number of bodies can share a shape.

Shapes are reference counted. Anything that holds a shape holds a `Ref<Shape>` to it. This includes bodies,
characters, shapes built from it, and your code. The last holder to let go frees it. A function that only reads a shape
takes a plain `Shape *`.

A shape's geometry is centred on its origin. The body's pose places it in the world.

### Convex shapes

<Snippet source="./cpp/shapes.cpp" select="convex" />

A box's or a cylinder's `convex_radius` rounds its edges a little. This makes collisions faster and steadier.

### Convex hulls

<Snippet source="./cpp/shapes.cpp" select="convex-hull" />

### Planes

<Snippet source="./cpp/shapes.cpp" select="plane" />

### Triangle meshes

<Snippet source="./cpp/shapes.cpp" select="triangle-mesh" />

Use meshes for static bodies. A mesh is hollow. It has no volume, no inside and no mass.

### Compounds

<Snippet source="./cpp/shapes.cpp" select="compound" />

### Decorators

<Snippet source="./cpp/shapes.cpp" select="decorators" />

### Empty

<Snippet source="./cpp/shapes.cpp" select="empty" />

### Freeing shapes

<Snippet source="./cpp/shapes.cpp" select="freeing" />

## Contact pairs

Events report every contact in every step. Contact pairs track only the contacts you care about, across frames.

Each pair is two body parts in contact. It has a stable handle while the contact lasts, and is marked added, persisted
or removed each frame. A frame can cover several world steps and the character moves between them.

<Snippet source="./cpp/contacts.cpp" select="observe" />

<Snippet source="./cpp/contacts.cpp" select="frame" />

<Snippet source="./cpp/contacts.cpp" select="read" />

A removed pair's handle can be reused once the next frame begins.

## Queries

A query searches the broadphase under a filter and tests every body it reaches. It reports hits to a collector:

- `CollectorKind::CLOSEST` keeps the nearest hit.
- `CollectorKind::ANY` keeps the first hit found, then stops.
- `CollectorKind::ALL` keeps every hit. `sort` orders them nearest first.
- `CollectorKind::CUSTOM` calls your `custom_add_hit` and `custom_add_miss`, with a `context`. Lower
  `early_out_fraction` to let shapes skip hits that cannot beat it. Set `force_early_out` to stop the query.

### Rays

<Snippet source="./cpp/queries.cpp" select="cast-ray" />

### Points

<Snippet source="./cpp/queries.cpp" select="collide-point" />

### Overlaps

<Snippet source="./cpp/queries.cpp" select="collide-shape" />

### Sweeps

<Snippet source="./cpp/queries.cpp" select="cast-shape" />

### Bounds only

`broadphase_query` finds bodies by their bounds alone, without testing their shapes.

- A box, a sphere, a point or an oriented box finds the bodies it overlaps.
- A ray or a moving box finds the bodies it enters, and where.

They take the same filter. An overlap has no nearest hit, so the overlap queries take `CollectorKind::ALL`, `ANY` or
`CUSTOM`. The casts also take `CollectorKind::CLOSEST`.

<Snippet source="./cpp/queries.cpp" select="broadphase" />

### Filters

<Snippet source="./cpp/queries.cpp" select="filters" />

<Snippet source="./cpp/queries.cpp" select="body-filter" />

## Constraints

A constraint holds two bodies relative to each other. `<type>_constraint::create(world, body_a, body_b, settings)`
returns its id.

`get(world, id)` returns the constraint's record, so you can change its limits and motors. The record is valid until
the next create of that type. Get it again each time instead of keeping it.

Removing either body removes the constraint. You can also remove any constraint with `constraints::remove(world, id)`.

Points and axes are in world space by default. With `.space = ConstraintSpace::LOCAL`, they are relative to each body.
The two sides of a constraint are named `_a` and `_b`, like its bodies.

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

A motor drives a hinge, slider, swing-twist or six-dof axis.

- `MotorState::VELOCITY` drives to a target velocity.
- `MotorState::POSITION` drives to a target position, using the motor's spring.

A motor's torque and force are unbounded unless its `MotorSettings` limit them. Set a limit on a motor that drives
against a constraint limit. Otherwise it pushes past the constraint limit.

A spring with `frequency_or_stiffness` above 0 makes a limit or a distance soft.

## Character controller

`kcc` moves a character through the world with sweeps. It walks up slopes and stairs, slides along walls, sticks to the
floor going down, and stands on moving bodies.

A character is not a rigid body. It can have an optional inner body. This is a kinematic body that follows the
character, so other bodies, queries and contact pairs can see it.

<Snippet source="./cpp/character.cpp" select="create" />

<Snippet source="./cpp/character.cpp" select="update" />

`character.ground` describes what the character stands on. Its `state` is one of `KCC::GroundState::ON_GROUND`,
`ON_STEEP_GROUND`, `NOT_SUPPORTED` and `IN_AIR`. It also has a `body_id`, a `normal`, and the ground's `velocity`.

### Changing shape

<Snippet source="./cpp/character.cpp" select="crouch" />

Put the origin of every shape at the character's feet. Then you can swap shapes freely.

### Listeners and policies

A `KCC::Listener` sets how the character treats the bodies it meets. Each of its callbacks is optional:

- `on_adjust_body_velocity` changes a body's velocity as the character sees it, such as a conveyor belt.
- `on_contact_validate` lets the character pass through a contact.
- `on_contact_added` and `on_contact_persisted` set a contact's push settings.
- `on_contact_solve` changes the character's velocity against a contact.
- `on_contact_removed` reports a contact that ended.

Pass the listener to each call, or null for none.

<Snippet source="./cpp/character.cpp" select="listener" />

A policy is per body data. `kcc_policy::listener` turns a policy into a listener:

<Snippet source="./cpp/character.cpp" select="policy" />

A policy sets how a character treats each body:

- `ignore`: the character passes through the body.
- `can_push_character = false`: the body blocks the character but does not move it.
- `can_receive_impulses = false`: the character does not push the body.
- `slide_on_walkable = false`: the character does not slide on a still, walkable surface of the body.
- `surface_linear_velocity` and `surface_angular_velocity`: the body moves the character like a conveyor belt.

The JS package's characters use policies only. A listener's callbacks would have to call from the wasm into JS.

### Your own loop

<Snippet source="./cpp/character.cpp" select="toolkit" />

`collide`, `solve` and `cast` are the phases of a move. `move` runs them once per collision iteration.

Between `collide` and `solve`, you can set a contact's `was_discarded` and `linear_velocity`. The solve, the sweep and
the ground check skip discarded contacts.

`update` runs the same phases, plus a few more:

- Contact tracking, with `begin_contact_tracking` and `end_contact_tracking`.
- `cancel_velocity_towards_steep_slopes` before the move.
- `stick_to_floor` and `walk_stairs` after it.

### Contacts

<Snippet source="./cpp/character.cpp" select="events" />

<Snippet source="./cpp/character.cpp" select="contacts" />

<Snippet source="./cpp/character.cpp" select="remove" />

## Custom shapes

A shape type of your own takes one of eight user slots, from `ShapeType::USER_1` to `ShapeType::USER_8`. Its def lists
its functions, and `dispatch::init` adds it next to the built-in types.

Its shapes are a struct of yours that derives from `Shape`, like every built-in shape such as `BoxShape`.

- `shapes::allocate<T>` makes one.
- `shapes::as<T>` reads a shape as your type.
- `shapes::release<T>` frees one. Call it from the def's `destroy`, which runs when the last `Ref` to the shape lets go.

This example is an ellipsoid:

<Snippet source="./cpp/custom_shapes.cpp" select="data" />

A convex type only needs a support function. With it, gjk and epa collide the type with every other convex shape.

<Snippet source="./cpp/custom_shapes.cpp" select="support" />

<Snippet source="./cpp/custom_shapes.cpp" select="functions" />

<Snippet source="./cpp/custom_shapes.cpp" select="def" />

<Snippet source="./cpp/custom_shapes.cpp" select="register" />

<Snippet source="./cpp/custom_shapes.cpp" select="use" />

A type that is not convex leaves `set_support` null. It sets its own pair functions with `shapes::set_collide_shape_fn`
and `shapes::set_cast_shape_fn`, after `dispatch::init`.

A contact's material comes from its shape's `get_material`, for the contact's sub shape id. Hits carry no material.
`rigid_body::get_material` returns a hit's material. So a type made of parts with their own materials, such as the
cells of a grid, picks the material by sub shape id.

Some types use sub shape ids that name state you keep between steps, such as a pool of hits you clear each step. Such a
type sets `skips_pair_cache`. Normally, a pair that barely moved reuses the last step's contacts, which would name state
you have cleared. With the flag set, the type's pairs always collide again.

## Debug drawing

<Snippet source="./cpp/debug_draw.cpp" select="draw" />

<Snippet source="./cpp/debug_draw.cpp" select="read" />

`debug::character_contacts` draws a character's contacts. A custom shape draws through its def's `draw` using
`debug::line`. Without a `draw`, it draws as its bounds.

## Determinism

<Snippet source="./cpp/determinism.cpp" select="replay" />

tumble uses no fused multiply-add, no fast math and no platform math library. Its trigonometry is its own. So the same
steps give exactly the same results natively and in WebAssembly, with SIMD and without. The tests check every build
against one hash of a simulated pile.

To keep results the same:

- Build tumble with its own flags, as its CMake does. That means `-ffp-contract=off`, or `/fp:precise` on MSVC, and no
  fast math.
- Build your own code that calls tumble's math headers with `-ffp-contract=off` too. The math is inline, so your flags
  compile it.
- Create bodies and constraints and step in the same order, with the same values.
