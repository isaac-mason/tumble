<!-- rendered by docs/build.mjs from docs/cpp.template.md: edit that, not this -->

# tumble for C++

tumble is a 3D rigid body physics engine. It is C++20 with no dependencies: no C++ standard library, no exceptions, no
RTTI. The same sources build to WebAssembly for the JS package, and simulate the same bits there as natively.

Every code block here is part of a program in `docs/cpp/` that the tests build and run, and each program checks what
it shows.

- [Getting tumble](#getting-tumble)
  - [CMake](#cmake)
  - [Without CMake](#without-cmake)
- [Quick start](#quick-start)
- [Conventions](#conventions)
  - [Public API](#public-api)
- [Memory](#memory)
  - [Errors](#errors)
- [The world](#the-world)
  - [Layers](#layers)
  - [Settings](#settings)
  - [Stepping](#stepping)
  - [Events and answers](#events-and-answers)
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
  - [Materials](#materials)
  - [Sensors](#sensors)
  - [Changing a body's shape](#changing-a-bodys-shape)
  - [Changing a body's layer](#changing-a-bodys-layer)
- [Shapes](#shapes)
  - [Convex shapes](#convex-shapes)
  - [Convex hulls](#convex-hulls)
  - [Planes](#planes)
  - [Triangle meshes](#triangle-meshes)
  - [Compounds](#compounds)
  - [Decorators](#decorators)
  - [Empty](#empty)
  - [Freeing shapes](#freeing-shapes)
- [Contact pairs](#contact-pairs)
- [Queries](#queries)
  - [Rays](#rays)
  - [Points](#points)
  - [Overlaps](#overlaps)
  - [Sweeps](#sweeps)
  - [Bounds only](#bounds-only)
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
- [Character controller](#character-controller)
  - [Changing shape](#changing-shape)
  - [Listeners and policies](#listeners-and-policies)
  - [Your own loop](#your-own-loop)
  - [Contacts](#contacts)
- [Custom shapes](#custom-shapes)
- [Debug drawing](#debug-drawing)
- [Determinism](#determinism)

## Getting tumble

### CMake

tumble's `CMakeLists.txt` makes the `tumble` library and its alias `tumble::tumble`. Fetch it, or add a checkout (a git
submodule, say) with `add_subdirectory(path/to/tumble)`, then link the target:

```cmake
cmake_minimum_required(VERSION 3.20)
project(game LANGUAGES CXX)

include(FetchContent)
FetchContent_Declare(tumble GIT_REPOSITORY https://github.com/isaac-mason/tumble.git GIT_TAG main)
FetchContent_MakeAvailable(tumble)

add_executable(game main.cpp)
target_link_libraries(game PRIVATE tumble::tumble)
```

The tests build only when tumble is the top-level project. Its options:

- `TUMBLE_DEBUG`: checks the library's asserts.
- `TUMBLE_NO_SIMD`: the scalar backend where SIMD is available. Both backends compute the same bits.

### Without CMake

Compile every `.cpp` under `tumble/` with the repository root on the include path (headers are included as
`"tumble/physics/world.h"`), as C++20, with `-ffp-contract=off` (MSVC: `/fp:precise`) and without fast math.

## Quick start

A world, a floor, and a box and a ball dropped onto it:

```cpp
#include <stdio.h>

#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/dispatch.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/world.h"

using namespace tumble;

int main() {
    // the shape types' functions and every pair of them, once before anything collides
    dispatch::init();

    // two object layers: things that move, and things that do not, each in a broadphase tree of its own
    world_settings::WorldSettings settings = world_settings::create();
    const uint32_t broadphase_moving = world_settings::add_broadphase_layer(settings);
    const uint32_t broadphase_not_moving = world_settings::add_broadphase_layer(settings);
    const uint32_t moving = world_settings::add_object_layer(settings, broadphase_moving);
    const uint32_t not_moving = world_settings::add_object_layer(settings, broadphase_not_moving);
    world_settings::enable_collision(settings, moving, moving);
    world_settings::enable_collision(settings, moving, not_moving);

    // the world copies the settings
    world::World *world = world::create(settings);
    world_settings::release(settings);

    // a static floor, its top at y = 0, and a box and a ball dropped onto it
    shapes::Shape *floor_shape = box::create({.half_extents = {20.0f, 0.5f, 20.0f}});
    shapes::Shape *box_shape = box::create({.half_extents = {0.5f, 0.5f, 0.5f}});
    shapes::Shape *ball_shape = sphere::create({.radius = 0.5f});
    rigid_body::create(*world, {.shape = floor_shape, .object_layer = not_moving, .motion_type = motion_type::MOTION_STATIC,
                                .position = {0.0f, -0.5f, 0.0f}});
    rigid_body::RigidBody *box = rigid_body::create(*world, {.shape = box_shape, .object_layer = moving,
                                                              .motion_type = motion_type::MOTION_DYNAMIC, .position = {0.0f, 4.0f, 0.0f}});
    rigid_body::RigidBody *ball = rigid_body::create(*world, {.shape = ball_shape, .object_layer = moving,
                                                               .motion_type = motion_type::MOTION_DYNAMIC, .position = {0.2f, 6.0f, 0.0f}});

    // two seconds at 60 steps a second
    for (int i = 0; i < 120; i++) world::step(*world, 1.0f / 60.0f);
    printf("box at y %.2f, ball at y %.2f\n", box->position.y, ball->position.y);

    // the world frees its bodies; the shapes are yours, freed once nothing uses them
    world::destroy(world);
    sphere::destroy(ball_shape);
    box::destroy(box_shape);
    box::destroy(floor_shape);
    return 0;
}
```

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

### Public API

These headers are tumble's API. The rest are its internals, which change without notice, and nothing in them is needed
to use tumble:

- **The world:** `world.h`, `world_settings.h`, `layers.h`, `filter.h`, `update.h` (the phases of a step you answer
  between), `events.h`, `contact_pairs.h` and `debug.h`.
- **Bodies:** `body/rigid_body.h`, `body/body_id.h`, `body/motion_type.h`, `body/dof.h`, `body/mass_properties.h` and
  `body/sub_shape.h`. A body's fields are yours to read; change them through `rigid_body`'s calls, which keep the
  broadphase, the pairs and the sleep state in step.
- **Shapes:** each shape's header under `shapes/`, `shapes/dispatch.h`, and the builders and data in `shapes/utils/`:
  `convex_hull_builder.h`, `triangle_mesh_builder.h` and `triangle_mesh_data.h`.
- **Queries:** `query.h`, `broadphase_query.h`, and the collectors, settings and hits in `collision/collector.h`,
  `collision/cast_ray_vs_shape.h`, `collision/cast_shape_vs_shape.h`, `collision/collide_point_vs_shape.h` and
  `collision/collide_shape_vs_shape.h`; also `collision/estimate_collision_response.h`.
- **Constraints:** each constraint's header under `constraints/`, `constraints/constraint_id.h`,
  `constraints/contact_settings.h`, `constraints/constraint_part/motor_settings.h` and
  `constraints/constraint_part/spring_settings.h`. `contact_constraints::Material` is what `world::set_materials`
  takes; the rest of `contact_constraints.h` is internal.
- **Characters:** `character/kcc.h`, `character/kcc_policy.h`, and `character/character_constraints.h` for a
  controller of your own.
- **Custom shapes:** `shapes/shapes.h` (`ShapeDef`, `allocate`, the collide and cast tables), `collision/narrowphase.h`
  to collide or cast the shapes inside yours, `collision/support.h` for a convex type's support, and `utils/face.h`.
- **Maths and memory:** `tumble/math/`, `util/alloc.h`, `util/error.h` and `util/collections.h`.

## Memory

tumble takes all its memory through one allocator. A hosted build starts with aligned `malloc` and `free`. Set your own
before anything allocates. Every block comes back with the size it was asked for, so your allocator needs no headers of
its own:

```cpp
/** a bump allocator over a block of its own: frees count what came back, but nothing is reused */
struct Arena {
    alignas(64) unsigned char memory[1 << 20];
    size_t used;
    size_t freed;
};

void *arena_allocate(size_t size, size_t alignment, void *user) {
    Arena &arena = *static_cast<Arena *>(user);
    const size_t at = util::align_up(arena.used, alignment);
    if (at + size > sizeof(arena.memory)) return nullptr;
    arena.used = at + size;
    return arena.memory + at;
}

void arena_free(void *, size_t size, void *user) { static_cast<Arena *>(user)->freed += size; }

static Arena arena;
```

```cpp
// before anything allocates: every block tumble takes from here on comes from the arena
util::set_allocator({arena_allocate, arena_free, &arena});
```

`util::allocated_bytes` and `util::allocated_blocks` count what tumble holds:

```cpp
world_settings::WorldSettings settings = world_settings::create();
world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
world::World *world = world::create(settings);
world_settings::release(settings);
printf("a world holds %zu bytes in %zu blocks\n", util::allocated_bytes(), util::allocated_blocks());

world::destroy(world);
printf("after destroy: %zu bytes\n", util::allocated_bytes());
```

### Errors

A call that cannot do what it was asked sets the error word and backs out, freeing anything it took. The first error
stands until you clear it:

```cpp
// the arena is nearly spent: the next world cannot be made
arena.used = sizeof(arena.memory) - 64;
world_settings::WorldSettings more = world_settings::create();
world::World *refused = world::create(more);
if (refused == nullptr && util::error() == util::ERROR_OUT_OF_MEMORY) {
    printf("out of memory: the call backed out and nothing leaked\n");
    util::error_clear();
}
```

## The world

A world holds bodies, constraints and the simulation's state. Several worlds can live side by side.

### Layers

Object layers decide what collides with what. Each object layer belongs to a broadphase layer, and each broadphase
layer is a tree of its own: a query or the pair search visits only the trees whose layers can collide. Keep moving and
static bodies in separate broadphase layers.

```cpp
// object layers say what collides with what; each maps to a broadphase layer, a tree of its own, so a query or the
// pair search walks only the trees whose layers can collide
struct Layers {
    uint32_t moving;
    uint32_t not_moving;
    uint32_t debris;
};

world_settings::WorldSettings make_settings(Layers &out) {
    world_settings::WorldSettings settings = world_settings::create();
    const uint32_t broadphase_moving = world_settings::add_broadphase_layer(settings);
    const uint32_t broadphase_not_moving = world_settings::add_broadphase_layer(settings);
    out.moving = world_settings::add_object_layer(settings, broadphase_moving);
    out.not_moving = world_settings::add_object_layer(settings, broadphase_not_moving);
    out.debris = world_settings::add_object_layer(settings, broadphase_moving);
    world_settings::enable_collision(settings, out.moving, out.moving);
    world_settings::enable_collision(settings, out.moving, out.not_moving);
    // debris lands on the ground and nothing else
    world_settings::enable_collision(settings, out.debris, out.not_moving);
    return settings;
}
```

### Settings

`world_settings::WorldSettings` holds the layers, gravity, and the solver, narrowphase, sleeping and CCD settings, with
defaults that suit most games. `world::create` copies them, so you can release them right away, or reuse them for
another world:

```cpp
Layers layers;
world_settings::WorldSettings settings = make_settings(layers);
settings.gravity = {0.0f, -20.0f, 0.0f};
settings.solver.velocity_iterations = 8;
settings.sleeping.time_before_sleep = 1.0f;
world::World *world = world::create(settings);
// a second world from the same settings is a world of its own
world::World *preview = world::create(settings);
world_settings::release(settings);
```

### Stepping

`world::step(world, delta_time)` advances the world one step. Step by a fixed time and catch up a few steps when a frame
runs long, so the simulation does not depend on the frame rate:

```cpp
// whatever the frame time, the world advances in steps of 1/60 s; a long frame catches up a few steps at most
const float step_time = 1.0f / 60.0f;
float accumulator = 0.0f;
const float frame_times[] = {0.016f, 0.017f, 0.033f, 0.2f};
for (float frame_time : frame_times) {
    accumulator += frame_time;
    for (int steps = 0; accumulator >= step_time && steps < 4; steps++) {
        world::step(*world, step_time);
        accumulator -= step_time;
    }
    if (accumulator > step_time) accumulator = 0.0f;
}
```

Bodies resting on others sink up to `settings.solver.penetration_slop` (2 cm) into them: the solver allows it to keep
stacks from jittering.

### Events and answers

There are no callbacks. A step is a series of phases. Between phases you read what the last phase found from
`world.events` and write your answers to `world.answers`, which the next phase reads. `world::step` runs the phases and
answers nothing. Run them yourself to answer:

```cpp
// world::step is these phases, answering nothing. between them the world's events say what a phase found, and its
// answers say what to do about it: here pairs with a body whose user data is 7 are rejected, so it passes through
// everything, and no contact bounces
void step_with_answers(world::World &world, float delta_time) {
    update::step_begin(world, delta_time);
    do {
        update::find_pairs(world);
        for (uint32_t i = 0; i < world.events.pairs.length; i++) {
            const events::PairEvent &pair = world.events.pairs[i];
            if (pair.user_data_a == 7 || pair.user_data_b == 7) events::reject_pair(world.answers, i);
        }
        update::narrowphase(world);
    } while (world.events.woken > 0);
    // after the last narrowphase: one answer per contacts_added entry, then per contacts_persisted entry
    for (uint32_t i = 0; i < world.answers.contact_override.length; i++) {
        events::override_contact(world.answers, world.events, i).settings.combined_restitution = 0.0f;
    }
    update::solve(world);
    update::step_end(world);
}
```

- `events.pairs`, after `find_pairs`: the pairs of bodies whose bounds overlap. `events::reject_pair` drops one before
  the narrowphase.
- `events.contacts_added` and `events.contacts_persisted`, after `narrowphase`: the contacts it made, each with its
  manifold and the combined material. `events::override_contact` changes how one is solved, or rejects it.
- `events.woken`, after `narrowphase`: the bodies its contacts woke. While it is above zero, find their pairs and run
  the narrowphase again.

After a step, the events say what it did:

```cpp
// after a step: the contacts it found and kept, the ones it lost, and the bodies it moved
void print_events(const world::World &world) {
    for (uint32_t i = 0; i < world.events.contacts_added.length; i++) {
        const events::ContactEvent &contact = world.events.contacts_added[i];
        printf("contact %u - %u, %u points, %.3f deep\n", body_id::index(contact.body_id_a), body_id::index(contact.body_id_b),
               contact.num_points, contact.penetration);
    }
    printf("%u contacts kept, %u removed, %u bodies moved\n", world.events.contacts_persisted.length, world.events.contacts_removed.length,
           world.events.moved.length);
}
```

`contacts_removed` also holds the contacts of bodies removed since the step before, so a removal's events come with the
next step.

## Rigid bodies

### Creating and removing

```cpp
// a body is in the world from create, found by the next step; the shape is borrowed and outlives it
rigid_body::RigidBody *crate = rigid_body::create(*world, {.shape = box_shape,
                                                           .object_layer = moving,
                                                           .motion_type = motion_type::MOTION_DYNAMIC,
                                                           .position = {0.0f, 2.0f, 0.0f},
                                                           .friction = 0.6f});

// its id names it for as long as it lives; get resolves an id to the body, or null once it is gone
const body_id::BodyId id = crate->id;
rigid_body::get(*world, id);

// out of the world: its contacts' removal events come with the next step, its constraints go with it
rigid_body::remove(*world, crate);
```

A body pointer stays valid while the body lives. Its id outlives it safely: `rigid_body::get` returns null for a removed
body's id, even once its slot is reused.

### Motion types

```cpp
// static bodies never move; kinematic ones move by their velocity and push dynamic ones aside; dynamic ones are
// moved by gravity, forces and contacts
rigid_body::RigidBody *platform = rigid_body::create(*world, {.shape = box_shape, .object_layer = moving,
                                                              .motion_type = motion_type::MOTION_KINEMATIC, .position = {4.0f, 1.0f, 0.0f}});
// and a body can change what it is: `wake` wakes one that keeps moving
rigid_body::set_motion_type(*world, platform, motion_type::MOTION_DYNAMIC, true);
```

### Position and rotation

```cpp
// a teleport: the broadphase and the pairs follow, and `wake` wakes a sleeping body
rigid_body::set_position(*world, body, vec3f::make(0.0f, 3.0f, 0.0f), true);
rigid_body::set_quaternion(*world, body, quatf::set_axis_angle(vec3f::axis_y(), 0.5f), true);
rigid_body::set_transform(*world, body, vec3f::make(0.0f, 0.5f, 0.0f), quatf::identity(), true);

// the body's own fields read its pose: position is the shape's origin, center_of_mass_position its centre of mass
printf("at %.2f %.2f %.2f\n", body->position.x, body->position.y, body->position.z);
```

Teleporting a body skips the simulation in between: nothing it passes through stops it. Move a kinematic body by
velocity instead (see [Kinematic bodies](#kinematic-bodies)).

### Velocity

```cpp
// metres and radians per second, world space; setting either wakes the body
rigid_body::set_linear_velocity(*world, body, vec3f::make(2.0f, 0.0f, 0.0f));
rigid_body::set_angular_velocity(*world, body, vec3f::make(0.0f, 1.0f, 0.0f));
rigid_body::add_linear_velocity(*world, body, vec3f::make(0.0f, 1.0f, 0.0f));

// the velocity of any point on it, from the body's linear and angular velocity
const Vec3f corner_velocity = rigid_body::get_velocity_at_point(body, vec3f::make(0.5f, 1.0f, 0.5f));
```

### Forces and impulses

```cpp
// a force acts over the next step and is then cleared; an impulse changes the velocity at once
rigid_body::add_force(*world, body, vec3f::make(0.0f, 2000.0f, 0.0f), true);
rigid_body::add_torque(*world, body, vec3f::make(0.0f, 10.0f, 0.0f), true);
rigid_body::add_force_at_position(*world, body, vec3f::make(100.0f, 0.0f, 0.0f), vec3f::make(0.0f, 1.0f, 0.0f), true);
rigid_body::add_impulse(*world, body, vec3f::make(0.0f, 500.0f, 0.0f));
rigid_body::add_angular_impulse(*world, body, vec3f::make(0.0f, 50.0f, 0.0f));
```

Forces and impulses only move dynamic bodies.

### Mass

```cpp
// from the shape's density by default (1000 kg/m3: the unit box weighs 1000 kg). `mass` keeps the shape's
// inertia scaled to a mass of your own; mass_properties_override gives both outright
rigid_body::RigidBody *light = rigid_body::create(*world, {.shape = box_shape, .object_layer = moving,
                                                           .motion_type = motion_type::MOTION_DYNAMIC, .position = {3.0f, 0.5f, 3.0f},
                                                           .mass = 10.0f});
mass_properties::MassProperties hollow = mass_properties::create();
mass_properties::set_mass_and_inertia_of_solid_box(hollow, vec3f::make(1.0f, 1.0f, 1.0f), 200.0f);
rigid_body::RigidBody *given = rigid_body::create(*world, {.shape = box_shape, .object_layer = moving,
                                                           .motion_type = motion_type::MOTION_DYNAMIC, .position = {-3.0f, 0.5f, 3.0f},
                                                           .mass_properties_override = &hollow});
```

A triangle mesh has no volume to take a mass from: a dynamic body with one needs `mass_properties_override`.

### Damping, limits and degrees of freedom

```cpp
// damping slows a body a little every step; the velocities are clamped to these limits; gravity_factor scales
// gravity for this body alone (0 floats); allowed_degrees_of_freedom locks axes: this one only slides in x and y
rigid_body::RigidBody *puck = rigid_body::create(*world, {.shape = box_shape,
                                                          .object_layer = moving,
                                                          .motion_type = motion_type::MOTION_DYNAMIC,
                                                          .position = {6.0f, 3.0f, 0.0f},
                                                          .allowed_degrees_of_freedom = dof::TRANSLATION_X | dof::TRANSLATION_Y,
                                                          .gravity_factor = 0.0f,
                                                          .linear_damping = 0.0f,
                                                          .angular_damping = 0.0f,
                                                          .max_linear_velocity = 20.0f});
rigid_body::set_linear_velocity(*world, puck, vec3f::make(30.0f, 0.0f, 5.0f));
```

### Sleeping

```cpp
// a body that has barely moved for a while sleeps, costing nothing until something touches or wakes it.
// allow_sleeping = false keeps it awake; sleep and wake do it by hand
rigid_body::sleep(*world, body);
rigid_body::wake(*world, body);
```

### Kinematic bodies

```cpp
// a kinematic body follows a path by velocities: move_kinematic sets the ones that reach the target in a step
rigid_body::RigidBody *lift = rigid_body::create(*world, {.shape = box_shape, .object_layer = moving,
                                                          .motion_type = motion_type::MOTION_KINEMATIC, .position = {-6.0f, 0.5f, 0.0f}});
for (int i = 1; i <= 60; i++) {
    const float height = 0.5f + 2.0f * (float)i / 60.0f;
    rigid_body::move_kinematic(*world, lift, vec3f::make(-6.0f, height, 0.0f), quatf::identity(), 1.0f / 60.0f);
    world::step(*world, 1.0f / 60.0f);
}
```

`move_kinematic` wakes the body, so a lift that waited at rest moves when told.

### Continuous collision detection

```cpp
// a small fast body can pass through a thin one between two steps; linear casting sweeps it, at a cost, so it
// stops at what it would hit
shapes::Shape *pellet_shape = sphere::create({.radius = 0.05f});
rigid_body::RigidBody *pellet = rigid_body::create(*world, {.shape = pellet_shape,
                                                            .object_layer = moving,
                                                            .motion_type = motion_type::MOTION_DYNAMIC,
                                                            .position = {10.0f, 5.0f, 0.0f},
                                                            .motion_quality = motion_properties::MOTION_QUALITY_LINEAR_CAST});
rigid_body::set_linear_velocity(*world, pellet, vec3f::make(0.0f, -200.0f, 0.0f));
```

### User data

```cpp
// a value of yours, carried into every event the body is in: an entity id, an index, a pointer
rigid_body::RigidBody *tagged = rigid_body::create(*world, {.shape = box_shape, .object_layer = moving,
                                                            .motion_type = motion_type::MOTION_DYNAMIC, .user_data = 42,
                                                            .position = {0.0f, 0.5f, 6.0f}});
tagged->user_data = 43;
```

### Collision groups and masks

```cpp
// inside one object layer, finer filtering: a pair collides when each body's mask has a bit of the other's groups
constexpr uint32_t PLAYERS = 1u << 0;
constexpr uint32_t PROJECTILES = 1u << 1;
rigid_body::RigidBody *player = rigid_body::create(*world, {.shape = box_shape, .object_layer = moving,
                                                            .motion_type = motion_type::MOTION_DYNAMIC, .position = {-10.0f, 0.5f, 0.0f},
                                                            .collision_groups = PLAYERS, .collision_mask = ~PROJECTILES});
// a player's own shots pass through it
rigid_body::RigidBody *shot = rigid_body::create(*world, {.shape = box_shape, .object_layer = moving,
                                                          .motion_type = motion_type::MOTION_DYNAMIC, .position = {-10.0f, 3.0f, 0.0f},
                                                          .collision_groups = PROJECTILES, .collision_mask = ~PLAYERS});
```

### Materials

```cpp
// friction and restitution meet in a contact through each body's combine mode; the more restrictive mode wins
rigid_body::RigidBody *ball = rigid_body::create(*world, {.shape = box_shape,
                                                          .object_layer = moving,
                                                          .motion_type = motion_type::MOTION_DYNAMIC,
                                                          .position = {12.0f, 4.0f, 0.0f},
                                                          .friction = 0.9f,
                                                          .restitution = 0.8f,
                                                          .restitution_combine_mode = combine_material::MAX});
```

`world::set_materials` gives the world a table of surfaces. A shape's or a triangle's `material_id` picks a surface,
which then stands in for the body's friction and restitution on that side of a contact.

### Sensors

```cpp
// a sensor reports its contacts and pushes nothing: a trigger volume
rigid_body::RigidBody *trigger = rigid_body::create(*world, {.shape = box_shape, .object_layer = moving,
                                                             .motion_type = motion_type::MOTION_KINEMATIC, .position = {14.0f, 0.5f, 0.0f},
                                                             .sensor = true});
```

### Changing a body's shape

```cpp
// a new shape: mass and bounds follow, and the constraints keep their anchors where they were on the body
shapes::Shape *bigger = box::create({.half_extents = {1.0f, 1.0f, 1.0f}});
rigid_body::set_shape(*world, body, bigger);

// a shape changed in place: its centre of mass before the change keeps the anchors put
const Vec3f previous_center_of_mass = vec3f::load(bigger->center_of_mass);
bigger->box.half_extents = {1.0f, 2.0f, 1.0f};
box::update(bigger);
rigid_body::update_shape(*world, body, previous_center_of_mass);
```

### Changing a body's layer

```cpp
// into another object layer: the body moves to that layer's broadphase tree and finds its pairs again
rigid_body::set_object_layer(*world, body, not_moving);
```

## Shapes

Shapes are created by each type's `create`, shared by any number of bodies, and freed by its `destroy` once nothing uses
them. Each body's pose places its shape: a shape's own geometry is centred on its origin.

### Convex shapes

```cpp
// the convex primitives, centred on their origin; a body takes its mass from their volume and density
shapes::Shape *ball = sphere::create({.radius = 0.5f});
shapes::Shape *crate = box::create({.half_extents = {0.5f, 0.25f, 0.5f}, .density = 500.0f});
// y is the long axis of a capsule and a cylinder; a capsule's radius is its convex radius
shapes::Shape *pill = capsule::create({.half_height_of_cylinder = 0.5f, .radius = 0.25f});
shapes::Shape *can = cylinder::create({.half_height = 0.4f, .radius = 0.3f});
```

A box's and a cylinder's `convex_radius` rounds their edges a little, which makes collisions faster and steadier.

### Convex hulls

```cpp
// any point cloud: build finds its hull, which any number of shapes can share
const float points[] = {
    -0.5f, 0.0f, -0.5f, 0.5f, 0.0f, -0.5f, 0.5f, 0.0f, 0.5f, -0.5f, 0.0f, 0.5f, 0.0f, 0.8f, 0.0f, 0.1f, 0.2f, 0.1f,
};
convex_hull::BuildResult result;
convex_hull::ConvexHullData *pyramid_hull = convex_hull::build({.positions = points, .num_positions = 6}, &result);
if (pyramid_hull == nullptr) printf("no hull: build result %u\n", (uint32_t)result);
shapes::Shape *pyramid = convex_hull::create({.data = pyramid_hull});
shapes::Shape *heavy_pyramid = convex_hull::create({.data = pyramid_hull, .density = 4000.0f});
```

### Planes

```cpp
// an infinite ground: point . normal + constant = 0, the normal its front
shapes::Shape *ground = plane::create({.plane = {{0.0f, 1.0f, 0.0f}, 0.0f}});
rigid_body::create(*world, {.shape = ground, .object_layer = not_moving, .motion_type = motion_type::MOTION_STATIC});
```

### Triangle meshes

```cpp
// static geometry from triangles, counter clockwise seen from the front; the builder makes the bvh and finds the
// edges a sliding body must not catch on
const float ramp_positions[] = {-2.0f, 0.0f, -2.0f, 2.0f, 0.0f, -2.0f, 2.0f, 1.0f, 2.0f, -2.0f, 1.0f, 2.0f};
const uint32_t ramp_indices[] = {0, 2, 1, 0, 3, 2};
triangle_mesh_data::TriangleMeshData *ramp_data =
    triangle_mesh_builder::build({.positions = ramp_positions, .num_positions = 4, .indices = ramp_indices, .num_indices = 6});
shapes::Shape *ramp = triangle_mesh::create({.data = ramp_data});
rigid_body::create(*world, {.shape = ramp, .object_layer = not_moving, .motion_type = motion_type::MOTION_STATIC,
                            .position = {20.0f, 0.0f, 0.0f}});
```

Meshes are for static bodies: they are hollow, with no volume, no inside and no mass.

### Compounds

```cpp
// several shapes as one body, each at its own pose; the shapes are borrowed and outlive the compound
const compound::CompoundShapeChild dumbbell_parts[] = {
    {.position = {-0.6f, 0.0f, 0.0f}, .quaternion = {0.0f, 0.0f, 0.0f, 1.0f}, .shape = ball},
    {.position = {0.6f, 0.0f, 0.0f}, .quaternion = {0.0f, 0.0f, 0.0f, 1.0f}, .shape = ball},
    {.position = {0.0f, 0.0f, 0.0f}, .quaternion = {0.0f, 0.0f, 0.70710678f, 0.70710678f}, .shape = pill},
};
shapes::Shape *dumbbell = compound::create({.children = dumbbell_parts, .num_children = 3});

// a static compound is built once into a bvh: for many children that never change
compound::CompoundShapeChild fence_posts[16];
for (uint32_t i = 0; i < 16; i++) fence_posts[i] = {.position = {(float)i, 0.4f, 0.0f}, .quaternion = {0.0f, 0.0f, 0.0f, 1.0f}, .shape = can};
shapes::Shape *fence = static_compound::create({.children = fence_posts, .num_children = 16});
rigid_body::create(*world, {.shape = fence, .object_layer = not_moving, .motion_type = motion_type::MOTION_STATIC,
                            .position = {-20.0f, 0.0f, 0.0f}});
```

### Decorators

```cpp
// wrappers over a shape they borrow: moved and turned within the body, scaled, or with the centre of mass moved
shapes::Shape *raised = transformed::create({.shape = crate, .position = {0.0f, 1.0f, 0.0f}, .quaternion = {0.0f, 0.0f, 0.0f, 1.0f}});
shapes::Shape *big_ball = scaled::create({.shape = ball, .scale = {2.0f, 2.0f, 2.0f}});
// a low centre of mass keeps a tall box upright
shapes::Shape *ballasted = offset_center_of_mass::create({.shape = crate, .offset = {0.0f, -0.2f, 0.0f}});
```

### Empty

```cpp
// a body with no geometry: a point for constraints to hold, or one waiting for its shape
shapes::Shape *nothing = empty::create();
rigid_body::create(*world, {.shape = nothing, .object_layer = moving, .motion_type = motion_type::MOTION_STATIC});
```

### Freeing shapes

```cpp
// shapes, hulls and meshes are yours: free them after the bodies that use them, wrappers before what they wrap
world::destroy(world);
empty::destroy(nothing);
transformed::destroy(raised);
scaled::destroy(big_ball);
offset_center_of_mass::destroy(ballasted);
compound::destroy(dumbbell);
static_compound::destroy(fence);
triangle_mesh::destroy(ramp);
triangle_mesh_data::destroy(ramp_data);
convex_hull::destroy(pyramid);
convex_hull::destroy(heavy_pyramid);
convex_hull::release(pyramid_hull);
sphere::destroy(ball);
box::destroy(crate);
capsule::destroy(pill);
cylinder::destroy(can);
plane::destroy(ground);
```

## Contact pairs

Events report every contact of every step. Contact pairs keep the few you care about, between frames: each pair of body
parts in contact, with a stable handle while the pair lasts, added, persisted or removed each frame. A frame can span
several world steps and the characters' moves between them.

```cpp
// only the pairs a body observing contacts is a side of are kept: the few a game reacts to, not every contact
rigid_body::RigidBody *ball = rigid_body::create(*world, {.shape = ball_shape, .object_layer = moving,
                                                           .motion_type = motion_type::MOTION_DYNAMIC, .position = {0.0f, 2.0f, 0.0f},
                                                           .restitution = 0.6f, .observes_contacts = true});
```

```cpp
// a frame of contact pairs: begin, record what moved (a world step's contact events, a character's move), end. a pair
// is added the frame it starts, persisted while it lasts, and removed the frame after it was last recorded
void frame(world::World &world, float frame_time, int steps) {
    contact_pairs::begin(world);
    for (int i = 0; i < steps; i++) {
        world::step(world, frame_time / (float)steps);
        contact_pairs::record_world_contacts(world, 0, 0);
    }
    contact_pairs::end(world);
}
```

```cpp
void print_pairs(const world::World &world) {
    const contact_pairs::ContactPairs &pairs = world.contact_pairs;
    for (uint32_t i = 0; i < pairs.active.length; i++) {
        const contact_pairs::ContactPair &pair = pairs.pairs[pairs.active[i]];
        printf("%s: body %u touches body %u at %.2f %.2f %.2f, %.2f m/s apart\n", pair.added ? "added" : "persisted",
               body_id::index(pair.a.body_id), body_id::index(pair.b.body_id), pair.point.x, pair.point.y, pair.point.z, pair.relative_velocity.y);
    }
    for (uint32_t i = 0; i < pairs.removed.length; i++) {
        const contact_pairs::ContactPair &pair = pairs.pairs[pairs.removed[i]];
        printf("removed: body %u and body %u\n", body_id::index(pair.a.body_id), body_id::index(pair.b.body_id));
    }
}
```

A removed pair's handle is reused after the next frame begins.

## Queries

Each query walks the broadphase under a filter and runs against every body it reaches, reporting to a collector:

- `collector::COLLECT_CLOSEST`: the nearest hit.
- `collector::COLLECT_ANY`: the first hit found, then stops.
- `collector::COLLECT_ALL`: every hit; `sort` orders them nearest first.

### Rays

```cpp
// down from above the crate, 10 m: the closest hit, its point and the surface normal there
filter::Filter filter = filter::create(world->settings.layers);
cast_ray::CastRayCollector collector = cast_ray::create_collector(collector::COLLECT_CLOSEST);
const cast_ray::Rayf ray = {vec3f::make(0.0f, 5.0f, 0.0f), vec3f::make(0.0f, -1.0f, 0.0f), 10.0f};
query::cast_ray(*world, collector, cast_ray::default_settings(), ray, filter);
if (const cast_ray::CastRayHit *hit = cast_ray::hit(collector)) {
    const Vec3f point = vec3f::scale_and_add(ray.origin, ray.direction, ray.length * hit->fraction);
    const rigid_body::RigidBody *body = rigid_body::get(*world, hit->body_id_b);
    const Vec3f normal = rigid_body::get_world_space_surface_normal(body, hit->sub_shape_id, point);
    printf("hit at y %.2f, normal y %.2f\n", vec3f::y(point), vec3f::y(normal));
}

// every hit along it, nearest first
cast_ray::CastRayCollector all = cast_ray::create_collector(collector::COLLECT_ALL);
query::cast_ray(*world, all, cast_ray::default_settings(), ray, filter);
cast_ray::sort(all);
printf("%u hits\n", all.hits.length);

// the collectors and the filter own memory
cast_ray::release(all);
cast_ray::release(collector);
filter::release(filter);
```

### Points

```cpp
// which bodies hold this point
collide_point::CollidePointCollector inside = collide_point::create_collector(collector::COLLECT_ALL);
query::collide_point(*world, inside, collide_point::default_settings(), vec3f::make(0.2f, 0.5f, 0.1f), filter);
```

### Overlaps

```cpp
// everything a 2 m sphere at the crate's corner overlaps, and how deep: an explosion's reach
shapes::Shape *blast = sphere::create({.radius = 2.0f});
filter::Filter filter = filter::create(world->settings.layers);
collide_shape::CollideShapeCollector overlaps = collide_shape::create_collector(collector::COLLECT_ALL);
const Transformf at = {{1.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
query::collide_shape(*world, overlaps, collide_shape::default_settings(), blast, at, filter);
for (uint32_t i = 0; i < overlaps.hits.length; i++) {
    printf("overlaps body %u by %.2f m\n", body_id::index(overlaps.hits[i].body_id_b), overlaps.hits[i].penetration);
}
```

### Sweeps

```cpp
// the sphere swept along a path: where it first touches, for a projectile with a size
filter::Filter only_moving = filter::create(world->settings.layers);
filter::disable_object_layer(only_moving, world->settings.layers, not_moving);
cast_shape::CastShapeCollector sweep = cast_shape::create_collector(collector::COLLECT_CLOSEST);
const shapes::Shape *small_ball = blast;
const Transformf start = {{-10.0f, 0.5f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {0.25f, 0.25f, 0.25f}};
query::cast_shape(*world, sweep, cast_shape::default_settings(), small_ball, start, vec3f::make(20.0f, 0.0f, 0.0f), only_moving);
if (const cast_shape::CastShapeHit *hit = cast_shape::hit(sweep)) printf("stops after %.2f m\n", 20.0f * hit->fraction);
```

### Bounds only

`broadphase_query` finds bodies by their bounds alone, without testing their shapes, as Jolt's `BroadPhaseQuery`: a
box, a sphere, a point or an oriented box for the bodies it overlaps, and a ray or a moving box for the bodies it enters
and where. They take the same filter. The overlaps' collector takes `COLLECT_ALL`, `COLLECT_ANY` or `COLLECT_CUSTOM`, as
an overlap has no nearest; the casts' takes `COLLECT_CLOSEST` too.

```cpp
// the bodies whose bounds a 1 m box around the origin overlaps, their shapes not tested
filter::Filter filter = filter::create(world->settings.layers);
broadphase_query::BodyCollector near_origin = broadphase_query::create_body_collector(collector::COLLECT_ALL);
broadphase_query::collide_aabb(*world, near_origin, aabox::make(vec3f::splat(-0.5f), vec3f::splat(0.5f)), filter);

// a sphere and a turned box, likewise
broadphase_query::reset(near_origin);
broadphase_query::collide_sphere(*world, near_origin, vec3f::make(0.0f, 3.0f, 0.0f), 2.1f, filter);
const Quatf turn = quatf::set_axis_angle(vec3f::make(0.0f, 1.0f, 0.0f), 0.7f);
const OrientedBox turned = {mat4f::from_rotation_translation(turn, vec3f::make(0.0f, 0.5f, 0.0f)), vec3f::make(0.2f, 0.2f, 0.2f)};
broadphase_query::BodyCollector in_turned = broadphase_query::create_body_collector(collector::COLLECT_ALL);
broadphase_query::collide_oriented_box(*world, in_turned, turned, filter);

// a box swept down from above: the first body whose bounds it enters, and where
broadphase_query::BodyCastCollector first = broadphase_query::create_body_cast_collector(collector::COLLECT_CLOSEST);
const AABox falling = aabox::make(vec3f::make(-0.1f, 4.9f, -0.1f), vec3f::make(0.1f, 5.1f, 0.1f));
broadphase_query::cast_aabb(*world, first, falling, vec3f::make(0.0f, -10.0f, 0.0f), filter);
if (const broadphase_query::BodyCastHit *hit = broadphase_query::hit(first)) printf("bounds entered at %.2f\n", hit->fraction);
```

### Filters

```cpp
// a filter starts with every layer, then narrows: by object or broadphase layer, by group and mask, and by a
// function of the body
filter::Filter filter = filter::create(world->settings.layers);
filter::disable_object_layer(filter, world->settings.layers, moving);
filter.body_filter = not_this_body;
filter.body_filter_context = ground;
cast_ray::CastRayCollector collector = cast_ray::create_collector(collector::COLLECT_ANY);
query::cast_ray(*world, collector, cast_ray::default_settings(), {vec3f::make(0.0f, 5.0f, 0.0f), vec3f::make(0.0f, -1.0f, 0.0f), 10.0f},
                filter);
// the crate's layer is off and the ground is filtered out: nothing hit
```

```cpp
/** keeps every body but one: the caller's own, say */
bool not_this_body(const rigid_body::RigidBody *body, void *context) { return body != static_cast<const rigid_body::RigidBody *>(context); }
```

## Constraints

A constraint holds two bodies relative to each other. `<type>_constraint::create(world, body_a, body_b, settings)`
returns its id. `get(world, id)` returns its record for changing it (limits, motors), valid until the next create of
that type, so get it again rather than keeping it. A constraint goes with either of its bodies, or with
`remove(world, id)`.

Points and axes are in world space by default. With `.space = constraints::SPACE_LOCAL` they are relative to each body.
The two sides of a constraint are always `_a` and `_b`, as its bodies are.

### Hinge

```cpp
// a door on a hinge along y at its left edge, opening a quarter turn at most, driven open by its motor. a motor's
// torque is unbounded unless limited, and an unbounded motor driving into a limit pushes past it: give it a limit
rigid_body::RigidBody *frame = rigid_body::create(*world, {.shape = nothing, .object_layer = not_moving,
                                                           .motion_type = motion_type::MOTION_STATIC, .position = {0.0f, 1.0f, 0.0f}});
rigid_body::RigidBody *door = rigid_body::create(*world, {.shape = door_shape, .object_layer = moving,
                                                          .motion_type = motion_type::MOTION_DYNAMIC, .position = {0.5f, 1.0f, 0.0f}});
const constraint_id::ConstraintId hinge_id = hinge_constraint::create(*world, frame, door,
                                                                      {.point_a = {0.0f, 1.0f, 0.0f},
                                                                       .point_b = {0.0f, 1.0f, 0.0f},
                                                                       .hinge_axis_a = {0.0f, 1.0f, 0.0f},
                                                                       .hinge_axis_b = {0.0f, 1.0f, 0.0f},
                                                                       .normal_axis_a = {1.0f, 0.0f, 0.0f},
                                                                       .normal_axis_b = {1.0f, 0.0f, 0.0f},
                                                                       .limits_min = 0.0f,
                                                                       .limits_max = 0.5f * scalar::PI,
                                                                       .motor_settings = {.min_torque_limit = -100.0f, .max_torque_limit = 100.0f}});

// the id names it; get gives its record, which changes it, until the next create of a hinge
hinge_constraint::HingeConstraint *hinge = hinge_constraint::get(*world, hinge_id);
hinge_constraint::set_motor_state(*hinge, motor_settings::VELOCITY);
hinge_constraint::set_target_angular_velocity(*hinge, 2.0f);
```

```cpp
// gone, or with either body: a stale id resolves to nothing
hinge_constraint::remove(*world, hinge_id);
```

### Point

```cpp
// a pendulum: a ball held 2 m below a point it swings about. a static body with an empty shape is a fixed anchor
rigid_body::RigidBody *anchor = rigid_body::create(*world, {.shape = nothing, .object_layer = not_moving,
                                                            .motion_type = motion_type::MOTION_STATIC, .position = {10.0f, 5.0f, 0.0f}});
rigid_body::RigidBody *bob = rigid_body::create(*world, {.shape = ball_shape, .object_layer = moving,
                                                         .motion_type = motion_type::MOTION_DYNAMIC, .position = {12.0f, 5.0f, 0.0f}});
point_constraint::create(*world, anchor, bob, {.point_a = {10.0f, 5.0f, 0.0f}, .point_b = {10.0f, 5.0f, 0.0f}});
```

### Distance

```cpp
// a bungee: no further than 3 m, pulled back by a spring; min and max of -1 keep the distance it starts at
rigid_body::RigidBody *jumper = rigid_body::create(*world, {.shape = ball_shape, .object_layer = moving,
                                                            .motion_type = motion_type::MOTION_DYNAMIC, .position = {14.0f, 5.0f, 0.0f}});
distance_constraint::create(*world, anchor, jumper,
                            {.point_a = {10.0f, 5.0f, 0.0f},
                             .point_b = {14.0f, 5.0f, 0.0f},
                             .min_distance = 0.0f,
                             .max_distance = 3.0f,
                             .spring_settings = {.mode = spring_settings::FREQUENCY_AND_DAMPING, .frequency_or_stiffness = 2.0f, .damping = 0.5f}});
```

### Slider

```cpp
// a lift: a platform that slides along y only, its position motor holding it 2 m up the rail. a position motor
// drives through a spring, which a load sags: this one is stiff enough (10 Hz) to sag by millimetres
rigid_body::RigidBody *rail = rigid_body::create(*world, {.shape = nothing, .object_layer = not_moving,
                                                          .motion_type = motion_type::MOTION_STATIC, .position = {20.0f, 0.0f, 0.0f}});
rigid_body::RigidBody *platform = rigid_body::create(*world, {.shape = block_shape, .object_layer = moving,
                                                              .motion_type = motion_type::MOTION_DYNAMIC, .position = {20.0f, 0.0f, 0.0f}});
const constraint_id::ConstraintId lift_id = slider_constraint::create(*world, rail, platform,
                                                                      {.slider_axis_a = {0.0f, 1.0f, 0.0f},
                                                                       .slider_axis_b = {0.0f, 1.0f, 0.0f},
                                                                       .normal_axis_a = {1.0f, 0.0f, 0.0f},
                                                                       .normal_axis_b = {1.0f, 0.0f, 0.0f},
                                                                       .auto_detect_point = true,
                                                                       .limits_min = 0.0f,
                                                                       .limits_max = 5.0f,
                                                                       .motor_settings = {.spring = {.frequency_or_stiffness = 10.0f, .damping = 1.0f}}});
slider_constraint::SliderConstraint *lift = slider_constraint::get(*world, lift_id);
slider_constraint::set_motor_state(*lift, motor_settings::POSITION);
slider_constraint::set_target_position(*lift, 2.0f);
```

### Fixed

```cpp
// a weld: two bodies held as they are, moving as one
rigid_body::RigidBody *left = rigid_body::create(*world, {.shape = block_shape, .object_layer = moving,
                                                          .motion_type = motion_type::MOTION_DYNAMIC, .position = {30.0f, 3.0f, 0.0f}});
rigid_body::RigidBody *right = rigid_body::create(*world, {.shape = block_shape, .object_layer = moving,
                                                           .motion_type = motion_type::MOTION_DYNAMIC, .position = {31.0f, 3.0f, 0.0f}});
fixed_constraint::create(*world, left, right, {.auto_detect_point = true});
```

### Swing-twist and cone

```cpp
// a shoulder: the arm swings within a 45 degree cone about the shoulder's twist axis (x here), twisting up to 30
// degrees either way. a cone constraint is the same without the twist limits
rigid_body::RigidBody *torso = rigid_body::create(*world, {.shape = nothing, .object_layer = not_moving,
                                                           .motion_type = motion_type::MOTION_STATIC, .position = {40.0f, 3.0f, 0.0f}});
rigid_body::RigidBody *arm = rigid_body::create(*world, {.shape = ball_shape, .object_layer = moving,
                                                         .motion_type = motion_type::MOTION_DYNAMIC, .position = {40.5f, 3.0f, 0.0f}});
swing_twist_constraint::create(*world, torso, arm,
                               {.point_a = {40.0f, 3.0f, 0.0f},
                                .point_b = {40.0f, 3.0f, 0.0f},
                                .twist_axis_a = {1.0f, 0.0f, 0.0f},
                                .plane_axis_a = {0.0f, 0.0f, 1.0f},
                                .twist_axis_b = {1.0f, 0.0f, 0.0f},
                                .plane_axis_b = {0.0f, 0.0f, 1.0f},
                                .normal_half_cone_angle = 0.25f * scalar::PI,
                                .plane_half_cone_angle = 0.25f * scalar::PI,
                                .twist_min_angle = -scalar::PI / 6.0f,
                                .twist_max_angle = scalar::PI / 6.0f});
```

```cpp
cone_constraint::create(*world, cone_anchor, cone_arm,
                        {.point_a = {40.0f, 6.0f, 0.0f},
                         .twist_axis_a = {1.0f, 0.0f, 0.0f},
                         .point_b = {40.0f, 6.0f, 0.0f},
                         .twist_axis_b = {1.0f, 0.0f, 0.0f},
                         .half_cone_angle = 0.25f * scalar::PI});
```

### Six degrees of freedom

```cpp
// every axis its own: here free to slide along x within a metre either way and to turn about y, the rest locked
rigid_body::RigidBody *base = rigid_body::create(*world, {.shape = nothing, .object_layer = not_moving,
                                                          .motion_type = motion_type::MOTION_STATIC, .position = {50.0f, 3.0f, 0.0f}});
rigid_body::RigidBody *cart = rigid_body::create(*world, {.shape = block_shape, .object_layer = moving,
                                                          .motion_type = motion_type::MOTION_DYNAMIC, .position = {50.0f, 3.0f, 0.0f}});
six_dof_constraint::SixDOFConstraintSettings rails;
rails.point_a = {50.0f, 3.0f, 0.0f};
rails.point_b = {50.0f, 3.0f, 0.0f};
for (uint32_t axis = 0; axis < six_dof_constraint::NUM; axis++) rails.limit_min[axis] = rails.limit_max[axis] = 0.0f;
rails.limit_min[six_dof_constraint::TRANSLATION_X] = -1.0f;
rails.limit_max[six_dof_constraint::TRANSLATION_X] = 1.0f;
rails.limit_min[six_dof_constraint::ROTATION_Y] = -scalar::PI;
rails.limit_max[six_dof_constraint::ROTATION_Y] = scalar::PI;
six_dof_constraint::create(*world, base, cart, rails);
rigid_body::set_linear_velocity(*world, cart, vec3f::make(5.0f, 0.0f, 0.0f));
```

### Motors and springs

A motor drives a hinge, slider, swing-twist or six-dof axis. `motor_settings::VELOCITY` drives to a target velocity;
`POSITION` drives to a target position through the motor's spring. A motor's torque and force are unbounded unless
limited by its `MotorSettings`. Limit one that drives into a limit, or it pushes past the limit. A spring's
`frequency_or_stiffness` above 0 makes a limit or a distance soft.

## Character controller

`kcc` moves a character through the world by sweeps: it walks up slopes and stairs, slides along walls, sticks to the
floor going down, and stands on moving bodies. It is not a rigid body. Its optional inner body, a kinematic body that
follows it, is its presence in the world for bodies, queries and contact pairs.

```cpp
// a capsule standing on its feet: the character's position is its feet, the capsule raised 0.8 m to stand on them.
// the inner body is its presence in the world, which rigid bodies and queries see and contact pairs name it by
shapes::Shape *standing_capsule = capsule::create({.half_height_of_cylinder = 0.5f, .radius = 0.3f});
shapes::Shape *standing = transformed::create({.shape = standing_capsule, .position = {0.0f, 0.8f, 0.0f}});
kcc::KCCSettings character_settings;
character_settings.shape = standing;
character_settings.max_slope_angle = 45.0f * scalar::DEGREES_TO_RADIANS;
character_settings.inner_rigid_body_shape = standing;
character_settings.inner_rigid_body_object_layer = moving;
character_settings.observes_contacts = true;
kcc::KCC *character = kcc::create(character_settings, vec3f::make(0.0f, 0.0f, 0.0f), quatf::identity());
kcc::add(*world, *character);

// what the character collides with: everything its layer does
filter::Filter filter = filter::create(world->settings.layers);
filter::set_from_layer(filter, world->settings.layers, moving, 0xffffffffu, 0xffffffffu);
```

```cpp
// a frame of a game: the velocity from the ground's when standing, else falling on, plus the input; then the
// character moves (sliding along walls, up stairs, sticking to floors going down), then the world steps
kcc::UpdateSettings update_settings;
for (int frame = 0; frame < 180; frame++) {
    const Vec3f input = vec3f::make(3.0f, 0.0f, 0.0f);
    Vec3f velocity = character->ground.state == kcc::ON_GROUND
                         ? character->ground.velocity
                         : vec3f::scale(character->up, vec3f::dot(character->linear_velocity, character->up));
    velocity = vec3f::scale_and_add(velocity, gravity, delta_time);
    character->linear_velocity = vec3f::add(velocity, input);
    kcc::update(*world, *character, delta_time, gravity, update_settings, &listener, filter);
    world::step(*world, delta_time);
}
printf("at x %.2f y %.2f, %s\n", vec3f::x(character->position), vec3f::y(character->position),
       character->ground.state == kcc::ON_GROUND ? "on the ground" : "not on the ground");
```

`character.ground` says what it stands on: its `state` (`ON_GROUND`, `ON_STEEP_GROUND`, `NOT_SUPPORTED`, `IN_AIR`), its
`body_id`, the `normal` and the ground's `velocity`.

### Changing shape

```cpp
// a shorter shape, on the same feet, where it fits: refused, false, if it would sink deeper than the given depth
shapes::Shape *crouching_capsule = capsule::create({.half_height_of_cylinder = 0.2f, .radius = 0.3f});
shapes::Shape *crouching = transformed::create({.shape = crouching_capsule, .position = {0.0f, 0.5f, 0.0f}});
const bool crouched = kcc::set_shape(*world, *character, crouching, filter, &listener, 0.01f);
if (crouched) kcc_policy::set(policy, crate->id, {.can_receive_impulses = false});
```

Give every shape the character's feet as its origin, and swap them freely.

### Listeners and policies

How the character treats the bodies it meets is a `CharacterListener`'s: crashcat's and Jolt's callbacks, each
optional. `on_adjust_body_velocity` changes a body's velocity as the character sees it (a conveyor's belt),
`on_contact_validate` lets the character pass through a contact, `on_contact_added` and `on_contact_persisted` set a
contact's push settings, `on_contact_solve` changes the character's velocity against a contact, and
`on_contact_removed` reports a contact gone. Pass the listener to each call, or null for none.

```cpp
// the hooks themselves, as crashcat's and Jolt's: a listener's callbacks, each optional, called as the character meets
// bodies. here Jolt's sample's rule for standing still: on a still, walkable surface, the character does not slide
kcc::CharacterListener standing_still = {};
standing_still.on_contact_solve = [](const kcc::KCC &self, const rigid_body::RigidBody &, sub_shape::SubShapeId, Vec3f, Vec3f contact_normal,
                                     Vec3f contact_velocity, Vec3f, Vec3f &io_new_character_velocity, void *) {
    if (vec3f::squared_length(contact_velocity) <= 1e-12f && !kcc::is_slope_too_steep(self, contact_normal)) {
        io_new_character_velocity = vec3f::zero();
    }
};
```

A policy is per body data that `kcc_policy::listener` makes into a listener:

```cpp
// how characters treat the bodies they meet, by body, the rest taking the policy's defaults; one policy serves any
// number of characters. here the crate may be pushed (until the character crouches, below)
kcc_policy::Policy policy = kcc_policy::create();
kcc_policy::set(policy, crate->id, {.can_receive_impulses = true});
// the character asks a listener: this one reads the policy, changes to it taking effect at once
const kcc::CharacterListener listener = kcc_policy::listener(policy);
```

A body's policy: `ignore` passes through it, `can_push_character` false blocks without moving the character,
`can_receive_impulses` false keeps the character from pushing it, `slide_on_walkable` false stops the character on a
still, walkable surface of it, and `surface_linear_velocity` and `surface_angular_velocity` move the character as a
conveyor's belt would. The JS package's characters take policies: a listener's callbacks would have to call from the
wasm into JS.

### Your own loop

```cpp
// update is the whole move in one call; the toolkit is its phases, for a loop of your own that edits the contacts
// between collide and solve. here a one-way platform overhead, jumped up through: from below, its contacts and the
// sweep's hit on it are passed over
shapes::Shape *platform_shape = box::create({.half_extents = {1.5f, 0.1f, 1.5f}});
rigid_body::RigidBody *platform = rigid_body::create(*world, {.shape = platform_shape, .object_layer = not_moving,
                                                              .motion_type = motion_type::MOTION_STATIC,
                                                              .position = {vec3f::x(character->position), 1.8f, 0.0f}});
const auto from_below = [&](const kcc::CharacterContact &contact) {
    return contact.body_id == platform->id && vec3f::y(contact.contact_normal) < 0.5f;
};
character->linear_velocity = vec3f::make(0.0f, 7.0f, 0.0f);
for (int frame = 0; frame < 90; frame++) {
    character->linear_velocity = vec3f::scale_and_add(character->linear_velocity, gravity, delta_time);
    Vec3f position = character->position;
    Vec3f velocity = character->linear_velocity;
    float time_remaining = delta_time;
    for (uint32_t iteration = 0; iteration < character->max_collision_iterations && time_remaining >= character->min_time_remaining; iteration++) {
        kcc::collide(*world, *character, position, vec3f::normalize(velocity), filter, &listener);
        for (uint32_t i = 0; i < character->contacts.length; i++) {
            if (from_below(character->contacts[i])) character->contacts[i].was_discarded = true;
        }
        Vec3f displacement;
        float time_simulated = kcc::solve(*world, *character, velocity, delta_time, time_remaining, &listener, displacement);
        kcc::CharacterContact hit;
        if (kcc::cast(hit, *world, *character, position, displacement, filter, &listener) && !from_below(hit)) {
            displacement = vec3f::scale(displacement, hit.fraction);
            time_simulated *= hit.fraction;
        }
        position = vec3f::add(position, displacement);
        time_remaining -= time_simulated;
    }
    kcc::set_position(*world, *character, position);
    kcc::update_ground(*world, *character, delta_time, false, &listener);
    // landed, coming down
    if (character->ground.state == kcc::ON_GROUND && vec3f::y(character->linear_velocity) <= 0.0f) {
        character->linear_velocity = vec3f::make(0.0f, 0.0f, 0.0f);
    }
    world::step(*world, delta_time);
}
```

`collide`, `solve` and `cast` are a move's phases, as `move` runs them each collision iteration. Between `collide` and
`solve` a contact's `was_discarded` and `linear_velocity` are yours: the solve, the sweep and the ground pass over a
discarded one. `update` is the same phases with contact tracking (`begin_contact_tracking`, `end_contact_tracking`),
`cancel_velocity_towards_steep_slopes` before, and `stick_to_floor` and `walk_stairs` after.

### Contacts

```cpp
// what the character met in its last update (or refresh_contacts, or set_shape): each contact added or persisted as
// it was first touched, and each gone by the end
uint32_t touching = 0;
for (uint32_t i = 0; i < character->contact_events.length; i++) {
    const kcc::ContactEvent &event = character->contact_events[i];
    if (event.kind != kcc::CONTACT_REMOVED) touching++;
}
printf("touching %u bodies\n", touching);
```

```cpp
// its contacts as data, through its inner body, in a contact pairs frame
contact_pairs::begin(*world);
kcc::refresh_contacts(*world, *character, filter, &listener);
contact_pairs::record_character_contacts(*world, *character);
contact_pairs::end(*world);
```

```cpp
kcc::remove(*world, *character);
kcc::destroy(character);
```

## Custom shapes

A shape type of your own takes one of eight user slots, `shapes::SHAPE_USER_1` to `SHAPE_USER_8`. Its def gives its
functions, its shapes carry your payload in `shape->user.data`, and `dispatch::init` puts it beside the built-ins. This
one is an ellipsoid:

```cpp
// an ellipsoid: a convex shape tumble does not have, in user slot 1. its payload is the shape's `user.data`
constexpr shapes::ShapeType SHAPE_ELLIPSOID = shapes::SHAPE_USER_1;

struct Ellipsoid {
    Float3 radii;
    float density;
};

const Ellipsoid &ellipsoid_of(const shapes::Shape *shape) { return *static_cast<const Ellipsoid *>(shape->user.data); }
```

A convex type gives a support: gjk and epa need nothing else to collide it with every other convex shape.

```cpp
// gjk and epa see a convex shape only through its support: the farthest point in a direction. for radii r and a local
// direction d it is r^2 d / |r d|, per component
Vec3f ellipsoid_support(support::Support &support, Vec3f direction) {
    const Vec3f local = support::to_local_direction(support, direction);
    const Ellipsoid &ellipsoid = *static_cast<const Ellipsoid *>(support.user.data);
    const Vec3f radii = vec3f::multiply(vec3f::load(ellipsoid.radii), vec3f::abs(support.user.scale));
    const Vec3f stretched = vec3f::multiply(radii, local);
    const float length = vec3f::length(stretched);
    const Vec3f point = length > 0.0f ? vec3f::scale(vec3f::multiply(radii, stretched), 1.0f / length) : vec3f::zero();
    return support::to_caller_space(support, support::add_radius_along(point, local, support.add_radius));
}

// no rounding, so both modes give the whole shape
void ellipsoid_set_support(support::Support &out, const shapes::Shape *shape, support::SupportFunctionMode, Vec3f scale) {
    out.get_support = ellipsoid_support;
    out.has_transform = false;
    out.add_radius = 0.0f;
    out.convex_radius = 0.0f;
    out.user.data = shape->user.data;
    out.user.scale = scale;
}
```

```cpp
void ellipsoid_mass_properties(mass_properties::MassProperties &out, const shapes::Shape *shape) {
    const Ellipsoid &ellipsoid = ellipsoid_of(shape);
    const float a = ellipsoid.radii.x, b = ellipsoid.radii.y, c = ellipsoid.radii.z;
    out.mass = 4.0f / 3.0f * scalar::PI * a * b * c * ellipsoid.density;
    const float k = out.mass / 5.0f;
    out.inertia = {{k * (b * b + c * c), 0, 0, 0, 0, k * (a * a + c * c), 0, 0, 0, 0, k * (a * a + b * b), 0, 0, 0, 0, 1.0f}};
}

// the gradient of x^2/a^2 + y^2/b^2 + z^2/c^2 at the point
void ellipsoid_surface_normal(shapes::SurfaceNormalResult &io, const shapes::Shape *shape, sub_shape::SubShapeId) {
    const Vec3f radii = vec3f::load(ellipsoid_of(shape).radii);
    const Vec3f gradient = vec3f::divide(io.position, vec3f::multiply(radii, radii));
    io.normal = vec3f::squared_length(gradient) > 0.0f ? vec3f::normalize(gradient) : vec3f::axis_y();
}

// curved: no face to build a manifold from, so contacts are single points
void ellipsoid_supporting_face(shapes::SupportingFaceResult &io, Vec3f, const shapes::Shape *, sub_shape::SubShapeId) {
    io.face.num_vertices = 0;
}

int32_t ellipsoid_material(const shapes::Shape *shape, sub_shape::SubShapeId) { return shape->material_id; }

float ellipsoid_inner_radius(const shapes::Shape *shape) {
    const Float3 &r = ellipsoid_of(shape).radii;
    return scalar::min(r.x, scalar::min(r.y, r.z));
}

void ellipsoid_collide_point(collide_point::CollidePointCollector &collector, const collide_point::CollidePointSettings &, Vec3f point,
                             const shapes::Shape *shape, const sub_shape::SubShapeIdBuilder &sub_shape_id, const Transformf &transform) {
    // into the shape's space, then inside when the scaled coordinates are within the unit sphere
    const Vec3f local = vec3f::transform_quat(vec3f::subtract(point, vec3f::load(transform.position)), quatf::conjugate(quatf::load(transform.quaternion)));
    const Vec3f radii = vec3f::multiply(vec3f::load(ellipsoid_of(shape).radii), vec3f::abs(vec3f::load(transform.scale)));
    if (vec3f::squared_length(vec3f::divide(local, radii)) > 1.0f) return;
    collide_point::CollidePointHit hit = collide_point::create_hit();
    hit.sub_shape_id_b = sub_shape_id.value;
    hit.body_id_b = collector.body_id_b;
    hit.material_id = shape->material_id;
    collide_point::add_hit(collector, hit);
}

// three rings, one in each axis plane
void ellipsoid_draw(debug::DebugBuffers &out, const shapes::Shape *shape, const Transformf &transform, Vec3f color) {
    const Vec3f radii = vec3f::multiply(vec3f::load(ellipsoid_of(shape).radii), vec3f::load(transform.scale));
    const Quatf rotation = quatf::load(transform.quaternion);
    const Vec3f centre = vec3f::load(transform.position);
    for (int plane = 0; plane < 3; plane++) {
        Vec3f previous = vec3f::zero();
        for (int i = 0; i <= 24; i++) {
            const float angle = 2.0f * scalar::PI * (float)i / 24.0f;
            const float u = scalar::cos(angle), v = scalar::sin(angle);
            const Vec3f ring = plane == 0 ? vec3f::make(u, v, 0.0f) : plane == 1 ? vec3f::make(u, 0.0f, v) : vec3f::make(0.0f, u, v);
            const Vec3f point = vec3f::add(centre, vec3f::transform_quat(vec3f::multiply(ring, radii), rotation));
            if (i > 0) debug::line(out, previous, point, color);
            previous = point;
        }
    }
}
```

```cpp
shapes::ShapeDef ellipsoid_def() {
    shapes::ShapeDef def = {};
    def.type = SHAPE_ELLIPSOID;
    def.set_support = ellipsoid_set_support;
    def.compute_mass_properties = ellipsoid_mass_properties;
    def.get_surface_normal = ellipsoid_surface_normal;
    def.get_supporting_face = ellipsoid_supporting_face;
    def.get_material = ellipsoid_material;
    def.get_inner_radius = ellipsoid_inner_radius;
    // a convex type's rays go through its support
    def.cast_ray = convex::cast_ray_vs_convex;
    def.collide_point = ellipsoid_collide_point;
    def.draw = ellipsoid_draw;
    // define_shape fills what a def leaves out with the defaults
    return shapes::define_shape(def);
}

/** a shape of the type: its payload, and the bounds, centre of mass and volume every shape carries */
shapes::Shape *create_ellipsoid(Ellipsoid *ellipsoid) {
    shapes::Shape *shape = shapes::allocate(SHAPE_ELLIPSOID);
    if (shape == nullptr) return nullptr;
    shape->user.data = ellipsoid;
    const Float3 &r = ellipsoid->radii;
    shape->aabb = {{-r.x, -r.y, -r.z}, {r.x, r.y, r.z}};
    shape->center_of_mass = {0.0f, 0.0f, 0.0f};
    shape->volume = 4.0f / 3.0f * scalar::PI * r.x * r.y * r.z;
    shape->material_id = -1;
    return shape;
}
```

```cpp
// the type beside the built-ins: it collides with every convex type, mesh and plane, in compounds and under
// wrappers. a pair of its own (an ellipsoid against a mesh by a faster rule, say) goes in after, over the generic one
const shapes::ShapeDef def = ellipsoid_def();
dispatch::init(&def, 1);
```

```cpp
// an egg, 0.3 m wide and 0.5 m tall, dropped on its side: it rolls to lie on its long axis
static Ellipsoid egg = {.radii = {0.3f, 0.5f, 0.3f}, .density = 1000.0f};
shapes::Shape *egg_shape = create_ellipsoid(&egg);
rigid_body::RigidBody *body = rigid_body::create(*world, {.shape = egg_shape, .object_layer = moving,
                                                          .motion_type = motion_type::MOTION_DYNAMIC, .position = {0.0f, 2.0f, 0.0f},
                                                          .quaternion = {0.0f, 0.0f, 0.38268343f, 0.92387953f}});
for (int i = 0; i < 300; i++) world::step(*world, 1.0f / 60.0f);
printf("the egg rests at y %.2f\n", body->position.y);
```

A type that is not convex leaves `set_support` null and gives its own pairs with `shapes::set_collide_shape_fn` and
`shapes::set_cast_shape_fn` after `dispatch::init`.

A type whose sub shape ids name state you keep between steps, such as a pool of hits you clear before each step, sets
`skips_pair_cache`. A pair that barely moved normally reuses last step's contacts, which would name state you have
since cleared. With it set, the type's pairs always collide anew.

## Debug drawing

```cpp
// buffers that grow as they are drawn into and keep their memory when cleared, for a frame's worth each frame
debug::DebugBuffers *buffers = debug::create();
debug::clear(*buffers);
// the bodies, coloured by motion type, with their velocities
debug::bodies(*buffers, *world, {.color_mode = debug::COLOR_MOTION_TYPE, .show_linear_velocity = true});
// each contact's points and normal, and the constraints' anchors and axes
debug::contacts(*buffers, *world);
debug::joints(*buffers, *world, {});
// a shape anywhere: a preview of where something would go
debug::shape(*buffers, ball_shape, vec3f::make(3.0f, 0.5f, 0.0f), quatf::identity(), {.color = {0.0f, 1.0f, 0.0f}});
draw_lines(*buffers);
printf("%u lines\n", buffers->line_count);
debug::release(buffers);
```

```cpp
// line_count lines of two vertices each, a vertex x y z r g b: a vertex array as it stands
void draw_lines(const debug::DebugBuffers &buffers) { upload_line_vertices(buffers.lines.data, buffers.line_count * 2); }
```

`debug::character_contacts` draws a character's contacts. A custom shape draws through its def's `draw`, with
`debug::line`, or as its bounds without one.

## Determinism

```cpp
// the same scene, made in the same order and stepped by the same steps, is the same to the last bit: in another
// world, another run, another machine, or the browser
world::World *first = make_pile(ground_shape, ball_shape);
world::World *second = make_pile(ground_shape, ball_shape);
for (int i = 0; i < 300; i++) {
    world::step(*first, 1.0f / 60.0f);
    world::step(*second, 1.0f / 60.0f);
}
bool same = true;
for (uint32_t i = 0; i < first->bodies.pool.length; i++) {
    const rigid_body::RigidBody &a = *first->bodies.pool[i];
    const rigid_body::RigidBody &b = *second->bodies.pool[i];
    same = same && memcmp(&a.position, &b.position, sizeof(Float3)) == 0 && memcmp(&a.quaternion, &b.quaternion, sizeof(Float4)) == 0;
}
printf("%s\n", same ? "bit for bit the same" : "they differ");
```

tumble uses no fused multiply-add, no fast math and none of the platform's math library: its trigonometry is its own.
So the same steps give the same bits natively and in WebAssembly, with SIMD and without; its tests hold every build to
one digest of a simulated pile. To keep them:

- Build tumble with its own flags, as its CMake does: `-ffp-contract=off` (MSVC: `/fp:precise`), no fast math.
- Build your code that calls tumble's math headers with `-ffp-contract=off` too. The math is inline, so it is compiled
  with your flags.
- Create bodies and constraints, and step, in the same order with the same values.
