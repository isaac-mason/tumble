#include "app/app.h"
#include "app/examples.h"
#include "app/random.h"
#include "imgui.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/plane.h"

using namespace tumble;

static constexpr float SPAWN_HEIGHT = 10.0f;
static constexpr float SPAWN_AREA = 2.5f;

static struct {
    app::Layers layers;
    shapes::Shape *cube;
    collections::Vec<rigid_body::RigidBody *> cubes;
    int count;
    Random random;
} state;

static void spawn(App &app) {
    rigid_body::RigidBodySettings settings;
    settings.shape = state.cube;
    settings.object_layer = state.layers.moving;
    settings.motion_type = motion_type::MOTION_DYNAMIC;
    settings.position = {random::range(state.random, -SPAWN_AREA, SPAWN_AREA), random::range(state.random, 0.0f, SPAWN_HEIGHT),
                         random::range(state.random, -SPAWN_AREA, SPAWN_AREA)};
    settings.friction = 0.5f;
    settings.mass = 1.0f;
    state.cubes.push(rigid_body::create(*app.world, settings));
}

static void match_count(App &app) {
    while ((int)state.cubes.length < state.count) spawn(app);
    while ((int)state.cubes.length > state.count) rigid_body::remove(*app.world, state.cubes[--state.cubes.length]);
}

static void init(App &app) {
    state.cubes.length = 0;
    state.count = 200;
    state.random = {1};
    // strong gravity, for a livelier heap
    app.world = app::create_world(state.layers, {0.0f, -50.0f, 0.0f});
    rigid_body::RigidBodySettings ground;
    ground.shape = app::keep(app, plane::create({.plane = {{0.0f, 1.0f, 0.0f}, 0.0f}, .half_extent = 50.0f}));
    ground.object_layer = state.layers.not_moving;
    ground.motion_type = motion_type::MOTION_STATIC;
    ground.friction = 0.5f;
    rigid_body::create(*app.world, ground);
    state.cube = app::keep(app, box::create({.half_extents = {0.25f, 0.25f, 0.25f}, .convex_radius = 0.05f}));
    match_count(app);
    camera::look_at(app.camera, {8.0f, 8.0f, 12.0f}, {0.0f, 1.5f, 0.0f});
}

static void step(App &app, float) {
    // one cube a step dropped back in over the middle, keeping the heap moving
    if (state.cubes.length == 0) return;
    rigid_body::RigidBody *cube = state.cubes[(uint32_t)(random::next(state.random) * (float)state.cubes.length) % state.cubes.length];
    rigid_body::set_position(*app.world, cube, vec3f::make(0.0f, random::range(state.random, 0.0f, SPAWN_HEIGHT), 0.0f), true);
    rigid_body::set_linear_velocity(*app.world, cube, vec3f::zero());
    rigid_body::set_angular_velocity(*app.world, cube, vec3f::zero());
}

static void ui(App &app) {
    if (ImGui::SliderInt("cubes", &state.count, 0, 1000)) match_count(app);
}

static void shutdown(App &) { state.cubes.release(); }

static const Example EXAMPLE_CUBE_HEAP = {"Cube Heap", init, step, nullptr, ui, shutdown};
static const examples::Registration REGISTERED(EXAMPLE_CUBE_HEAP);
