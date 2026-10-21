#include "app/app.h"
#include "app/examples.h"
#include "app/model.h"
#include "app/random.h"
#include "app/vec.h"
#include "imgui.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/shapes/triangle_mesh.h"
#include "tumble/physics/shapes/utils/triangle_mesh_builder.h"

using namespace tumble;

static constexpr float SPAWN_HEIGHT = 5.0f;
static constexpr float SPAWN_AREA = 5.0f;
static constexpr int KIND_COUNT = 2;

static struct {
    app::Layers layers;
    shapes::Shape *kinds[KIND_COUNT];
    collections::Vec<rigid_body::RigidBody *> bodies;
    int count;
    int respawn_interval_ms;
    float since_respawn_ms;
    uint32_t respawn_index;
    Random random;
} state;

static void make_jungle(App &app) {
    Model jungle = model::load("low_poly_environment_jungle_scene.glb");
    if (!jungle.loaded) return;
    triangle_mesh_data::TriangleMeshData *data = app::keep(
        app, triangle_mesh_builder::build({.positions = jungle.positions.data,
                                           .num_positions = jungle.positions.length / 3,
                                           .indices = jungle.indices.data,
                                           .num_indices = jungle.indices.length}));
    model::release(jungle);
    rigid_body::RigidBodySettings settings;
    settings.shape = app::keep(app, triangle_mesh::create({.data = data}));
    settings.object_layer = state.layers.not_moving;
    settings.motion_type = motion_type::MOTION_STATIC;
    settings.restitution = 0.2f;
    settings.friction = 0.7f;
    rigid_body::create(*app.world, settings);
}

/** a sphere or a box, dropped somewhere over the middle at any angle */
static rigid_body::RigidBody *spawn(App &app) {
    const Float3 axis = {random::next(state.random) - 0.5f, random::next(state.random) - 0.5f, random::next(state.random) - 0.5f};
    rigid_body::RigidBodySettings settings;
    settings.shape = state.kinds[(uint32_t)(random::next(state.random) * KIND_COUNT) % KIND_COUNT];
    settings.object_layer = state.layers.moving;
    settings.motion_type = motion_type::MOTION_DYNAMIC;
    settings.position = {random::range(state.random, -SPAWN_AREA, SPAWN_AREA), SPAWN_HEIGHT, random::range(state.random, -SPAWN_AREA, SPAWN_AREA)};
    settings.quaternion = vec::quat_axis_angle(axis, random::next(state.random) * 2.0f * scalar::PI);
    settings.restitution = 0.4f;
    settings.friction = 0.6f;
    return rigid_body::create(*app.world, settings);
}

static void match_count(App &app) {
    while ((int)state.bodies.length < state.count) state.bodies.push(spawn(app));
    while ((int)state.bodies.length > state.count) rigid_body::remove(*app.world, state.bodies[--state.bodies.length]);
}

static void init(App &app) {
    state.bodies.length = 0;
    state.count = 30;
    state.respawn_interval_ms = 500;
    state.since_respawn_ms = 0.0f;
    state.respawn_index = 0;
    state.random = {5};
    app.world = app::create_world(state.layers);
    make_jungle(app);
    state.kinds[0] = app::keep(app, sphere::create({.radius = 0.5f}));
    state.kinds[1] = app::keep(app, box::create({.half_extents = {0.5f, 0.5f, 0.5f}, .convex_radius = 0.05f}));
    match_count(app);
    app.drawing.grid = false;
    camera::look_at(app.camera, {20.0f, 15.0f, -20.0f}, {0.0f, 5.0f, 0.0f});
}

static void step(App &app, float delta_time) {
    // round robin: the oldest body goes, a new one of either kind drops in
    state.since_respawn_ms += delta_time * 1000.0f;
    if (state.bodies.length == 0 || state.since_respawn_ms < (float)state.respawn_interval_ms) return;
    state.since_respawn_ms = 0.0f;
    rigid_body::RigidBody *&oldest = state.bodies[state.respawn_index++ % state.bodies.length];
    rigid_body::remove(*app.world, oldest);
    oldest = spawn(app);
}

static void ui(App &app) {
    if (ImGui::SliderInt("bodies", &state.count, 0, 200)) match_count(app);
    ImGui::SliderInt("respawn (ms)", &state.respawn_interval_ms, 100, 5000);
}

static void shutdown(App &) { state.bodies.release(); }

static const Example EXAMPLE_TRIANGLE_MESH = {"Triangle Mesh", init, step, nullptr, ui, shutdown, nullptr};
static const examples::Registration REGISTERED(EXAMPLE_TRIANGLE_MESH);
