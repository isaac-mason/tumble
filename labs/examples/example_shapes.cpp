#include "app/app.h"
#include "app/examples.h"
#include "app/random.h"
#include "app/vec.h"
#include "imgui.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/convex_hull.h"
#include "tumble/physics/shapes/cylinder.h"
#include "tumble/physics/shapes/plane.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/shapes/static_compound.h"
#include "tumble/physics/shapes/transformed.h"

using namespace tumble;

static constexpr float SPAWN_HEIGHT = 15.0f;
static constexpr float SPAWN_AREA = 10.0f;
static constexpr int KIND_COUNT = 8;
/** one body respawned every this many steps, half a second at 60 */
static constexpr int RESPAWN_STEPS = 30;

struct Kind {
    shapes::Shape *shape;
    Color color;
};

struct Spawned {
    rigid_body::RigidBody *body;
};

static struct {
    app::Layers layers;
    Kind kinds[KIND_COUNT];
    collections::Vec<Spawned> spawned;
    int count;
    int respawn_steps;
    int steps;
    uint32_t next_respawn;
    Random random;
} state;

static Color hex(uint32_t rgb) { return {(float)((rgb >> 16) & 255) / 255.0f, (float)((rgb >> 8) & 255) / 255.0f, (float)(rgb & 255) / 255.0f, 1.0f}; }

static const Float4 IDENTITY = {0.0f, 0.0f, 0.0f, 1.0f};

static void make_kinds(App &app) {
    shapes::Shape *ball = app::keep(app, sphere::create({.radius = 0.5f}));
    state.kinds[0] = {ball, hex(0xff6b6b)};
    state.kinds[1] = {app::keep(app, box::create({.half_extents = {0.5f, 0.5f, 0.5f}, .convex_radius = 0.05f})), hex(0x4ecdc4)};
    state.kinds[2] = {app::keep(app, capsule::create({.half_height_of_cylinder = 1.0f, .radius = 1.0f})), hex(0xdfe6e9)};
    state.kinds[3] = {app::keep(app, cylinder::create({.half_height = 0.75f, .radius = 0.5f})), hex(0x74b9ff)};

    // two balls on a bar along x
    shapes::Shape *bar = app::keep(app, capsule::create({.half_height_of_cylinder = 0.8f, .radius = 0.3f}));
    const compound::CompoundShapeChild dumbbell[] = {
        {.position = {-1.1f, 0.0f, 0.0f}, .quaternion = IDENTITY, .shape = ball},
        {.position = {1.1f, 0.0f, 0.0f}, .quaternion = IDENTITY, .shape = ball},
        {.position = {0.0f, 0.0f, 0.0f}, .quaternion = vec::quat_axis_angle({0.0f, 0.0f, 1.0f}, 0.5f * 3.14159265f), .shape = bar},
    };
    state.kinds[4] = {app::keep(app, compound::create({.children = dumbbell, .num_children = 3})), hex(0x95e1d3)};

    // two rails and four rungs, built once into a static compound's tree
    shapes::Shape *rail = app::keep(app, box::create({.half_extents = {0.1f, 1.5f, 0.1f}, .convex_radius = 0.05f}));
    shapes::Shape *rung = app::keep(app, box::create({.half_extents = {0.6f, 0.1f, 0.1f}, .convex_radius = 0.05f}));
    compound::CompoundShapeChild ladder[6] = {
        {.position = {-0.5f, 0.0f, 0.0f}, .quaternion = IDENTITY, .shape = rail},
        {.position = {0.5f, 0.0f, 0.0f}, .quaternion = IDENTITY, .shape = rail},
    };
    for (int i = 0; i < 4; i++) ladder[2 + i] = {.position = {0.0f, -0.75f + 0.5f * (float)i, 0.0f}, .quaternion = IDENTITY, .shape = rung};
    state.kinds[5] = {app::keep(app, static_compound::create({.children = ladder, .num_children = 6})), hex(0x95e1d3)};

    // a handle, and a heavy head raised by a transformed shape rather than by the compound's child pose
    shapes::Shape *handle = app::keep(app, box::create({.half_extents = {0.2f, 1.5f, 0.2f}, .convex_radius = 0.05f}));
    shapes::Shape *head_box = app::keep(app, box::create({.half_extents = {0.8f, 0.3f, 0.3f}, .convex_radius = 0.05f, .density = 10000.0f}));
    shapes::Shape *head = app::keep(app, transformed::create({.shape = head_box, .position = {0.0f, 1.5f, 0.0f}, .quaternion = IDENTITY}));
    const compound::CompoundShapeChild hammer[] = {
        {.position = {0.0f, 0.0f, 0.0f}, .quaternion = IDENTITY, .shape = handle},
        {.position = {0.0f, 0.0f, 0.0f}, .quaternion = IDENTITY, .shape = head},
    };
    state.kinds[6] = {app::keep(app, compound::create({.children = hammer, .num_children = 2})), hex(0xf39c12)};

    // a square base and an apex over it
    const float pyramid[] = {-0.5f, 0.0f, -0.5f, 0.5f, 0.0f, -0.5f, 0.5f, 0.0f, 0.5f, -0.5f, 0.0f, 0.5f, 0.0f, 1.0f, 0.0f};
    convex_hull::ConvexHullData *hull = app::keep(app, convex_hull::build({.positions = pyramid, .num_positions = 5, .convex_radius = 0.05f}, nullptr));
    state.kinds[7] = {app::keep(app, convex_hull::create({.data = hull})), hex(0x9b59b6)};
}

static rigid_body::RigidBody *spawn(App &app, float restitution) {
    const Kind &kind = state.kinds[(uint32_t)(random::next(state.random) * KIND_COUNT) % KIND_COUNT];
    const Float3 axis = {random::next(state.random) - 0.5f, random::next(state.random) - 0.5f, random::next(state.random) - 0.5f};
    rigid_body::RigidBodySettings settings;
    settings.shape = kind.shape;
    settings.object_layer = state.layers.moving;
    settings.motion_type = motion_type::MOTION_DYNAMIC;
    settings.position = {random::range(state.random, -SPAWN_AREA, SPAWN_AREA), SPAWN_HEIGHT, random::range(state.random, -SPAWN_AREA, SPAWN_AREA)};
    settings.quaternion = vec::quat_axis_angle(axis, random::next(state.random) * 2.0f * 3.14159265f);
    settings.restitution = restitution;
    settings.friction = 0.5f;
    rigid_body::RigidBody *body = rigid_body::create(*app.world, settings);
    app::set_color(app, body, kind.color);
    return body;
}

static void match_count(App &app) {
    while ((int)state.spawned.length < state.count) state.spawned.push({spawn(app, 0.0f)});
    while ((int)state.spawned.length > state.count) rigid_body::remove(*app.world, state.spawned[--state.spawned.length].body);
}

static void init(App &app) {
    state.spawned.length = 0;
    state.count = 50;
    state.respawn_steps = RESPAWN_STEPS;
    state.steps = 0;
    state.next_respawn = 0;
    state.random = {7};
    app.world = app::create_world(state.layers);
    make_kinds(app);
    rigid_body::RigidBodySettings ground;
    ground.shape = app::keep(app, plane::create({.plane = {{0.0f, 1.0f, 0.0f}, 0.0f}, .half_extent = 50.0f}));
    ground.object_layer = state.layers.not_moving;
    ground.motion_type = motion_type::MOTION_STATIC;
    ground.friction = 0.5f;
    rigid_body::create(*app.world, ground);
    match_count(app);
    camera::look_at(app.camera, {0.0f, 10.0f, 30.0f}, {0.0f, 5.0f, 0.0f});
}

static void step(App &app, float) {
    // round robin: the oldest body goes, a new one of any kind drops in
    if (++state.steps < state.respawn_steps || state.spawned.length == 0) return;
    state.steps = 0;
    Spawned &oldest = state.spawned[state.next_respawn++ % state.spawned.length];
    rigid_body::remove(*app.world, oldest.body);
    oldest.body = spawn(app, 0.3f);
}

static void ui(App &app) {
    if (ImGui::SliderInt("bodies", &state.count, 0, 1000)) match_count(app);
    ImGui::SliderInt("respawn every (steps)", &state.respawn_steps, 6, 300);
}

static void shutdown(App &) { state.spawned.release(); }

const Example EXAMPLE_SHAPES = {"shapes", init, step, nullptr, ui, shutdown};
