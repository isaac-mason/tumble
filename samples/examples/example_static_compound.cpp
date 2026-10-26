#include <math.h>

#include "app/app.h"
#include "app/examples.h"
#include "app/random.h"
#include "imgui.h"
#include "tumble/math/eulerf.h"
#include "tumble/math/quatf.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/shapes/static_compound.h"

using namespace tumble;

static constexpr float BRIDGE_LENGTH = 36.0f;
static constexpr float BRIDGE_WIDTH = 3.0f;
static constexpr float BRIDGE_HEIGHT = 8.0f;
static constexpr float BEAM_THICKNESS = 0.12f;
static constexpr float DECK_THICKNESS = 0.15f;
static constexpr int PLANKS = 12;
static constexpr int POSTS = 9;
static constexpr int TRUSSES = 8;
static constexpr float SUPPORT_HEIGHT = 3.5f;
static constexpr float BOX_HALF_EXTENT = 0.25f;
static constexpr float CORNER_SPHERE_RADIUS = 0.12f;

static const Float4 IDENTITY = {0.0f, 0.0f, 0.0f, 1.0f};

static struct {
    app::Layers layers;
    Shape *sphere_box;
    collections::Array<RigidBody *> boxes;
    int count;
    int respawn_interval_ms;
    float since_respawn_ms;
    uint32_t respawn_index;
    Random random;
} state;

static void make_bridge(App &app) {
    collections::Array<CompoundShapeSettings::Child> children = {};
    const auto add = [&](Shape *shape, Float3 position, Float4 quaternion) {
        children.push({.position = position, .quaternion = quaternion, .shape = shape});
    };
    const auto beam = [&](Float3 half_extents) { return app::keep(app, box::create({.half_extents = half_extents, .convex_radius = 0.01f})); };

    Shape *plank = beam({BRIDGE_LENGTH / PLANKS / 2.0f + 0.05f, DECK_THICKNESS / 2.0f, BRIDGE_WIDTH / 2.0f});
    for (int i = 0; i < PLANKS; i++) {
        add(plank, {-BRIDGE_LENGTH / 2.0f + (float)i / (float)(PLANKS - 1) * BRIDGE_LENGTH, BRIDGE_HEIGHT, 0.0f}, IDENTITY);
    }

    Shape *rail = beam({BRIDGE_LENGTH / 2.0f, BEAM_THICKNESS, BEAM_THICKNESS});
    Shape *post = beam({BEAM_THICKNESS, 0.6f, BEAM_THICKNESS});
    for (int side = 0; side < 2; side++) {
        const float z = (side == 0 ? -1.0f : 1.0f) * (BRIDGE_WIDTH / 2.0f + 0.2f);
        add(rail, {0.0f, BRIDGE_HEIGHT + 0.3f, z}, IDENTITY);
        add(rail, {0.0f, BRIDGE_HEIGHT + 1.2f, z}, IDENTITY);
        for (int i = 0; i < POSTS; i++) {
            add(post, {-BRIDGE_LENGTH / 2.0f + (float)i / (float)(POSTS - 1) * BRIDGE_LENGTH, BRIDGE_HEIGHT + 0.75f, z}, IDENTITY);
        }
    }

    // the trusses underneath: a support each side, and an x of bracing to the next truss
    const float truss_spacing = BRIDGE_LENGTH / (float)(TRUSSES - 1);
    const float cross_length = sqrtf(truss_spacing * truss_spacing + BRIDGE_WIDTH * BRIDGE_WIDTH + 1.5f * 1.5f);
    Shape *support = beam({BEAM_THICKNESS * 1.5f, SUPPORT_HEIGHT / 2.0f, BEAM_THICKNESS * 1.5f});
    Shape *brace = beam({cross_length / 2.0f, BEAM_THICKNESS * 0.8f, BEAM_THICKNESS * 0.8f});
    for (int i = 0; i < TRUSSES; i++) {
        const float x = -BRIDGE_LENGTH / 2.0f + (float)i / (float)(TRUSSES - 1) * BRIDGE_LENGTH;
        for (int side = 0; side < 2; side++) {
            const float z = (side == 0 ? -1.0f : 1.0f) * (BRIDGE_WIDTH / 2.0f - 0.3f);
            add(support, {x, BRIDGE_HEIGHT - SUPPORT_HEIGHT / 2.0f, z}, IDENTITY);
        }
        if (i == TRUSSES - 1) continue;
        const float next_x = -BRIDGE_LENGTH / 2.0f + (float)(i + 1) / (float)(TRUSSES - 1) * BRIDGE_LENGTH;
        for (int cross = 0; cross < 2; cross++) {
            const float angle = (cross == 0 ? 1.0f : -1.0f) * atan2f(BRIDGE_WIDTH, next_x - x);
            const float tilt = atan2f(1.5f, next_x - x);
            Eulerf euler;
            Float4 quaternion;
            quatf::store(quaternion, quatf::from_euler(eulerf::set(euler, tilt, angle, 0.0f, EulerOrder::XYZ)));
            add(brace, {(x + next_x) / 2.0f, BRIDGE_HEIGHT - 1.5f, 0.0f}, quaternion);
        }
    }

    // two beams along the bottom, and the cross beams between them
    Shape *bottom_beam = beam({BRIDGE_LENGTH / 2.0f, BEAM_THICKNESS * 1.5f, BEAM_THICKNESS * 1.5f});
    for (int side = 0; side < 2; side++) {
        add(bottom_beam, {0.0f, BRIDGE_HEIGHT - SUPPORT_HEIGHT, (side == 0 ? -1.0f : 1.0f) * (BRIDGE_WIDTH / 2.0f - 0.3f)}, IDENTITY);
    }
    Shape *cross_beam = beam({BEAM_THICKNESS, BEAM_THICKNESS * 1.5f, BRIDGE_WIDTH / 2.0f - 0.3f});
    for (int i = 0; i < POSTS; i++) {
        add(cross_beam, {-BRIDGE_LENGTH / 2.0f + (float)i / (float)(POSTS - 1) * BRIDGE_LENGTH, BRIDGE_HEIGHT - SUPPORT_HEIGHT, 0.0f}, IDENTITY);
    }

    RigidBodySettings bridge;
    bridge.shape = app::keep(app, static_compound::create({.children = children.data, .num_children = children.length}));
    bridge.object_layer = state.layers.not_moving;
    bridge.motion_type = MotionType::STATIC;
    bridge.restitution = 0.3f;
    bridge.friction = 0.5f;
    rigid_body::create(*app.world, bridge);
    children.release();
}

/** a hollow box: a sphere at each of its eight corners */
static Shape *make_sphere_box(App &app) {
    Shape *corner = app::keep(app, sphere::create({.radius = CORNER_SPHERE_RADIUS}));
    CompoundShapeSettings::Child corners[8];
    for (int i = 0; i < 8; i++) {
        const float h = BOX_HALF_EXTENT;
        corners[i] = {.position = {(i & 4) ? -h : h, (i & 2) ? -h : h, (i & 1) ? -h : h}, .quaternion = IDENTITY, .shape = corner};
    }
    return app::keep(app, static_compound::create({.children = corners, .num_children = 8}));
}

static RigidBody *spawn_box(App &app) {
    RigidBodySettings settings;
    settings.shape = state.sphere_box;
    settings.object_layer = state.layers.moving;
    settings.motion_type = MotionType::DYNAMIC;
    settings.position = {(seeded::next(state.random) - 0.5f) * (BRIDGE_LENGTH - 4.0f), BRIDGE_HEIGHT + 5.0f + seeded::next(state.random) * 3.0f,
                         (seeded::next(state.random) - 0.5f) * (BRIDGE_WIDTH - 0.5f)};
    settings.restitution = 0.3f;
    settings.friction = 0.5f;
    return rigid_body::create(*app.world, settings);
}

static void match_count(App &app) {
    while ((int)state.boxes.length < state.count) state.boxes.push(spawn_box(app));
    while ((int)state.boxes.length > state.count) rigid_body::remove(*app.world, state.boxes[--state.boxes.length]);
}

static void init(App &app) {
    state.boxes.length = 0;
    state.count = 100;
    state.respawn_interval_ms = 200;
    state.since_respawn_ms = 0.0f;
    state.respawn_index = 0;
    state.random = {3};
    app.world = app::create_world(state.layers);
    make_bridge(app);
    state.sphere_box = make_sphere_box(app);
    match_count(app);
    app.drawing.grid = false;
    camera::look_at(app.camera, {0.0f, 22.0f, 30.0f}, {0.0f, 8.0f, 0.0f});
}

static void step(App &app, float delta_time) {
    // round robin: the oldest box goes, a new one drops onto the bridge
    state.since_respawn_ms += delta_time * 1000.0f;
    if (state.boxes.length == 0 || state.since_respawn_ms < (float)state.respawn_interval_ms) return;
    state.since_respawn_ms = 0.0f;
    RigidBody *&oldest = state.boxes[state.respawn_index++ % state.boxes.length];
    rigid_body::remove(*app.world, oldest);
    oldest = spawn_box(app);
}

static void ui(App &app) {
    if (ImGui::SliderInt("boxes", &state.count, 0, 500)) match_count(app);
    ImGui::SliderInt("respawn (ms)", &state.respawn_interval_ms, 50, 2000);
}

static void shutdown(App &) { state.boxes.release(); }

static const Example EXAMPLE_STATIC_COMPOUND = {"Static Compound Shapes", init, step, nullptr, ui, shutdown, nullptr};
static const examples::Registration REGISTERED(EXAMPLE_STATIC_COMPOUND);
