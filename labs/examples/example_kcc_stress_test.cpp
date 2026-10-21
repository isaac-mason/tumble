#include <math.h>

#include "app/app.h"
#include "app/examples.h"
#include "app/random.h"
#include "imgui.h"
#include "tumble/math/quatf.h"
#include "tumble/physics/character/kcc.h"
#include "tumble/physics/filter.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/transformed.h"
#include "tumble/physics/shapes/triangle_mesh.h"
#include "tumble/physics/shapes/utils/triangle_mesh_builder.h"

using namespace tumble;

static constexpr float PI = 3.14159265f;
static constexpr Float3 GRAVITY = {0.0f, -25.0f, 0.0f};
static constexpr float AGENT_SPEED = 4.0f;
static constexpr float AGENT_RADIUS = 0.4f;
static constexpr float AGENT_HALF_HEIGHT = 0.5f;
static constexpr float SPAWN_Y = 10.0f;
static constexpr float FALL_THRESHOLD = -4.0f;
/** agents spawn well inside the walls at +-31 m */
static constexpr float SPAWN_BOUND = 24.0f;

struct Agent {
    kcc::KCC *character;
    float target_angle;
    float turn_timer;
    Float3 spawn_position;
};

static struct {
    app::Layers layers;
    filter::Filter filter;
    kcc::UpdateSettings update_settings;
    shapes::Shape *agent_shape;
    collections::Vec<Agent> agents;
    int agent_count;
    double update_milliseconds;
    Random random;
} state;

/** a 32 by 32 grid of 2 m cells, rolling hills */
static shapes::Shape *make_terrain(App &app) {
    const int grid = 32;
    const float cell = 2.0f, height = 2.0f;
    const auto height_at = [&](float x, float z) {
        return sinf(x * 0.3f) * cosf(z * 0.3f) * height * 0.5f + sinf(x * 0.7f + 1.0f) * sinf(z * 0.7f) * height * 0.3f +
               cosf((x + z) * 0.15f) * height * 0.2f;
    };
    collections::Owned<float> positions;
    collections::Owned<uint32_t> indices;
    for (int iz = 0; iz <= grid; iz++) {
        for (int ix = 0; ix <= grid; ix++) {
            const float x = (float)(ix - grid / 2) * cell, z = (float)(iz - grid / 2) * cell;
            positions.v.push(x);
            positions.v.push(height_at(x, z));
            positions.v.push(z);
        }
    }
    const uint32_t row = grid + 1;
    for (uint32_t iz = 0; iz < (uint32_t)grid; iz++) {
        for (uint32_t ix = 0; ix < (uint32_t)grid; ix++) {
            const uint32_t bl = iz * row + ix, br = bl + 1, tl = bl + row, tr = tl + 1;
            const uint32_t quad[6] = {bl, tl, br, br, tl, tr};
            for (uint32_t index : quad) indices.v.push(index);
        }
    }
    triangle_mesh_data::TriangleMeshData *data = app::keep(
        app, triangle_mesh_builder::build({.positions = positions.v.data, .num_positions = positions.v.length / 3, .indices = indices.v.data,
                                           .num_indices = indices.v.length}));
    return app::keep(app, triangle_mesh::create({.data = data}));
}

static void add_static(App &app, shapes::Shape *shape, Float3 position) {
    rigid_body::RigidBodySettings settings;
    settings.shape = shape;
    settings.object_layer = state.layers.not_moving;
    settings.motion_type = motion_type::MOTION_STATIC;
    settings.position = position;
    rigid_body::create(*app.world, settings);
}

static Agent make_agent(App &app, float x, float z) {
    kcc::KCCSettings settings;
    settings.shape = state.agent_shape;
    settings.mass = 70.0f;
    settings.max_slope_angle = 45.0f * PI / 180.0f;
    settings.character_padding = 0.02f;
    settings.inner_rigid_body_shape = state.agent_shape;
    settings.inner_rigid_body_object_layer = state.layers.moving;
    settings.has_supporting_volume_plane = true;
    settings.supporting_volume_plane = {{0.0f, 1.0f, 0.0f}, -AGENT_RADIUS};
    const Float3 position = {x, SPAWN_Y, z};
    kcc::KCC *character = kcc::create(settings, vec3f::load(position), quatf::identity());
    kcc::add(*app.world, *character);
    app::set_color(app, rigid_body::get(*app.world, character->inner_rigid_body_id), {0.95f, 0.75f, 0.3f, 1.0f});
    return {character, seeded::next(state.random) * 2 * PI, 1 + seeded::next(state.random) * 2, position};
}

static void release_agents(App &app) {
    for (uint32_t i = 0; i < state.agents.length; i++) {
        kcc::remove(*app.world, *state.agents[i].character);
        kcc::destroy(state.agents[i].character);
    }
    state.agents.length = 0;
}

/** all agents again, on a grid that fits the new count */
static void respawn_agents(App &app) {
    release_agents(app);
    const int count = state.agent_count;
    const int columns = (int)ceilf(sqrtf((float)count));
    const int rows = columns > 0 ? (count + columns - 1) / columns : 0;
    for (int i = 0; i < count; i++) {
        const int column = i % columns, row = i / columns;
        const float x = columns > 1 ? ((float)column / (float)(columns - 1) - 0.5f) * 2 * SPAWN_BOUND : 0;
        const float z = rows > 1 ? ((float)row / (float)(rows - 1) - 0.5f) * 2 * SPAWN_BOUND : 0;
        state.agents.push(make_agent(app, x, z));
    }
}

static void init(App &app) {
    state.agents.length = 0;
    state.agent_count = 50;
    state.random = {11};
    state.update_settings = {};
    app.world = app::create_world(state.layers, GRAVITY);
    state.filter = filter::create(app.world->settings.layers);
    add_static(app, make_terrain(app), {0, 0, 0});
    // walls just inside the terrain's edge at +-32 m
    const float bound = 31, height = 6, thickness = 1;
    const float walls[4][4] = {{0, bound, bound + thickness, thickness},
                               {0, -bound, bound + thickness, thickness},
                               {bound, 0, thickness, bound + thickness},
                               {-bound, 0, thickness, bound + thickness}};
    for (const float *wall : walls) add_static(app, app::keep(app, box::create({.half_extents = {wall[2], height, wall[3]}})), {wall[0], height, wall[1]});
    shapes::Shape *pill = app::keep(app, capsule::create({.half_height_of_cylinder = AGENT_HALF_HEIGHT, .radius = AGENT_RADIUS}));
    state.agent_shape =
        app::keep(app, transformed::create({.shape = pill, .position = {0, AGENT_HALF_HEIGHT + AGENT_RADIUS, 0}, .quaternion = {0, 0, 0, 1}}));
    respawn_agents(app);
    camera::look_at(app.camera, {0, 40, 70}, {0, 0, 0});
}

static void steer(App &app, Agent &agent, float delta_time) {
    kcc::KCC &character = *agent.character;
    if (vec3f::y(character.position) < FALL_THRESHOLD) {
        kcc::set_position(*app.world, character, vec3f::load(agent.spawn_position));
        character.linear_velocity = vec3f::zero();
        return;
    }
    agent.turn_timer -= delta_time;
    if (agent.turn_timer <= 0) {
        // a new heading, up to 90 degrees either way
        agent.target_angle += (seeded::next(state.random) - 0.5f) * PI;
        agent.turn_timer = 1 + seeded::next(state.random) * 2;
    }
    kcc::update_ground_velocity(*app.world, character, nullptr);
    const Vec3f up = character.up;
    Vec3f velocity = character.ground.state == kcc::ON_GROUND ? character.ground.velocity
                                                               : vec3f::scale(up, vec3f::dot(character.linear_velocity, up));
    velocity = vec3f::add(velocity, vec3f::scale(vec3f::load(GRAVITY), delta_time));
    velocity = vec3f::add(velocity, vec3f::make(cosf(agent.target_angle) * AGENT_SPEED, 0, sinf(agent.target_angle) * AGENT_SPEED));
    character.linear_velocity = velocity;
}

static void step(App &app, float delta_time) {
    const double start = app::now_milliseconds();
    for (uint32_t i = 0; i < state.agents.length; i++) {
        Agent &agent = state.agents[i];
        steer(app, agent, delta_time);
        kcc::update(*app.world, *agent.character, delta_time, vec3f::load(GRAVITY), state.update_settings, nullptr, state.filter);
    }
    state.update_milliseconds = app::now_milliseconds() - start;
}

static void ui(App &app) {
    if (ImGui::SliderInt("agents", &state.agent_count, 0, 100)) respawn_agents(app);
    ImGui::Text("characters %.2f ms", state.update_milliseconds);
}

static void shutdown(App &app) {
    release_agents(app);
    state.agents.release();
    filter::release(state.filter);
}

static const Example EXAMPLE_KCC_STRESS_TEST = {"KCC Stress Test", init, step, nullptr, ui, shutdown};
static const examples::Registration REGISTERED(EXAMPLE_KCC_STRESS_TEST);
