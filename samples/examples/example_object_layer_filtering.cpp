#include <math.h>

#include "app/app.h"
#include "app/examples.h"
#include "app/random.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/sphere.h"

using namespace tumble;

/** a tenth of a second at 60 steps a second */
static constexpr int SPAWN_STEPS = 6;
/** five seconds */
static constexpr int LIFETIME_STEPS = 300;
static constexpr float DEBRIS_RADIUS = 0.05f;
static constexpr float SPAWN_HEIGHT = 5.0f;
static constexpr float SPAWN_RADIUS = 1.5f;

struct Debris {
    RigidBody *body;
    int spawned_at;
};

static struct {
    uint32_t environment;
    uint32_t characters;
    uint32_t debris_layer;
    Shape *debris_shape;
    collections::Array<Debris> debris;
    int steps;
    int spawn_steps;
    Random random;
} state;

static void init(App &app) {
    state.debris.length = 0;
    state.steps = 0;
    state.spawn_steps = 0;
    state.random = {3};

    WorldSettings settings = world_settings::create();
    const uint32_t broadphase_moving = world_settings::add_broadphase_layer(settings);
    const uint32_t broadphase_not_moving = world_settings::add_broadphase_layer(settings);
    state.environment = world_settings::add_object_layer(settings, broadphase_not_moving);
    state.characters = world_settings::add_object_layer(settings, broadphase_moving);
    state.debris_layer = world_settings::add_object_layer(settings, broadphase_moving);
    // debris passes through characters: that pair is never enabled
    world_settings::enable_collision(settings, state.environment, state.environment);
    world_settings::enable_collision(settings, state.environment, state.characters);
    world_settings::enable_collision(settings, state.environment, state.debris_layer);
    world_settings::enable_collision(settings, state.characters, state.characters);
    world_settings::enable_collision(settings, state.debris_layer, state.debris_layer);
    app.world = world::create(settings);

    RigidBodySettings floor;
    floor.shape = app::keep(app, box::create({.half_extents = {5.0f, 0.1f, 5.0f}}));
    floor.object_layer = state.environment;
    floor.motion_type = MotionType::STATIC;
    floor.friction = 0.5f;
    rigid_body::create(*app.world, floor);

    RigidBodySettings character;
    character.shape = app::keep(app, capsule::create({.half_height_of_cylinder = 0.5f, .radius = 0.5f}));
    character.object_layer = state.characters;
    character.motion_type = MotionType::KINEMATIC;
    character.position = {0.0f, 1.3f, 0.0f};
    character.friction = 0.5f;
    rigid_body::create(*app.world, character);

    state.debris_shape = app::keep(app, sphere::create({.radius = DEBRIS_RADIUS}));
    camera::look_at(app.camera, {5.0f, 5.0f, 10.0f}, {0.0f, 2.0f, 0.0f});
}

static void step(App &app, float) {
    state.steps++;
    if (++state.spawn_steps >= SPAWN_STEPS) {
        state.spawn_steps = 0;
        // anywhere in a circle over the character
        const float angle = seeded::next(state.random) * 2.0f * 3.14159265f;
        const float radius = seeded::next(state.random) * SPAWN_RADIUS;
        RigidBodySettings settings;
        settings.shape = state.debris_shape;
        settings.object_layer = state.debris_layer;
        settings.motion_type = MotionType::DYNAMIC;
        settings.position = {cosf(angle) * radius, SPAWN_HEIGHT, sinf(angle) * radius};
        settings.allowed_degrees_of_freedom = dof::DOF_TRANSLATION_ONLY;
        settings.friction = 0.3f;
        settings.restitution = 0.2f;
        state.debris.push({rigid_body::create(*app.world, settings), state.steps});
    }

    // the oldest first: they expire in the order they came
    uint32_t expired = 0;
    while (expired < state.debris.length && state.steps - state.debris[expired].spawned_at > LIFETIME_STEPS) {
        rigid_body::remove(*app.world, state.debris[expired].body);
        expired++;
    }
    if (expired == 0) return;
    for (uint32_t i = expired; i < state.debris.length; i++) state.debris[i - expired] = state.debris[i];
    state.debris.length -= expired;
}

static void shutdown(App &) { state.debris.release(); }

static const Example EXAMPLE_OBJECT_LAYER_FILTERING = {"Object Layer Filtering", init, step, nullptr, nullptr, shutdown};
static const examples::Registration REGISTERED(EXAMPLE_OBJECT_LAYER_FILTERING);
