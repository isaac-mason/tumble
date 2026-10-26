#include <math.h>

#include "app/app.h"
#include "app/examples.h"
#include "app/vec.h"
#include "imgui.h"
#include "tumble/physics/query.h"
#include "tumble/physics/shapes/box.h"

using namespace tumble;

static constexpr float AMPLITUDE = 6.0f;
static constexpr float FREQUENCY = 1.0f;

static struct {
    uint32_t layer;
    Shape *query_shape;
    Filter filter;
    CollideShapeCollector collector;
    bool hit;
    int steps;
} state;

static Color hex(uint32_t rgb, float alpha = 1.0f) {
    return {(float)((rgb >> 16) & 255) / 255.0f, (float)((rgb >> 8) & 255) / 255.0f, (float)(rgb & 255) / 255.0f, alpha};
}

/** a world-space label as crashcat's sprites are: a box `width` by `height` facing the camera, its text a fraction of the height */
static void init(App &app) {
    state.steps = 0;
    state.hit = false;

    WorldSettings settings = world_settings::create();
    state.layer = world_settings::add_object_layer(settings, world_settings::add_broadphase_layer(settings));
    world_settings::enable_collision(settings, state.layer, state.layer);
    settings.gravity = {0.0f, 0.0f, 0.0f};
    app.world = world::create(settings);

    RigidBodySettings box_settings;
    box_settings.shape = app::keep(app, box::create({.half_extents = {2.0f, 2.0f, 2.0f}}));
    box_settings.object_layer = state.layer;
    box_settings.motion_type = MotionType::STATIC;
    box_settings.position = {0.0f, 2.0f, 0.0f};
    rigid_body::create(*app.world, box_settings);

    state.query_shape = app::keep(app, box::create({.half_extents = {0.5f, 0.5f, 0.5f}}));
    state.filter = filter::create(app.world->settings.layers);
    state.collector = collide_shape::create_collector(CollectorKind::ALL);

    // no ground here
    app.drawing.grid = false;
    camera::look_at(app.camera, {0.0f, 5.0f, 15.0f}, {0.0f, 2.0f, 0.0f});
}

static void step(App &, float) { state.steps++; }

static void update(App &app, float) {
    // crashcat's clock, as steps
    const float time = (float)state.steps / 60.0f;
    const float x = sinf(time * FREQUENCY) * AMPLITUDE;
    const Transformf transform = {{x, 2.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};

    collide_shape::reset(state.collector);
    query::collide_shape(*app.world, state.collector, collide_shape::default_settings(), state.query_shape, transform, state.filter);
    state.hit = state.collector.hits.length > 0;

    renderer::mesh(app.renderer, renderer::mesh_of(app.renderer, state.query_shape), matrix::compose(transform.position, transform.quaternion),
                   state.hit ? hex(0x00ff00, 0.7f) : hex(0xff00ff, 0.7f));
    app::label(app, {x, 4.5f, 0.0f}, state.hit ? "hit" : "no hit", {4.0f, 1.0f, 0.5f, hex(0x5f5f5f), state.hit ? hex(0x4caf50) : hex(0xf44336)});
}

static void shutdown(App &) {
    state.collector = {};
    state.filter = {};
}

static const Example EXAMPLE_COLLIDE_SHAPE = {"Collide Shape", init, step, update, nullptr, shutdown};
static const examples::Registration REGISTERED(EXAMPLE_COLLIDE_SHAPE);
