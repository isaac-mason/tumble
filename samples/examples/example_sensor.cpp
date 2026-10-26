#include <math.h>

#include "app/app.h"
#include "app/examples.h"
#include "app/vec.h"
#include "imgui.h"
#include "tumble/physics/shapes/box.h"

using namespace tumble;

static constexpr float AMPLITUDE = 6.0f;
static constexpr float FREQUENCY = 1.0f;

static struct {
    app::Layers layers;
    RigidBody *sensor;
    bool intersecting;
    int steps;
} state;

static Color hex(uint32_t rgb, float alpha = 1.0f) {
    return {(float)((rgb >> 16) & 255) / 255.0f, (float)((rgb >> 8) & 255) / 255.0f, (float)(rgb & 255) / 255.0f, alpha};
}

/** a world-space label as crashcat's sprites are: a box `width` by `height` facing the camera, its text a fraction of the height */
static void init(App &app) {
    state.steps = 0;
    state.intersecting = false;
    app.world = app::create_world(state.layers, {0.0f, 0.0f, 0.0f});

    // a kinematic sensor swept side to side
    RigidBodySettings sensor;
    sensor.shape = app::keep(app, box::create({.half_extents = {1.5f, 1.0f, 1.5f}}));
    sensor.object_layer = state.layers.moving;
    sensor.motion_type = MotionType::KINEMATIC;
    sensor.position = {-8.0f, 1.0f, 0.0f};
    sensor.sensor = true;
    state.sensor = rigid_body::create(*app.world, sensor);

    Shape *cube = app::keep(app, box::create({.half_extents = {0.5f, 0.5f, 0.5f}}));
    RigidBodySettings dynamic_box;
    dynamic_box.shape = cube;
    dynamic_box.object_layer = state.layers.moving;
    dynamic_box.motion_type = MotionType::DYNAMIC;
    dynamic_box.position = {-2.0f, 1.0f, 0.0f};
    dynamic_box.restitution = 0.3f;
    dynamic_box.friction = 0.5f;
    rigid_body::create(*app.world, dynamic_box);

    RigidBodySettings static_box = dynamic_box;
    static_box.object_layer = state.layers.not_moving;
    static_box.motion_type = MotionType::STATIC;
    static_box.position = {2.0f, 1.0f, 0.0f};
    rigid_body::create(*app.world, static_box);

    // no ground here
    app.drawing.grid = false;
    camera::look_at(app.camera, {0.0f, 5.0f, 15.0f}, {0.0f, 2.0f, 0.0f});
}

static void step(App &app, float) {
    // crashcat's clock, as steps
    state.steps++;
    const float time = (float)state.steps / 60.0f;
    rigid_body::set_position(*app.world, state.sensor, vec3f::make(sinf(time * FREQUENCY) * AMPLITUDE, 1.0f, 0.0f), false);
}

static bool touches_sensor(const ContactEvent &contact) { return contact.body_id_a == state.sensor->id || contact.body_id_b == state.sensor->id; }

static void update(App &app, float) {
    // the last step's contacts, added or kept
    const WorldEvents &events = app.world->events;
    state.intersecting = false;
    for (uint32_t i = 0; i < events.contacts_added.length; i++) state.intersecting = state.intersecting || touches_sensor(events.contacts_added[i]);
    for (uint32_t i = 0; i < events.contacts_persisted.length; i++) state.intersecting = state.intersecting || touches_sensor(events.contacts_persisted[i]);

    app::label(app, {state.sensor->position.x, 3.5f, 0.0f}, state.intersecting ? "hit" : "no hit", {4.0f, 1.0f, 0.5f, hex(0x5f5f5f),
          state.intersecting ? hex(0x4caf50) : hex(0xf44336)});
}

static void ui(App &) { ImGui::Checkbox("Collide with Non-Dynamic", &state.sensor->collide_kinematic_vs_non_dynamic); }

static const Example EXAMPLE_SENSOR = {"Sensor", init, step, update, ui, nullptr};
static const examples::Registration REGISTERED(EXAMPLE_SENSOR);
