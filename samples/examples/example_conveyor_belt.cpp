#include <stdio.h>

#include "app/app.h"
#include "app/examples.h"
#include "app/vec.h"
#include "imgui.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/cylinder.h"
#include "tumble/physics/update.h"

using namespace tumble;

static constexpr float BELT_WIDTH = 10.0f;
static constexpr float BELT_LENGTH = 50.0f;
static constexpr float PI = 3.14159265f;
static constexpr int MAX_BELTS = 6;
static constexpr int MAX_LABELS = 18;

/** a body whose surface moves: along a local direction, or turning about a local axis */
struct Belt {
    BodyId body_id;
    bool angular;
    Float3 local_velocity;
};

namespace {

struct Label {
    Float3 position;
    float friction;
};

}  // namespace

static struct {
    app::Layers layers;
    Belt belts[MAX_BELTS];
    int belt_count;
    Label labels[MAX_LABELS];
    int label_count;
} state;

static const Float4 IDENTITY = {0.0f, 0.0f, 0.0f, 1.0f};

static const Belt *belt_of(BodyId id) {
    for (int i = 0; i < state.belt_count; i++) {
        if (state.belts[i].body_id == id) return &state.belts[i];
    }
    return nullptr;
}

static void add_belt(const RigidBody *body, bool angular, Float3 local_velocity) {
    state.belts[state.belt_count++] = {body->id, angular, local_velocity};
}

static RigidBody *add_body(World &world, Shape *shape, bool moving, Float3 position, Float4 quaternion, float friction) {
    RigidBodySettings settings;
    settings.shape = shape;
    settings.object_layer = moving ? state.layers.moving : state.layers.not_moving;
    settings.motion_type = moving ? MotionType::DYNAMIC : MotionType::STATIC;
    settings.position = position;
    settings.quaternion = quaternion;
    settings.restitution = 0.0f;
    settings.friction = friction;
    return rigid_body::create(world, settings);
}

/** the surface velocity of b relative to a, from whichever of them is a belt */
static void move_surfaces(ContactSettings &settings, const RigidBody *a, const RigidBody *b,
                          const Belt *belt_a, const Belt *belt_b) {
    const bool linear_a = belt_a != nullptr && !belt_a->angular;
    const bool linear_b = belt_b != nullptr && !belt_b->angular;
    if (linear_a || linear_b) {
        const Float3 velocity_a = linear_a ? vec::rotate(a->quaternion, belt_a->local_velocity) : Float3{0.0f, 0.0f, 0.0f};
        const Float3 velocity_b = linear_b ? vec::rotate(b->quaternion, belt_b->local_velocity) : Float3{0.0f, 0.0f, 0.0f};
        settings.relative_linear_surface_velocity = vec::subtract(velocity_b, velocity_a);
    }
    const bool angular_a = belt_a != nullptr && belt_a->angular;
    const bool angular_b = belt_b != nullptr && belt_b->angular;
    if (angular_a || angular_b) {
        const Float3 turn_a = angular_a ? vec::rotate(a->quaternion, belt_a->local_velocity) : Float3{0.0f, 0.0f, 0.0f};
        const Float3 turn_b = angular_b ? vec::rotate(b->quaternion, belt_b->local_velocity) : Float3{0.0f, 0.0f, 0.0f};
        // the angular velocity is about a's centre of mass: b's turning moves a's centre relative to b's
        settings.relative_linear_surface_velocity =
            angular_b ? vec::cross(turn_b, vec::subtract(a->center_of_mass_position, b->center_of_mass_position)) : Float3{0.0f, 0.0f, 0.0f};
        settings.relative_angular_surface_velocity = vec::subtract(turn_b, turn_a);
    }
}

/** the step's phases, the belts' contacts given their surface velocities between the narrowphase and the solve */
static void step_world(App &app, float delta_time) {
    World &world = *app.world;
    update::step_begin(world, delta_time);
    do {
        update::find_pairs(world);
        update::narrowphase(world);
    } while (world.events.woken > 0);
    for (uint32_t i = 0; i < world.answers.contact_override.length; i++) {
        const ContactEvent &contact = events::contact_event(world.events, i);
        const Belt *belt_a = belt_of(contact.body_id_a);
        const Belt *belt_b = belt_of(contact.body_id_b);
        if (belt_a == nullptr && belt_b == nullptr) continue;
        const RigidBody *a = rigid_body::get(world, contact.body_id_a);
        const RigidBody *b = rigid_body::get(world, contact.body_id_b);
        move_surfaces(events::override_contact(world.answers, world.events, i).settings, a, b, belt_a, belt_b);
    }
    update::solve(world);
    update::step_end(world);
}

static void init(App &app) {
    state.belt_count = 0;
    state.label_count = 0;
    app.world = app::create_world(state.layers);

    add_body(*app.world, app::keep(app, box::create({.half_extents = {100.0f, 0.5f, 100.0f}})), false, {0.0f, -0.5f, 0.0f}, IDENTITY, 1.0f);

    // four belts around a square, each turned a quarter from the last and tilted up a degree, rougher each time
    Shape *belt_shape = app::keep(app, box::create({.half_extents = {BELT_WIDTH, 0.1f, BELT_LENGTH}}));
    for (int i = 0; i < 4; i++) {
        const Float4 turn = vec::quat_axis_angle({0.0f, 1.0f, 0.0f}, 0.5f * PI * (float)i);
        const Float4 tilt = vec::quat_axis_angle({1.0f, 0.0f, 0.0f}, PI / 180.0f);
        const Float4 rotation = vec::quat_multiply(turn, tilt);
        const Float3 position = vec::rotate(rotation, {BELT_LENGTH, 6.0f, BELT_WIDTH});
        add_belt(add_body(*app.world, belt_shape, false, position, rotation, 0.25f * (float)(i + 1)), false, {0.0f, 0.0f, -10.0f});
    }

    Shape *cargo = app::keep(app, box::create({.half_extents = {2.0f, 2.0f, 2.0f}}));
    for (int i = 0; i <= 10; i++) {
        const float friction = 1.0f - 0.1f * (float)i;
        const float x = -BELT_LENGTH + (float)i * 10.0f;
        add_body(*app.world, cargo, true, {x, 10.0f, -BELT_LENGTH}, IDENTITY, friction);
        state.labels[state.label_count++] = {{x, 14.0f, -BELT_LENGTH}, friction};
    }

    // a belt that is itself dynamic, lying on two loose rollers
    Shape *roller = app::keep(app, cylinder::create({.half_height = 6.0f, .radius = 1.0f}));
    const Float4 roller_rotation = vec::quat_axis_angle({0.0f, 0.0f, 1.0f}, 0.5f * PI);
    add_body(*app.world, roller, true, {-25.0f, 1.0f, -20.0f}, roller_rotation, 0.1f);
    add_body(*app.world, roller, true, {-25.0f, 1.0f, 20.0f}, roller_rotation, 0.1f);
    Shape *dynamic_belt = app::keep(app, box::create({.half_extents = {5.0f, 0.1f, 25.0f}}));
    add_belt(add_body(*app.world, dynamic_belt, true, {-25.0f, 3.0f, 0.0f}, IDENTITY, 1.0f), false, {0.0f, 0.0f, -10.0f});
    add_body(*app.world, cargo, true, {-25.0f, 6.0f, 15.0f}, IDENTITY, 1.0f);

    // a lazy susan, turning 10 degrees a second
    Shape *turntable = app::keep(app, box::create({.half_extents = {20.0f, 0.1f, 20.0f}}));
    add_belt(add_body(*app.world, turntable, false, {10.0f, 3.0f, 0.0f}, IDENTITY, 1.0f), true, {0.0f, 10.0f * PI / 180.0f, 0.0f});
    for (int i = 0; i <= 6; i++) {
        const float friction = 1.0f - 0.1f * (float)i;
        const float z = -15.0f + 5.0f * (float)i;
        add_body(*app.world, cargo, true, {10.0f, 10.0f, z}, IDENTITY, friction);
        state.labels[state.label_count++] = {{10.0f, 14.0f, z}, friction};
    }

    app.drawing.grid = false;
    camera::look_at(app.camera, {60.0f, 20.0f, 50.0f}, {0.0f, 0.0f, 0.0f});
}

/** each belt's friction over it */
static void ui(App &app) {
    for (int i = 0; i < state.label_count; i++) {
        char text[16];
        snprintf(text, sizeof(text), "u=%.1f", (double)state.labels[i].friction);
        app::label(app, state.labels[i].position, text, {2.0f, 1.0f, 0.44f});
    }
}

static const Example EXAMPLE_CONVEYOR_BELT = {"Conveyor Belt", init, nullptr, nullptr, ui, nullptr, step_world};
static const examples::Registration REGISTERED(EXAMPLE_CONVEYOR_BELT);
