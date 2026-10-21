#include "app/app.h"
#include "app/examples.h"
#include "imgui.h"
#include "tumble/physics/constraints/cone_constraint.h"
#include "tumble/physics/constraints/distance_constraint.h"
#include "tumble/physics/constraints/fixed_constraint.h"
#include "tumble/physics/constraints/hinge_constraint.h"
#include "tumble/physics/constraints/point_constraint.h"
#include "tumble/physics/constraints/six_dof_constraint.h"
#include "tumble/physics/constraints/slider_constraint.h"
#include "tumble/physics/constraints/swing_twist_constraint.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/update.h"

using namespace tumble;

static constexpr int CHAIN_LENGTH = 10;
static constexpr float START_Y = 20.0f;

enum ChainKind { FIXED, POINT, DISTANCE, HINGE, SLIDER, CONE, SWING_TWIST, SIX_DOF, CHAIN_KIND_COUNT };

static const char *const CHAIN_NAMES[CHAIN_KIND_COUNT] = {"Fixed", "Point", "Distance", "Hinge", "Slider", "Cone", "Swing-Twist", "6DOF"};

static struct {
    app::Layers layers;
    shapes::Shape *link;
} state;

/** text centred on a world point, over the scene */
static void connect(App &app, ChainKind kind, rigid_body::RigidBody *a, rigid_body::RigidBody *b, float x, float z) {
    world::World &world = *app.world;
    switch (kind) {
        case FIXED:
            fixed_constraint::create(world, a, b, {.point_a = {x, START_Y, z - 0.5f}, .point_b = {x, START_Y, z - 0.5f}});
            break;
        case POINT:
            point_constraint::create(world, a, b, {.point_a = {x, START_Y, z - 0.5f}, .point_b = {x, START_Y, z - 0.5f}});
            break;
        case DISTANCE:
            distance_constraint::create(world, a, b,
                                        {.point_a = {x, START_Y, z - 0.5f}, .point_b = {x, START_Y, z - 0.5f}, .min_distance = 0.0f, .max_distance = 1.0f});
            break;
        case HINGE:
            hinge_constraint::create(world, a, b,
                                     {.point_a = {x, START_Y - 0.5f, z - 0.5f},
                                      .point_b = {x, START_Y - 0.5f, z - 0.5f},
                                      .hinge_axis_a = {1.0f, 0.0f, 0.0f},
                                      .hinge_axis_b = {1.0f, 0.0f, 0.0f},
                                      .normal_axis_a = {0.0f, 1.0f, 0.0f},
                                      .normal_axis_b = {0.0f, 1.0f, 0.0f}});
            break;
        case SLIDER: {
            const Vec3f slider_axis = vec3f::normalize(vec3f::make(0.0f, -1.0f, 1.0f));
            Float3 axis, normal;
            vec3f::store(axis, slider_axis);
            vec3f::store(normal, vec3f::perpendicular(slider_axis));
            slider_constraint::create(world, a, b,
                                      {.point_a = {x, START_Y, z - 0.5f},
                                       .point_b = {x, START_Y, z - 0.5f},
                                       .slider_axis_a = axis,
                                       .slider_axis_b = axis,
                                       .normal_axis_a = normal,
                                       .normal_axis_b = normal,
                                       .limits_min = 0.0f,
                                       .limits_max = 1.0f,
                                       .limits_spring_settings = {.mode = spring_settings::STIFFNESS_AND_DAMPING, .frequency_or_stiffness = 0.0f}});
            break;
        }
        case CONE:
            cone_constraint::create(world, a, b,
                                    {.point_a = {x, START_Y, z},
                                     .twist_axis_a = {0.0f, 0.0f, 1.0f},
                                     .point_b = {x, START_Y, z},
                                     .twist_axis_b = {0.0f, 0.0f, 1.0f},
                                     .half_cone_angle = 0.1f * scalar::PI});
            break;
        case SWING_TWIST:
            swing_twist_constraint::create(world, a, b,
                                           {.point_a = {x, START_Y, z},
                                            .point_b = {x, START_Y, z},
                                            .twist_axis_a = {0.0f, 0.0f, 1.0f},
                                            .plane_axis_a = {1.0f, 0.0f, 0.0f},
                                            .twist_axis_b = {0.0f, 0.0f, 1.0f},
                                            .plane_axis_b = {1.0f, 0.0f, 0.0f},
                                            .normal_half_cone_angle = 0.2f * scalar::PI,
                                            .plane_half_cone_angle = 0.1f * scalar::PI,
                                            .twist_min_angle = -0.3f * scalar::PI,
                                            .twist_max_angle = 0.4f * scalar::PI});
            break;
        case SIX_DOF: {
            const float max_angle = 10.0f * scalar::DEGREES_TO_RADIANS;
            six_dof_constraint::SixDOFConstraintSettings settings;
            settings.point_a = {x, START_Y, z};
            settings.point_b = {x, START_Y, z};
            settings.axis_x_a = {0.0f, 0.0f, 1.0f};
            settings.axis_y_a = {1.0f, 0.0f, 0.0f};
            settings.axis_x_b = {0.0f, 0.0f, 1.0f};
            settings.axis_y_b = {1.0f, 0.0f, 0.0f};
            for (uint32_t axis = 0; axis < six_dof_constraint::NUM; axis++) {
                const float limit = axis < six_dof_constraint::NUM_TRANSLATION ? 0.1f : max_angle;
                settings.limit_min[axis] = -limit;
                settings.limit_max[axis] = limit;
            }
            six_dof_constraint::create(world, a, b, settings);
            break;
        }
        default:
            break;
    }
}

/** a chain hanging from a static first link, the last link pushed sideways */
static void make_chain(App &app, ChainKind kind) {
    const float x = -15.0f + 5.0f * (float)kind;
    rigid_body::RigidBody *previous = nullptr;
    for (int z = 0; z < CHAIN_LENGTH; z++) {
        rigid_body::RigidBodySettings settings;
        settings.shape = state.link;
        settings.object_layer = z == 0 ? state.layers.not_moving : state.layers.moving;
        settings.motion_type = z == 0 ? motion_type::MOTION_STATIC : motion_type::MOTION_DYNAMIC;
        settings.position = {x, START_Y, (float)z};
        settings.restitution = 0.0f;
        settings.friction = 0.5f;
        rigid_body::RigidBody *body = rigid_body::create(*app.world, settings);
        if (previous != nullptr) connect(app, kind, previous, body, x, (float)z);
        if (z == CHAIN_LENGTH - 1) rigid_body::add_impulse(*app.world, body, vec3f::make(1000.0f, 0.0f, 0.0f));
        previous = body;
    }
}

static void init(App &app) {
    app.world = app::create_world(state.layers);
    rigid_body::RigidBodySettings ground;
    ground.shape = app::keep(app, box::create({.half_extents = {50.0f, 0.5f, 10.0f}}));
    ground.object_layer = state.layers.not_moving;
    ground.motion_type = motion_type::MOTION_STATIC;
    ground.position = {0.0f, -0.5f, 0.0f};
    ground.restitution = 0.0f;
    ground.friction = 0.5f;
    rigid_body::create(*app.world, ground);
    state.link = app::keep(app, box::create({.half_extents = {0.5f, 0.5f, 0.5f}, .convex_radius = 0.05f}));
    for (int kind = 0; kind < CHAIN_KIND_COUNT; kind++) make_chain(app, (ChainKind)kind);
    camera::look_at(app.camera, {10.0f, 30.0f, 30.0f}, {0.0f, 5.0f, 0.0f});
}

static void ui(App &app) {
    for (int kind = 0; kind < CHAIN_KIND_COUNT; kind++) {
        app::label(app, {-15.0f + 5.0f * (float)kind, START_Y + 2.0f, 0.5f * (float)CHAIN_LENGTH}, CHAIN_NAMES[kind], {4.0f, 1.0f, 0.375f});
    }
}

/** crashcat's listener: bodies held by a constraint between them do not collide */
static void step_world(App &app, float delta_time) {
    world::World &world = *app.world;
    update::step_begin(world, delta_time);
    do {
        update::find_pairs(world);
        for (uint32_t i = 0; i < world.events.pairs.length; i++) {
            const events::PairEvent &pair = world.events.pairs[i];
            if (rigid_body::bodies_share_constraint(world.bodies.pool[pair.body_index_a], world.bodies.pool[pair.body_index_b])) {
                events::reject_pair(world.answers, i);
            }
        }
        update::narrowphase(world);
    } while (world.events.woken > 0);
    update::solve(world);
    update::step_end(world);
}

static const Example EXAMPLE_CONSTRAINTS = {"Constraints", init, nullptr, nullptr, ui, nullptr, step_world};
static const examples::Registration REGISTERED(EXAMPLE_CONSTRAINTS);
