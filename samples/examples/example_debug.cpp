#include <math.h>

#include "app/app.h"
#include "app/examples.h"
#include "app/vec.h"
#include "imgui.h"
#include "tumble/physics/events.h"
#include "tumble/physics/update.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/constraints/cone_constraint.h"
#include "tumble/physics/constraints/distance_constraint.h"
#include "tumble/physics/constraints/fixed_constraint.h"
#include "tumble/physics/constraints/hinge_constraint.h"
#include "tumble/physics/constraints/point_constraint.h"
#include "tumble/physics/constraints/six_dof_constraint.h"
#include "tumble/physics/constraints/slider_constraint.h"
#include "tumble/physics/constraints/swing_twist_constraint.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/convex_hull.h"
#include "tumble/physics/shapes/cylinder.h"
#include "tumble/physics/shapes/plane.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/shapes/static_compound.h"

using namespace tumble;

static constexpr float PI = 3.14159265f;
static constexpr int CHAIN_LENGTH = 5;
static constexpr int MAX_LABELS = 24;


namespace {

struct Label {
    const char *text;
    Float3 position;
};

}  // namespace

static struct {
    app::Layers layers;
    Shape *link;
    Label labels[MAX_LABELS];
    int label_count;
} state;

static const Float4 IDENTITY = {0.0f, 0.0f, 0.0f, 1.0f};

static void add_label(const char *text, float x, float y, float z) {
    if (state.label_count < MAX_LABELS) state.labels[state.label_count++] = {text, {x, y, z}};
}

/** a world-space label as crashcat's sprites are: a box `width` by `height` facing the camera, its text a fraction of the height */
static RigidBody *create_body(App &app, Shape *shape, Float3 position, MotionType motion_type) {
    RigidBodySettings settings;
    settings.shape = shape;
    settings.object_layer = motion_type == MotionType::STATIC ? state.layers.not_moving : state.layers.moving;
    settings.motion_type = motion_type;
    settings.position = position;
    settings.restitution = 0.0f;
    settings.friction = 0.5f;
    return rigid_body::create(*app.world, settings);
}

/** five boxes in a row along z from a static first, each joined to the one before, the last pushed along x */
template <typename Join>
static void chain(App &app, float start_x, const char *name, Join join) {
    const float start_y = 20.0f;
    const float start_z = 0.0f;
    RigidBody *previous = nullptr;
    for (int z = 0; z < CHAIN_LENGTH; z++) {
        const MotionType motion = z == 0 ? MotionType::STATIC : MotionType::DYNAMIC;
        RigidBody *body = create_body(app, state.link, {start_x, start_y, start_z + (float)z}, motion);
        if (previous != nullptr) join(previous, body, start_x, start_y, start_z + (float)z);
        if (z == CHAIN_LENGTH - 1) rigid_body::add_impulse(*app.world, body, vec3f::make(500.0f, 0.0f, 0.0f));
        previous = body;
    }
    add_label(name, start_x, start_y + 2.0f, start_z + (float)CHAIN_LENGTH / 2.0f);
}

static void make_shapes(App &app) {
    add_label("Shapes", 0.0f, 12.0f, -15.0f);
    const float start_x = -12.0f, start_y = 1.0f, spacing = 4.0f;
    const MotionType dynamic = MotionType::DYNAMIC;
    create_body(app, app::keep(app, sphere::create({.radius = 1.0f})), {start_x, start_y, 0.0f}, dynamic);
    add_label("Sphere", start_x, start_y + 2.5f, 0.0f);
    create_body(app, app::keep(app, box::create({.half_extents = {1.0f, 1.0f, 1.0f}, .convex_radius = 0.05f})), {start_x + spacing, start_y, 0.0f}, dynamic);
    add_label("Box", start_x + spacing, start_y + 2.5f, 0.0f);
    create_body(app, app::keep(app, capsule::create({.half_height_of_cylinder = 1.0f, .radius = 0.8f})), {start_x + spacing * 2.0f, start_y + 1.0f, 0.0f},
                dynamic);
    add_label("Capsule", start_x + spacing * 2.0f, start_y + 3.5f, 0.0f);
    create_body(app, app::keep(app, cylinder::create({.half_height = 1.0f, .radius = 0.8f})), {start_x + spacing * 3.0f, start_y, 0.0f}, dynamic);
    add_label("Cylinder", start_x + spacing * 3.0f, start_y + 2.5f, 0.0f);

    add_label("Static Shapes", -10.0f, 6.0f, -8.0f);
    const float static_x = -14.0f, static_y = 0.5f;
    const MotionType fixed = MotionType::STATIC;
    create_body(app, app::keep(app, box::create({.half_extents = {1.5f, 0.5f, 1.5f}, .convex_radius = 0.05f})), {static_x, static_y, -10.0f}, fixed);
    add_label("Static Box", static_x, static_y + 2.0f, -10.0f);
    create_body(app, app::keep(app, sphere::create({.radius = 1.0f})), {static_x + spacing, static_y + 1.0f, -10.0f}, fixed);
    add_label("Static Sphere", static_x + spacing, static_y + 3.0f, -10.0f);

    Shape *ball = app::keep(app, sphere::create({.radius = 0.6f}));
    Shape *bar = app::keep(app, cylinder::create({.half_height = 0.5f, .radius = 0.3f}));
    const CompoundShapeSettings::Child dumbbell[] = {
        {.position = {-0.6f, 0.0f, 0.0f}, .quaternion = IDENTITY, .shape = ball},
        {.position = {0.6f, 0.0f, 0.0f}, .quaternion = IDENTITY, .shape = ball},
        {.position = {0.0f, 0.0f, 0.0f}, .quaternion = vec::quat_axis_angle({0.0f, 0.0f, 1.0f}, PI * 0.5f), .shape = bar},
    };
    create_body(app, app::keep(app, compound::create({.children = dumbbell, .num_children = 3})), {10.0f, 1.0f, -10.0f}, dynamic);
    add_label("Compound", 10.0f, 4.0f, -10.0f);

    Shape *post = app::keep(app, box::create({.half_extents = {0.3f, 1.0f, 0.3f}, .convex_radius = 0.05f}));
    Shape *rung = app::keep(app, box::create({.half_extents = {1.2f, 0.1f, 0.1f}, .convex_radius = 0.05f}));
    const CompoundShapeSettings::Child frame[] = {
        {.position = {-1.0f, 0.0f, 0.0f}, .quaternion = IDENTITY, .shape = post},
        {.position = {1.0f, 0.0f, 0.0f}, .quaternion = IDENTITY, .shape = post},
        {.position = {0.0f, -0.5f, -0.5f}, .quaternion = IDENTITY, .shape = rung},
        {.position = {0.0f, -0.5f, 0.5f}, .quaternion = IDENTITY, .shape = rung},
    };
    create_body(app, app::keep(app, static_compound::create({.children = frame, .num_children = 4})), {10.0f, 1.0f, -4.0f}, fixed);
    add_label("Static Compound", 10.0f, 4.0f, -4.0f);

    const float pyramid[] = {-0.8f, 0.0f, -0.8f, 0.8f, 0.0f, -0.8f, 0.8f, 0.0f, 0.8f, -0.8f, 0.0f, 0.8f, 0.0f, 1.2f, 0.0f};
    create_body(app, app::keep(app, convex_hull::create({.positions = pyramid, .num_positions = 5, .convex_radius = 0.05f})), {10.0f, 1.0f, 2.0f}, dynamic);
    add_label("Convex Hull", 10.0f, 4.0f, 2.0f);
}

static void make_chains(App &app) {
    add_label("Constraints", -10.0f, 15.0f, 0.0f);
    World &world = *app.world;

    chain(app, -15.0f, "Fixed", [&](RigidBody *a, RigidBody *b, float x, float y, float z) {
        fixed_constraint::create(world, a, b, {.point_a = {x, y, z - 0.5f}, .point_b = {x, y, z - 0.5f}});
    });
    chain(app, -10.0f, "Point", [&](RigidBody *a, RigidBody *b, float x, float y, float z) {
        point_constraint::create(world, a, b, {.point_a = {x, y, z - 0.5f}, .point_b = {x, y, z - 0.5f}});
    });
    chain(app, -5.0f, "Distance", [&](RigidBody *a, RigidBody *b, float x, float y, float z) {
        distance_constraint::create(world, a, b, {.point_a = {x, y, z - 0.5f}, .point_b = {x, y, z - 0.5f}, .min_distance = 0.0f, .max_distance = 1.0f});
    });
    chain(app, 0.0f, "Hinge", [&](RigidBody *a, RigidBody *b, float x, float y, float z) {
        hinge_constraint::create(world, a, b,
                                 {.point_a = {x, y - 0.5f, z - 0.5f},
                                  .point_b = {x, y - 0.5f, z - 0.5f},
                                  .hinge_axis_a = {1.0f, 0.0f, 0.0f},
                                  .hinge_axis_b = {1.0f, 0.0f, 0.0f},
                                  .normal_axis_a = {0.0f, 1.0f, 0.0f},
                                  .normal_axis_b = {0.0f, 1.0f, 0.0f}});
    });
    const Vec3f slider_axis = vec3f::normalize(vec3f::make(0.0f, -1.0f, 1.0f));
    const Vec3f normal_axis = vec3f::perpendicular(slider_axis);
    const Float3 slider = {vec3f::x(slider_axis), vec3f::y(slider_axis), vec3f::z(slider_axis)};
    const Float3 normal = {vec3f::x(normal_axis), vec3f::y(normal_axis), vec3f::z(normal_axis)};
    chain(app, 5.0f, "Slider", [&](RigidBody *a, RigidBody *b, float x, float y, float z) {
        slider_constraint::create(world, a, b,
                                  {.point_a = {x, y, z - 0.5f},
                                   .point_b = {x, y, z - 0.5f},
                                   .slider_axis_a = slider,
                                   .slider_axis_b = slider,
                                   .normal_axis_a = normal,
                                   .normal_axis_b = normal,
                                   .limits_min = 0.0f,
                                   .limits_max = 1.0f});
    });
    chain(app, 10.0f, "Cone", [&](RigidBody *a, RigidBody *b, float x, float y, float z) {
        cone_constraint::create(world, a, b,
                                {.point_a = {x, y, z}, .twist_axis_a = {0.0f, 0.0f, 1.0f}, .point_b = {x, y, z}, .twist_axis_b = {0.0f, 0.0f, 1.0f}, .half_cone_angle = 0.1f * PI});
    });
    chain(app, 15.0f, "Swing-Twist", [&](RigidBody *a, RigidBody *b, float x, float y, float z) {
        swing_twist_constraint::create(world, a, b,
                                       {.point_a = {x, y, z},
                                        .point_b = {x, y, z},
                                        .twist_axis_a = {0.0f, 0.0f, 1.0f},
                                        .plane_axis_a = {1.0f, 0.0f, 0.0f},
                                        .twist_axis_b = {0.0f, 0.0f, 1.0f},
                                        .plane_axis_b = {1.0f, 0.0f, 0.0f},
                                        .normal_half_cone_angle = 0.2f * PI,
                                        .plane_half_cone_angle = 0.1f * PI,
                                        .twist_min_angle = -0.3f * PI,
                                        .twist_max_angle = 0.4f * PI});
    });
    const float max_angle = PI / 18.0f;
    chain(app, 20.0f, "6DOF", [&](RigidBody *a, RigidBody *b, float x, float y, float z) {
        SixDOFConstraintSettings settings;
        settings.point_a = {x, y, z};
        settings.point_b = {x, y, z};
        settings.axis_x_a = {0.0f, 0.0f, 1.0f};
        settings.axis_y_a = {1.0f, 0.0f, 0.0f};
        settings.axis_x_b = {0.0f, 0.0f, 1.0f};
        settings.axis_y_b = {1.0f, 0.0f, 0.0f};
        for (uint32_t axis = 0; axis < SixDOFConstraint::Axis::NUM; axis++) {
            const float limit = axis < SixDOFConstraint::Axis::NUM_TRANSLATION ? 0.1f : max_angle;
            settings.limit_min[axis] = -limit;
            settings.limit_max[axis] = limit;
        }
        six_dof_constraint::create(world, a, b, settings);
    });
}

static void init(App &app) {
    state.label_count = 0;
    app.world = app::create_world(state.layers);

    RigidBodySettings ground;
    ground.shape = app::keep(app, plane::create({.plane = {{0.0f, 1.0f, 0.0f}, 0.0f}, .half_extent = 50.0f}));
    ground.object_layer = state.layers.not_moving;
    ground.motion_type = MotionType::STATIC;
    ground.restitution = 0.0f;
    ground.friction = 0.5f;
    rigid_body::create(*app.world, ground);

    state.link = app::keep(app, box::create({.half_extents = {0.5f, 0.5f, 0.5f}, .convex_radius = 0.05f}));
    make_shapes(app);
    make_chains(app);

    // crashcat draws this one as the library's debug lines alone: bodies by motion type, contacts, manifolds and joints
    app.drawing.grid = false;
    app.drawing.meshes = false;
    app.drawing.wireframes = true;
    app.drawing.velocities = false;
    app.drawing.contacts = true;
    app.drawing.contact_manifolds = true;
    app.drawing.joints = true;
    app.drawing.color_mode = DebugDraw::BodyColorMode::MOTION_TYPE;
    camera::look_at(app.camera, {0.0f, 15.0f, 30.0f}, {0.0f, 5.0f, 0.0f});
}

static void update(App &app, float) {
    for (int i = 0; i < state.label_count; i++) app::label(app, state.labels[i].position, state.labels[i].text, {4.0f, 1.0f, 0.375f});
}

/** a step by its phases, joined bodies' pairs rejected so the links of a chain pass through each other */
static void step_world(App &app, float delta_time) {
    World &world = *app.world;
    tumble::update::step_begin(world, delta_time);
    do {
        tumble::update::find_pairs(world);
        for (uint32_t i = 0; i < world.events.pairs.length; i++) {
            PairEvent &pair = world.events.pairs[i];
            if (rigid_body::bodies_share_constraint(world.bodies.pool[pair.body_index_a], world.bodies.pool[pair.body_index_b])) {
                pair.rejected = true;
            }
        }
        tumble::update::narrowphase(world);
    } while (world.events.woken > 0);
    tumble::update::solve(world);
    tumble::update::step_end(world);
}

static const Example EXAMPLE_DEBUG = {"Debug Renderer", init, nullptr, ::update, nullptr, nullptr, step_world};
static const examples::Registration REGISTERED(EXAMPLE_DEBUG);
