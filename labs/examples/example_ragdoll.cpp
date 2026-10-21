#include <math.h>

#include "app/app.h"
#include "app/examples.h"
#include "app/vec.h"
#include "imgui.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/physics/constraints/distance_constraint.h"
#include "tumble/physics/constraints/swing_twist_constraint.h"
#include "tumble/physics/query.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/update.h"

using namespace tumble;

enum Part : uint32_t {
    UPPER_BODY,
    HEAD,
    UPPER_LEFT_ARM,
    LOWER_LEFT_ARM,
    UPPER_RIGHT_ARM,
    LOWER_RIGHT_ARM,
    PELVIS,
    UPPER_LEFT_LEG,
    LOWER_LEFT_LEG,
    UPPER_RIGHT_LEG,
    LOWER_RIGHT_LEG,
    PART_COUNT,
};

static constexpr uint32_t NO_PARENT = PART_COUNT;
static constexpr int RAGDOLL_COUNT = 3;
static constexpr int JOINT_COUNT = 10;
static constexpr float PI = 3.14159265f;
static constexpr float RAY_LENGTH = 1000.0f;

struct PartSettings {
    Float3 half_extents;
    Float3 position;
};

/** a swing twist joint from part a to part b, its pivots and twist axes local to each */
struct JointSettings {
    Part part_a;
    Part part_b;
    Float3 pivot_a;
    Float3 pivot_b;
    Float3 axis_a;
    Float3 axis_b;
    float angle;
    float twist_angle;
};

struct RagdollSettings {
    PartSettings parts[PART_COUNT];
    JointSettings joints[JOINT_COUNT];
    /** each part's parent in the skeleton, the pelvis its root */
    uint32_t parents[PART_COUNT];
    float density;
};

struct Ragdoll {
    rigid_body::RigidBody *bodies[PART_COUNT];
};

static struct {
    app::Layers layers;
    RagdollSettings settings;
    Ragdoll ragdolls[RAGDOLL_COUNT];
    rigid_body::RigidBody *pointer;
    constraint_id::ConstraintId drag;
    bool dragging;
    /** the plane the dragged point moves in, facing the camera as it was when grabbed */
    Float3 plane_point;
    Float3 plane_normal;
    Float3 pointer_position;
    filter::Filter filter;
    cast_ray::CastRayCollector collector;
} state;

static RagdollSettings make_settings(float scale, float angle_a, float angle_b, float twist_angle) {
    const float shoulders_distance = 0.45f * scale;
    const float upper_arm_length = 0.4f * scale;
    const float lower_arm_length = 0.4f * scale;
    const float upper_arm_size = 0.15f * scale;
    const float lower_arm_size = 0.15f * scale;
    const float neck_length = 0.1f * scale;
    const float head_radius = 0.2f * scale;
    const float upper_body_length = 0.6f * scale;
    const float pelvis_length = 0.2f * scale;
    const float pelvis_size = 0.25f * scale;
    const float upper_leg_length = 0.5f * scale;
    const float upper_leg_size = 0.15f * scale;
    const float lower_leg_size = 0.15f * scale;
    const float lower_leg_length = 0.5f * scale;

    const float lower_leg_y = lower_leg_length / 2.0f;
    const float upper_leg_y = lower_leg_y + lower_leg_length / 2.0f + upper_leg_length / 2.0f;
    const float pelvis_y = upper_leg_y + upper_leg_length / 2.0f + pelvis_length / 2.0f;
    const float upper_body_y = pelvis_y + pelvis_length / 2.0f + upper_body_length / 2.0f;
    const float head_y = upper_body_y + upper_body_length / 2.0f + head_radius / 2.0f + neck_length;
    const float arm_y = upper_body_y + upper_body_length / 2.0f;
    const float upper_arm_x = shoulders_distance / 2.0f + upper_arm_length / 2.0f;
    const float lower_arm_x = upper_arm_x + lower_arm_length / 2.0f + upper_arm_length / 2.0f;
    const float leg_x = shoulders_distance / 3.0f;

    RagdollSettings out = {};
    out.density = scale;
    out.parts[LOWER_LEFT_LEG] = {{lower_leg_size * 0.5f, lower_leg_length * 0.5f, lower_leg_size * 0.5f}, {-leg_x, lower_leg_y, 0.0f}};
    out.parts[LOWER_RIGHT_LEG] = {{lower_leg_size * 0.5f, lower_leg_length * 0.5f, lower_leg_size * 0.5f}, {leg_x, lower_leg_y, 0.0f}};
    out.parts[UPPER_LEFT_LEG] = {{upper_leg_size * 0.5f, upper_leg_length * 0.5f, upper_leg_size * 0.5f}, {-leg_x, upper_leg_y, 0.0f}};
    out.parts[UPPER_RIGHT_LEG] = {{upper_leg_size * 0.5f, upper_leg_length * 0.5f, upper_leg_size * 0.5f}, {leg_x, upper_leg_y, 0.0f}};
    out.parts[PELVIS] = {{shoulders_distance * 0.5f, pelvis_length * 0.5f, pelvis_size * 0.5f}, {0.0f, pelvis_y, 0.0f}};
    out.parts[UPPER_BODY] = {{shoulders_distance * 0.5f, upper_body_length * 0.5f, lower_arm_size * 0.75f}, {0.0f, upper_body_y, 0.0f}};
    out.parts[HEAD] = {{head_radius * 0.6f, head_radius * 0.7f, head_radius * 0.6f}, {0.0f, head_y, 0.0f}};
    out.parts[UPPER_LEFT_ARM] = {{upper_arm_length * 0.5f, upper_arm_size * 0.5f, upper_arm_size * 0.5f}, {-upper_arm_x, arm_y, 0.0f}};
    out.parts[UPPER_RIGHT_ARM] = {{upper_arm_length * 0.5f, upper_arm_size * 0.5f, upper_arm_size * 0.5f}, {upper_arm_x, arm_y, 0.0f}};
    out.parts[LOWER_LEFT_ARM] = {{lower_arm_length * 0.5f, lower_arm_size * 0.5f, lower_arm_size * 0.5f}, {-lower_arm_x, arm_y, 0.0f}};
    out.parts[LOWER_RIGHT_ARM] = {{lower_arm_length * 0.5f, lower_arm_size * 0.5f, lower_arm_size * 0.5f}, {lower_arm_x, arm_y, 0.0f}};

    const Float3 up = {0.0f, 1.0f, 0.0f};
    const Float3 side = {1.0f, 0.0f, 0.0f};
    out.joints[0] = {HEAD, UPPER_BODY, {0.0f, -head_radius - neck_length / 2.0f, 0.0f}, {0.0f, upper_body_length / 2.0f, 0.0f}, up, up, angle_a, twist_angle};
    out.joints[1] = {LOWER_LEFT_LEG, UPPER_LEFT_LEG, {0.0f, lower_leg_length / 2.0f, 0.0f}, {0.0f, -upper_leg_length / 2.0f, 0.0f}, up, up, angle_a, twist_angle};
    out.joints[2] = {LOWER_RIGHT_LEG, UPPER_RIGHT_LEG, {0.0f, lower_leg_length / 2.0f, 0.0f}, {0.0f, -upper_leg_length / 2.0f, 0.0f}, up, up, angle_a, twist_angle};
    out.joints[3] = {UPPER_LEFT_LEG, PELVIS, {0.0f, upper_leg_length / 2.0f, 0.0f}, {-leg_x, -pelvis_length / 2.0f, 0.0f}, up, up, angle_a, twist_angle};
    out.joints[4] = {UPPER_RIGHT_LEG, PELVIS, {0.0f, upper_leg_length / 2.0f, 0.0f}, {leg_x, -pelvis_length / 2.0f, 0.0f}, up, up, angle_a, twist_angle};
    out.joints[5] = {PELVIS, UPPER_BODY, {0.0f, pelvis_length / 2.0f, 0.0f}, {0.0f, -upper_body_length / 2.0f, 0.0f}, up, up, angle_a, twist_angle};
    out.joints[6] = {UPPER_BODY, UPPER_LEFT_ARM, {-shoulders_distance / 2.0f, upper_body_length / 2.0f, 0.0f}, {upper_arm_length / 2.0f, 0.0f, 0.0f}, side, side, angle_b, twist_angle};
    out.joints[7] = {UPPER_BODY, UPPER_RIGHT_ARM, {shoulders_distance / 2.0f, upper_body_length / 2.0f, 0.0f}, {-upper_arm_length / 2.0f, 0.0f, 0.0f}, side, side, angle_b, twist_angle};
    out.joints[8] = {LOWER_LEFT_ARM, UPPER_LEFT_ARM, {lower_arm_length / 2.0f, 0.0f, 0.0f}, {-upper_arm_length / 2.0f, 0.0f, 0.0f}, side, side, angle_a, twist_angle};
    out.joints[9] = {LOWER_RIGHT_ARM, UPPER_RIGHT_ARM, {-lower_arm_length / 2.0f, 0.0f, 0.0f}, {upper_arm_length / 2.0f, 0.0f, 0.0f}, side, side, angle_a, twist_angle};

    out.parents[PELVIS] = NO_PARENT;
    out.parents[UPPER_BODY] = PELVIS;
    out.parents[HEAD] = UPPER_BODY;
    out.parents[UPPER_LEFT_ARM] = UPPER_BODY;
    out.parents[LOWER_LEFT_ARM] = UPPER_LEFT_ARM;
    out.parents[UPPER_RIGHT_ARM] = UPPER_BODY;
    out.parents[LOWER_RIGHT_ARM] = UPPER_RIGHT_ARM;
    out.parents[UPPER_LEFT_LEG] = PELVIS;
    out.parents[LOWER_LEFT_LEG] = UPPER_LEFT_LEG;
    out.parents[UPPER_RIGHT_LEG] = PELVIS;
    out.parents[LOWER_RIGHT_LEG] = UPPER_RIGHT_LEG;
    return out;
}

/**
 * jolt's ragdoll stabilisation ("stop my constraints from blowing up", oliver strunk): child to parent mass ratios
 * kept within [0.8, 1.2], and each parent's inertia raised towards the sum of its children's
 */
static void stabilize(Ragdoll &ragdoll, const RagdollSettings &settings) {
    constexpr float MIN_MASS_RATIO = 0.8f;
    constexpr float MAX_MASS_RATIO = 1.2f;
    constexpr float MAX_INERTIA_INCREASE = 2.0f;

    // the parts breadth first from the root, so every parent comes before its children
    uint32_t chain[PART_COUNT];
    uint32_t chain_length = 0;
    chain[chain_length++] = PELVIS;
    for (uint32_t i = 0; i < chain_length; i++) {
        for (uint32_t part = 0; part < PART_COUNT; part++) {
            if (settings.parents[part] == chain[i]) chain[chain_length++] = part;
        }
    }

    float mass_ratios[PART_COUNT];
    mass_ratios[chain[0]] = 1.0f;
    float total_mass_ratio = 1.0f;
    float total_mass = ragdoll.bodies[chain[0]]->mass_properties.mass;
    for (uint32_t i = 1; i < chain_length; i++) {
        const uint32_t part = chain[i];
        const uint32_t parent = settings.parents[part];
        const float ratio = ragdoll.bodies[part]->mass_properties.mass / ragdoll.bodies[parent]->mass_properties.mass;
        mass_ratios[part] = mass_ratios[parent] * fmaxf(MIN_MASS_RATIO, fminf(MAX_MASS_RATIO, ratio));
        total_mass_ratio += mass_ratios[part];
        total_mass += ragdoll.bodies[part]->mass_properties.mass;
    }
    const float ratio_to_mass = total_mass / total_mass_ratio;
    for (uint32_t i = 0; i < chain_length; i++) {
        const uint32_t part = chain[i];
        mass_properties::scale_to_mass(ragdoll.bodies[part]->mass_properties, mass_ratios[part] * ratio_to_mass);
    }

    Mat4f rotations[PART_COUNT];
    Vec3f diagonals[PART_COUNT];
    float child_sums[PART_COUNT] = {};
    bool decomposed[PART_COUNT] = {};
    for (uint32_t i = 0; i < chain_length; i++) {
        const uint32_t part = chain[i];
        decomposed[part] = motion_properties::decompose_principal_moments_of_inertia(mat4f::load(ragdoll.bodies[part]->mass_properties.inertia),
                                                                                     rotations[part], diagonals[part]);
    }
    for (uint32_t i = chain_length - 1; i > 0; i--) {
        const uint32_t part = chain[i];
        const uint32_t parent = settings.parents[part];
        if (decomposed[part] && decomposed[parent]) child_sums[parent] += vec3f::x(diagonals[part]) + child_sums[part];
    }
    for (uint32_t i = 0; i < chain_length; i++) {
        const uint32_t part = chain[i];
        if (!decomposed[part] || child_sums[part] == 0.0f) continue;
        const float minimum = fminf(MAX_INERTIA_INCREASE * vec3f::x(diagonals[part]), child_sums[part]);
        const Vec3f diagonal = vec3f::max(diagonals[part], vec3f::splat(minimum));
        const Mat4f inertia = mat4f::multiply(mat4f::multiply(rotations[part], mat4f::from_scaling(diagonal)), mat4f::transpose(rotations[part]));
        Float4x4 &stored = ragdoll.bodies[part]->mass_properties.inertia;
        mat4f::store(stored, inertia);
        stored.m[3] = stored.m[7] = stored.m[11] = stored.m[12] = stored.m[13] = stored.m[14] = 0.0f;
        stored.m[15] = 1.0f;
    }

    for (uint32_t part = 0; part < PART_COUNT; part++) {
        rigid_body::RigidBody *body = ragdoll.bodies[part];
        body->mass_properties_override = rigid_body::MASS_AND_INERTIA_PROVIDED;
        motion_properties::set_mass_properties(body->motion_properties, body->motion_properties.allowed_degrees_of_freedom, body->mass_properties);
    }
}

static Float3 tangent(Float3 axis) {
    const float ax = fabsf(axis.x), ay = fabsf(axis.y), az = fabsf(axis.z);
    if (ax <= ay && ax <= az) return vec::normalize({0.0f, -axis.z, axis.y});
    if (ay <= az) return vec::normalize({axis.z, 0.0f, -axis.x});
    return vec::normalize({-axis.y, axis.x, 0.0f});
}

static void create_ragdoll(App &app, Ragdoll &out, shapes::Shape *const *shapes, Float3 offset) {
    const RagdollSettings &settings = state.settings;
    for (uint32_t part = 0; part < PART_COUNT; part++) {
        rigid_body::RigidBodySettings body;
        body.shape = shapes[part];
        body.object_layer = state.layers.moving;
        body.motion_type = motion_type::MOTION_DYNAMIC;
        body.position = vec::add(settings.parts[part].position, offset);
        body.linear_damping = 0.05f;
        body.angular_damping = 0.05f;
        body.restitution = 0.0f;
        out.bodies[part] = rigid_body::create(*app.world, body);
    }
    stabilize(out, settings);
    for (int i = 0; i < JOINT_COUNT; i++) {
        const JointSettings &joint = settings.joints[i];
        swing_twist_constraint::SwingTwistConstraintSettings constraint;
        constraint.point_a = joint.pivot_a;
        constraint.point_b = joint.pivot_b;
        constraint.twist_axis_a = joint.axis_a;
        constraint.plane_axis_a = tangent(joint.axis_a);
        constraint.twist_axis_b = joint.axis_b;
        constraint.plane_axis_b = tangent(joint.axis_b);
        constraint.space = constraints::SPACE_LOCAL;
        constraint.normal_half_cone_angle = joint.angle;
        constraint.plane_half_cone_angle = joint.angle;
        constraint.twist_min_angle = -joint.twist_angle;
        constraint.twist_max_angle = joint.twist_angle;
        swing_twist_constraint::create(*app.world, out.bodies[joint.part_a], out.bodies[joint.part_b], constraint);
    }
}

static void init(App &app) {
    state.dragging = false;
    state.settings = make_settings(1.8f, PI / 4.0f, PI / 4.0f, 0.0f);
    app.world = app::create_world(state.layers, {0.0f, -20.0f, 0.0f});
    state.filter = filter::create(app.world->settings.layers);
    state.collector = cast_ray::create_collector(collector::COLLECT_CLOSEST);
    // the ground is at y = -4, below the grid
    app.drawing.grid = false;

    rigid_body::RigidBodySettings ground;
    ground.shape = app::keep(app, box::create({.half_extents = {500.0f, 1.0f, 500.0f}}));
    ground.object_layer = state.layers.not_moving;
    ground.motion_type = motion_type::MOTION_STATIC;
    ground.position = {0.0f, -5.0f, 0.0f};
    rigid_body::create(*app.world, ground);

    shapes::Shape *shapes[PART_COUNT];
    for (uint32_t part = 0; part < PART_COUNT; part++) {
        shapes[part] = app::keep(app, box::create({.half_extents = state.settings.parts[part].half_extents, .convex_radius = 0.05f,
                                                   .density = state.settings.density}));
    }
    for (int i = 0; i < RAGDOLL_COUNT; i++) {
        create_ragdoll(app, state.ragdolls[i], shapes, {-5.0f + (float)i * 4.0f, (float)i * 3.0f, 0.0f});
    }
    for (int i = 0; i < RAGDOLL_COUNT; i++) {
        for (uint32_t part = 0; part < PART_COUNT; part++) {
            rigid_body::add_linear_velocity(*app.world, state.ragdolls[i].bodies[part], vec3f::make(0.0f, 5.0f, 0.0f));
            rigid_body::add_angular_velocity(*app.world, state.ragdolls[i].bodies[part], vec3f::make(0.0f, 0.0f, 20.0f));
        }
    }

    // what the dragged point is held to: a static body with no shape
    rigid_body::RigidBodySettings pointer;
    pointer.object_layer = state.layers.moving;
    pointer.motion_type = motion_type::MOTION_STATIC;
    pointer.position = {0.0f, 0.0f, -10000.0f};
    state.pointer = rigid_body::create(*app.world, pointer);

    camera::look_at(app.camera, {0.0f, 5.0f, 15.0f}, {0.0f, 0.0f, 0.0f});
}

static void release_drag(App &app) {
    if (!state.dragging) return;
    distance_constraint::remove(*app.world, state.drag);
    state.dragging = false;
}

/** crashcat grabs on pointer down and lets go on pointer up; the app's left drag orbits, so a click grabs and the next lets go */
static void grab(App &app, const Ray &pointer) {
    const cast_ray::Rayf ray = {vec3f::load(pointer.origin), vec3f::load(pointer.direction), RAY_LENGTH};
    cast_ray::reset(state.collector);
    query::cast_ray(*app.world, state.collector, cast_ray::default_settings(), ray, state.filter);
    const cast_ray::CastRayHit *hit = cast_ray::hit(state.collector);
    if (hit == nullptr) return;
    rigid_body::RigidBody *body = rigid_body::get(*app.world, hit->body_id_b);
    if (body == nullptr || body->motion_type != motion_type::MOTION_DYNAMIC) return;

    const Float3 point = vec::scale_and_add(pointer.origin, pointer.direction, RAY_LENGTH * hit->fraction);
    state.plane_point = point;
    state.plane_normal = app.camera.forward;
    state.pointer_position = point;
    rigid_body::set_position(*app.world, state.pointer, vec3f::load(point), false);
    state.drag = distance_constraint::create(*app.world, state.pointer, body,
                                             {.point_a = point, .point_b = point, .min_distance = 0.0f, .max_distance = 1.0f,
                                              .constraint_priority = 100});
    state.dragging = true;
    rigid_body::wake(*app.world, body);
}

static void step(App &app, float) {
    if (!state.dragging) return;
    const Ray &ray = app.pointer.ray;
    const float facing = vec::dot(ray.direction, state.plane_normal);
    if (fabsf(facing) < 1e-6f) return;
    const float t = vec::dot(vec::subtract(state.plane_point, ray.origin), state.plane_normal) / facing;
    if (t <= 0.0f) return;
    state.pointer_position = vec::scale_and_add(ray.origin, ray.direction, t);
    rigid_body::set_position(*app.world, state.pointer, vec3f::load(state.pointer_position), false);
}

static void update_frame(App &app, float) {
    if (app.click.clicked && !app.click.secondary) {
        if (state.dragging) release_drag(app);
        else grab(app, app.click.ray);
    }
    if (state.dragging) renderer::point(app::frame(app), state.pointer_position, 0.2f, {1.0f, 0.0f, 0.0f, 1.0f});
}

static void ui(App &) { ImGui::TextUnformatted(state.dragging ? "click to let go" : "click a ragdoll to grab it"); }

/** world::step, rejecting the pairs of bodies a constraint joins: the joined parts of a ragdoll pass through each other */
static void step_world(App &app, float delta_time) {
    world::World &world = *app.world;
    update::step_begin(world, delta_time);
    do {
        const uint32_t first_pair = world.events.pairs.length;
        update::find_pairs(world);
        for (uint32_t i = first_pair; i < world.events.pairs.length; i++) {
            const events::PairEvent &pair = world.events.pairs[i];
            const rigid_body::RigidBody *body_a = rigid_body::get(world, pair.body_id_a);
            const rigid_body::RigidBody *body_b = rigid_body::get(world, pair.body_id_b);
            if (body_a && body_b && rigid_body::bodies_share_constraint(body_a, body_b)) events::reject_pair(world.answers, i);
        }
        update::narrowphase(world);
    } while (world.events.woken > 0);
    update::solve(world);
    update::step_end(world);
}

static void shutdown(App &) {
    cast_ray::release(state.collector);
    filter::release(state.filter);
}

static const Example EXAMPLE_RAGDOLL = {"Ragdoll", init, step, update_frame, ui, shutdown, step_world};
static const examples::Registration REGISTERED(EXAMPLE_RAGDOLL);
