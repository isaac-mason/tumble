#include <math.h>

#include "app/app.h"
#include "app/examples.h"
#include "imgui.h"
#include "sokol_app.h"
#include "tumble/math/quatf.h"
#include "tumble/physics/constraints/distance_constraint.h"
#include "tumble/physics/query.h"
#include "tumble/physics/shapes/box.h"
#include "tumble/physics/shapes/capsule.h"
#include "tumble/physics/shapes/compound.h"
#include "tumble/physics/shapes/sphere.h"
#include "tumble/physics/shapes/triangle_mesh.h"
#include "tumble/physics/shapes/utils/triangle_mesh_builder.h"

using namespace tumble;

// after pmndrs/ecctrl: a dynamic capsule held above the ground by a spring, pushed about by impulses

static constexpr float PI = 3.14159265f;

struct Character {
    RigidBody *body;

    float capsule_radius;
    float capsule_half_height;
    float float_height;
    float float_spring_k;
    float float_damping_c;

    float max_walk_speed;
    float max_run_speed;
    float acceleration_time;
    float turn_speed;
    float turn_vel_multiplier;
    float air_control_factor;
    float reject_vel_mult;
    float drag_damping_c;
    float move_impulse_point_y;

    float max_slope_angle;
    float slope_ray_offset;
    float slope_up_extra_force;
    float slope_down_extra_force;

    bool enable_auto_balance;
    float balance_spring_k;
    float balance_damping_c;
    float balance_spring_on_y;
    float balance_damping_on_y;

    float jump_velocity;
    float jump_force_to_ground_mult;
    float slope_jump_mult;
    float sprint_jump_mult;

    float normal_gravity_scale;
    float falling_gravity_scale;
    float max_fall_speed;

    float ray_hit_forgiveness;

    bool is_on_ground;
    bool is_falling;
    bool can_jump;
    float slope_angle;
    Vec3f actual_slope_normal;
    float actual_slope_angle;
    /** INVALID_BODY_ID when off the ground */
    BodyId ground_body_id;
    SubShapeId ground_sub_shape_id;
    Vec3f ground_position;
    float ground_distance;

    bool is_on_moving_object;
    float mass_ratio;
    Vec3f moving_object_velocity;

    /** the facing the character turns towards, about y, apart from the body's own rotation */
    float indicator_y;
    bool rotated;

    Vec3f body_contact_force;
    Vec3f character_mass_force;

    Vec3f move_direction;
    bool want_to_run;
    bool want_to_jump;
};

static struct {
    app::Layers layers;
    Character character;
    Filter filter;
    CastRayCollector collector;
    RigidBody *spinning_platform;
    RigidBody *sliding_platform;
    RigidBody *diagonal_platform;
    RigidBody *windmill;
    int steps;
} state;

static Color hex(uint32_t rgb) { return {(float)((rgb >> 16) & 255) / 255.0f, (float)((rgb >> 8) & 255) / 255.0f, (float)(rgb & 255) / 255.0f, 1.0f}; }

static float clamp_unit(float value) { return fmaxf(-1.0f, fminf(1.0f, value)); }

static Float3 stored(Vec3f v) {
    Float3 out;
    vec3f::store(out, v);
    return out;
}

static bool not_this_body(const RigidBody *body, void *context) { return body != static_cast<const RigidBody *>(context); }

static void make_character(App &app, Vec3f position) {
    Character &c = state.character;
    c = {};
    c.capsule_radius = 0.5f;
    c.capsule_half_height = 0.5f;
    c.float_height = 0.3f;
    c.float_spring_k = 1.2f;
    c.float_damping_c = 0.08f;
    c.max_walk_speed = 8.0f;
    c.max_run_speed = 13.0f;
    c.acceleration_time = 8.0f;
    c.turn_speed = 40.0f;
    c.turn_vel_multiplier = 0.2f;
    c.air_control_factor = 0.2f;
    c.reject_vel_mult = 4.0f;
    c.drag_damping_c = 0.15f;
    c.move_impulse_point_y = 0.5f;
    c.max_slope_angle = 1.0f;
    c.slope_up_extra_force = 0.1f;
    c.slope_down_extra_force = 0.2f;
    c.enable_auto_balance = true;
    c.balance_spring_k = 0.2f;
    c.balance_damping_c = 0.1f;
    c.balance_spring_on_y = 0.3f;
    c.balance_damping_on_y = 0.05f;
    c.jump_velocity = 6.0f;
    c.jump_force_to_ground_mult = 5.0f;
    c.slope_jump_mult = 0.25f;
    c.sprint_jump_mult = 1.2f;
    c.normal_gravity_scale = 1.0f;
    c.falling_gravity_scale = 2.5f;
    c.max_fall_speed = 20.0f;
    c.ray_hit_forgiveness = 0.1f;
    c.slope_ray_offset = c.capsule_radius - 0.03f;
    c.actual_slope_normal = vec3f::axis_y();
    c.ground_body_id = body_id::INVALID_BODY_ID;
    c.mass_ratio = 1.0f;
    c.rotated = true;

    RigidBodySettings settings;
    settings.shape = app::keep(app, capsule::create({.half_height_of_cylinder = c.capsule_half_height, .radius = c.capsule_radius}));
    settings.motion_type = MotionType::DYNAMIC;
    settings.position = position;
    settings.object_layer = state.layers.moving;
    settings.friction = 0.5f;
    settings.restitution = 0.0f;
    settings.linear_damping = 0.0f;
    settings.angular_damping = 0.0f;
    settings.mass = 1.0f;
    settings.allowed_degrees_of_freedom = c.enable_auto_balance ? dof::DOF_ALL : dof::DOF_TRANSLATION_ONLY;
    settings.gravity_factor = c.normal_gravity_scale;
    c.body = rigid_body::create(*app.world, settings);
    app::set_color(app, c.body, hex(0x4488ff));
}

/** the closest hit straight down from origin, ignoring the character; its distance, or -1 */
static float cast_down(App &app, Vec3f origin, float length) {
    const Rayf ray = {origin, vec3f::make(0.0f, -1.0f, 0.0f), length};
    cast_ray::reset(state.collector);
    query::cast_ray(*app.world, state.collector, cast_ray::default_settings(), ray, state.filter);
    const CastRayHit *hit = cast_ray::hit(state.collector);
    return hit ? hit->fraction * length : -1.0f;
}

static void leave_ground(Character &c) {
    c.is_on_ground = false;
    c.can_jump = false;
    c.ground_body_id = body_id::INVALID_BODY_ID;
    c.ground_sub_shape_id = 0;
    c.ground_distance = 0.0f;
    c.actual_slope_angle = 0.0f;
}

static void update_ground_detection(App &app, Character &c) {
    const Vec3f origin = vec3f::subtract(c.body->position, vec3f::make(0.0f, c.capsule_half_height, 0.0f));
    const float distance = cast_down(app, origin, c.capsule_radius + 2.0f);
    const float floating_distance = c.capsule_radius + c.float_height;
    if (distance < 0.0f || distance >= floating_distance + c.ray_hit_forgiveness) {
        leave_ground(c);
        return;
    }
    const CastRayHit *hit = cast_ray::hit(state.collector);
    c.is_on_ground = true;
    c.ground_distance = distance;
    c.ground_position = vec3f::subtract(origin, vec3f::make(0.0f, distance, 0.0f));
    c.ground_body_id = hit->body_id_b;
    c.ground_sub_shape_id = hit->sub_shape_id;
}

static Vec3f body_forward(const Character &c) { return vec3f::transform_quat(vec3f::axis_z(), c.body->quaternion); }

static void update_slope_detection(App &app, Character &c) {
    if (!c.is_on_ground) {
        c.slope_angle = 0.0f;
        c.actual_slope_normal = vec3f::axis_y();
        c.can_jump = false;
        return;
    }
    const Vec3f ahead = vec3f::scale(body_forward(c), c.slope_ray_offset);
    const Vec3f origin = vec3f::add(c.body->position, vec3f::make(vec3f::x(ahead), -c.capsule_half_height, vec3f::z(ahead)));
    const float distance = cast_down(app, origin, c.capsule_radius + 3.0f);
    const float floating_distance = c.capsule_radius + c.float_height;
    if (distance < 0.0f || distance >= floating_distance + 0.5f) {
        c.slope_angle = 0.0f;
        c.actual_slope_normal = vec3f::axis_y();
        c.actual_slope_angle = 0.0f;
        c.can_jump = false;
        return;
    }
    // crashcat rounds it to two places
    c.slope_angle = roundf(atanf((c.ground_distance - distance) / c.slope_ray_offset) * 100.0f) / 100.0f;
    const RigidBody *ground = rigid_body::get(*app.world, c.ground_body_id);
    if (ground == nullptr) {
        c.can_jump = false;
        return;
    }
    c.actual_slope_normal = rigid_body::get_world_space_surface_normal(ground, c.ground_sub_shape_id, c.ground_position);
    c.actual_slope_angle = acosf(clamp_unit(vec3f::y(c.actual_slope_normal)));
    c.can_jump = c.actual_slope_angle < c.max_slope_angle;
}

static bool is_moving_body(const RigidBody *body) {
    return body->motion_type == MotionType::DYNAMIC || body->motion_type == MotionType::KINEMATIC;
}

static void apply_floating_force(App &app, Character &c) {
    if (!c.is_on_ground) return;
    RigidBody *ground = rigid_body::get(*app.world, c.ground_body_id);
    if (ground == nullptr) return;
    const float displacement = c.capsule_radius + c.float_height - c.ground_distance;
    // a spring and damper, applied as this step's impulse
    const float force = c.float_spring_k * displacement - c.float_damping_c * vec3f::y(c.body->motion_properties.linear_velocity);
    rigid_body::add_impulse(*app.world, c.body, vec3f::make(0.0f, force, 0.0f));
    c.character_mass_force = vec3f::make(0.0f, force > 0.0f ? -force : 0.0f, 0.0f);
    // and the ground pushed back
    if (is_moving_body(ground)) {
        rigid_body::add_impulse_at_position(*app.world, ground, c.character_mass_force, c.ground_position);
    }
}

static void apply_movement_force(App &app, Character &c) {
    const bool run = c.want_to_run;
    const float max_velocity = run ? c.max_run_speed : c.max_walk_speed;

    Vec3f moving_direction = vec3f::axis_z();
    if (c.actual_slope_angle < c.max_slope_angle && fabsf(c.slope_angle) > 0.2f && fabsf(c.slope_angle) < c.max_slope_angle) {
        moving_direction = vec3f::make(0.0f, sinf(c.slope_angle), cosf(c.slope_angle));
    } else if (c.actual_slope_angle >= c.max_slope_angle) {
        // too steep: only a little forward
        const float rise = sinf(c.slope_angle);
        moving_direction = vec3f::make(0.0f, rise > 0.0f ? 0.0f : rise, rise > 0.0f ? 0.1f : 1.0f);
    }
    moving_direction = vec3f::transform_quat(moving_direction, quatf::set_axis_angle(vec3f::axis_y(), c.indicator_y));

    const Vec3f velocity = c.body->motion_properties.linear_velocity;
    const float object_along = vec3f::dot(c.moving_object_velocity, moving_direction);
    const Vec3f object_velocity_along = vec3f::scale(moving_direction, object_along);
    const float angle_to_object =
        acosf(clamp_unit(object_along / (vec3f::length(c.moving_object_velocity) * vec3f::length(moving_direction) + 0.0001f)));

    // the velocity across the way it wants to go, cancelled on the ground
    const Vec3f flat_velocity = vec3f::with_y(velocity, 0.0f);
    const Vec3f flat_direction = vec3f::with_y(moving_direction, 0.0f);
    const Vec3f reject = vec3f::subtract(flat_velocity, vec3f::scale(flat_direction, vec3f::dot(flat_velocity, flat_direction)));
    const float reject_mult = c.is_on_moving_object ? 0.0f : c.reject_vel_mult;
    const float object_sin = sinf(angle_to_object);
    const Vec3f wanted = vec3f::multiply(moving_direction, vec3f::add(vec3f::splat(max_velocity), object_velocity_along));
    const Vec3f current = vec3f::add(vec3f::subtract(velocity, vec3f::scale(c.moving_object_velocity, object_sin)), vec3f::scale(reject, reject_mult));
    const Vec3f acceleration = vec3f::with_y(vec3f::scale(vec3f::subtract(wanted, current), 1.0f / c.acceleration_time), 0.0f);

    const float control_mult = c.rotated ? 1.0f : c.turn_vel_multiplier;
    const float air_mult = c.can_jump ? 1.0f : c.air_control_factor;
    float slope_impulse_y = 0.0f;
    if (c.slope_angle != 0.0f) {
        const float rise = vec3f::y(moving_direction);
        slope_impulse_y = rise * (rise > 0.0f ? c.slope_up_extra_force : c.slope_down_extra_force) * (run ? c.max_run_speed / c.max_walk_speed : 1.0f);
    }
    // a mass of 1: the acceleration is the force
    const Vec3f impulse = vec3f::with_y(vec3f::scale(acceleration, control_mult * air_mult), slope_impulse_y);
    const Vec3f point = vec3f::add(c.body->position, vec3f::make(0.0f, c.move_impulse_point_y, 0.0f));
    rigid_body::add_impulse_at_position(*app.world, c.body, impulse, point);

    // the platform stood on is dragged back
    if (c.is_on_moving_object) {
        RigidBody *ground = rigid_body::get(*app.world, c.ground_body_id);
        if (ground != nullptr && ground->motion_type == MotionType::DYNAMIC) {
            const float share = fminf(1.0f, 1.0f / c.mass_ratio);
            rigid_body::add_impulse_at_position(*app.world, ground, vec3f::scale(vec3f::with_y(impulse, 0.0f), -share), c.ground_position);
        }
    }
}

static void apply_auto_balance_torque(App &app, Character &c) {
    if (!c.enable_auto_balance) return;
    const Vec3f up = vec3f::transform_quat(vec3f::axis_y(), c.body->quaternion);
    const Vec3f forward = body_forward(c);
    const Vec3f desired_up = vec3f::axis_y();
    const Vec3f desired_forward = vec3f::length(c.move_direction) > 0.001f ? vec3f::normalize(c.move_direction) : forward;

    const Vec3f balance_on_x = vec3f::with_x(up, 0.0f);
    const Vec3f balance_on_z = vec3f::with_z(up, 0.0f);
    const Vec3f facing_on_y = vec3f::with_y(forward, 0.0f);
    const float sign_x = vec3f::x(vec3f::cross(desired_up, balance_on_x)) < 0.0f ? 1.0f : -1.0f;
    const float sign_y = vec3f::y(vec3f::cross(desired_forward, facing_on_y)) < 0.0f ? 1.0f : -1.0f;
    const float sign_z = vec3f::z(vec3f::cross(desired_up, balance_on_z)) < 0.0f ? 1.0f : -1.0f;
    const float angle_x = acosf(clamp_unit(vec3f::dot(balance_on_x, desired_up)));
    const float angle_y = acosf(clamp_unit(vec3f::dot(facing_on_y, desired_forward)));
    const float angle_z = acosf(clamp_unit(vec3f::dot(balance_on_z, desired_up)));

    const Vec3f spring =
        vec3f::make(sign_x * c.balance_spring_k * angle_x, sign_y * c.balance_spring_on_y * angle_y, sign_z * c.balance_spring_k * angle_z);
    const Vec3f damping =
        vec3f::multiply(c.body->motion_properties.angular_velocity, vec3f::make(c.balance_damping_c, c.balance_damping_on_y, c.balance_damping_c));
    rigid_body::add_angular_impulse(*app.world, c.body, vec3f::subtract(spring, damping));
}

static void stand_still(Character &c) {
    c.mass_ratio = 1.0f;
    c.is_on_moving_object = false;
    c.moving_object_velocity = vec3f::zero();
    c.body_contact_force = vec3f::zero();
}

static void update_moving_platform(App &app, Character &c, bool moving) {
    if (!c.can_jump) {
        stand_still(c);
        return;
    }
    RigidBody *ground = rigid_body::get(*app.world, c.ground_body_id);
    if (ground == nullptr || !is_moving_body(ground)) {
        stand_still(c);
        return;
    }
    c.is_on_moving_object = true;
    c.mass_ratio = c.body->mass_properties.mass / ground->mass_properties.mass;

    // the platform's velocity where the character stands
    const Vec3f offset = vec3f::subtract(c.body->position, ground->position);
    const Vec3f spin = vec3f::cross(ground->motion_properties.angular_velocity, offset);
    const float share = fminf(1.0f, 1.0f / c.mass_ratio);
    c.moving_object_velocity = vec3f::scale(vec3f::add(ground->motion_properties.linear_velocity, vec3f::with_y(spin, 0.0f)), share);
    const float difference = vec3f::length(vec3f::subtract(c.moving_object_velocity, c.body->motion_properties.linear_velocity));
    if (difference > 30.0f) c.moving_object_velocity = vec3f::scale(c.moving_object_velocity, 1.0f / difference);

    // standing still on a dynamic platform: its drag comes from the contact force (moving, apply_movement_force drags it)
    if (ground->motion_type == MotionType::DYNAMIC && !moving && vec3f::squared_length(c.move_direction) < 0.001f) {
        rigid_body::add_impulse_at_position(*app.world, ground, vec3f::scale(c.body_contact_force, -share), c.ground_position);
        c.body_contact_force = vec3f::zero();
    }
}

static void update_gravity_scale(Character &c) {
    const float vertical = vec3f::y(c.body->motion_properties.linear_velocity);
    c.is_falling = vertical < 0.0f && !c.can_jump;
    float &gravity_factor = c.body->motion_properties.gravity_factor;
    if (vertical < -c.max_fall_speed) gravity_factor = 0.0f;
    else if (c.is_falling) gravity_factor = c.falling_gravity_scale;
    else gravity_factor = c.normal_gravity_scale;
}

static void handle_jump(App &app, Character &c) {
    if (!c.want_to_jump || !c.can_jump) return;
    const float jump = c.want_to_run ? c.sprint_jump_mult * c.jump_velocity : c.jump_velocity;
    const Vec3f jump_velocity = vec3f::with_y(c.body->motion_properties.linear_velocity, jump);
    // a little more along the slope's normal
    Vec3f along_normal = vec3f::make(0.0f, jump * c.slope_jump_mult, 0.0f);
    const float normal_length = vec3f::length(c.actual_slope_normal);
    if (normal_length > 0.0001f) {
        along_normal = vec3f::scale(c.actual_slope_normal, vec3f::dot(along_normal, c.actual_slope_normal) / (normal_length * normal_length));
    }
    rigid_body::set_linear_velocity(*app.world, c.body, vec3f::add(jump_velocity, along_normal));

    RigidBody *ground = rigid_body::get(*app.world, c.ground_body_id);
    if (ground != nullptr && is_moving_body(ground)) {
        const Vec3f push = vec3f::with_y(c.character_mass_force, vec3f::y(c.character_mass_force) * c.jump_force_to_ground_mult);
        rigid_body::add_impulse_at_position(*app.world, ground, push, c.ground_position);
    }
}

static void update_rotation(Character &c, float delta_time) {
    if (vec3f::length(c.move_direction) <= 0.001f) {
        c.rotated = true;
        return;
    }
    float delta = atan2f(vec3f::x(c.move_direction), vec3f::z(c.move_direction)) - c.indicator_y;
    while (delta > PI) delta -= 2.0f * PI;
    while (delta < -PI) delta += 2.0f * PI;
    const float max_step = c.turn_speed * delta_time;
    c.indicator_y += fmaxf(-max_step, fminf(max_step, delta));
    c.rotated = fabsf(delta) < 0.01f;
}

static void apply_drag_force(App &app, Character &c) {
    if (!c.can_jump) return;
    // towards rest, or towards the platform's velocity
    const Vec3f target = c.is_on_moving_object ? c.moving_object_velocity : vec3f::zero();
    const Vec3f drag = vec3f::scale(vec3f::subtract(target, c.body->motion_properties.linear_velocity), c.drag_damping_c);
    rigid_body::add_impulse(*app.world, c.body, vec3f::with_y(drag, 0.0f));
}

static void update_character(App &app, Character &c, Vec3f direction, bool run, bool jump, float delta_time) {
    c.want_to_run = run;
    c.want_to_jump = jump;
    const Vec3f flat = vec3f::with_y(direction, 0.0f);
    c.move_direction = vec3f::length(flat) > 0.001f ? vec3f::normalize(flat) : flat;
    const bool moving = vec3f::length(c.move_direction) > 0.001f;

    update_rotation(c, delta_time);
    update_ground_detection(app, c);
    update_slope_detection(app, c);
    update_moving_platform(app, c, moving);
    if (moving) apply_movement_force(app, c);
    else apply_drag_force(app, c);
    apply_floating_force(app, c);
    apply_auto_balance_torque(app, c);
    handle_jump(app, c);
    update_gravity_scale(c);
}

static RigidBody *add_static(App &app, Shape *shape, Vec3f position, Quatf quaternion = quatf::identity()) {
    RigidBodySettings settings;
    settings.shape = shape;
    settings.position = position;
    settings.quaternion = quaternion;
    settings.motion_type = MotionType::STATIC;
    settings.object_layer = state.layers.not_moving;
    return rigid_body::create(*app.world, settings);
}

static RigidBody *add_kinematic(App &app, Shape *shape, Vec3f position) {
    RigidBodySettings settings;
    settings.shape = shape;
    settings.position = position;
    settings.motion_type = MotionType::KINEMATIC;
    settings.object_layer = state.layers.moving;
    return rigid_body::create(*app.world, settings);
}

static Shape *make_box(App &app, Vec3f half_extents) { return app::keep(app, box::create({.half_extents = half_extents})); }

static void make_terrain(App &app) {
    constexpr int GRID = 16;
    constexpr float CELL = 2.0f;
    constexpr float HEIGHT = 2.0f;
    float positions[(GRID + 1) * (GRID + 1) * 3];
    uint32_t indices[GRID * GRID * 6];
    int at = 0;
    for (int iz = 0; iz <= GRID; iz++) {
        for (int ix = 0; ix <= GRID; ix++) {
            const float x = (float)(ix - GRID / 2) * CELL;
            const float z = (float)(iz - GRID / 2) * CELL;
            positions[at++] = x;
            positions[at++] = sinf(x * 0.3f) * cosf(z * 0.3f) * HEIGHT * 0.5f + sinf(x * 0.7f + 1.0f) * sinf(z * 0.7f) * HEIGHT * 0.3f +
                              cosf((x + z) * 0.15f) * HEIGHT * 0.2f;
            positions[at++] = z;
        }
    }
    at = 0;
    for (int iz = 0; iz < GRID; iz++) {
        for (int ix = 0; ix < GRID; ix++) {
            const uint32_t bottom_left = (uint32_t)(iz * (GRID + 1) + ix);
            const uint32_t bottom_right = bottom_left + 1;
            const uint32_t top_left = bottom_left + GRID + 1;
            const uint32_t top_right = top_left + 1;
            const uint32_t quad[] = {bottom_left, top_left, bottom_right, bottom_right, top_left, top_right};
            for (uint32_t index : quad) indices[at++] = index;
        }
    }
    add_static(app,
               app::keep(app, triangle_mesh::create({.positions = positions,
                                                     .num_positions = (GRID + 1) * (GRID + 1),
                                                     .indices = indices,
                                                     .num_indices = GRID * GRID * 6})),
               vec3f::make(-30.0f, 2.0f, 30.0f));
}

static void make_level(App &app) {
    add_static(app, make_box(app, vec3f::make(50.0f, 0.5f, 50.0f)), vec3f::make(0.0f, -0.5f, 0.0f));
    Shape *wall = make_box(app, vec3f::make(0.5f, 2.0f, 45.0f));
    add_static(app, wall, vec3f::make(-45.0f, 1.0f, 0.0f));
    add_static(app, wall, vec3f::make(45.0f, 1.0f, 0.0f));
    // the conveyor belt's box, which crashcat leaves still
    add_static(app, make_box(app, vec3f::make(10.0f, 0.25f, 2.0f)), vec3f::make(0.0f, 0.0f, -10.0f));

    // five flights of stairs, each with steps taller than the last
    for (int j = 0; j < 5; j++) {
        const float step_height = 0.3f + 0.1f * (float)j;
        Shape *stair = make_box(app, vec3f::make(2.0f, step_height / 2.0f, 2.0f));
        for (int i = 1; i < 10; i++) {
            add_static(app, stair, vec3f::make(15.0f + 5.0f * (float)j, (float)i * step_height - 0.5f + step_height / 2.0f, -20.0f - (float)i * 3.0f));
        }
    }

    // slopes from 70 degrees down to 25
    Shape *slope = make_box(app, vec3f::make(2.5f, 0.6f, 8.0f));
    for (int i = 0; i < 10; i++) {
        const float angle = (70.0f - (float)i * 5.0f) * PI / 180.0f;
        add_static(app, slope, vec3f::make(-40.0f + 5.0f * (float)i, 2.0f, -25.0f), quatf::set_axis_angle(vec3f::axis_x(), angle));
    }

    RigidBodySettings block;
    block.shape = make_box(app, vec3f::splat(0.75f));
    block.position = vec3f::make(-10.0f, 5.0f, 10.0f);
    block.motion_type = MotionType::DYNAMIC;
    block.object_layer = state.layers.moving;
    block.friction = 0.1f;
    block.mass = 1.0f;
    rigid_body::create(*app.world, block);
    RigidBodySettings ball = block;
    ball.shape = app::keep(app, sphere::create({.radius = 0.75f}));
    ball.position = vec3f::make(-8.0f, 2.0f, 10.0f);
    rigid_body::create(*app.world, ball);

    state.spinning_platform = add_kinematic(app, make_box(app, vec3f::make(4.0f, 0.2f, 4.0f)), vec3f::make(0.0f, 2.0f, 15.0f));
    Shape *platform = make_box(app, vec3f::make(3.0f, 0.2f, 3.0f));
    state.sliding_platform = add_kinematic(app, platform, vec3f::make(20.0f, 2.0f, -5.0f));
    state.diagonal_platform = add_kinematic(app, platform, vec3f::make(20.0f, 3.0f, 5.0f));

    // a hub and four blades about z, sweeping through where the character walks
    Shape *hub = make_box(app, vec3f::make(0.3f, 0.3f, 0.5f));
    Shape *vertical_blade = make_box(app, vec3f::make(0.5f, 3.0f, 0.5f));
    Shape *horizontal_blade = make_box(app, vec3f::make(3.0f, 0.5f, 0.5f));
    const Quatf identity = quatf::identity();
    const CompoundShapeSettings::Child blades[] = {
        {.position = vec3f::make(0.0f, 0.0f, 0.0f), .quaternion = identity, .shape = hub},
        {.position = vec3f::make(0.0f, 3.3f, 0.0f), .quaternion = identity, .shape = vertical_blade},
        {.position = vec3f::make(0.0f, -3.3f, 0.0f), .quaternion = identity, .shape = vertical_blade},
        {.position = vec3f::make(3.3f, 0.0f, 0.0f), .quaternion = identity, .shape = horizontal_blade},
        {.position = vec3f::make(-3.3f, 0.0f, 0.0f), .quaternion = identity, .shape = horizontal_blade},
    };
    state.windmill = add_kinematic(app, app::keep(app, compound::create({.children = blades, .num_children = 5})), vec3f::make(-15.0f, 6.5f, 0.0f));
    add_static(app, make_box(app, vec3f::make(0.25f, 6.5f, 0.25f)), vec3f::make(-15.0f, 0.0f, 0.0f));

    make_terrain(app);

    // a bumpy floor of half sunk spheres
    const Float3 bumps[] = {
        {25.0f, 28.0f, 0.8f},  {29.0f, 32.0f, 0.9f},  {23.0f, 34.0f, 0.7f},  {31.0f, 28.0f, 0.85f}, {27.0f, 30.0f, 1.0f},  {22.0f, 30.0f, 0.5f},
        {32.0f, 30.0f, 0.55f}, {25.0f, 26.0f, 0.45f}, {29.0f, 34.0f, 0.6f},  {21.0f, 32.0f, 0.5f},  {33.0f, 32.0f, 0.48f}, {27.0f, 35.0f, 0.52f},
        {27.0f, 25.0f, 0.55f}, {24.0f, 31.0f, 0.35f}, {30.0f, 29.0f, 0.3f},  {26.0f, 33.0f, 0.38f}, {28.0f, 27.0f, 0.32f}, {22.0f, 28.0f, 0.28f},
        {32.0f, 34.0f, 0.34f}, {20.0f, 30.0f, 0.4f},  {34.0f, 30.0f, 0.36f}, {23.0f, 36.0f, 0.33f}, {31.0f, 26.0f, 0.31f},
    };
    for (const Float3 &bump : bumps) {
        add_static(app, app::keep(app, sphere::create({.radius = bump.z})), vec3f::make(bump.x, bump.z * -0.5f, bump.y));
    }

    // boxes each twice the size of the last
    float x = 5.0f;
    for (int i = 0; i < 5; i++) {
        const float half_extent = 0.25f * (float)(1 << i);
        add_static(app, make_box(app, vec3f::splat(half_extent)), vec3f::make(x, half_extent, 30.0f));
        x += half_extent * 3.0f;
    }

    // a platform hung from four springs
    const Vec3f centre = vec3f::make(-10.0f, 5.0f, 35.0f);
    const Vec3f half_extents = vec3f::make(3.0f, 0.3f, 3.0f);
    RigidBodySettings hung;
    hung.shape = make_box(app, half_extents);
    hung.position = centre;
    hung.motion_type = MotionType::DYNAMIC;
    hung.object_layer = state.layers.moving;
    hung.linear_damping = 0.5f;
    hung.angular_damping = 0.5f;
    RigidBody *suspended = rigid_body::create(*app.world, hung);
    const float spring_length = 4.0f;
    for (int i = 0; i < 4; i++) {
        // a top corner of the platform, and the anchor above it
        const Vec3f corner = vec3f::multiply(half_extents, vec3f::make((i & 1) ? 1.0f : -1.0f, 1.0f, (i & 2) ? 1.0f : -1.0f));
        const Vec3f point_b = vec3f::add(centre, corner);
        const Vec3f anchor_position = vec3f::with_y(point_b, 12.0f);
        RigidBody *anchor = add_static(app, nullptr, anchor_position);
        distance_constraint::create(*app.world, anchor, suspended,
                                    {.point_a = anchor_position,
                                     .point_b = point_b,
                                     .min_distance = spring_length * 0.5f,
                                     .max_distance = spring_length * 2.0f,
                                     .spring_settings = {.mode = SpringMode::FREQUENCY_AND_DAMPING, .frequency_or_stiffness = 0.8f, .damping = 0.1f}});
    }
}

static void init(App &app) {
    state.steps = 0;
    app.world = app::create_world(state.layers);
    state.filter = filter::create(app.world->settings.layers);
    state.collector = cast_ray::create_collector(CollectorKind::CLOSEST);
    make_level(app);
    make_character(app, vec3f::make(0.0f, 5.0f, 0.0f));
    state.filter.body_filter = not_this_body;
    state.filter.body_filter_context = state.character.body;
    camera::look_at(app.camera, {0.0f, 5.0f, 10.0f}, {0.0f, 5.0f, 0.0f});
}

static void move_platforms(App &app, float delta_time) {
    const float time = (float)state.steps * delta_time;

    RigidBody *spinning = state.spinning_platform;
    const Quatf spun = quatf::multiply(spinning->quaternion, quatf::set_axis_angle(vec3f::axis_y(), 1.0f * delta_time));
    rigid_body::move_kinematic(*app.world, spinning, spinning->position, spun, delta_time);

    RigidBody *sliding = state.sliding_platform;
    const float slide = sinf(time * 0.15f * PI * 2.0f) * 8.0f;
    rigid_body::move_kinematic(*app.world, sliding, vec3f::make(20.0f + slide, 2.0f, -5.0f), sliding->quaternion, delta_time);

    RigidBody *diagonal = state.diagonal_platform;
    const float phase = time * 0.12f * PI * 2.0f;
    rigid_body::move_kinematic(*app.world, diagonal, vec3f::make(20.0f + sinf(phase) * 6.0f, 3.0f + sinf(phase * 2.0f) * 2.0f, 5.0f),
                               diagonal->quaternion, delta_time);

    RigidBody *windmill = state.windmill;
    const Quatf turned = quatf::multiply(windmill->quaternion, quatf::set_axis_angle(vec3f::axis_z(), 1.0f * delta_time));
    rigid_body::move_kinematic(*app.world, windmill, windmill->position, turned, delta_time);
}

static void step(App &app, float delta_time) {
    // w a s d move relative to the camera, space jumps, shift runs
    const bool *keys = app.keys;
    const float forward = keys[SAPP_KEYCODE_W] ? 1.0f : keys[SAPP_KEYCODE_S] ? -1.0f : 0.0f;
    const float right = keys[SAPP_KEYCODE_D] ? 1.0f : keys[SAPP_KEYCODE_A] ? -1.0f : 0.0f;
    const Vec3f camera_forward = vec3f::normalize(vec3f::make(app.camera.forward.x, 0.0f, app.camera.forward.z));
    const Vec3f camera_right = vec3f::normalize(vec3f::make(app.camera.side.x, 0.0f, app.camera.side.z));
    const Vec3f direction = vec3f::add(vec3f::scale(camera_right, right), vec3f::scale(camera_forward, forward));
    const bool run = keys[SAPP_KEYCODE_LEFT_SHIFT] || keys[SAPP_KEYCODE_RIGHT_SHIFT];

    update_character(app, state.character, direction, run, keys[SAPP_KEYCODE_SPACE], delta_time);
    move_platforms(app, delta_time);
    state.steps++;
}

static void update(App &app, float) {
    const Character &c = state.character;
    // the camera follows the character as drawn, keeping its orbit
    app.camera.target = app::pose(app, *c.body).position;

    DrawList &frame = app::frame(app);
    const Vec3f feet = vec3f::subtract(c.body->position, vec3f::make(0.0f, c.capsule_half_height, 0.0f));
    const float ground_length = c.ground_distance > 0.0f ? c.ground_distance : 1.0f;
    const Color ground_color = c.is_on_ground ? hex(0x00ff00) : hex(0xff0000);
    renderer::arrow(frame, stored(feet), stored(vec3f::subtract(feet, vec3f::make(0.0f, ground_length, 0.0f))), ground_color);
    const Vec3f slope_origin = vec3f::add(feet, vec3f::with_y(vec3f::scale(body_forward(c), c.slope_ray_offset), 0.0f));
    renderer::arrow(frame, stored(slope_origin), stored(vec3f::subtract(slope_origin, vec3f::axis_y())), hex(0x00ff00));
    if (c.is_on_ground) {
        renderer::arrow(frame, stored(c.ground_position), stored(vec3f::add(c.ground_position, c.actual_slope_normal)), hex(0x0000ff));
    }
}

static void set_auto_balance(App &app, Character &c) {
    RigidBody *body = c.body;
    motion_properties::set_mass_properties(body->motion_properties, c.enable_auto_balance ? dof::DOF_ALL : dof::DOF_TRANSLATION_ONLY,
                                           body->mass_properties);
    if (c.enable_auto_balance) return;
    // upright and not turning
    rigid_body::set_angular_velocity(*app.world, body, vec3f::zero());
    rigid_body::set_quaternion(*app.world, body, quatf::identity(), true);
    app::snap(app, *body);
}

static void ui(App &app) {
    Character &c = state.character;
    ImGui::TextUnformatted("wasd move, space jumps, shift runs");
    ImGui::Text("%s, slope %.2f", c.can_jump ? "on the ground" : c.is_on_ground ? "on steep ground" : "in the air", c.slope_angle);
    if (ImGui::CollapsingHeader("movement")) {
        ImGui::SliderFloat("walk speed", &c.max_walk_speed, 0.0f, 10.0f);
        ImGui::SliderFloat("run speed", &c.max_run_speed, 0.0f, 20.0f);
        ImGui::SliderFloat("acceleration time", &c.acceleration_time, 1.0f, 20.0f);
        ImGui::SliderFloat("turn speed", &c.turn_speed, 5.0f, 50.0f);
        ImGui::SliderFloat("turn penalty", &c.turn_vel_multiplier, 0.0f, 1.0f);
        ImGui::SliderFloat("air control", &c.air_control_factor, 0.0f, 1.0f);
    }
    if (ImGui::CollapsingHeader("floating")) {
        ImGui::SliderFloat("float height", &c.float_height, 0.0f, 2.0f);
        ImGui::SliderFloat("spring k##float", &c.float_spring_k, 0.0f, 5.0f);
        ImGui::SliderFloat("damping c##float", &c.float_damping_c, 0.0f, 0.5f);
        ImGui::SliderFloat("drag damping", &c.drag_damping_c, 0.0f, 1.0f);
    }
    if (ImGui::CollapsingHeader("slope")) {
        ImGui::SliderFloat("max slope angle", &c.max_slope_angle, 0.0f, PI / 2.0f);
        ImGui::SliderFloat("uphill force", &c.slope_up_extra_force, 0.0f, 1.0f);
        ImGui::SliderFloat("downhill force", &c.slope_down_extra_force, 0.0f, 1.0f);
        ImGui::SliderFloat("reject vel mult", &c.reject_vel_mult, 0.0f, 10.0f);
    }
    if (ImGui::CollapsingHeader("auto balance")) {
        if (ImGui::Checkbox("enable", &c.enable_auto_balance)) set_auto_balance(app, c);
        ImGui::SliderFloat("spring k##balance", &c.balance_spring_k, 0.0f, 2.0f);
        ImGui::SliderFloat("damping c##balance", &c.balance_damping_c, 0.0f, 0.2f);
        ImGui::SliderFloat("spring k y", &c.balance_spring_on_y, 0.0f, 2.0f);
        ImGui::SliderFloat("damping c y", &c.balance_damping_on_y, 0.0f, 0.2f);
    }
    if (ImGui::CollapsingHeader("jump")) {
        ImGui::SliderFloat("jump velocity", &c.jump_velocity, 0.0f, 10.0f);
        ImGui::SliderFloat("force to ground", &c.jump_force_to_ground_mult, 0.0f, 20.0f);
        ImGui::SliderFloat("slope jump mult", &c.slope_jump_mult, 0.0f, 1.0f);
        ImGui::SliderFloat("sprint jump mult", &c.sprint_jump_mult, 1.0f, 2.0f);
    }
    if (ImGui::CollapsingHeader("gravity")) {
        if (ImGui::SliderFloat("normal scale", &c.normal_gravity_scale, 0.0f, 3.0f) && !c.is_falling) {
            c.body->motion_properties.gravity_factor = c.normal_gravity_scale;
        }
        if (ImGui::SliderFloat("falling scale", &c.falling_gravity_scale, 0.0f, 5.0f) && c.is_falling) {
            c.body->motion_properties.gravity_factor = c.falling_gravity_scale;
        }
        ImGui::SliderFloat("max fall speed", &c.max_fall_speed, 0.0f, 50.0f);
    }
}

static void shutdown(App &) {
    state.collector = {};
    state.filter = {};
}

static const Example EXAMPLE_FLOATING_CHARACTER_CONTROLLER = {"Floating Character Controller", init, step, update, ui, shutdown};
static const examples::Registration REGISTERED(EXAMPLE_FLOATING_CHARACTER_CONTROLLER);
