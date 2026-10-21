#include <math.h>

#include "app/app.h"
#include "app/examples.h"
#include "app/vec.h"
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
    rigid_body::RigidBody *body;

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
    Float3 actual_slope_normal;
    float actual_slope_angle;
    /** INVALID_BODY_ID when off the ground */
    body_id::BodyId ground_body_id;
    sub_shape::SubShapeId ground_sub_shape_id;
    Float3 ground_position;
    float ground_distance;

    bool is_on_moving_object;
    float mass_ratio;
    Float3 moving_object_velocity;

    /** the facing the character turns towards, about y, apart from the body's own rotation */
    float indicator_y;
    bool rotated;

    Float3 body_contact_force;
    Float3 character_mass_force;

    Float3 move_direction;
    bool want_to_run;
    bool want_to_jump;
};

static struct {
    app::Layers layers;
    Character character;
    filter::Filter filter;
    cast_ray::CastRayCollector collector;
    rigid_body::RigidBody *spinning_platform;
    rigid_body::RigidBody *sliding_platform;
    rigid_body::RigidBody *diagonal_platform;
    rigid_body::RigidBody *windmill;
    int steps;
} state;

static Color hex(uint32_t rgb) { return {(float)((rgb >> 16) & 255) / 255.0f, (float)((rgb >> 8) & 255) / 255.0f, (float)(rgb & 255) / 255.0f, 1.0f}; }

static float clamp_unit(float value) { return fmaxf(-1.0f, fminf(1.0f, value)); }

static bool not_this_body(const rigid_body::RigidBody *body, void *context) { return body != static_cast<const rigid_body::RigidBody *>(context); }

static void make_character(App &app, Float3 position) {
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
    c.actual_slope_normal = {0.0f, 1.0f, 0.0f};
    c.ground_body_id = body_id::INVALID_BODY_ID;
    c.mass_ratio = 1.0f;
    c.rotated = true;

    rigid_body::RigidBodySettings settings;
    settings.shape = app::keep(app, capsule::create({.half_height_of_cylinder = c.capsule_half_height, .radius = c.capsule_radius}));
    settings.motion_type = motion_type::MOTION_DYNAMIC;
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
static float cast_down(App &app, Float3 origin, float length) {
    const cast_ray::Rayf ray = {vec3f::load(origin), vec3f::make(0.0f, -1.0f, 0.0f), length};
    cast_ray::reset(state.collector);
    query::cast_ray(*app.world, state.collector, cast_ray::default_settings(), ray, state.filter);
    const cast_ray::CastRayHit *hit = cast_ray::hit(state.collector);
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
    const Float3 origin = {c.body->position.x, c.body->position.y - c.capsule_half_height, c.body->position.z};
    const float distance = cast_down(app, origin, c.capsule_radius + 2.0f);
    const float floating_distance = c.capsule_radius + c.float_height;
    if (distance < 0.0f || distance >= floating_distance + c.ray_hit_forgiveness) {
        leave_ground(c);
        return;
    }
    const cast_ray::CastRayHit *hit = cast_ray::hit(state.collector);
    c.is_on_ground = true;
    c.ground_distance = distance;
    c.ground_position = {origin.x, origin.y - distance, origin.z};
    c.ground_body_id = hit->body_id_b;
    c.ground_sub_shape_id = hit->sub_shape_id;
}

static Float3 body_forward(const Character &c) { return vec::rotate(c.body->quaternion, {0.0f, 0.0f, 1.0f}); }

static void update_slope_detection(App &app, Character &c) {
    if (!c.is_on_ground) {
        c.slope_angle = 0.0f;
        c.actual_slope_normal = {0.0f, 1.0f, 0.0f};
        c.can_jump = false;
        return;
    }
    const Float3 forward = body_forward(c);
    const Float3 origin = {c.body->position.x + forward.x * c.slope_ray_offset, c.body->position.y - c.capsule_half_height,
                           c.body->position.z + forward.z * c.slope_ray_offset};
    const float distance = cast_down(app, origin, c.capsule_radius + 3.0f);
    const float floating_distance = c.capsule_radius + c.float_height;
    if (distance < 0.0f || distance >= floating_distance + 0.5f) {
        c.slope_angle = 0.0f;
        c.actual_slope_normal = {0.0f, 1.0f, 0.0f};
        c.actual_slope_angle = 0.0f;
        c.can_jump = false;
        return;
    }
    // crashcat rounds it to two places
    c.slope_angle = roundf(atanf((c.ground_distance - distance) / c.slope_ray_offset) * 100.0f) / 100.0f;
    const rigid_body::RigidBody *ground = rigid_body::get(*app.world, c.ground_body_id);
    if (ground == nullptr) {
        c.can_jump = false;
        return;
    }
    rigid_body::get_surface_normal(c.actual_slope_normal, ground, c.ground_position, c.ground_sub_shape_id);
    c.actual_slope_angle = acosf(clamp_unit(c.actual_slope_normal.y));
    c.can_jump = c.actual_slope_angle < c.max_slope_angle;
}

static bool is_moving_body(const rigid_body::RigidBody *body) {
    return body->motion_type == motion_type::MOTION_DYNAMIC || body->motion_type == motion_type::MOTION_KINEMATIC;
}

static void apply_floating_force(App &app, Character &c) {
    if (!c.is_on_ground) return;
    rigid_body::RigidBody *ground = rigid_body::get(*app.world, c.ground_body_id);
    if (ground == nullptr) return;
    const float displacement = c.capsule_radius + c.float_height - c.ground_distance;
    // a spring and damper, applied as this step's impulse
    const float force = c.float_spring_k * displacement - c.float_damping_c * c.body->motion_properties.linear_velocity.y;
    rigid_body::add_impulse(*app.world, c.body, vec3f::make(0.0f, force, 0.0f));
    c.character_mass_force = {0.0f, force > 0.0f ? -force : 0.0f, 0.0f};
    // and the ground pushed back
    if (is_moving_body(ground)) {
        rigid_body::add_impulse_at_position(*app.world, ground, vec3f::load(c.character_mass_force), vec3f::load(c.ground_position));
    }
}

static void apply_movement_force(App &app, Character &c) {
    const bool run = c.want_to_run;
    const float max_velocity = run ? c.max_run_speed : c.max_walk_speed;

    Float3 moving_direction = {0.0f, 0.0f, 1.0f};
    if (c.actual_slope_angle < c.max_slope_angle && fabsf(c.slope_angle) > 0.2f && fabsf(c.slope_angle) < c.max_slope_angle) {
        moving_direction = {0.0f, sinf(c.slope_angle), cosf(c.slope_angle)};
    } else if (c.actual_slope_angle >= c.max_slope_angle) {
        // too steep: only a little forward
        const float rise = sinf(c.slope_angle);
        moving_direction = {0.0f, rise > 0.0f ? 0.0f : rise, rise > 0.0f ? 0.1f : 1.0f};
    }
    moving_direction = vec::rotate(vec::quat_axis_angle({0.0f, 1.0f, 0.0f}, c.indicator_y), moving_direction);

    const Float3 velocity = c.body->motion_properties.linear_velocity;
    const Float3 object_velocity_along = vec::scale(moving_direction, vec::dot(c.moving_object_velocity, moving_direction));
    const float angle_to_object = acosf(clamp_unit(vec::dot(c.moving_object_velocity, moving_direction) /
                                                   (vec::length(c.moving_object_velocity) * vec::length(moving_direction) + 0.0001f)));

    // the velocity across the way it wants to go, cancelled on the ground
    const float along = velocity.x * moving_direction.x + velocity.z * moving_direction.z;
    const Float3 reject = {velocity.x - moving_direction.x * along, 0.0f, velocity.z - moving_direction.z * along};
    const float reject_mult = c.is_on_moving_object ? 0.0f : c.reject_vel_mult;
    const float object_sin = sinf(angle_to_object);
    const Float3 acceleration = {
        (moving_direction.x * (max_velocity + object_velocity_along.x) -
         (velocity.x - c.moving_object_velocity.x * object_sin + reject.x * reject_mult)) / c.acceleration_time,
        0.0f,
        (moving_direction.z * (max_velocity + object_velocity_along.z) -
         (velocity.z - c.moving_object_velocity.z * object_sin + reject.z * reject_mult)) / c.acceleration_time,
    };

    const float control_mult = c.rotated ? 1.0f : c.turn_vel_multiplier;
    const float air_mult = c.can_jump ? 1.0f : c.air_control_factor;
    float slope_impulse_y = 0.0f;
    if (c.slope_angle != 0.0f) {
        slope_impulse_y = moving_direction.y * (moving_direction.y > 0.0f ? c.slope_up_extra_force : c.slope_down_extra_force) *
                          (run ? c.max_run_speed / c.max_walk_speed : 1.0f);
    }
    // a mass of 1: the acceleration is the force
    const Float3 impulse = {acceleration.x * control_mult * air_mult, slope_impulse_y, acceleration.z * control_mult * air_mult};
    const Float3 point = {c.body->position.x, c.body->position.y + c.move_impulse_point_y, c.body->position.z};
    rigid_body::add_impulse_at_position(*app.world, c.body, vec3f::load(impulse), vec3f::load(point));

    // the platform stood on is dragged back
    if (c.is_on_moving_object) {
        rigid_body::RigidBody *ground = rigid_body::get(*app.world, c.ground_body_id);
        if (ground != nullptr && ground->motion_type == motion_type::MOTION_DYNAMIC) {
            const float share = fminf(1.0f, 1.0f / c.mass_ratio);
            rigid_body::add_impulse_at_position(*app.world, ground, vec3f::make(-impulse.x * share, 0.0f, -impulse.z * share),
                                                vec3f::load(c.ground_position));
        }
    }
}

static void apply_auto_balance_torque(App &app, Character &c) {
    if (!c.enable_auto_balance) return;
    const Float3 up = vec::rotate(c.body->quaternion, {0.0f, 1.0f, 0.0f});
    const Float3 forward = body_forward(c);
    const Float3 desired_up = {0.0f, 1.0f, 0.0f};
    const Float3 desired_forward = vec::length(c.move_direction) > 0.001f ? vec::normalize(c.move_direction) : forward;

    const Float3 balance_on_x = {0.0f, up.y, up.z};
    const Float3 balance_on_z = {up.x, up.y, 0.0f};
    const Float3 facing_on_y = {forward.x, 0.0f, forward.z};
    const Float3 cross_x = vec::cross(desired_up, balance_on_x);
    const Float3 cross_y = vec::cross(desired_forward, facing_on_y);
    const Float3 cross_z = vec::cross(desired_up, balance_on_z);
    const float angle_x = acosf(clamp_unit(vec::dot(balance_on_x, desired_up)));
    const float angle_y = acosf(clamp_unit(vec::dot(facing_on_y, desired_forward)));
    const float angle_z = acosf(clamp_unit(vec::dot(balance_on_z, desired_up)));

    const Float3 angular_velocity = c.body->motion_properties.angular_velocity;
    const Float3 torque = {
        (cross_x.x < 0.0f ? 1.0f : -1.0f) * c.balance_spring_k * angle_x - angular_velocity.x * c.balance_damping_c,
        (cross_y.y < 0.0f ? 1.0f : -1.0f) * c.balance_spring_on_y * angle_y - angular_velocity.y * c.balance_damping_on_y,
        (cross_z.z < 0.0f ? 1.0f : -1.0f) * c.balance_spring_k * angle_z - angular_velocity.z * c.balance_damping_c,
    };
    rigid_body::add_angular_impulse(*app.world, c.body, vec3f::load(torque));
}

static void stand_still(Character &c) {
    c.mass_ratio = 1.0f;
    c.is_on_moving_object = false;
    c.moving_object_velocity = {0.0f, 0.0f, 0.0f};
    c.body_contact_force = {0.0f, 0.0f, 0.0f};
}

static void update_moving_platform(App &app, Character &c, bool moving) {
    if (!c.can_jump) {
        stand_still(c);
        return;
    }
    rigid_body::RigidBody *ground = rigid_body::get(*app.world, c.ground_body_id);
    if (ground == nullptr || !is_moving_body(ground)) {
        stand_still(c);
        return;
    }
    c.is_on_moving_object = true;
    c.mass_ratio = c.body->mass_properties.mass / ground->mass_properties.mass;

    // the platform's velocity where the character stands
    const Float3 offset = vec::subtract(c.body->position, ground->position);
    const Float3 linear = ground->motion_properties.linear_velocity;
    const Float3 spin = vec::cross(ground->motion_properties.angular_velocity, offset);
    const float share = fminf(1.0f, 1.0f / c.mass_ratio);
    c.moving_object_velocity = vec::scale({linear.x + spin.x, linear.y, linear.z + spin.z}, share);
    const float difference = vec::length(vec::subtract(c.moving_object_velocity, c.body->motion_properties.linear_velocity));
    if (difference > 30.0f) c.moving_object_velocity = vec::scale(c.moving_object_velocity, 1.0f / difference);

    // standing still on a dynamic platform: its drag comes from the contact force (moving, apply_movement_force drags it)
    if (ground->motion_type == motion_type::MOTION_DYNAMIC && !moving && vec::dot(c.move_direction, c.move_direction) < 0.001f) {
        rigid_body::add_impulse_at_position(*app.world, ground, vec3f::scale(vec3f::load(c.body_contact_force), -share),
                                            vec3f::load(c.ground_position));
        c.body_contact_force = {0.0f, 0.0f, 0.0f};
    }
}

static void update_gravity_scale(Character &c) {
    const float vertical = c.body->motion_properties.linear_velocity.y;
    c.is_falling = vertical < 0.0f && !c.can_jump;
    float &gravity_factor = c.body->motion_properties.gravity_factor;
    if (vertical < -c.max_fall_speed) gravity_factor = 0.0f;
    else if (c.is_falling) gravity_factor = c.falling_gravity_scale;
    else gravity_factor = c.normal_gravity_scale;
}

static void handle_jump(App &app, Character &c) {
    if (!c.want_to_jump || !c.can_jump) return;
    const Float3 velocity = c.body->motion_properties.linear_velocity;
    const float jump = c.want_to_run ? c.sprint_jump_mult * c.jump_velocity : c.jump_velocity;
    Float3 jump_velocity = {velocity.x, jump, velocity.z};
    // a little more along the slope's normal
    Float3 along_normal = {0.0f, jump * c.slope_jump_mult, 0.0f};
    const float normal_length = vec::length(c.actual_slope_normal);
    if (normal_length > 0.0001f) {
        along_normal = vec::scale(c.actual_slope_normal, vec::dot(along_normal, c.actual_slope_normal) / (normal_length * normal_length));
    }
    jump_velocity = vec::add(jump_velocity, along_normal);
    rigid_body::set_linear_velocity(*app.world, c.body, vec3f::load(jump_velocity));

    rigid_body::RigidBody *ground = rigid_body::get(*app.world, c.ground_body_id);
    if (ground != nullptr && is_moving_body(ground)) {
        const Float3 push = {c.character_mass_force.x, c.character_mass_force.y * c.jump_force_to_ground_mult, c.character_mass_force.z};
        rigid_body::add_impulse_at_position(*app.world, ground, vec3f::load(push), vec3f::load(c.ground_position));
    }
}

static void update_rotation(Character &c, float delta_time) {
    if (vec::length(c.move_direction) <= 0.001f) {
        c.rotated = true;
        return;
    }
    float delta = atan2f(c.move_direction.x, c.move_direction.z) - c.indicator_y;
    while (delta > PI) delta -= 2.0f * PI;
    while (delta < -PI) delta += 2.0f * PI;
    const float max_step = c.turn_speed * delta_time;
    c.indicator_y += fmaxf(-max_step, fminf(max_step, delta));
    c.rotated = fabsf(delta) < 0.01f;
}

static void apply_drag_force(App &app, Character &c) {
    if (!c.can_jump) return;
    const Float3 velocity = c.body->motion_properties.linear_velocity;
    // towards rest, or towards the platform's velocity
    const Float3 target = c.is_on_moving_object ? c.moving_object_velocity : Float3{0.0f, 0.0f, 0.0f};
    rigid_body::add_impulse(*app.world, c.body,
                            vec3f::make((target.x - velocity.x) * c.drag_damping_c, 0.0f, (target.z - velocity.z) * c.drag_damping_c));
}

static void update_character(App &app, Character &c, Float3 direction, bool run, bool jump, float delta_time) {
    c.want_to_run = run;
    c.want_to_jump = jump;
    direction.y = 0.0f;
    c.move_direction = vec::length(direction) > 0.001f ? vec::normalize(direction) : direction;
    const bool moving = vec::length(c.move_direction) > 0.001f;

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

static rigid_body::RigidBody *add_static(App &app, shapes::Shape *shape, Float3 position, Float4 quaternion = {0.0f, 0.0f, 0.0f, 1.0f}) {
    rigid_body::RigidBodySettings settings;
    settings.shape = shape;
    settings.position = position;
    settings.quaternion = quaternion;
    settings.motion_type = motion_type::MOTION_STATIC;
    settings.object_layer = state.layers.not_moving;
    return rigid_body::create(*app.world, settings);
}

static rigid_body::RigidBody *add_kinematic(App &app, shapes::Shape *shape, Float3 position) {
    rigid_body::RigidBodySettings settings;
    settings.shape = shape;
    settings.position = position;
    settings.motion_type = motion_type::MOTION_KINEMATIC;
    settings.object_layer = state.layers.moving;
    return rigid_body::create(*app.world, settings);
}

static shapes::Shape *make_box(App &app, Float3 half_extents) { return app::keep(app, box::create({.half_extents = half_extents})); }

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
    triangle_mesh_data::TriangleMeshData *data = app::keep(
        app, triangle_mesh_builder::build({.positions = positions, .num_positions = (GRID + 1) * (GRID + 1), .indices = indices, .num_indices = GRID * GRID * 6}));
    add_static(app, app::keep(app, triangle_mesh::create({.data = data})), {-30.0f, 2.0f, 30.0f});
}

static void make_level(App &app) {
    add_static(app, make_box(app, {50.0f, 0.5f, 50.0f}), {0.0f, -0.5f, 0.0f});
    shapes::Shape *wall = make_box(app, {0.5f, 2.0f, 45.0f});
    add_static(app, wall, {-45.0f, 1.0f, 0.0f});
    add_static(app, wall, {45.0f, 1.0f, 0.0f});
    // the conveyor belt's box, which crashcat leaves still
    add_static(app, make_box(app, {10.0f, 0.25f, 2.0f}), {0.0f, 0.0f, -10.0f});

    // five flights of stairs, each with steps taller than the last
    for (int j = 0; j < 5; j++) {
        const float step_height = 0.3f + 0.1f * (float)j;
        shapes::Shape *stair = make_box(app, {2.0f, step_height / 2.0f, 2.0f});
        for (int i = 1; i < 10; i++) {
            add_static(app, stair, {15.0f + 5.0f * (float)j, (float)i * step_height - 0.5f + step_height / 2.0f, -20.0f - (float)i * 3.0f});
        }
    }

    // slopes from 70 degrees down to 25
    shapes::Shape *slope = make_box(app, {2.5f, 0.6f, 8.0f});
    for (int i = 0; i < 10; i++) {
        const float angle = (70.0f - (float)i * 5.0f) * PI / 180.0f;
        add_static(app, slope, {-40.0f + 5.0f * (float)i, 2.0f, -25.0f}, vec::quat_axis_angle({1.0f, 0.0f, 0.0f}, angle));
    }

    rigid_body::RigidBodySettings block;
    block.shape = make_box(app, {0.75f, 0.75f, 0.75f});
    block.position = {-10.0f, 5.0f, 10.0f};
    block.motion_type = motion_type::MOTION_DYNAMIC;
    block.object_layer = state.layers.moving;
    block.friction = 0.1f;
    block.mass = 1.0f;
    rigid_body::create(*app.world, block);
    rigid_body::RigidBodySettings ball = block;
    ball.shape = app::keep(app, sphere::create({.radius = 0.75f}));
    ball.position = {-8.0f, 2.0f, 10.0f};
    rigid_body::create(*app.world, ball);

    state.spinning_platform = add_kinematic(app, make_box(app, {4.0f, 0.2f, 4.0f}), {0.0f, 2.0f, 15.0f});
    shapes::Shape *platform = make_box(app, {3.0f, 0.2f, 3.0f});
    state.sliding_platform = add_kinematic(app, platform, {20.0f, 2.0f, -5.0f});
    state.diagonal_platform = add_kinematic(app, platform, {20.0f, 3.0f, 5.0f});

    // a hub and four blades about z, sweeping through where the character walks
    shapes::Shape *hub = make_box(app, {0.3f, 0.3f, 0.5f});
    shapes::Shape *vertical_blade = make_box(app, {0.5f, 3.0f, 0.5f});
    shapes::Shape *horizontal_blade = make_box(app, {3.0f, 0.5f, 0.5f});
    const Float4 identity = {0.0f, 0.0f, 0.0f, 1.0f};
    const compound::CompoundShapeChild blades[] = {
        {.position = {0.0f, 0.0f, 0.0f}, .quaternion = identity, .shape = hub},
        {.position = {0.0f, 3.3f, 0.0f}, .quaternion = identity, .shape = vertical_blade},
        {.position = {0.0f, -3.3f, 0.0f}, .quaternion = identity, .shape = vertical_blade},
        {.position = {3.3f, 0.0f, 0.0f}, .quaternion = identity, .shape = horizontal_blade},
        {.position = {-3.3f, 0.0f, 0.0f}, .quaternion = identity, .shape = horizontal_blade},
    };
    state.windmill = add_kinematic(app, app::keep(app, compound::create({.children = blades, .num_children = 5})), {-15.0f, 6.5f, 0.0f});
    add_static(app, make_box(app, {0.25f, 6.5f, 0.25f}), {-15.0f, 0.0f, 0.0f});

    make_terrain(app);

    // a bumpy floor of half sunk spheres
    const Float3 bumps[] = {
        {25.0f, 28.0f, 0.8f},  {29.0f, 32.0f, 0.9f},  {23.0f, 34.0f, 0.7f},  {31.0f, 28.0f, 0.85f}, {27.0f, 30.0f, 1.0f},  {22.0f, 30.0f, 0.5f},
        {32.0f, 30.0f, 0.55f}, {25.0f, 26.0f, 0.45f}, {29.0f, 34.0f, 0.6f},  {21.0f, 32.0f, 0.5f},  {33.0f, 32.0f, 0.48f}, {27.0f, 35.0f, 0.52f},
        {27.0f, 25.0f, 0.55f}, {24.0f, 31.0f, 0.35f}, {30.0f, 29.0f, 0.3f},  {26.0f, 33.0f, 0.38f}, {28.0f, 27.0f, 0.32f}, {22.0f, 28.0f, 0.28f},
        {32.0f, 34.0f, 0.34f}, {20.0f, 30.0f, 0.4f},  {34.0f, 30.0f, 0.36f}, {23.0f, 36.0f, 0.33f}, {31.0f, 26.0f, 0.31f},
    };
    for (const Float3 &bump : bumps) add_static(app, app::keep(app, sphere::create({.radius = bump.z})), {bump.x, bump.z * -0.5f, bump.y});

    // boxes each twice the size of the last
    float x = 5.0f;
    for (int i = 0; i < 5; i++) {
        const float half_extent = 0.25f * (float)(1 << i);
        add_static(app, make_box(app, {half_extent, half_extent, half_extent}), {x, half_extent, 30.0f});
        x += half_extent * 3.0f;
    }

    // a platform hung from four springs
    const Float3 centre = {-10.0f, 5.0f, 35.0f};
    const Float3 half_extents = {3.0f, 0.3f, 3.0f};
    rigid_body::RigidBodySettings hung;
    hung.shape = make_box(app, half_extents);
    hung.position = centre;
    hung.motion_type = motion_type::MOTION_DYNAMIC;
    hung.object_layer = state.layers.moving;
    hung.linear_damping = 0.5f;
    hung.angular_damping = 0.5f;
    rigid_body::RigidBody *suspended = rigid_body::create(*app.world, hung);
    const float spring_length = 4.0f;
    for (int i = 0; i < 4; i++) {
        const float dx = (i & 1) ? half_extents.x : -half_extents.x;
        const float dz = (i & 2) ? half_extents.z : -half_extents.z;
        const Float3 anchor_position = {centre.x + dx, 12.0f, centre.z + dz};
        rigid_body::RigidBody *anchor = add_static(app, nullptr, anchor_position);
        distance_constraint::create(*app.world, anchor, suspended,
                                    {.point_a = anchor_position,
                                     .point_b = {centre.x + dx, centre.y + half_extents.y, centre.z + dz},
                                     .min_distance = spring_length * 0.5f,
                                     .max_distance = spring_length * 2.0f,
                                     .spring_settings = {.mode = spring_settings::FREQUENCY_AND_DAMPING, .frequency_or_stiffness = 0.8f, .damping = 0.1f}});
    }
}

static void init(App &app) {
    state.steps = 0;
    app.world = app::create_world(state.layers);
    state.filter = filter::create(app.world->settings.layers);
    state.collector = cast_ray::create_collector(collector::COLLECT_CLOSEST);
    make_level(app);
    make_character(app, {0.0f, 5.0f, 0.0f});
    state.filter.body_filter = not_this_body;
    state.filter.body_filter_context = state.character.body;
    camera::look_at(app.camera, {0.0f, 5.0f, 10.0f}, {0.0f, 5.0f, 0.0f});
}

static void move_platforms(App &app, float delta_time) {
    const float time = (float)state.steps * delta_time;

    rigid_body::RigidBody *spinning = state.spinning_platform;
    const Quatf spun = quatf::multiply(quatf::load(spinning->quaternion), quatf::set_axis_angle(vec3f::axis_y(), 1.0f * delta_time));
    rigid_body::move_kinematic(*app.world, spinning, vec3f::load(spinning->position), spun, delta_time);

    rigid_body::RigidBody *sliding = state.sliding_platform;
    const float slide = sinf(time * 0.15f * PI * 2.0f) * 8.0f;
    rigid_body::move_kinematic(*app.world, sliding, vec3f::make(20.0f + slide, 2.0f, -5.0f), quatf::load(sliding->quaternion), delta_time);

    rigid_body::RigidBody *diagonal = state.diagonal_platform;
    const float phase = time * 0.12f * PI * 2.0f;
    rigid_body::move_kinematic(*app.world, diagonal, vec3f::make(20.0f + sinf(phase) * 6.0f, 3.0f + sinf(phase * 2.0f) * 2.0f, 5.0f),
                               quatf::load(diagonal->quaternion), delta_time);

    rigid_body::RigidBody *windmill = state.windmill;
    const Quatf turned = quatf::multiply(quatf::load(windmill->quaternion), quatf::set_axis_angle(vec3f::axis_z(), 1.0f * delta_time));
    rigid_body::move_kinematic(*app.world, windmill, vec3f::load(windmill->position), turned, delta_time);
}

static void step(App &app, float delta_time) {
    // w a s d move relative to the camera, space jumps, shift runs
    const bool *keys = app.keys;
    const float forward = keys[SAPP_KEYCODE_W] ? 1.0f : keys[SAPP_KEYCODE_S] ? -1.0f : 0.0f;
    const float right = keys[SAPP_KEYCODE_D] ? 1.0f : keys[SAPP_KEYCODE_A] ? -1.0f : 0.0f;
    const Float3 camera_forward = vec::normalize({app.camera.forward.x, 0.0f, app.camera.forward.z});
    const Float3 camera_right = vec::normalize({app.camera.side.x, 0.0f, app.camera.side.z});
    const Float3 direction = vec::add(vec::scale(camera_right, right), vec::scale(camera_forward, forward));
    const bool run = keys[SAPP_KEYCODE_LEFT_SHIFT] || keys[SAPP_KEYCODE_RIGHT_SHIFT];

    update_character(app, state.character, direction, run, keys[SAPP_KEYCODE_SPACE], delta_time);
    move_platforms(app, delta_time);
    state.steps++;
}

static void update(App &app, float) {
    const Character &c = state.character;
    // the camera follows the character, keeping its orbit
    app.camera.target = c.body->position;

    DrawList &frame = app::frame(app);
    const Float3 feet = {c.body->position.x, c.body->position.y - c.capsule_half_height, c.body->position.z};
    const float ground_length = c.ground_distance > 0.0f ? c.ground_distance : 1.0f;
    renderer::arrow(frame, feet, {feet.x, feet.y - ground_length, feet.z}, c.is_on_ground ? hex(0x00ff00) : hex(0xff0000));
    const Float3 forward = body_forward(c);
    const Float3 slope_origin = {feet.x + forward.x * c.slope_ray_offset, feet.y, feet.z + forward.z * c.slope_ray_offset};
    renderer::arrow(frame, slope_origin, {slope_origin.x, slope_origin.y - 1.0f, slope_origin.z}, hex(0x00ff00));
    if (c.is_on_ground) renderer::arrow(frame, c.ground_position, vec::add(c.ground_position, c.actual_slope_normal), hex(0x0000ff));
}

static void set_auto_balance(App &app, Character &c) {
    rigid_body::RigidBody *body = c.body;
    motion_properties::set_mass_properties(body->motion_properties, c.enable_auto_balance ? dof::DOF_ALL : dof::DOF_TRANSLATION_ONLY,
                                           body->mass_properties);
    if (c.enable_auto_balance) return;
    // upright and not turning
    rigid_body::set_angular_velocity(*app.world, body, vec3f::zero());
    rigid_body::set_quaternion(*app.world, body, quatf::identity(), true);
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
    cast_ray::release(state.collector);
    filter::release(state.filter);
}

static const Example EXAMPLE_FLOATING_CHARACTER_CONTROLLER = {"Floating Character Controller", init, step, update, ui, shutdown};
static const examples::Registration REGISTERED(EXAMPLE_FLOATING_CHARACTER_CONTROLLER);
