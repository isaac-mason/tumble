#include "tumble/physics/constraints/slider_constraint.h"

#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec2f.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/motion_properties.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/world.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

namespace slider_constraint {

using constraint_id::ConstraintId;
using motor_settings::MotorState;
using rigid_body::RigidBody;

static bool is_body_live(const bodies::Bodies &bodies, uint32_t index) {
    return index < bodies.pool.length && bodies.pool[index] != nullptr && bodies.sequences[index] != bodies::SLOT_FREE;
}

/** creates default slider constraint */
static SliderConstraint make_slider_constraint() {
    SliderConstraint constraint;
    constraint.base = constraints::make_constraint_base();
    constraint.local_space_position_a = vec3f::zero();
    constraint.local_space_position_b = vec3f::zero();
    constraint.local_space_slider_axis_a = vec3f::axis_x();
    constraint.local_space_normal_a = vec3f::axis_y();
    constraint.local_space_normal_b = vec3f::axis_z();
    constraint.inv_initial_orientation = quatf::identity();
    constraint.r1 = vec3f::zero();
    constraint.r2 = vec3f::zero();
    constraint.u = vec3f::zero();
    constraint.world_space_slider_axis = vec3f::zero();
    constraint.n1 = vec3f::zero();
    constraint.n2 = vec3f::zero();
    constraint.d = 0.0f;
    constraint.has_limits = false;
    constraint.limits_min = -scalar::FLOAT_MAX;
    constraint.limits_max = scalar::FLOAT_MAX;
    constraint.limits_spring_settings = spring_settings::create();
    constraint.motor_state = motor_settings::OFF;
    constraint.target_velocity = 0.0f;
    constraint.target_position = 0.0f;
    constraint.max_friction_force = 0.0f;
    constraint.motor_settings = motor_settings::create();
    constraint.position_constraint_part = dual_axis_constraint_part::create();
    constraint.rotation_constraint_part = rotation_euler_constraint_part::create();
    constraint.position_limits_constraint_part = axis_constraint_part::create();
    constraint.motor_constraint_part = axis_constraint_part::create();
    return constraint;
}

/** reset constraint state for pooling reuse */
static void reset_constraint(SliderConstraint &constraint) {
    constraint.base.enabled = true;
    constraint.base.sleeping = false;
    constraint.base.pooled = false;
    dual_axis_constraint_part::deactivate(constraint.position_constraint_part);
    rotation_euler_constraint_part::deactivate(constraint.rotation_constraint_part);
    axis_constraint_part::deactivate(constraint.position_limits_constraint_part);
    axis_constraint_part::deactivate(constraint.motor_constraint_part);
}

ConstraintId create(world::World &world, RigidBody *body_a, RigidBody *body_b,
                    const SliderConstraintSettings &settings) {
    constraints::Constraints &constraints = world.constraints;
    TUMBLE_ASSERT(is_body_live(world.bodies, body_a->index) && world.bodies.pool[body_a->index] == body_a);
    TUMBLE_ASSERT(is_body_live(world.bodies, body_b->index) && world.bodies.pool[body_b->index] == body_b);
    constraints::ConstraintPool &pool = constraints::ensure_pool(constraints, constraint_id::SLIDER);

    const uint32_t index = constraints::allocate_index(pool);
    if (index == ~0u) return constraint_id::INVALID_CONSTRAINT_ID;
    const uint32_t sequence = constraints::next_sequence(pool, index);
    SliderConstraint &constraint = *reinterpret_cast<SliderConstraint *>(constraints::base_at(pool, index));
    constraint = make_slider_constraint();
    reset_constraint(constraint);

    constraint.base.id = constraint_id::ser(index, constraint_id::SLIDER, sequence);
    constraint.base.index = index;
    constraint.base.sequence = sequence;

    constraint.base.constraint_priority = settings.constraint_priority;
    constraint.base.num_velocity_steps_override = settings.num_velocity_steps_override;
    constraint.base.num_position_steps_override = settings.num_position_steps_override;

    constraint.base.body_index_a = body_a->index;
    constraint.base.body_index_b = body_b->index;

    // normalize axes
    const Vec3f slider_axis1 = vec3f::normalize(vec3f::load(settings.slider_axis_a));
    const Vec3f slider_axis2 = vec3f::normalize(vec3f::load(settings.slider_axis_b));
    const Vec3f normal_axis1 = vec3f::normalize(vec3f::load(settings.normal_axis_a));
    const Vec3f normal_axis2 = vec3f::normalize(vec3f::load(settings.normal_axis_b));

    // initial orientation from the slider (x) and normal (y) axes
    constraint.inv_initial_orientation =
        rotation_euler_constraint_part::get_inv_initial_orientation_xy(slider_axis1, normal_axis1, slider_axis2, normal_axis2);

    if (settings.space == constraints::SPACE_WORLD) {
        const Quatf inv_quat_a = quatf::conjugate(quatf::load(body_a->quaternion));
        const Quatf inv_quat_b = quatf::conjugate(quatf::load(body_b->quaternion));

        if (settings.auto_detect_point) {
            // if either body is static, anchor on the other body's center of mass. otherwise weight the anchor
            // towards the lighter body (higher inv_mass = more influence)
            const float inv_mass_a = body_a->motion_properties.inv_mass;
            const float inv_mass_b = body_b->motion_properties.inv_mass;
            const float total_inv_mass = inv_mass_a + inv_mass_b;

            const Vec3f center_of_mass_a = vec3f::load(body_a->center_of_mass_position);
            const Vec3f center_of_mass_b = vec3f::load(body_b->center_of_mass_position);
            Vec3f anchor;
            if (body_a->motion_type == motion_type::MOTION_STATIC) {
                anchor = center_of_mass_b;
            } else if (body_b->motion_type == motion_type::MOTION_STATIC) {
                anchor = center_of_mass_a;
            } else if (total_inv_mass != 0.0f) {
                anchor = vec3f::scale_and_add(vec3f::zero(), center_of_mass_a, inv_mass_a / total_inv_mass);
                anchor = vec3f::scale_and_add(anchor, center_of_mass_b, inv_mass_b / total_inv_mass);
            } else {
                // both bodies have infinite mass, use body a's position
                anchor = center_of_mass_a;
            }

            constraint.local_space_position_a = vec3f::transform_quat(vec3f::subtract(anchor, center_of_mass_a), inv_quat_a);
            constraint.local_space_position_b = vec3f::transform_quat(vec3f::subtract(anchor, center_of_mass_b), inv_quat_b);
        } else {
            constraint.local_space_position_a = vec3f::transform_quat(vec3f::subtract(vec3f::load(settings.point_a), vec3f::load(body_a->center_of_mass_position)), inv_quat_a);

            constraint.local_space_position_b = vec3f::transform_quat(vec3f::subtract(vec3f::load(settings.point_b), vec3f::load(body_b->center_of_mass_position)), inv_quat_b);
        }

        // slider axis and normal to local space
        constraint.local_space_slider_axis_a = vec3f::normalize(vec3f::transform_quat(slider_axis1, inv_quat_a));
        constraint.local_space_normal_a = vec3f::normalize(vec3f::transform_quat(normal_axis1, inv_quat_a));

        constraint.local_space_normal_b = vec3f::cross(constraint.local_space_slider_axis_a, constraint.local_space_normal_a);

        // the axes were given in world space: r0^-1 = q20^-1 (c2 c1^-1) q10
        constraint.inv_initial_orientation =
            quatf::multiply(quatf::multiply(inv_quat_b, constraint.inv_initial_orientation), quatf::load(body_a->quaternion));
    } else {
        constraint.local_space_position_a = vec3f::load(settings.point_a);
        constraint.local_space_position_b = vec3f::load(settings.point_b);
        constraint.local_space_slider_axis_a = slider_axis1;
        constraint.local_space_normal_a = normal_axis1;
        constraint.local_space_normal_b = vec3f::cross(constraint.local_space_slider_axis_a, constraint.local_space_normal_a);
    }

    // equal limits without a spring should be a fixed constraint
    TUMBLE_ASSERT(settings.limits_min != settings.limits_max || settings.limits_spring_settings.frequency_or_stiffness > 0.0f);
    set_limits(constraint, settings.limits_min, settings.limits_max);
    constraint.limits_spring_settings = settings.limits_spring_settings;

    constraint.motor_state = motor_settings::OFF;
    constraint.target_velocity = 0.0f;
    constraint.target_position = 0.0f;
    constraint.max_friction_force = settings.max_friction_force;
    constraint.motor_settings = settings.motor_settings;

    // track constraint on both bodies
    const ConstraintId id = constraint.base.id;
    bool linked = body_a->constraint_ids.push(id);
    if (linked && body_a != body_b) linked = body_b->constraint_ids.push(id);
    if (linked) linked = constraints::add_to_list(constraints, constraint.base);
    if (!linked) {
        constraints::remove(world, id);
        return constraint_id::INVALID_CONSTRAINT_ID;
    }

    return id;
}

SliderConstraint *get(world::World &world, ConstraintId id) {
    if (constraint_id::type(id) != constraint_id::SLIDER) return nullptr;
    return reinterpret_cast<SliderConstraint *>(constraints::get(world.constraints, id));
}

float get_current_position(const world::World &world, const SliderConstraint &constraint) {
    const bodies::Bodies &bodies = world.bodies;
    // the cached value when a body is gone
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) {
        return constraint.d;
    }
    const RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    const RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    const Quatf rotation_a = quatf::load(body_a.quaternion);
    const Vec3f r1 = vec3f::transform_quat(constraint.local_space_position_a, rotation_a);
    const Vec3f r2 = vec3f::transform_quat(constraint.local_space_position_b, quatf::load(body_b.quaternion));

    // u = x2 + r2 - x1 - r1
    const Vec3f u = vec3f::subtract(
        vec3f::add(vec3f::subtract(vec3f::load(body_b.center_of_mass_position), vec3f::load(body_a.center_of_mass_position)), r2), r1);

    return vec3f::dot(u, vec3f::transform_quat(constraint.local_space_slider_axis_a, rotation_a));
}

void set_limits(SliderConstraint &constraint, float min, float max) {
    TUMBLE_ASSERT(min <= 0.0f);
    TUMBLE_ASSERT(max >= 0.0f);
    constraint.limits_min = min;
    constraint.limits_max = max;
    constraint.has_limits = min != -scalar::FLOAT_MAX || max != scalar::FLOAT_MAX;
}

void set_motor_state(SliderConstraint &constraint, MotorState state) {
    TUMBLE_ASSERT(state == motor_settings::OFF || motor_settings::is_valid(constraint.motor_settings));
    constraint.motor_state = state;
}

void set_target_velocity(SliderConstraint &constraint, float velocity) { constraint.target_velocity = velocity; }

void set_target_position(SliderConstraint &constraint, float position) {
    if (constraint.has_limits) {
        constraint.target_position = scalar::max(constraint.limits_min, scalar::min(constraint.limits_max, position));
    } else {
        constraint.target_position = position;
    }
}

void set_max_friction_force(SliderConstraint &constraint, float force) { constraint.max_friction_force = force; }

float get_max_friction_force(const SliderConstraint &constraint) { return constraint.max_friction_force; }

/** the body's inverse mass and world space inverse inertia as the axis part takes them: zero unless dynamic */
static void get_inverse_mass_and_inertia(float &out_inv_mass, Mat4f &out_inv_inertia, RigidBody &body, uint32_t step_stamp) {
    if (body.motion_type == motion_type::MOTION_DYNAMIC) {
        out_inv_mass = body.motion_properties.inv_mass;
        out_inv_inertia = motion_properties::get_world_inverse_inertia(body.motion_properties, quatf::load(body.quaternion), step_stamp);
    } else {
        out_inv_mass = 0.0f;
        out_inv_inertia = mat4f::zero();
    }
}

/** r1 = rotation1 x local position 1, r2 = rotation2 x local position 2, u = x2 + r2 - x1 - r1 */
static void calculate_r1_r2_u(SliderConstraint &constraint, const RigidBody &body_a, const RigidBody &body_b,
                              const Mat4f &rotation_a, const Mat4f &rotation_b) {
    constraint.r1 = mat4f::multiply3x3_vec(rotation_a, constraint.local_space_position_a);
    constraint.r2 = mat4f::multiply3x3_vec(rotation_b, constraint.local_space_position_b);

    const Vec3f delta_com = vec3f::subtract(vec3f::load(body_b.center_of_mass_position), vec3f::load(body_a.center_of_mass_position));
    constraint.u = vec3f::subtract(vec3f::add(delta_com, constraint.r2), constraint.r1);
}

/** the world space slider axis and the distance along it, when limits, a motor or friction need them */
static void calculate_sliding_axis_and_position(SliderConstraint &constraint, const Mat4f &rotation_a) {
    if (constraint.has_limits || constraint.motor_state != motor_settings::OFF || constraint.max_friction_force > 0.0f) {
        constraint.world_space_slider_axis = mat4f::multiply3x3_vec(rotation_a, constraint.local_space_slider_axis_a);
        constraint.d = vec3f::dot(constraint.u, constraint.world_space_slider_axis);
    }
}

/** the dual axis part perpendicular to the slider */
static void calculate_position_constraint_properties(SliderConstraint &constraint, RigidBody &body_a, RigidBody &body_b,
                                                     const Mat4f &rotation_a, uint32_t step_stamp) {
    constraint.n1 = mat4f::multiply3x3_vec(rotation_a, constraint.local_space_normal_a);
    constraint.n2 = mat4f::multiply3x3_vec(rotation_a, constraint.local_space_normal_b);

    // r1 + u: from body a's center of mass to body b's attachment point
    const Vec3f r1_plus_u = vec3f::add(constraint.r1, constraint.u);

    dual_axis_constraint_part::calculate_constraint_properties(constraint.position_constraint_part, body_a, r1_plus_u, body_b,
                                                               constraint.r2, constraint.n1, constraint.n2, step_stamp);
}

static void calculate_position_limits_constraint_properties(SliderConstraint &constraint, RigidBody &body_a, RigidBody &body_b,
                                                            float delta_time, uint32_t step_stamp) {
    const bool below_min = constraint.d <= constraint.limits_min;
    if (constraint.has_limits && (below_min || constraint.d >= constraint.limits_max)) {
        const float error = below_min ? constraint.d - constraint.limits_min : constraint.d - constraint.limits_max;

        const Vec3f r1_plus_u = vec3f::add(constraint.r1, constraint.u);

        float inv_mass_a;
        float inv_mass_b;
        Mat4f inv_inertia_a;
        Mat4f inv_inertia_b;
        get_inverse_mass_and_inertia(inv_mass_a, inv_inertia_a, body_a, step_stamp);
        get_inverse_mass_and_inertia(inv_mass_b, inv_inertia_b, body_b, step_stamp);

        if (constraint.limits_spring_settings.frequency_or_stiffness > 0.0f) {
            axis_constraint_part::calculate_constraint_properties_with_settings(
                constraint.position_limits_constraint_part, delta_time, body_a, body_b, inv_mass_a, inv_mass_b, inv_inertia_a,
                inv_inertia_b, r1_plus_u, constraint.r2, constraint.world_space_slider_axis, 0.0f, error,
                constraint.limits_spring_settings);
        } else {
            axis_constraint_part::calculate_constraint_properties(constraint.position_limits_constraint_part, body_a, body_b,
                                                                  inv_mass_a, inv_mass_b, inv_inertia_a, inv_inertia_b, r1_plus_u,
                                                                  constraint.r2, constraint.world_space_slider_axis, 0.0f);
        }
    } else {
        axis_constraint_part::deactivate(constraint.position_limits_constraint_part);
    }
}

static void calculate_motor_constraint_properties(SliderConstraint &constraint, RigidBody &body_a, RigidBody &body_b,
                                                  float delta_time, uint32_t step_stamp) {
    float inv_mass_a;
    float inv_mass_b;
    Mat4f inv_inertia_a;
    Mat4f inv_inertia_b;
    get_inverse_mass_and_inertia(inv_mass_a, inv_inertia_a, body_a, step_stamp);
    get_inverse_mass_and_inertia(inv_mass_b, inv_inertia_b, body_b, step_stamp);

    const Vec3f r1_plus_u = vec3f::add(constraint.r1, constraint.u);

    switch (constraint.motor_state) {
        case motor_settings::OFF:
            if (constraint.max_friction_force > 0.0f) {
                axis_constraint_part::calculate_constraint_properties(constraint.motor_constraint_part, body_a, body_b, inv_mass_a,
                                                                      inv_mass_b, inv_inertia_a, inv_inertia_b, r1_plus_u,
                                                                      constraint.r2, constraint.world_space_slider_axis, 0.0f);
            } else {
                axis_constraint_part::deactivate(constraint.motor_constraint_part);
            }
            break;

        case motor_settings::VELOCITY:
            axis_constraint_part::calculate_constraint_properties(constraint.motor_constraint_part, body_a, body_b, inv_mass_a,
                                                                  inv_mass_b, inv_inertia_a, inv_inertia_b, r1_plus_u,
                                                                  constraint.r2, constraint.world_space_slider_axis,
                                                                  -constraint.target_velocity);
            break;

        case motor_settings::POSITION:
            if (spring_settings::has_stiffness(constraint.motor_settings.spring)) {
                axis_constraint_part::calculate_constraint_properties_with_settings_for_motor(
                    constraint.motor_constraint_part, delta_time, body_a, body_b, inv_mass_a, inv_mass_b, inv_inertia_a,
                    inv_inertia_b, r1_plus_u, constraint.r2, constraint.world_space_slider_axis, 0.0f,
                    constraint.d - constraint.target_position, constraint.motor_settings.spring);
            } else {
                axis_constraint_part::deactivate(constraint.motor_constraint_part);
            }
            break;

        case motor_settings::POSITION_AND_VELOCITY:
            if (spring_settings::has_stiffness_or_damping(constraint.motor_settings.spring)) {
                axis_constraint_part::calculate_constraint_properties_with_settings_for_motor(
                    constraint.motor_constraint_part, delta_time, body_a, body_b, inv_mass_a, inv_mass_b, inv_inertia_a,
                    inv_inertia_b, r1_plus_u, constraint.r2, constraint.world_space_slider_axis, -constraint.target_velocity,
                    constraint.d - constraint.target_position, constraint.motor_settings.spring);
            } else {
                axis_constraint_part::deactivate(constraint.motor_constraint_part);
            }
            break;
    }
}

void setup_velocity(SliderConstraint &constraint, bodies::Bodies &bodies, float delta_time) {
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    const Mat4f rotation_a = mat4f::from_quat(quatf::load(body_a.quaternion));
    const Mat4f rotation_b = mat4f::from_quat(quatf::load(body_b.quaternion));

    calculate_r1_r2_u(constraint, body_a, body_b, rotation_a, rotation_b);
    calculate_position_constraint_properties(constraint, body_a, body_b, rotation_a, bodies.step_stamp);
    rotation_euler_constraint_part::calculate_constraint_properties(constraint.rotation_constraint_part, body_a, body_b, bodies.step_stamp);
    calculate_sliding_axis_and_position(constraint, rotation_a);
    calculate_position_limits_constraint_properties(constraint, body_a, body_b, delta_time, bodies.step_stamp);
    calculate_motor_constraint_properties(constraint, body_a, body_b, delta_time, bodies.step_stamp);
}

void warm_start_velocity(SliderConstraint &constraint, bodies::Bodies &bodies, float warm_start_impulse_ratio) {
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    const float inv_mass_a = body_a.motion_type == motion_type::MOTION_DYNAMIC ? body_a.motion_properties.inv_mass : 0.0f;
    const float inv_mass_b = body_b.motion_type == motion_type::MOTION_DYNAMIC ? body_b.motion_properties.inv_mass : 0.0f;

    if (axis_constraint_part::is_active(constraint.motor_constraint_part)) {
        axis_constraint_part::warm_start(constraint.motor_constraint_part, body_a, body_b, inv_mass_a, inv_mass_b,
                                         constraint.world_space_slider_axis, warm_start_impulse_ratio);
    }
    dual_axis_constraint_part::warm_start(constraint.position_constraint_part, body_a, body_b, constraint.n1, constraint.n2,
                                          warm_start_impulse_ratio);
    rotation_euler_constraint_part::warm_start(constraint.rotation_constraint_part, body_a, body_b, warm_start_impulse_ratio);
    if (axis_constraint_part::is_active(constraint.position_limits_constraint_part)) {
        axis_constraint_part::warm_start(constraint.position_limits_constraint_part, body_a, body_b, inv_mass_a, inv_mass_b,
                                         constraint.world_space_slider_axis, warm_start_impulse_ratio);
    }
}

bool solve_velocity(SliderConstraint &constraint, bodies::Bodies &bodies, float delta_time) {
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return false;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    bool motor = false;
    if (axis_constraint_part::is_active(constraint.motor_constraint_part)) {
        switch (constraint.motor_state) {
            case motor_settings::OFF: {
                const float max_lambda = constraint.max_friction_force * delta_time;
                motor = axis_constraint_part::solve_velocity_constraint(constraint.motor_constraint_part, body_a, body_b,
                                                                        constraint.world_space_slider_axis, -max_lambda,
                                                                        max_lambda);
                break;
            }
            case motor_settings::VELOCITY:
            case motor_settings::POSITION:
            case motor_settings::POSITION_AND_VELOCITY:
                motor = axis_constraint_part::solve_velocity_constraint(
                    constraint.motor_constraint_part, body_a, body_b, constraint.world_space_slider_axis,
                    delta_time * constraint.motor_settings.min_force_limit, delta_time * constraint.motor_settings.max_force_limit);
                break;
        }
    }

    const bool pos = dual_axis_constraint_part::solve_velocity_constraint(constraint.position_constraint_part, body_a, body_b,
                                                                          constraint.n1, constraint.n2);
    const bool rot = rotation_euler_constraint_part::solve_velocity_constraint(constraint.rotation_constraint_part, body_a, body_b);

    bool limit = false;
    if (axis_constraint_part::is_active(constraint.position_limits_constraint_part)) {
        float min_lambda;
        float max_lambda;
        if (constraint.limits_min == constraint.limits_max) {
            min_lambda = -scalar::FLOAT_MAX;
            max_lambda = scalar::FLOAT_MAX;
        } else if (constraint.d <= constraint.limits_min) {
            min_lambda = 0.0f;
            max_lambda = scalar::FLOAT_MAX;
        } else {
            min_lambda = -scalar::FLOAT_MAX;
            max_lambda = 0.0f;
        }
        limit = axis_constraint_part::solve_velocity_constraint(constraint.position_limits_constraint_part, body_a, body_b,
                                                                constraint.world_space_slider_axis, min_lambda, max_lambda);
    }

    return motor || pos || rot || limit;
}

bool solve_position(SliderConstraint &constraint, bodies::Bodies &bodies, float delta_time, float baumgarte) {
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return false;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    // the motor works on velocities only

    Mat4f rotation_a = mat4f::from_quat(quatf::load(body_a.quaternion));
    Mat4f rotation_b = mat4f::from_quat(quatf::load(body_b.quaternion));
    calculate_r1_r2_u(constraint, body_a, body_b, rotation_a, rotation_b);
    calculate_position_constraint_properties(constraint, body_a, body_b, rotation_a, motion_properties::STEP_STAMP_NONE);
    const bool pos = dual_axis_constraint_part::solve_position_constraint(constraint.position_constraint_part, body_a, body_b,
                                                                          constraint.u, constraint.n1, constraint.n2, baumgarte);

    rotation_euler_constraint_part::calculate_constraint_properties(constraint.rotation_constraint_part, body_a, body_b,
                                                                    motion_properties::STEP_STAMP_NONE);
    const bool rot = rotation_euler_constraint_part::solve_position_constraint(constraint.rotation_constraint_part, body_a, body_b,
                                                                               constraint.inv_initial_orientation, baumgarte);

    // soft limits have no position solve
    bool limit = false;
    if (constraint.has_limits && constraint.limits_spring_settings.frequency_or_stiffness <= 0.0f) {
        rotation_a = mat4f::from_quat(quatf::load(body_a.quaternion));
        rotation_b = mat4f::from_quat(quatf::load(body_b.quaternion));
        calculate_r1_r2_u(constraint, body_a, body_b, rotation_a, rotation_b);
        calculate_sliding_axis_and_position(constraint, rotation_a);
        calculate_position_limits_constraint_properties(constraint, body_a, body_b, delta_time, motion_properties::STEP_STAMP_NONE);
        if (axis_constraint_part::is_active(constraint.position_limits_constraint_part)) {
            const float position_error = constraint.d <= constraint.limits_min ? constraint.d - constraint.limits_min
                                                                               : constraint.d - constraint.limits_max;
            limit = axis_constraint_part::solve_position_constraint(constraint.position_limits_constraint_part, body_a, body_b,
                                                                    constraint.world_space_slider_axis, position_error, baumgarte);
        }
    }

    return pos || rot || limit;
}

void notify_shape_changed(SliderConstraint &constraint, body_id::BodyId body_id, Vec3f delta_com) {
    // separate tests, not an else-if: a constraint between a body and itself moves both points
    const uint32_t body_index = body_id::index(body_id);
    if (constraint.base.body_index_a == body_index)
        constraint.local_space_position_a = vec3f::subtract(constraint.local_space_position_a, delta_com);
    if (constraint.base.body_index_b == body_index)
        constraint.local_space_position_b = vec3f::subtract(constraint.local_space_position_b, delta_com);
}

void reset_warm_start(SliderConstraint &constraint) {
    dual_axis_constraint_part::deactivate(constraint.position_constraint_part);
    rotation_euler_constraint_part::deactivate(constraint.rotation_constraint_part);
    axis_constraint_part::deactivate(constraint.position_limits_constraint_part);
    axis_constraint_part::deactivate(constraint.motor_constraint_part);
}

void get_iteration_overrides(constraints::ConstraintIterationOverrides &out, const SliderConstraint &constraint) {
    out.velocity = constraint.base.num_velocity_steps_override;
    out.position = constraint.base.num_position_steps_override;
}

void get_sort_fields(constraints::ConstraintSortFields &out, const SliderConstraint &constraint) {
    out.priority = constraint.base.constraint_priority;
    out.index = constraint.base.list_index;
}

Vec2f &get_total_lambda_position(Vec2f &out, const SliderConstraint &constraint) {
    return vec2f::copy(out, constraint.position_constraint_part.total_lambda);
}

float get_total_lambda_position_limits(const SliderConstraint &constraint) {
    return axis_constraint_part::get_total_lambda_value(constraint.position_limits_constraint_part);
}

Vec3f get_total_lambda_rotation(const SliderConstraint &constraint) { return constraint.rotation_constraint_part.total_lambda; }

float get_total_lambda_motor(const SliderConstraint &constraint) {
    return axis_constraint_part::get_total_lambda_value(constraint.motor_constraint_part);
}

}  // namespace slider_constraint

TUMBLE_NAMESPACE_END
