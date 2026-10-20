#include "tumble/physics/constraints/hinge_constraint.h"

#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/math/vec4f.h"
#include "tumble/physics/body/motion_properties.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

namespace hinge_constraint {

using constraint_id::ConstraintId;
using motor_settings::MotorState;
using rigid_body::RigidBody;

static bool is_body_live(const bodies::Bodies &bodies, uint32_t index) {
    return index < bodies.pool.length && bodies.pool[index] != nullptr && bodies.sequences[index] != bodies::SLOT_FREE;
}

/** creates default hinge constraint */
static HingeConstraint make_hinge_constraint() {
    HingeConstraint constraint;
    constraint.base = constraints::make_constraint_base();
    constraint.local_space_position1 = vec3f::zero();
    constraint.local_space_position2 = vec3f::zero();
    constraint.local_space_hinge_axis1 = vec3f::axis_y();
    constraint.local_space_hinge_axis2 = vec3f::axis_y();
    constraint.local_space_normal_axis1 = vec3f::axis_x();
    constraint.local_space_normal_axis2 = vec3f::axis_x();
    constraint.inv_initial_orientation = quatf::identity();
    constraint.theta = 0.0f;
    constraint.world_space_hinge_axis1 = vec3f::zero();
    constraint.has_limits = false;
    constraint.limits_min = -scalar::PI;
    constraint.limits_max = scalar::PI;
    constraint.limits_spring_settings = spring_settings::create();
    constraint.motor_state = motor_settings::OFF;
    constraint.target_angular_velocity = 0.0f;
    constraint.target_angle = 0.0f;
    constraint.max_friction_torque = 0.0f;
    constraint.motor_settings = motor_settings::create();
    constraint.point_constraint_part = point_constraint_part::create();
    constraint.rotation_constraint_part = hinge_rotation_constraint_part::create();
    constraint.rotation_limits_constraint_part = angle_constraint_part::create();
    constraint.motor_constraint_part = angle_constraint_part::create();
    return constraint;
}

/** the rotation whose matrix has columns normal, hinge x normal and hinge */
static Quatf rotation_of_axes_xz(Vec3f normal_axis, Vec3f hinge_axis) {
    const Vec3f y = vec3f::cross(hinge_axis, normal_axis);
    return quatf::from_mat4(mat4f::make_columns({normal_axis.v}, {y.v}, {hinge_axis.v}, vec4f::make(0.0f, 0.0f, 0.0f, 1.0f)));
}

/** inverse initial orientation for angle tracking, from a normal (x) and hinge (z) axis per body: c2 * c1^-1 */
static Quatf get_inv_initial_orientation_xz(Vec3f normal_axis1, Vec3f hinge_axis1, Vec3f normal_axis2, Vec3f hinge_axis2) {
    // if axes are identical, return identity
    if (vec3f::exact_equals(normal_axis1, normal_axis2) && vec3f::exact_equals(hinge_axis1, hinge_axis2)) return quatf::identity();

    // q2 * conjugate(q1)
    return quatf::multiply(rotation_of_axes_xz(normal_axis2, hinge_axis2), quatf::conjugate(rotation_of_axes_xz(normal_axis1, hinge_axis1)));
}

/** reset constraint state for pooling reuse */
static void reset_constraint(HingeConstraint &constraint) {
    constraint.base.enabled = true;
    constraint.base.sleeping = false;
    constraint.base.pooled = false;
    point_constraint_part::deactivate(constraint.point_constraint_part);
    hinge_rotation_constraint_part::deactivate(constraint.rotation_constraint_part);
    angle_constraint_part::deactivate(constraint.rotation_limits_constraint_part);
    angle_constraint_part::deactivate(constraint.motor_constraint_part);
}

ConstraintId create(constraints::Constraints &constraints, bodies::Bodies &bodies, RigidBody *body_a, RigidBody *body_b,
                    const HingeConstraintSettings &settings) {
    TUMBLE_ASSERT(is_body_live(bodies, body_a->index) && bodies.pool[body_a->index] == body_a);
    TUMBLE_ASSERT(is_body_live(bodies, body_b->index) && bodies.pool[body_b->index] == body_b);
    constraints::ConstraintPool &pool = constraints::ensure_pool(constraints, constraint_id::HINGE);

    const uint32_t index = constraints::allocate_index(pool);
    if (index == ~0u) return constraint_id::INVALID_CONSTRAINT_ID;
    const uint32_t sequence = constraints::next_sequence(pool, index);
    HingeConstraint &constraint = *reinterpret_cast<HingeConstraint *>(constraints::base_at(pool, index));
    constraint = make_hinge_constraint();
    reset_constraint(constraint);

    constraint.base.id = constraint_id::ser(index, constraint_id::HINGE, sequence);
    constraint.base.index = index;
    constraint.base.sequence = sequence;

    constraint.base.constraint_priority = settings.constraint_priority;
    constraint.base.num_velocity_steps_override = settings.num_velocity_steps_override;
    constraint.base.num_position_steps_override = settings.num_position_steps_override;

    constraint.base.body_index_a = body_a->index;
    constraint.base.body_index_b = body_b->index;

    // normalize axes
    const Vec3f hinge_axis1 = vec3f::normalize(vec3f::load(settings.hinge_axis_a));
    const Vec3f hinge_axis2 = vec3f::normalize(vec3f::load(settings.hinge_axis_b));
    const Vec3f normal_axis1 = vec3f::normalize(vec3f::load(settings.normal_axis_a));
    const Vec3f normal_axis2 = vec3f::normalize(vec3f::load(settings.normal_axis_b));

    // inverse of the initial rotation from body 1 to body 2 in body 1 space, from the axes as given
    const Quatf r0 = get_inv_initial_orientation_xz(normal_axis1, hinge_axis1, normal_axis2, hinge_axis2);

    if (settings.space == constraints::SPACE_WORLD) {
        const Quatf inv_quat_a = quatf::conjugate(quatf::load(body_a->quaternion));
        const Quatf inv_quat_b = quatf::conjugate(quatf::load(body_b->quaternion));

        // transform positions to local space
        constraint.local_space_position1 = vec3f::transform_quat(vec3f::subtract(vec3f::load(settings.point_a), vec3f::load(body_a->center_of_mass_position)), inv_quat_a);

        constraint.local_space_position2 = vec3f::transform_quat(vec3f::subtract(vec3f::load(settings.point_b), vec3f::load(body_b->center_of_mass_position)), inv_quat_b);

        // transform axes to local space
        constraint.local_space_hinge_axis1 = vec3f::transform_quat(hinge_axis1, inv_quat_a);
        constraint.local_space_hinge_axis2 = vec3f::transform_quat(hinge_axis2, inv_quat_b);
        constraint.local_space_normal_axis1 = vec3f::transform_quat(normal_axis1, inv_quat_a);
        constraint.local_space_normal_axis2 = vec3f::transform_quat(normal_axis2, inv_quat_b);

        // the axes were given in world space, so r0^-1 = q20^-1 (c2 c1^-1) q10: at the initial orientations
        // diff = q2 * inv_initial_orientation * q1^-1 is then c2 c1^-1
        constraint.inv_initial_orientation = quatf::multiply(quatf::multiply(inv_quat_b, r0), quatf::load(body_a->quaternion));
    } else {
        constraint.local_space_position1 = vec3f::load(settings.point_a);
        constraint.local_space_position2 = vec3f::load(settings.point_b);
        constraint.local_space_hinge_axis1 = hinge_axis1;
        constraint.local_space_hinge_axis2 = hinge_axis2;
        constraint.local_space_normal_axis1 = normal_axis1;
        constraint.local_space_normal_axis2 = normal_axis2;

        // the axes are already relative to the bodies
        constraint.inv_initial_orientation = r0;
    }

    // equal limits without a spring should be a fixed constraint
    TUMBLE_ASSERT(settings.limits_min != settings.limits_max || settings.limits_spring_settings.frequency_or_stiffness > 0.0f);
    set_limits(constraint, settings.limits_min, settings.limits_max);
    constraint.limits_spring_settings = settings.limits_spring_settings;

    constraint.motor_state = motor_settings::OFF;
    constraint.target_angular_velocity = 0.0f;
    constraint.target_angle = 0.0f;
    constraint.max_friction_torque = settings.max_friction_torque;
    constraint.motor_settings = settings.motor_settings;

    // track constraint on both bodies
    const ConstraintId id = constraint.base.id;
    bool linked = body_a->constraint_ids.push(id);
    if (linked && body_a != body_b) linked = body_b->constraint_ids.push(id);
    if (!linked) {
        constraints::remove_constraint_by_id(constraints, bodies, id);
        return constraint_id::INVALID_CONSTRAINT_ID;
    }

    return id;
}

void remove(constraints::Constraints &constraints, bodies::Bodies &bodies, HingeConstraint &constraint) {
    constraints::ConstraintPool &pool = constraints::ensure_pool(constraints, constraint_id::HINGE);

    // remove from bodies' constraint_ids
    const uint32_t a = constraint.base.body_index_a;
    const uint32_t b = constraint.base.body_index_b;
    if (is_body_live(bodies, a)) constraints::remove_constraint_id_from_body(*bodies.pool[a], constraint.base.id);
    if (a != b && is_body_live(bodies, b)) constraints::remove_constraint_id_from_body(*bodies.pool[b], constraint.base.id);

    reset_constraint(constraint);

    constraint.base.pooled = true;
    constraint.base.id = constraint_id::INVALID_CONSTRAINT_ID;
    pool.free_indices.push(constraint.base.index);
}

HingeConstraint *get(constraints::Constraints &constraints, ConstraintId id) {
    if (constraint_id::type(id) != constraint_id::HINGE) return nullptr;
    return reinterpret_cast<HingeConstraint *>(constraints::get(constraints, id));
}

void set_limits(HingeConstraint &constraint, float min, float max) {
    TUMBLE_ASSERT(min <= 0.0f && min >= -scalar::PI);
    TUMBLE_ASSERT(max >= 0.0f && max <= scalar::PI);
    constraint.limits_min = min;
    constraint.limits_max = max;
    constraint.has_limits = min > -scalar::PI || max < scalar::PI;
}

void set_motor_state(HingeConstraint &constraint, MotorState state) {
    TUMBLE_ASSERT(state == motor_settings::OFF || motor_settings::is_valid(constraint.motor_settings));
    constraint.motor_state = state;
}

void set_target_angular_velocity(HingeConstraint &constraint, float velocity) { constraint.target_angular_velocity = velocity; }

void set_target_angle(HingeConstraint &constraint, float angle) {
    constraint.target_angle =
        constraint.has_limits ? scalar::max(constraint.limits_min, scalar::min(constraint.limits_max, angle)) : angle;
}

float get_current_angle(const HingeConstraint &constraint) { return constraint.theta; }

/** the world space hinge axis and the current hinge angle theta, when limits, a motor or friction need them */
static void calculate_a1_and_theta(HingeConstraint &constraint, const RigidBody &body_a, const RigidBody &body_b) {
    if (!constraint.has_limits && constraint.motor_state == motor_settings::OFF && constraint.max_friction_torque <= 0.0f) {
        return;
    }

    const Quatf rotation_a = quatf::load(body_a.quaternion);
    constraint.world_space_hinge_axis1 = vec3f::transform_quat(constraint.local_space_hinge_axis1, rotation_a);

    // relative rotation in world space: diff = q2 * inv_initial_orientation * q1^-1
    const Quatf diff =
        quatf::multiply(quatf::multiply(quatf::load(body_b.quaternion), constraint.inv_initial_orientation), quatf::conjugate(rotation_a));

    // rotation angle about the hinge axis: 2 atan(dot(axis, xyz) / w); atan, not atan2, for the [-pi, pi] range
    const float axis_dot = vec3f::dot(constraint.world_space_hinge_axis1, quatf::xyz(diff));
    const float w = quatf::w(diff);
    constraint.theta = w == 0.0f ? scalar::PI : 2.0f * scalar::atan(axis_dot / w);
}

/** wrap to [-pi, pi] */
static float center_angle_around_zero(float angle) {
    while (angle > scalar::PI) angle -= 2.0f * scalar::PI;
    while (angle < -scalar::PI) angle += 2.0f * scalar::PI;
    return angle;
}

/** the signed angle to the closest limit */
static float get_smallest_angle_to_limit(const HingeConstraint &constraint) {
    const float dist_to_min = center_angle_around_zero(constraint.theta - constraint.limits_min);
    const float dist_to_max = center_angle_around_zero(constraint.theta - constraint.limits_max);
    return scalar::abs(dist_to_min) < scalar::abs(dist_to_max) ? dist_to_min : dist_to_max;
}

static bool is_min_limit_closest(const HingeConstraint &constraint) {
    const float dist_to_min = center_angle_around_zero(constraint.theta - constraint.limits_min);
    const float dist_to_max = center_angle_around_zero(constraint.theta - constraint.limits_max);
    return scalar::abs(dist_to_min) < scalar::abs(dist_to_max);
}

static void calculate_rotation_limits_constraint_properties(HingeConstraint &constraint, RigidBody &body_a, RigidBody &body_b,
                                                            float delta_time) {
    // active at or beyond the limits
    if (constraint.has_limits && (constraint.theta <= constraint.limits_min || constraint.theta >= constraint.limits_max)) {
        angle_constraint_part::calculate_constraint_properties_with_settings(
            constraint.rotation_limits_constraint_part, delta_time, body_a, body_b, constraint.world_space_hinge_axis1, 0.0f,
            get_smallest_angle_to_limit(constraint), constraint.limits_spring_settings);
    } else {
        angle_constraint_part::deactivate(constraint.rotation_limits_constraint_part);
    }
}

static void calculate_motor_constraint_properties(HingeConstraint &constraint, RigidBody &body_a, RigidBody &body_b,
                                                  float delta_time) {
    switch (constraint.motor_state) {
        case motor_settings::OFF:
            if (constraint.max_friction_torque > 0.0f) {
                angle_constraint_part::calculate_constraint_properties(constraint.motor_constraint_part, body_a, body_b,
                                                                       constraint.world_space_hinge_axis1, 0.0f);
            } else {
                angle_constraint_part::deactivate(constraint.motor_constraint_part);
            }
            break;

        case motor_settings::VELOCITY:
            angle_constraint_part::calculate_constraint_properties(constraint.motor_constraint_part, body_a, body_b,
                                                                   constraint.world_space_hinge_axis1,
                                                                   -constraint.target_angular_velocity);
            break;

        case motor_settings::POSITION:
            if (spring_settings::has_stiffness(constraint.motor_settings.spring)) {
                angle_constraint_part::calculate_constraint_properties_with_settings_for_motor(
                    constraint.motor_constraint_part, delta_time, body_a, body_b, constraint.world_space_hinge_axis1, 0.0f,
                    center_angle_around_zero(constraint.theta - constraint.target_angle), constraint.motor_settings.spring);
            } else {
                angle_constraint_part::deactivate(constraint.motor_constraint_part);
            }
            break;

        case motor_settings::POSITION_AND_VELOCITY:
            if (spring_settings::has_stiffness_or_damping(constraint.motor_settings.spring)) {
                angle_constraint_part::calculate_constraint_properties_with_settings_for_motor(
                    constraint.motor_constraint_part, delta_time, body_a, body_b, constraint.world_space_hinge_axis1,
                    -constraint.target_angular_velocity, center_angle_around_zero(constraint.theta - constraint.target_angle),
                    constraint.motor_settings.spring);
            } else {
                angle_constraint_part::deactivate(constraint.motor_constraint_part);
            }
            break;
    }
}

void setup_velocity(HingeConstraint &constraint, bodies::Bodies &bodies, float delta_time) {
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    const Mat4f rotation_a = mat4f::from_quat(quatf::load(body_a.quaternion));
    const Mat4f rotation_b = mat4f::from_quat(quatf::load(body_b.quaternion));

    const Vec3f world_hinge_axis1 = vec3f::transform_quat(constraint.local_space_hinge_axis1, quatf::load(body_a.quaternion));
    const Vec3f world_hinge_axis2 = vec3f::transform_quat(constraint.local_space_hinge_axis2, quatf::load(body_b.quaternion));

    point_constraint_part::calculate_constraint_properties(constraint.point_constraint_part, body_a, rotation_a,
                                                           constraint.local_space_position1, body_b, rotation_b,
                                                           constraint.local_space_position2, bodies.step_stamp);
    hinge_rotation_constraint_part::calculate_constraint_properties(constraint.rotation_constraint_part, body_a, body_b,
                                                                    world_hinge_axis1, world_hinge_axis2, bodies.step_stamp);
    calculate_a1_and_theta(constraint, body_a, body_b);
    calculate_rotation_limits_constraint_properties(constraint, body_a, body_b, delta_time);
    calculate_motor_constraint_properties(constraint, body_a, body_b, delta_time);
}

void warm_start_velocity(HingeConstraint &constraint, bodies::Bodies &bodies, float warm_start_impulse_ratio) {
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    angle_constraint_part::warm_start(constraint.motor_constraint_part, body_a, body_b, warm_start_impulse_ratio);
    point_constraint_part::warm_start(constraint.point_constraint_part, body_a, body_b, warm_start_impulse_ratio);
    hinge_rotation_constraint_part::warm_start(constraint.rotation_constraint_part, body_a, body_b, warm_start_impulse_ratio);
    angle_constraint_part::warm_start(constraint.rotation_limits_constraint_part, body_a, body_b, warm_start_impulse_ratio);
}

bool solve_velocity(HingeConstraint &constraint, bodies::Bodies &bodies, float delta_time) {
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return false;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    bool motor = false;
    if (angle_constraint_part::is_active(constraint.motor_constraint_part)) {
        switch (constraint.motor_state) {
            case motor_settings::OFF: {
                const float max_lambda = constraint.max_friction_torque * delta_time;
                motor = angle_constraint_part::solve_velocity_constraint(constraint.motor_constraint_part, body_a, body_b,
                                                                         constraint.world_space_hinge_axis1, -max_lambda,
                                                                         max_lambda);
                break;
            }
            case motor_settings::VELOCITY:
            case motor_settings::POSITION:
            case motor_settings::POSITION_AND_VELOCITY:
                motor = angle_constraint_part::solve_velocity_constraint(
                    constraint.motor_constraint_part, body_a, body_b, constraint.world_space_hinge_axis1,
                    delta_time * constraint.motor_settings.min_torque_limit, delta_time * constraint.motor_settings.max_torque_limit);
                break;
        }
    }

    const bool pos = point_constraint_part::solve_velocity_constraint(constraint.point_constraint_part, body_a, body_b);
    const bool rot = hinge_rotation_constraint_part::solve_velocity_constraint(constraint.rotation_constraint_part, body_a, body_b);

    bool limit = false;
    if (angle_constraint_part::is_active(constraint.rotation_limits_constraint_part)) {
        float min_lambda;
        float max_lambda;
        if (constraint.limits_min == constraint.limits_max) {
            min_lambda = -scalar::FLOAT_MAX;
            max_lambda = scalar::FLOAT_MAX;
        } else if (is_min_limit_closest(constraint)) {
            min_lambda = 0.0f;
            max_lambda = scalar::FLOAT_MAX;
        } else {
            min_lambda = -scalar::FLOAT_MAX;
            max_lambda = 0.0f;
        }
        limit = angle_constraint_part::solve_velocity_constraint(constraint.rotation_limits_constraint_part, body_a, body_b,
                                                                 constraint.world_space_hinge_axis1, min_lambda, max_lambda);
    }

    return motor || pos || rot || limit;
}

bool solve_position(HingeConstraint &constraint, bodies::Bodies &bodies, float delta_time, float baumgarte) {
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return false;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    // the motor works on velocities only

    const Mat4f rotation_a = mat4f::from_quat(quatf::load(body_a.quaternion));
    const Mat4f rotation_b = mat4f::from_quat(quatf::load(body_b.quaternion));
    point_constraint_part::calculate_constraint_properties(constraint.point_constraint_part, body_a, rotation_a,
                                                           constraint.local_space_position1, body_b, rotation_b,
                                                           constraint.local_space_position2, motion_properties::STEP_STAMP_NONE);
    const bool pos = point_constraint_part::solve_position_constraint(constraint.point_constraint_part, body_a, body_b, baumgarte);

    // the bodies have moved: recalculate the axes
    const Vec3f world_hinge_axis1 = vec3f::transform_quat(constraint.local_space_hinge_axis1, quatf::load(body_a.quaternion));
    const Vec3f world_hinge_axis2 = vec3f::transform_quat(constraint.local_space_hinge_axis2, quatf::load(body_b.quaternion));
    hinge_rotation_constraint_part::calculate_constraint_properties(constraint.rotation_constraint_part, body_a, body_b,
                                                                    world_hinge_axis1, world_hinge_axis2,
                                                                    motion_properties::STEP_STAMP_NONE);
    const bool rot =
        hinge_rotation_constraint_part::solve_position_constraint(constraint.rotation_constraint_part, body_a, body_b, baumgarte);

    // soft limits have no position solve
    bool limit = false;
    if (constraint.has_limits && constraint.limits_spring_settings.frequency_or_stiffness <= 0.0f) {
        calculate_a1_and_theta(constraint, body_a, body_b);
        calculate_rotation_limits_constraint_properties(constraint, body_a, body_b, delta_time);
        if (angle_constraint_part::is_active(constraint.rotation_limits_constraint_part)) {
            limit = angle_constraint_part::solve_position_constraint(constraint.rotation_limits_constraint_part, body_a, body_b,
                                                                     get_smallest_angle_to_limit(constraint), baumgarte);
        }
    }

    return pos || rot || limit;
}

void notify_shape_changed(HingeConstraint &constraint, body_id::BodyId body_id, Vec3f delta_com) {
    // separate tests, not an else-if: a constraint between a body and itself moves both points
    const uint32_t body_index = body_id::index(body_id);
    if (constraint.base.body_index_a == body_index)
        constraint.local_space_position1 = vec3f::subtract(constraint.local_space_position1, delta_com);
    if (constraint.base.body_index_b == body_index)
        constraint.local_space_position2 = vec3f::subtract(constraint.local_space_position2, delta_com);
}

void reset_warm_start(HingeConstraint &constraint) {
    angle_constraint_part::deactivate(constraint.motor_constraint_part);
    point_constraint_part::deactivate(constraint.point_constraint_part);
    hinge_rotation_constraint_part::deactivate(constraint.rotation_constraint_part);
    angle_constraint_part::deactivate(constraint.rotation_limits_constraint_part);
}

void get_iteration_overrides(constraints::ConstraintIterationOverrides &out, const HingeConstraint &constraint) {
    out.velocity = constraint.base.num_velocity_steps_override;
    out.position = constraint.base.num_position_steps_override;
}

void get_sort_fields(constraints::ConstraintSortFields &out, const HingeConstraint &constraint) {
    out.priority = constraint.base.constraint_priority;
    out.index = constraint.base.index;
}

}  // namespace hinge_constraint

TUMBLE_NAMESPACE_END
