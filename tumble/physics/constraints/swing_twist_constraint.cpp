#include "physics/constraints/swing_twist_constraint.h"

#include "math/mat4f.h"
#include "math/quatf.h"
#include "math/vec3f.h"
#include "math/vec4f.h"
#include "physics/body/motion_properties.h"
#include "physics/body/rigid_body.h"
#include "util/assert.h"

namespace swing_twist_constraint {

using constraint_id::ConstraintId;
using motor_settings::MotorState;
using rigid_body::RigidBody;

static bool is_body_live(const bodies::Bodies &bodies, uint32_t index) {
    return index < bodies.pool.length && bodies.pool[index] != nullptr && bodies.sequences[index] != bodies::SLOT_FREE;
}

/** creates default swing twist constraint */
static SwingTwistConstraint make_swing_twist_constraint() {
    SwingTwistConstraint constraint;
    constraint.base = constraints::make_constraint_base();
    constraint.local_space_position1 = vec3f::zero();
    constraint.local_space_position2 = vec3f::zero();
    constraint.constraint_to_body1 = quatf::identity();
    constraint.constraint_to_body2 = quatf::identity();
    constraint.normal_half_cone_angle = 0.0f;
    constraint.plane_half_cone_angle = 0.0f;
    constraint.twist_min_angle = 0.0f;
    constraint.twist_max_angle = 0.0f;
    constraint.max_friction_torque = 0.0f;
    constraint.swing_motor_state = motor_settings::OFF;
    constraint.twist_motor_state = motor_settings::OFF;
    constraint.target_angular_velocity = vec3f::zero();
    constraint.target_orientation = quatf::identity();
    constraint.swing_motor_settings = motor_settings::create();
    constraint.twist_motor_settings = motor_settings::create();
    for (uint32_t i = 0; i < 3; i++) constraint.world_space_motor_axis[i] = vec3f::zero();
    constraint.point_constraint_part = point_constraint_part::create();
    constraint.swing_twist_constraint_part = swing_twist_constraint_part::create();
    for (uint32_t i = 0; i < 3; i++) constraint.motor_constraint_parts[i] = angle_constraint_part::create();
    return constraint;
}

/** reset constraint state for pooling reuse */
static void reset_constraint(SwingTwistConstraint &constraint) {
    constraint.base.enabled = true;
    constraint.base.sleeping = false;
    constraint.base.pooled = false;
    point_constraint_part::deactivate(constraint.point_constraint_part);
    swing_twist_constraint_part::deactivate(constraint.swing_twist_constraint_part);
    for (uint32_t i = 0; i < 3; i++) angle_constraint_part::deactivate(constraint.motor_constraint_parts[i]);
}

/** passes the limits on to the swing twist part */
static void update_limits(SwingTwistConstraint &constraint) {
    swing_twist_constraint_part::set_limits(constraint.swing_twist_constraint_part, constraint.twist_min_angle,
                                            constraint.twist_max_angle, -constraint.plane_half_cone_angle,
                                            constraint.plane_half_cone_angle, -constraint.normal_half_cone_angle,
                                            constraint.normal_half_cone_angle);
}

/** the rotation from constraint space (x = twist, y = plane x twist, z = plane) to body space */
static Quatf constraint_to_body(Vec3f twist_axis, Vec3f plane_axis) {
    const Vec3f normal_axis = vec3f::cross(plane_axis, twist_axis);
    return quatf::from_mat4(mat4f::make_columns({twist_axis.v}, {normal_axis.v}, {plane_axis.v}, vec4f::make(0.0f, 0.0f, 0.0f, 1.0f)));
}

ConstraintId create(constraints::Constraints &constraints, bodies::Bodies &bodies, RigidBody *body_a, RigidBody *body_b,
                    const SwingTwistConstraintSettings &settings) {
    CC_ASSERT(is_body_live(bodies, body_a->index) && bodies.pool[body_a->index] == body_a);
    CC_ASSERT(is_body_live(bodies, body_b->index) && bodies.pool[body_b->index] == body_b);
    constraints::ConstraintPool &pool = constraints::ensure_pool(constraints, constraint_id::SWING_TWIST);

    const uint32_t index = constraints::allocate_index(pool);
    if (index == ~0u) return constraint_id::INVALID_CONSTRAINT_ID;
    const uint32_t sequence = constraints::next_sequence(pool, index);
    SwingTwistConstraint &constraint = *reinterpret_cast<SwingTwistConstraint *>(constraints::base_at(pool, index));
    constraint = make_swing_twist_constraint();
    reset_constraint(constraint);

    constraint.base.id = constraint_id::ser(index, constraint_id::SWING_TWIST, sequence);
    constraint.base.index = index;
    constraint.base.sequence = sequence;

    constraint.base.constraint_priority = settings.constraint_priority;
    constraint.base.num_velocity_steps_override = settings.num_velocity_steps_override;
    constraint.base.num_position_steps_override = settings.num_position_steps_override;

    constraint.base.body_index_a = body_a->index;
    constraint.base.body_index_b = body_b->index;

    // normalize axes
    const Vec3f twist_axis1 = vec3f::normalize(vec3f::load(settings.twist_axis1));
    const Vec3f twist_axis2 = vec3f::normalize(vec3f::load(settings.twist_axis2));
    const Vec3f plane_axis1 = vec3f::normalize(vec3f::load(settings.plane_axis1));
    const Vec3f plane_axis2 = vec3f::normalize(vec3f::load(settings.plane_axis2));

    constraint.constraint_to_body1 = constraint_to_body(twist_axis1, plane_axis1);
    constraint.constraint_to_body2 = constraint_to_body(twist_axis2, plane_axis2);

    if (settings.space == constraints::SPACE_WORLD) {
        const Quatf inv_quat_a = quatf::conjugate(quatf::load(body_a->quaternion));
        const Quatf inv_quat_b = quatf::conjugate(quatf::load(body_b->quaternion));

        constraint.local_space_position1 = vec3f::transform_quat(vec3f::subtract(vec3f::load(settings.position1), vec3f::load(body_a->center_of_mass_position)), inv_quat_a);

        constraint.local_space_position2 = vec3f::transform_quat(vec3f::subtract(vec3f::load(settings.position2), vec3f::load(body_b->center_of_mass_position)), inv_quat_b);

        constraint.constraint_to_body1 = quatf::multiply(inv_quat_a, constraint.constraint_to_body1);
        constraint.constraint_to_body2 = quatf::multiply(inv_quat_b, constraint.constraint_to_body2);
    } else {
        constraint.local_space_position1 = vec3f::load(settings.position1);
        constraint.local_space_position2 = vec3f::load(settings.position2);
    }

    constraint.normal_half_cone_angle = settings.normal_half_cone_angle;
    constraint.plane_half_cone_angle = settings.plane_half_cone_angle;
    constraint.twist_min_angle = settings.twist_min_angle;
    constraint.twist_max_angle = settings.twist_max_angle;

    constraint.swing_twist_constraint_part.swing_type = settings.swing_type;
    update_limits(constraint);

    constraint.max_friction_torque = settings.max_friction_torque;

    constraint.swing_motor_state = motor_settings::OFF;
    constraint.twist_motor_state = motor_settings::OFF;
    constraint.target_angular_velocity = vec3f::zero();
    constraint.target_orientation = quatf::identity();
    constraint.swing_motor_settings = settings.swing_motor_settings;
    constraint.twist_motor_settings = settings.twist_motor_settings;

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

void remove(constraints::Constraints &constraints, bodies::Bodies &bodies, SwingTwistConstraint &constraint) {
    constraints::ConstraintPool &pool = constraints::ensure_pool(constraints, constraint_id::SWING_TWIST);

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

SwingTwistConstraint *get(constraints::Constraints &constraints, ConstraintId id) {
    if (constraint_id::type(id) != constraint_id::SWING_TWIST) return nullptr;
    return reinterpret_cast<SwingTwistConstraint *>(constraints::get(constraints, id));
}

void set_normal_half_cone_angle(SwingTwistConstraint &constraint, float angle) {
    constraint.normal_half_cone_angle = angle;
    update_limits(constraint);
}

void set_plane_half_cone_angle(SwingTwistConstraint &constraint, float angle) {
    constraint.plane_half_cone_angle = angle;
    update_limits(constraint);
}

void set_twist_min_angle(SwingTwistConstraint &constraint, float angle) {
    constraint.twist_min_angle = angle;
    update_limits(constraint);
}

void set_twist_max_angle(SwingTwistConstraint &constraint, float angle) {
    constraint.twist_max_angle = angle;
    update_limits(constraint);
}

void set_max_friction_torque(SwingTwistConstraint &constraint, float torque) { constraint.max_friction_torque = torque; }

void set_swing_motor_state(SwingTwistConstraint &constraint, MotorState state) {
    CC_ASSERT(state == motor_settings::OFF || motor_settings::is_valid(constraint.swing_motor_settings));
    if (constraint.swing_motor_state != state) {
        constraint.swing_motor_state = state;
        // the motor parts are repurposed between modes: no warm start into the new one
        for (uint32_t i = 0; i < 3; i++) angle_constraint_part::deactivate(constraint.motor_constraint_parts[i]);
    }
}

void set_twist_motor_state(SwingTwistConstraint &constraint, MotorState state) {
    CC_ASSERT(state == motor_settings::OFF || motor_settings::is_valid(constraint.twist_motor_settings));
    if (constraint.twist_motor_state != state) {
        constraint.twist_motor_state = state;
        angle_constraint_part::deactivate(constraint.motor_constraint_parts[0]);
    }
}

void set_target_angular_velocity_cs(SwingTwistConstraint &constraint, Vec3f velocity) { constraint.target_angular_velocity = velocity; }

void set_target_orientation_cs(SwingTwistConstraint &constraint, Quatf orientation) {
    // clamp to the limits
    Quatf q_swing;
    Quatf q_twist;
    swing_twist_constraint_part::get_swing_twist(q_swing, q_twist, orientation);
    const uint32_t clamped_axis = swing_twist_constraint_part::clamp_swing_twist(constraint.swing_twist_constraint_part, q_swing, q_twist);

    constraint.target_orientation = clamped_axis != 0 ? quatf::multiply(q_swing, q_twist) : orientation;
}

void set_target_orientation_bs(SwingTwistConstraint &constraint, Quatf orientation) {
    // body space to constraint space: c1^-1 * orientation * c2
    set_target_orientation_cs(
        constraint, quatf::multiply(quatf::multiply(quatf::conjugate(constraint.constraint_to_body1), orientation), constraint.constraint_to_body2));
}

Quatf get_rotation_in_constraint_space(const SwingTwistConstraint &constraint, const bodies::Bodies &bodies) {
    const RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    const RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    // q = (q1 * c1)^-1 * (q2 * c2)
    const Quatf q1 = quatf::multiply(quatf::load(body_a.quaternion), constraint.constraint_to_body1);
    const Quatf q2 = quatf::multiply(quatf::load(body_b.quaternion), constraint.constraint_to_body2);
    return quatf::multiply(quatf::conjugate(q1), q2);
}

void setup_velocity(SwingTwistConstraint &constraint, bodies::Bodies &bodies, float delta_time) {
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    const Mat4f rotation_a = mat4f::from_quat(quatf::load(body_a.quaternion));
    const Mat4f rotation_b = mat4f::from_quat(quatf::load(body_b.quaternion));
    point_constraint_part::calculate_constraint_properties(constraint.point_constraint_part, body_a, rotation_a,
                                                           constraint.local_space_position1, body_b, rotation_b,
                                                           constraint.local_space_position2, bodies.step_stamp);

    // the constraint rotation, q = (q1 * c1)^-1 * (q2 * c2), keeping the sub expressions
    const Quatf constraint_body1_to_world = quatf::multiply(quatf::load(body_a.quaternion), constraint.constraint_to_body1);
    const Quatf constraint_body2_to_world = quatf::multiply(quatf::load(body_b.quaternion), constraint.constraint_to_body2);
    const Quatf q = quatf::multiply(quatf::conjugate(constraint_body1_to_world), constraint_body2_to_world);

    swing_twist_constraint_part::calculate_constraint_properties(constraint.swing_twist_constraint_part, body_a, body_b, q,
                                                                 constraint_body1_to_world);

    if (constraint.swing_motor_state != motor_settings::OFF || constraint.twist_motor_state != motor_settings::OFF ||
        constraint.max_friction_torque > 0.0f) {
        // motor axes: the world space axes of body 2's constraint space
        const Mat4f ws_axis = mat4f::from_quat(constraint_body2_to_world);
        for (int i = 0; i < 3; i++) constraint.world_space_motor_axis[i] = mat4f::axis(ws_axis, i);

        Vec3f rotation_error = vec3f::zero();
        if (motor_settings::is_position_motor(constraint.swing_motor_state) ||
            motor_settings::is_position_motor(constraint.twist_motor_state)) {
            // the target orientation along the shortest path from q
            Quatf target_orientation = constraint.target_orientation;
            if (quatf::dot(q, target_orientation) < 0.0f) target_orientation = quatf::scale(target_orientation, -1.0f);

            // target_orientation = q * diff, so diff = q^-1 * target_orientation
            const Quatf diff = quatf::multiply(quatf::conjugate(q), target_orientation);

            // the imaginary part is axis * sin(angle / 2), so for small angles angle ~ 2 * axis; for large ones
            // at least the sign is right
            rotation_error = vec3f::scale(quatf::xyz(diff), -2.0f);
        }
        const float rotation_errors[3] = {vec3f::x(rotation_error), vec3f::y(rotation_error), vec3f::z(rotation_error)};
        const float target_angular_velocity[3] = {vec3f::x(constraint.target_angular_velocity), vec3f::y(constraint.target_angular_velocity),
                                                  vec3f::z(constraint.target_angular_velocity)};

        // swing motor
        switch (constraint.swing_motor_state) {
            case motor_settings::OFF:
                if (constraint.max_friction_torque > 0.0f) {
                    for (uint32_t i = 1; i < 3; i++) {
                        angle_constraint_part::calculate_constraint_properties(
                            constraint.motor_constraint_parts[i], body_a, body_b, constraint.world_space_motor_axis[i], 0.0f);
                    }
                } else {
                    for (uint32_t i = 0; i < 3; i++) angle_constraint_part::deactivate(constraint.motor_constraint_parts[i]);
                }
                break;

            case motor_settings::VELOCITY:
                for (uint32_t i = 1; i < 3; i++) {
                    angle_constraint_part::calculate_constraint_properties(constraint.motor_constraint_parts[i], body_a, body_b,
                                                                           constraint.world_space_motor_axis[i],
                                                                           -target_angular_velocity[i]);
                }
                break;

            case motor_settings::POSITION:
                if (spring_settings::has_stiffness(constraint.swing_motor_settings.spring)) {
                    for (uint32_t i = 1; i < 3; i++) {
                        angle_constraint_part::calculate_constraint_properties_with_settings_for_motor(
                            constraint.motor_constraint_parts[i], delta_time, body_a, body_b, constraint.world_space_motor_axis[i],
                            0.0f, rotation_errors[i], constraint.swing_motor_settings.spring);
                    }
                } else {
                    for (uint32_t i = 1; i < 3; i++) angle_constraint_part::deactivate(constraint.motor_constraint_parts[i]);
                }
                break;

            case motor_settings::POSITION_AND_VELOCITY:
                if (spring_settings::has_stiffness_or_damping(constraint.swing_motor_settings.spring)) {
                    for (uint32_t i = 1; i < 3; i++) {
                        angle_constraint_part::calculate_constraint_properties_with_settings_for_motor(
                            constraint.motor_constraint_parts[i], delta_time, body_a, body_b, constraint.world_space_motor_axis[i],
                            -target_angular_velocity[i], rotation_errors[i], constraint.swing_motor_settings.spring);
                    }
                } else {
                    for (uint32_t i = 1; i < 3; i++) angle_constraint_part::deactivate(constraint.motor_constraint_parts[i]);
                }
                break;
        }

        // twist motor
        switch (constraint.twist_motor_state) {
            case motor_settings::OFF:
                if (constraint.max_friction_torque > 0.0f) {
                    angle_constraint_part::calculate_constraint_properties(constraint.motor_constraint_parts[0], body_a, body_b,
                                                                           constraint.world_space_motor_axis[0], 0.0f);
                } else {
                    angle_constraint_part::deactivate(constraint.motor_constraint_parts[0]);
                }
                break;

            case motor_settings::VELOCITY:
                angle_constraint_part::calculate_constraint_properties(constraint.motor_constraint_parts[0], body_a, body_b,
                                                                       constraint.world_space_motor_axis[0],
                                                                       -target_angular_velocity[0]);
                break;

            case motor_settings::POSITION:
                if (spring_settings::has_stiffness(constraint.twist_motor_settings.spring)) {
                    angle_constraint_part::calculate_constraint_properties_with_settings_for_motor(
                        constraint.motor_constraint_parts[0], delta_time, body_a, body_b, constraint.world_space_motor_axis[0],
                        0.0f, rotation_errors[0], constraint.twist_motor_settings.spring);
                } else {
                    angle_constraint_part::deactivate(constraint.motor_constraint_parts[0]);
                }
                break;

            case motor_settings::POSITION_AND_VELOCITY:
                if (spring_settings::has_stiffness_or_damping(constraint.twist_motor_settings.spring)) {
                    angle_constraint_part::calculate_constraint_properties_with_settings_for_motor(
                        constraint.motor_constraint_parts[0], delta_time, body_a, body_b, constraint.world_space_motor_axis[0],
                        -target_angular_velocity[0], rotation_errors[0], constraint.twist_motor_settings.spring);
                } else {
                    angle_constraint_part::deactivate(constraint.motor_constraint_parts[0]);
                }
                break;
        }
    } else {
        for (uint32_t i = 0; i < 3; i++) angle_constraint_part::deactivate(constraint.motor_constraint_parts[i]);
    }
}

void warm_start_velocity(SwingTwistConstraint &constraint, bodies::Bodies &bodies, float warm_start_impulse_ratio) {
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    for (uint32_t i = 0; i < 3; i++) {
        angle_constraint_part::warm_start(constraint.motor_constraint_parts[i], body_a, body_b, warm_start_impulse_ratio);
    }
    swing_twist_constraint_part::warm_start(constraint.swing_twist_constraint_part, body_a, body_b, warm_start_impulse_ratio);
    point_constraint_part::warm_start(constraint.point_constraint_part, body_a, body_b, warm_start_impulse_ratio);
}

bool solve_velocity(SwingTwistConstraint &constraint, bodies::Bodies &bodies, float delta_time) {
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return false;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    bool impulse = false;

    // twist motor
    if (angle_constraint_part::is_active(constraint.motor_constraint_parts[0])) {
        float min_limit;
        float max_limit;
        if (constraint.twist_motor_state == motor_settings::OFF) {
            max_limit = delta_time * constraint.max_friction_torque;
            min_limit = -max_limit;
        } else {
            min_limit = delta_time * constraint.twist_motor_settings.min_torque_limit;
            max_limit = delta_time * constraint.twist_motor_settings.max_torque_limit;
        }
        impulse = angle_constraint_part::solve_velocity_constraint(constraint.motor_constraint_parts[0], body_a, body_b,
                                                                   constraint.world_space_motor_axis[0], min_limit, max_limit) ||
                  impulse;
    }

    // swing motors, on and off together
    if (angle_constraint_part::is_active(constraint.motor_constraint_parts[1])) {
        float min_limit;
        float max_limit;
        if (constraint.swing_motor_state == motor_settings::OFF) {
            max_limit = delta_time * constraint.max_friction_torque;
            min_limit = -max_limit;
        } else {
            min_limit = delta_time * constraint.swing_motor_settings.min_torque_limit;
            max_limit = delta_time * constraint.swing_motor_settings.max_torque_limit;
        }
        for (uint32_t i = 1; i < 3; i++) {
            impulse = angle_constraint_part::solve_velocity_constraint(constraint.motor_constraint_parts[i], body_a, body_b,
                                                                       constraint.world_space_motor_axis[i], min_limit,
                                                                       max_limit) ||
                      impulse;
        }
    }

    impulse = swing_twist_constraint_part::solve_velocity_constraint(constraint.swing_twist_constraint_part, body_a, body_b) ||
              impulse;
    impulse = point_constraint_part::solve_velocity_constraint(constraint.point_constraint_part, body_a, body_b) || impulse;

    return impulse;
}

bool solve_position(SwingTwistConstraint &constraint, bodies::Bodies &bodies, float delta_time, float baumgarte) {
    (void)delta_time;
    if (!is_body_live(bodies, constraint.base.body_index_a) || !is_body_live(bodies, constraint.base.body_index_b)) return false;
    RigidBody &body_a = *bodies.pool[constraint.base.body_index_a];
    RigidBody &body_b = *bodies.pool[constraint.base.body_index_b];

    bool impulse = false;

    // rotation violations
    const Quatf q = get_rotation_in_constraint_space(constraint, bodies);
    impulse = swing_twist_constraint_part::solve_position_constraint(constraint.swing_twist_constraint_part, body_a, body_b, q,
                                                                     constraint.constraint_to_body1,
                                                                     constraint.constraint_to_body2, baumgarte) ||
              impulse;

    // position violations
    const Mat4f rotation_a = mat4f::from_quat(quatf::load(body_a.quaternion));
    const Mat4f rotation_b = mat4f::from_quat(quatf::load(body_b.quaternion));
    point_constraint_part::calculate_constraint_properties(constraint.point_constraint_part, body_a, rotation_a,
                                                           constraint.local_space_position1, body_b, rotation_b,
                                                           constraint.local_space_position2, motion_properties::STEP_STAMP_NONE);
    impulse = point_constraint_part::solve_position_constraint(constraint.point_constraint_part, body_a, body_b, baumgarte) || impulse;

    return impulse;
}

void notify_shape_changed(SwingTwistConstraint &constraint, body_id::BodyId body_id, Vec3f delta_com) {
    // separate tests, not jolt's else-if: a constraint between a body and itself moves both points
    const uint32_t body_index = body_id::index(body_id);
    if (constraint.base.body_index_a == body_index)
        constraint.local_space_position1 = vec3f::subtract(constraint.local_space_position1, delta_com);
    if (constraint.base.body_index_b == body_index)
        constraint.local_space_position2 = vec3f::subtract(constraint.local_space_position2, delta_com);
}

void reset_warm_start(SwingTwistConstraint &constraint) {
    for (uint32_t i = 0; i < 3; i++) angle_constraint_part::deactivate(constraint.motor_constraint_parts[i]);
    swing_twist_constraint_part::deactivate(constraint.swing_twist_constraint_part);
    point_constraint_part::deactivate(constraint.point_constraint_part);
}

void get_iteration_overrides(constraints::ConstraintIterationOverrides &out, const SwingTwistConstraint &constraint) {
    out.velocity = constraint.base.num_velocity_steps_override;
    out.position = constraint.base.num_position_steps_override;
}

void get_sort_fields(constraints::ConstraintSortFields &out, const SwingTwistConstraint &constraint) {
    out.priority = constraint.base.constraint_priority;
    out.index = constraint.base.index;
}

Vec3f get_total_lambda_position(const SwingTwistConstraint &constraint) { return constraint.point_constraint_part.total_lambda; }

float get_total_lambda_twist(const SwingTwistConstraint &constraint) {
    return swing_twist_constraint_part::get_total_twist_lambda(constraint.swing_twist_constraint_part);
}

float get_total_lambda_swing_y(const SwingTwistConstraint &constraint) {
    return swing_twist_constraint_part::get_total_swing_y_lambda(constraint.swing_twist_constraint_part);
}

float get_total_lambda_swing_z(const SwingTwistConstraint &constraint) {
    return swing_twist_constraint_part::get_total_swing_z_lambda(constraint.swing_twist_constraint_part);
}

Vec3f get_total_lambda_motor(const SwingTwistConstraint &constraint) {
    return vec3f::make(constraint.motor_constraint_parts[0].total_lambda, constraint.motor_constraint_parts[1].total_lambda,
                       constraint.motor_constraint_parts[2].total_lambda);
}

}  // namespace swing_twist_constraint
