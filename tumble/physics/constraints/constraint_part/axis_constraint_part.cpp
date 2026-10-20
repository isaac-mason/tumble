#include "tumble/physics/constraints/constraint_part/axis_constraint_part.h"

#include "tumble/math/mat4f.h"
#include "tumble/math/scalar.h"
#include "tumble/physics/body/motion_properties.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/body/rigid_body_step.h"

TUMBLE_NAMESPACE_BEGIN

namespace axis_constraint_part {

/** inverse effective mass (cached jacobian terms) */
static float calculate_inverse_effective_mass(AxisConstraintPart &part, rigid_body::RigidBody &body_a,
                                              rigid_body::RigidBody &body_b, float inv_mass_a, float inv_mass_b,
                                              const Mat4f &inv_inertia_a, const Mat4f &inv_inertia_b,
                                              Vec3f r1_plus_u, Vec3f r2, Vec3f axis) {
    float inv_effective_mass = 0.0f;

    // the cross products are needed for every moving body (a kinematic body's angular velocity enters
    // the jacobian), the inverse inertia terms only for dynamic ones
    if (body_a.motion_type != motion_type::MOTION_STATIC) {
        // r1 x axis
        const Vec3f r1_plus_u_x_axis = vec3f::cross(r1_plus_u, axis);
        vec3f::store(part.r1_plus_u_x_axis, r1_plus_u_x_axis);

        if (body_a.motion_type == motion_type::MOTION_DYNAMIC) {
            // I1^-1 x (r1 x axis)
            const Vec3f inv_i1_r1_plus_u_x_axis = mat4f::multiply3x3_vec(inv_inertia_a, r1_plus_u_x_axis);
            vec3f::store(part.inv_i1_r1_plus_u_x_axis, inv_i1_r1_plus_u_x_axis);
            // inv_mass + (r x axis) . (I^-1 x (r x axis))
            inv_effective_mass += inv_mass_a + vec3f::dot(inv_i1_r1_plus_u_x_axis, r1_plus_u_x_axis);
        }
    }

    if (body_b.motion_type != motion_type::MOTION_STATIC) {
        // r2 x axis
        const Vec3f r2_x_axis = vec3f::cross(r2, axis);
        vec3f::store(part.r2_x_axis, r2_x_axis);

        if (body_b.motion_type == motion_type::MOTION_DYNAMIC) {
            // I2^-1 x (r2 x axis)
            const Vec3f inv_i2_r2_x_axis = mat4f::multiply3x3_vec(inv_inertia_b, r2_x_axis);
            vec3f::store(part.inv_i2_r2_x_axis, inv_i2_r2_x_axis);
            // inv_mass + (r x axis) . (I^-1 x (r x axis))
            inv_effective_mass += inv_mass_b + vec3f::dot(inv_i2_r2_x_axis, r2_x_axis);
        }
    }

    return inv_effective_mass;
}

void calculate_constraint_properties(AxisConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                                     float inv_mass_a, float inv_mass_b, const Mat4f &inv_inertia_a,
                                     const Mat4f &inv_inertia_b, Vec3f r1_plus_u, Vec3f r2, Vec3f axis,
                                     float bias) {
    const float inv_effective_mass = calculate_inverse_effective_mass(part, body_a, body_b, inv_mass_a, inv_mass_b,
                                                                      inv_inertia_a, inv_inertia_b, r1_plus_u, r2, axis);

    if (inv_effective_mass < scalar::FLOAT_MIN_NORMAL) {
        deactivate(part);
    } else {
        part.effective_mass = 1.0f / inv_effective_mass;
        spring_part::calculate_spring_properties_with_bias(part.spring, bias);
    }
}

void calculate_constraint_properties_with_frequency_and_damping(AxisConstraintPart &part, float delta_time,
                                                                rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                                                                float inv_mass_a, float inv_mass_b, const Mat4f &inv_inertia_a,
                                                                const Mat4f &inv_inertia_b, Vec3f r1_plus_u,
                                                                Vec3f r2, Vec3f axis, float bias, float c,
                                                                float frequency, float damping) {
    const float inv_effective_mass = calculate_inverse_effective_mass(part, body_a, body_b, inv_mass_a, inv_mass_b,
                                                                      inv_inertia_a, inv_inertia_b, r1_plus_u, r2, axis);

    if (inv_effective_mass < scalar::FLOAT_MIN_NORMAL) {
        deactivate(part);
    } else {
        part.effective_mass = spring_part::calculate_spring_properties_with_frequency_and_damping(
            part.spring, delta_time, inv_effective_mass, bias, c, frequency, damping);
    }
}

void calculate_constraint_properties_with_stiffness_and_damping(AxisConstraintPart &part, float delta_time,
                                                                rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                                                                float inv_mass_a, float inv_mass_b, const Mat4f &inv_inertia_a,
                                                                const Mat4f &inv_inertia_b, Vec3f r1_plus_u,
                                                                Vec3f r2, Vec3f axis, float bias, float c,
                                                                float stiffness, float damping) {
    const float inv_effective_mass = calculate_inverse_effective_mass(part, body_a, body_b, inv_mass_a, inv_mass_b,
                                                                      inv_inertia_a, inv_inertia_b, r1_plus_u, r2, axis);

    if (inv_effective_mass < scalar::FLOAT_MIN_NORMAL) {
        deactivate(part);
    } else {
        part.effective_mass = spring_part::calculate_spring_properties_with_stiffness_and_damping(
            part.spring, delta_time, inv_effective_mass, bias, c, stiffness, damping);
    }
}

void calculate_constraint_properties_with_settings(AxisConstraintPart &part, float delta_time, rigid_body::RigidBody &body_a,
                                                   rigid_body::RigidBody &body_b, float inv_mass_a, float inv_mass_b,
                                                   const Mat4f &inv_inertia_a, const Mat4f &inv_inertia_b,
                                                   Vec3f r1_plus_u, Vec3f r2, Vec3f axis, float bias,
                                                   float c, const spring_settings::SpringSettings &settings) {
    const float inv_effective_mass = calculate_inverse_effective_mass(part, body_a, body_b, inv_mass_a, inv_mass_b,
                                                                      inv_inertia_a, inv_inertia_b, r1_plus_u, r2, axis);

    if (inv_effective_mass < scalar::FLOAT_MIN_NORMAL) {
        deactivate(part);
    } else if (!spring_settings::has_stiffness(settings)) {
        part.effective_mass = 1.0f / inv_effective_mass;
        spring_part::calculate_spring_properties_with_bias(part.spring, bias);
    } else {
        part.effective_mass =
            spring_part::calculate_spring_properties_with_settings(part.spring, delta_time, inv_effective_mass, bias, c, settings);
    }
}

void calculate_constraint_properties_with_settings_for_motor(AxisConstraintPart &part, float delta_time,
                                                             rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                                                             float inv_mass_a, float inv_mass_b, const Mat4f &inv_inertia_a,
                                                             const Mat4f &inv_inertia_b, Vec3f r1_plus_u,
                                                             Vec3f r2, Vec3f axis, float bias, float c,
                                                             const spring_settings::SpringSettings &settings) {
    const float inv_effective_mass = calculate_inverse_effective_mass(part, body_a, body_b, inv_mass_a, inv_mass_b,
                                                                      inv_inertia_a, inv_inertia_b, r1_plus_u, r2, axis);

    if (inv_effective_mass < scalar::FLOAT_MIN_NORMAL) {
        deactivate(part);
    } else {
        part.effective_mass =
            spring_part::calculate_spring_properties_with_settings(part.spring, delta_time, inv_effective_mass, bias, c, settings);
    }
}

void set_total_lambda(AxisConstraintPart &part, float lambda) { part.total_lambda = lambda; }

float get_total_lambda_value(const AxisConstraintPart &part) { return part.total_lambda; }

/**
 * body a takes the impulse -lambda * axis, body b +lambda * axis, each through its inverse mass; the translation dof mask
 * applies to the result
 */
static void apply_velocity_step(const AxisConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                                float inv_mass_a, float inv_mass_b, Vec3f axis, float lambda) {
    const Vec3f impulse = vec3f::scale(axis, lambda);
    if (body_a.motion_type == motion_type::MOTION_DYNAMIC) {
        motion_properties::sub_linear_velocity_step(body_a.motion_properties, vec3f::scale(impulse, inv_mass_a));
        motion_properties::sub_angular_velocity_step(body_a.motion_properties, vec3f::scale(vec3f::load(part.inv_i1_r1_plus_u_x_axis), lambda));
    }
    if (body_b.motion_type == motion_type::MOTION_DYNAMIC) {
        motion_properties::add_linear_velocity_step(body_b.motion_properties, vec3f::scale(impulse, inv_mass_b));
        motion_properties::add_angular_velocity_step(body_b.motion_properties, vec3f::scale(vec3f::load(part.inv_i2_r2_x_axis), lambda));
    }
}

void warm_start(AxisConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b, float inv_mass_a,
                float inv_mass_b, Vec3f axis, float warm_start_ratio) {
    // scale previous frame's lambda
    part.total_lambda *= warm_start_ratio;

    if (part.total_lambda == 0.0f) return;

    apply_velocity_step(part, body_a, body_b, inv_mass_a, inv_mass_b, axis, part.total_lambda);
}

bool solve_velocity_constraint(AxisConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                               Vec3f axis, float min_lambda, float max_lambda) {
    const bool moving_a = body_a.motion_type != motion_type::MOTION_STATIC;
    const bool moving_b = body_b.motion_type != motion_type::MOTION_STATIC;
    const motion_properties::MotionProperties &mp_a = body_a.motion_properties;
    const motion_properties::MotionProperties &mp_b = body_b.motion_properties;

    // jacobian multiplied by velocity: the linear and angular terms summed per component, reduced once
    Vec3f acc;
    if (moving_a) {
        if (moving_b) {
            acc = vec3f::multiply(axis, vec3f::subtract(vec3f::load(mp_a.linear_velocity), vec3f::load(mp_b.linear_velocity)));
            acc = vec3f::subtract(acc, vec3f::multiply(vec3f::load(part.r2_x_axis), vec3f::load(mp_b.angular_velocity)));
        } else {
            acc = vec3f::multiply(axis, vec3f::load(mp_a.linear_velocity));
        }
        acc = vec3f::add(acc, vec3f::multiply(vec3f::load(part.r1_plus_u_x_axis), vec3f::load(mp_a.angular_velocity)));
    } else if (moving_b) {
        acc = vec3f::multiply(axis, vec3f::negate(vec3f::load(mp_b.linear_velocity)));
        acc = vec3f::subtract(acc, vec3f::multiply(vec3f::load(part.r2_x_axis), vec3f::load(mp_b.angular_velocity)));
    } else {
        acc = vec3f::zero();
    }
    const float jv = vec3f::reduce_sum(acc);

    // lambda = -K^-1 (J v + b), the accumulated impulse clamped (with inverted bounds the max wins)
    const float lambda = part.effective_mass * (jv - spring_part::get_spring_bias(part.spring, part.total_lambda));
    const float total_lambda = scalar::min(scalar::max(part.total_lambda + lambda, min_lambda), max_lambda);
    const float delta_lambda = total_lambda - part.total_lambda;
    part.total_lambda = total_lambda;
    if (delta_lambda == 0.0f) return false;

    const float inv_mass_a = body_a.motion_type == motion_type::MOTION_DYNAMIC ? mp_a.inv_mass : 0.0f;
    const float inv_mass_b = body_b.motion_type == motion_type::MOTION_DYNAMIC ? mp_b.inv_mass : 0.0f;
    apply_velocity_step(part, body_a, body_b, inv_mass_a, inv_mass_b, axis, delta_lambda);
    return true;
}

bool solve_position_constraint(AxisConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                               Vec3f axis, float c, float baumgarte) {
    // only apply position constraint when the constraint is hard, otherwise the velocity bias will fix the constraint
    if (c == 0.0f || !is_active(part) || spring_part::is_spring_active(part.spring)) return false;

    // lambda = -K^-1 * beta * C, the euler velocity and position steps folded together
    const float lambda = -part.effective_mass * baumgarte * c;

    // the impulse lambda * axis, through each dynamic body's own inverse mass
    const Vec3f impulse = vec3f::scale(axis, lambda);
    if (body_a.motion_type == motion_type::MOTION_DYNAMIC) {
        rigid_body_step::sub_position_step(&body_a, vec3f::scale(impulse, body_a.motion_properties.inv_mass));
        rigid_body_step::sub_rotation_step(&body_a, vec3f::scale(vec3f::load(part.inv_i1_r1_plus_u_x_axis), lambda));
    }
    if (body_b.motion_type == motion_type::MOTION_DYNAMIC) {
        rigid_body_step::add_position_step(&body_b, vec3f::scale(impulse, body_b.motion_properties.inv_mass));
        rigid_body_step::add_rotation_step(&body_b, vec3f::scale(vec3f::load(part.inv_i2_r2_x_axis), lambda));
    }
    return true;
}

}  // namespace axis_constraint_part

TUMBLE_NAMESPACE_END
