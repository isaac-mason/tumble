#include "tumble/physics/constraints/constraint_part/angle_constraint_part.h"

#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/motion_properties.h"
#include "tumble/physics/body/motion_type.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/body/rigid_body_step.h"

TUMBLE_NAMESPACE_BEGIN

namespace angle_constraint_part {

/** helper to calculate inverse effective mass */
static float calculate_inverse_effective_mass(AngleConstraintPart &part, RigidBody &body_a,
                                              RigidBody &body_b, Vec3f world_space_axis) {
    // calculate I^-1 * axis for both bodies
    part.inv_i1_axis = body_a.motion_type == MotionType::DYNAMIC
                           ? motion_properties::multiply_world_space_inverse_inertia_by_vector(body_a.motion_properties,
                                                                                               quatf::load(body_a.quaternion), world_space_axis)
                           : vec3f::zero();
    part.inv_i2_axis = body_b.motion_type == MotionType::DYNAMIC
                           ? motion_properties::multiply_world_space_inverse_inertia_by_vector(body_b.motion_properties,
                                                                                               quatf::load(body_b.quaternion), world_space_axis)
                           : vec3f::zero();

    // K = J M^-1 J^T = axis . (I1^-1 + I2^-1) . axis; since J = [0, -a, 0, a], K = a . I1^-1 . a + a . I2^-1 . a
    return vec3f::dot(world_space_axis, vec3f::add(part.inv_i1_axis, part.inv_i2_axis));
}

void calculate_constraint_properties(AngleConstraintPart &part, RigidBody &body_a, RigidBody &body_b,
                                     Vec3f world_space_axis, float bias) {
    const float inv_effective_mass = calculate_inverse_effective_mass(part, body_a, body_b, world_space_axis);

    if (inv_effective_mass < scalar::FLOAT_MIN_NORMAL) {
        deactivate(part);
    } else {
        part.effective_mass = 1.0f / inv_effective_mass;
        spring_part::calculate_spring_properties_with_bias(part.spring, bias);
    }
}

void calculate_constraint_properties_with_frequency_and_damping(AngleConstraintPart &part, float delta_time,
                                                                RigidBody &body_a, RigidBody &body_b,
                                                                Vec3f world_space_axis, float bias, float c,
                                                                float frequency, float damping) {
    const float inv_effective_mass = calculate_inverse_effective_mass(part, body_a, body_b, world_space_axis);

    if (inv_effective_mass < scalar::FLOAT_MIN_NORMAL) {
        deactivate(part);
    } else {
        part.effective_mass = spring_part::calculate_spring_properties_with_frequency_and_damping(
            part.spring, delta_time, inv_effective_mass, bias, c, frequency, damping);
    }
}

void calculate_constraint_properties_with_stiffness_and_damping(AngleConstraintPart &part, float delta_time,
                                                                RigidBody &body_a, RigidBody &body_b,
                                                                Vec3f world_space_axis, float bias, float c,
                                                                float stiffness, float damping) {
    const float inv_effective_mass = calculate_inverse_effective_mass(part, body_a, body_b, world_space_axis);

    if (inv_effective_mass < scalar::FLOAT_MIN_NORMAL) {
        deactivate(part);
    } else {
        part.effective_mass = spring_part::calculate_spring_properties_with_stiffness_and_damping(
            part.spring, delta_time, inv_effective_mass, bias, c, stiffness, damping);
    }
}

void calculate_constraint_properties_with_settings(AngleConstraintPart &part, float delta_time, RigidBody &body_a,
                                                   RigidBody &body_b, Vec3f world_space_axis, float bias,
                                                   float c, const SpringSettings &settings) {
    const float inv_effective_mass = calculate_inverse_effective_mass(part, body_a, body_b, world_space_axis);

    if (inv_effective_mass < scalar::FLOAT_MIN_NORMAL) {
        deactivate(part);
    } else if (!spring_settings::has_stiffness(settings)) {
        part.effective_mass = 1.0f / inv_effective_mass;
        spring_part::calculate_spring_properties_with_bias(part.spring, bias);
    } else {
        part.effective_mass = spring_part::calculate_spring_properties_with_settings(part.spring, delta_time,
                                                                                     inv_effective_mass, bias, c, settings);
    }
}

void calculate_constraint_properties_with_settings_for_motor(AngleConstraintPart &part, float delta_time,
                                                             RigidBody &body_a, RigidBody &body_b,
                                                             Vec3f world_space_axis, float bias, float c,
                                                             const SpringSettings &settings) {
    const float inv_effective_mass = calculate_inverse_effective_mass(part, body_a, body_b, world_space_axis);

    if (inv_effective_mass < scalar::FLOAT_MIN_NORMAL) {
        deactivate(part);
    } else {
        part.effective_mass = spring_part::calculate_spring_properties_with_settings(part.spring, delta_time,
                                                                                     inv_effective_mass, bias, c, settings);
    }
}

static bool apply_velocity_step(AngleConstraintPart &part, RigidBody &body_a, RigidBody &body_b,
                                float lambda) {
    if (lambda == 0.0f) return false;

    // apply impulse: P = J^T * lambda
    // body a: w -= lambda * I1^-1 * axis
    if (body_a.motion_type == MotionType::DYNAMIC) {
        motion_properties::sub_angular_velocity_step(body_a.motion_properties, vec3f::scale(part.inv_i1_axis, lambda));
    }

    // body b: w += lambda * I2^-1 * axis
    if (body_b.motion_type == MotionType::DYNAMIC) {
        motion_properties::add_angular_velocity_step(body_b.motion_properties, vec3f::scale(part.inv_i2_axis, lambda));
    }

    return true;
}

void warm_start(AngleConstraintPart &part, RigidBody &body_a, RigidBody &body_b,
                float warm_start_impulse_ratio) {
    part.total_lambda *= warm_start_impulse_ratio;
    apply_velocity_step(part, body_a, body_b, part.total_lambda);
}

bool solve_velocity_constraint(AngleConstraintPart &part, RigidBody &body_a, RigidBody &body_b,
                               Vec3f world_space_axis, float min_lambda, float max_lambda) {
    if (!is_active(part)) return false;

    // J * v = axis . (w1 - w2)
    float jv = 0.0f;
    if (body_a.motion_type != MotionType::STATIC) {
        jv += vec3f::dot(world_space_axis, vec3f::load(body_a.motion_properties.angular_velocity));
    }
    if (body_b.motion_type != MotionType::STATIC) {
        jv -= vec3f::dot(world_space_axis, vec3f::load(body_b.motion_properties.angular_velocity));
    }

    // lambda = -K^-1 * (J v + b)
    const float lambda = part.effective_mass * (jv - spring_part::get_spring_bias(part.spring, part.total_lambda));

    // clamp impulse
    const float new_lambda = scalar::max(min_lambda, scalar::min(max_lambda, part.total_lambda + lambda));
    const float delta_lambda = new_lambda - part.total_lambda;
    part.total_lambda = new_lambda;

    return apply_velocity_step(part, body_a, body_b, delta_lambda);
}

bool solve_position_constraint(AngleConstraintPart &part, RigidBody &body_a, RigidBody &body_b, float c,
                               float baumgarte) {
    // only hard constraints get a position correction; soft constraints (springs) use the velocity bias instead
    if (c == 0.0f || !is_active(part) || spring_part::is_spring_active(part.spring)) return false;

    // lambda = -K^-1 * beta * C. no division by dt: the position integration multiplies by it again
    const float lambda = -part.effective_mass * baumgarte * c;

    // body a: dq -= lambda * I1^-1 * axis
    if (body_a.motion_type == MotionType::DYNAMIC) rigid_body_step::sub_rotation_step(&body_a, vec3f::scale(part.inv_i1_axis, lambda));

    // body b: dq += lambda * I2^-1 * axis
    if (body_b.motion_type == MotionType::DYNAMIC) rigid_body_step::add_rotation_step(&body_b, vec3f::scale(part.inv_i2_axis, lambda));

    return true;
}

float get_total_lambda(const AngleConstraintPart &part) { return part.total_lambda; }

void set_total_lambda(AngleConstraintPart &part, float lambda) { part.total_lambda = lambda; }

}  // namespace angle_constraint_part

TUMBLE_NAMESPACE_END
