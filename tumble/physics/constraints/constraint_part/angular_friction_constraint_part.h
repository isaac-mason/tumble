#pragma once

// the twist friction of a contact manifold about its normal (jolt's AngularFrictionConstraintPart), solved over the
// angular velocities the caller loads once per constraint, templated on the bodies' motion types.

#include "tumble/math/mat4f.h"
#include "tumble/math/scalar.h"
#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/motion_type.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace angular_friction_constraint_part {

/** the smallest normal f32, jolt's FLT_MIN */
static constexpr float FLOAT_MIN_NORMAL = 1.17549435e-38f;

struct AngularFrictionConstraintPart {
    /** the axis through each body's inverse inertia */
    Float3 inv_i1_axis;
    Float3 inv_i2_axis;
    /** 1 / k; 0 when inactive */
    float effective_mass;
    float bias;
    /** the impulse applied so far this step */
    float total_lambda;
};

inline void deactivate(AngularFrictionConstraintPart &part) {
    part.effective_mass = 0.0f;
    part.total_lambda = 0.0f;
}

inline bool is_active(const AngularFrictionConstraintPart &part) { return part.effective_mass != 0.0f; }

/** body a takes -lambda about the axis, body b +lambda; true when lambda is not 0 */
template <motion_type::MotionType TYPE_A, motion_type::MotionType TYPE_B>
inline bool apply_velocity_step(const AngularFrictionConstraintPart &part, Vec3f &angular_velocity_a, Vec3f &angular_velocity_b,
                                float lambda) {
    if (lambda == 0.0f) return false;
    if constexpr (TYPE_A == motion_type::MOTION_DYNAMIC) {
        angular_velocity_a = vec3f::subtract(angular_velocity_a, vec3f::scale(vec3f::load(part.inv_i1_axis), lambda));
    }
    if constexpr (TYPE_B == motion_type::MOTION_DYNAMIC) {
        angular_velocity_b = vec3f::add(angular_velocity_b, vec3f::scale(vec3f::load(part.inv_i2_axis), lambda));
    }
    return true;
}

/**
 * the effective mass and the cached I^-1 axis terms. inv_inertia_a / inv_inertia_b are the world space inverse inertias,
 * world_space_axis the normalized axis the friction torque acts about (the contact normal), bias the velocity bias (the
 * relative angular surface velocity along the axis).
 */
template <motion_type::MotionType TYPE_A, motion_type::MotionType TYPE_B>
inline void calculate_constraint_properties(AngularFrictionConstraintPart &part, const Mat4f &inv_inertia_a, const Mat4f &inv_inertia_b,
                                            Vec3f world_space_axis, float bias) {
    part.bias = bias;

    float inv_effective_mass = 0.0f;
    if constexpr (TYPE_A == motion_type::MOTION_DYNAMIC) {
        const Vec3f inv_i1_axis = mat4f::multiply3x3_vec(inv_inertia_a, world_space_axis);
        vec3f::store(part.inv_i1_axis, inv_i1_axis);
        inv_effective_mass += vec3f::dot(world_space_axis, inv_i1_axis);
    }
    if constexpr (TYPE_B == motion_type::MOTION_DYNAMIC) {
        const Vec3f inv_i2_axis = mat4f::multiply3x3_vec(inv_inertia_b, world_space_axis);
        vec3f::store(part.inv_i2_axis, inv_i2_axis);
        inv_effective_mass += vec3f::dot(world_space_axis, inv_i2_axis);
    }

    if (inv_effective_mass < scalar::FLOAT_MIN_NORMAL) {
        deactivate(part);
    } else {
        part.effective_mass = 1.0f / inv_effective_mass;
    }
}

/** scale the previous step's impulse by warm_start_ratio (dt_new / dt_old) and apply it; true when it was not 0 */
template <motion_type::MotionType TYPE_A, motion_type::MotionType TYPE_B>
inline bool warm_start(AngularFrictionConstraintPart &part, Vec3f &angular_velocity_a, Vec3f &angular_velocity_b, float warm_start_ratio) {
    part.total_lambda *= warm_start_ratio;
    return apply_velocity_step<TYPE_A, TYPE_B>(part, angular_velocity_a, angular_velocity_b, part.total_lambda);
}

/** one velocity iteration with the total lambda clamped to [min_lambda, max_lambda]; true if an impulse was applied */
template <motion_type::MotionType TYPE_A, motion_type::MotionType TYPE_B>
inline bool solve_velocity_constraint(AngularFrictionConstraintPart &part, Vec3f &angular_velocity_a, Vec3f &angular_velocity_b,
                                      Vec3f world_space_axis, float min_lambda, float max_lambda) {
    static_assert(TYPE_A != motion_type::MOTION_STATIC || TYPE_B != motion_type::MOTION_STATIC, "static against static");

    float jv;
    if constexpr (TYPE_A != motion_type::MOTION_STATIC && TYPE_B != motion_type::MOTION_STATIC) {
        jv = vec3f::dot(world_space_axis, vec3f::subtract(angular_velocity_a, angular_velocity_b));
    } else if constexpr (TYPE_A != motion_type::MOTION_STATIC) {
        jv = vec3f::dot(world_space_axis, angular_velocity_a);
    } else {
        jv = -vec3f::dot(world_space_axis, angular_velocity_b);
    }

    const float lambda = part.effective_mass * (jv - part.bias);
    const float total_lambda = scalar::min(max_lambda, scalar::max(min_lambda, part.total_lambda + lambda));
    const float delta_lambda = total_lambda - part.total_lambda;
    part.total_lambda = total_lambda;
    return apply_velocity_step<TYPE_A, TYPE_B>(part, angular_velocity_a, angular_velocity_b, delta_lambda);
}

}  // namespace angular_friction_constraint_part

TUMBLE_NAMESPACE_END
