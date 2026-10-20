#pragma once

#include "tumble/math/mat4f.h"
#include "tumble/math/scalar.h"
#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/motion_type.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/body/rigid_body_step.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

// the axis part for contacts: hard, along a contact normal or friction tangent, over velocities the caller loads once
// per constraint, with the bodies' motion types as template arguments so a static or kinematic side costs nothing
namespace contact_constraint_part {

struct ContactConstraintPart {
    /** (r1 + u) x axis and r2 x axis, the jacobian's angular terms */
    Float3 r1_plus_u_x_axis;
    Float3 r2_x_axis;
    /** the angular terms through each body's inverse inertia */
    Float3 inv_i1_r1_plus_u_x_axis;
    Float3 inv_i2_r2_x_axis;
    /** 1 / k; 0 when inactive */
    float effective_mass;
    /** the impulse applied so far this step */
    float total_lambda;
    /** the velocity bias (restitution, speculative contacts, surface velocity) */
    float bias;
};

inline void deactivate(ContactConstraintPart &part) {
    part.effective_mass = 0.0f;
    part.total_lambda = 0.0f;
}

inline bool is_active(const ContactConstraintPart &part) { return part.effective_mass != 0.0f; }

/** body a takes -lambda along the axis, body b +lambda; true when lambda is not 0 */
template <motion_type::MotionType TYPE_A, motion_type::MotionType TYPE_B>
inline bool apply_velocity_step(const ContactConstraintPart &part, Vec3f &linear_velocity_a, Vec3f &angular_velocity_a,
                                Vec3f &linear_velocity_b, Vec3f &angular_velocity_b, float inv_mass_a, float inv_mass_b, Vec3f axis,
                                float lambda) {
    if (lambda == 0.0f) return false;
    if constexpr (TYPE_A == motion_type::MOTION_DYNAMIC) {
        linear_velocity_a = vec3f::subtract(linear_velocity_a, vec3f::scale(axis, lambda * inv_mass_a));
        angular_velocity_a = vec3f::subtract(angular_velocity_a, vec3f::scale(vec3f::load(part.inv_i1_r1_plus_u_x_axis), lambda));
    }
    if constexpr (TYPE_B == motion_type::MOTION_DYNAMIC) {
        linear_velocity_b = vec3f::add(linear_velocity_b, vec3f::scale(axis, lambda * inv_mass_b));
        angular_velocity_b = vec3f::add(angular_velocity_b, vec3f::scale(vec3f::load(part.inv_i2_r2_x_axis), lambda));
    }
    return true;
}

/**
 * the effective mass and cached jacobian terms: inv_mass and inv_inertia per body after the contact's scales (zero for a
 * non-dynamic body), r1_plus_u and r2 the moment arms from each centre of mass, axis normalized
 */
template <motion_type::MotionType TYPE_A, motion_type::MotionType TYPE_B>
inline void calculate_constraint_properties(ContactConstraintPart &part, float inv_mass_a, const Mat4f &inv_inertia_a, Vec3f r1_plus_u,
                                            float inv_mass_b, const Mat4f &inv_inertia_b, Vec3f r2, Vec3f axis, float bias) {
    part.bias = bias;

    // K = J M^-1 J^T; the cross products are needed for every moving body (a kinematic body's angular velocity enters the
    // jacobian), the inverse inertia terms only for dynamic ones
    float inv_effective_mass = 0.0f;
    if constexpr (TYPE_A != motion_type::MOTION_STATIC) {
        const Vec3f r1_plus_u_x_axis = vec3f::cross(r1_plus_u, axis);
        vec3f::store(part.r1_plus_u_x_axis, r1_plus_u_x_axis);
        if constexpr (TYPE_A == motion_type::MOTION_DYNAMIC) {
            const Vec3f inv_i1_r1_plus_u_x_axis = mat4f::multiply3x3_vec(inv_inertia_a, r1_plus_u_x_axis);
            vec3f::store(part.inv_i1_r1_plus_u_x_axis, inv_i1_r1_plus_u_x_axis);
            inv_effective_mass = inv_mass_a + vec3f::dot(inv_i1_r1_plus_u_x_axis, r1_plus_u_x_axis);
        }
    }
    if constexpr (TYPE_B != motion_type::MOTION_STATIC) {
        const Vec3f r2_x_axis = vec3f::cross(r2, axis);
        vec3f::store(part.r2_x_axis, r2_x_axis);
        if constexpr (TYPE_B == motion_type::MOTION_DYNAMIC) {
            const Vec3f inv_i2_r2_x_axis = mat4f::multiply3x3_vec(inv_inertia_b, r2_x_axis);
            vec3f::store(part.inv_i2_r2_x_axis, inv_i2_r2_x_axis);
            inv_effective_mass += inv_mass_b + vec3f::dot(inv_i2_r2_x_axis, r2_x_axis);
        }
    }

    if (inv_effective_mass < scalar::FLOAT_MIN_NORMAL) {
        deactivate(part);
    } else {
        part.effective_mass = 1.0f / inv_effective_mass;
    }
}

/** scale the previous step's impulse by warm_start_ratio (dt_new / dt_old) and apply it; true when it was not 0 */
template <motion_type::MotionType TYPE_A, motion_type::MotionType TYPE_B>
inline bool warm_start(ContactConstraintPart &part, Vec3f &linear_velocity_a, Vec3f &angular_velocity_a, Vec3f &linear_velocity_b,
                       Vec3f &angular_velocity_b, float inv_mass_a, float inv_mass_b, Vec3f axis, float warm_start_ratio) {
    part.total_lambda *= warm_start_ratio;
    return apply_velocity_step<TYPE_A, TYPE_B>(part, linear_velocity_a, angular_velocity_a, linear_velocity_b, angular_velocity_b,
                                               inv_mass_a, inv_mass_b, axis, part.total_lambda);
}

/** the total lambda the part would reach, unclamped, from the jacobian times the velocities */
template <motion_type::MotionType TYPE_A, motion_type::MotionType TYPE_B>
inline float solve_velocity_constraint_get_total_lambda(const ContactConstraintPart &part, Vec3f linear_velocity_a, Vec3f angular_velocity_a,
                                                        Vec3f linear_velocity_b, Vec3f angular_velocity_b, Vec3f axis) {
    static_assert(TYPE_A != motion_type::MOTION_STATIC || TYPE_B != motion_type::MOTION_STATIC, "static against static");

    // the linear and angular terms summed per component, reduced once
    Vec3f acc;
    if constexpr (TYPE_A != motion_type::MOTION_STATIC && TYPE_B != motion_type::MOTION_STATIC) {
        acc = vec3f::multiply(axis, vec3f::subtract(linear_velocity_a, linear_velocity_b));
    } else if constexpr (TYPE_A != motion_type::MOTION_STATIC) {
        acc = vec3f::multiply(axis, linear_velocity_a);
    } else {
        acc = vec3f::multiply(axis, vec3f::negate(linear_velocity_b));
    }
    if constexpr (TYPE_A != motion_type::MOTION_STATIC) {
        acc = vec3f::add(acc, vec3f::multiply(vec3f::load(part.r1_plus_u_x_axis), angular_velocity_a));
    }
    if constexpr (TYPE_B != motion_type::MOTION_STATIC) {
        acc = vec3f::subtract(acc, vec3f::multiply(vec3f::load(part.r2_x_axis), angular_velocity_b));
    }
    const float jv = vec3f::reduce_sum(acc);

    // lambda = -K^-1 (J v + b)
    return part.total_lambda + part.effective_mass * (jv - part.bias);
}

/** commit a total lambda and apply the change; true when it changed */
template <motion_type::MotionType TYPE_A, motion_type::MotionType TYPE_B>
inline bool solve_velocity_constraint_apply_lambda(ContactConstraintPart &part, Vec3f &linear_velocity_a, Vec3f &angular_velocity_a,
                                                   Vec3f &linear_velocity_b, Vec3f &angular_velocity_b, float inv_mass_a, float inv_mass_b,
                                                   Vec3f axis, float total_lambda) {
    const float delta_lambda = total_lambda - part.total_lambda;
    part.total_lambda = total_lambda;
    return apply_velocity_step<TYPE_A, TYPE_B>(part, linear_velocity_a, angular_velocity_a, linear_velocity_b, angular_velocity_b,
                                               inv_mass_a, inv_mass_b, axis, delta_lambda);
}

/** one velocity iteration with the total lambda clamped to [min_lambda, max_lambda] (inverted bounds give the max) */
template <motion_type::MotionType TYPE_A, motion_type::MotionType TYPE_B>
inline bool solve_velocity_constraint(ContactConstraintPart &part, Vec3f &linear_velocity_a, Vec3f &angular_velocity_a,
                                      Vec3f &linear_velocity_b, Vec3f &angular_velocity_b, float inv_mass_a, float inv_mass_b, Vec3f axis,
                                      float min_lambda, float max_lambda) {
    const float total_lambda = solve_velocity_constraint_get_total_lambda<TYPE_A, TYPE_B>(part, linear_velocity_a, angular_velocity_a,
                                                                                         linear_velocity_b, angular_velocity_b, axis);
    return solve_velocity_constraint_apply_lambda<TYPE_A, TYPE_B>(part, linear_velocity_a, angular_velocity_a, linear_velocity_b,
                                                                  angular_velocity_b, inv_mass_a, inv_mass_b, axis,
                                                                  scalar::min(scalar::max(total_lambda, min_lambda), max_lambda));
}

/**
 * baumgarte stabilization: c is the separation (negative when penetrating). the euler velocity step and the position
 * step are folded together and the velocity change discarded, so the correction adds no momentum (catto, "modeling and
 * solving constraints", gdc 2007). true if a correction was applied.
 */
template <motion_type::MotionType TYPE_A, motion_type::MotionType TYPE_B>
inline bool solve_position_constraint(const ContactConstraintPart &part, rigid_body::RigidBody &body_a, float inv_mass_a,
                                      rigid_body::RigidBody &body_b, float inv_mass_b, Vec3f axis, float c, float baumgarte) {
    if (c == 0.0f || !is_active(part)) return false;

    // lambda = -K^-1 * beta * C / dt, and the position step multiplies by dt again
    const float lambda = -part.effective_mass * baumgarte * c;
    if constexpr (TYPE_A == motion_type::MOTION_DYNAMIC) {
        rigid_body_step::sub_position_step(&body_a, vec3f::scale(axis, lambda * inv_mass_a));
        rigid_body_step::sub_rotation_step(&body_a, vec3f::scale(vec3f::load(part.inv_i1_r1_plus_u_x_axis), lambda));
    }
    if constexpr (TYPE_B == motion_type::MOTION_DYNAMIC) {
        rigid_body_step::add_position_step(&body_b, vec3f::scale(axis, lambda * inv_mass_b));
        rigid_body_step::add_rotation_step(&body_b, vec3f::scale(vec3f::load(part.inv_i2_r2_x_axis), lambda));
    }
    return true;
}

}  // namespace contact_constraint_part

TUMBLE_NAMESPACE_END
