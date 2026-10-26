#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"

TUMBLE_NAMESPACE_BEGIN

struct RigidBody;

// all three rotational degrees of freedom

/**
 * removes 3 rotational degrees of freedom.
 *
 * based on: "constraints derivation for rigid body simulation in 3d" - daniel chappuis, section 2.5.1
 *
 * constraint equation (eq 129): C = [d theta_x, d theta_y, d theta_z]
 * jacobian (eq 131): J = [0, -E, 0, E], where E is the identity matrix
 */
struct RotationEulerConstraintPart {
    /** I1^-1 in world space (for position solve) */
    Mat4f inv_i1;
    /** I2^-1 in world space (for position solve) */
    Mat4f inv_i2;
    /** 3x3 effective mass matrix K^-1 = (J M^-1 J^T)^-1 */
    Mat4f effective_mass;
    /** accumulated 3d angular impulse (for warm starting) */
    Vec3f total_lambda;
};

namespace rotation_euler_constraint_part {

inline RotationEulerConstraintPart create() {
    RotationEulerConstraintPart part = {};
    part.inv_i1 = mat4f::identity();
    part.inv_i2 = mat4f::identity();
    return part;
}

/** deactivates the constraint part (resets state) */
inline void deactivate(RotationEulerConstraintPart &part) {
    part.effective_mass = mat4f::zero();
    part.total_lambda = vec3f::zero();
}

/** checks if the constraint part is active */
inline bool is_active(const RotationEulerConstraintPart &part) { return v4f::lane<3>(part.effective_mass.c[3]) != 0.0f; }

/**
 * inverse of the initial rotation from body 1 to body 2 in body 1 space.
 *
 * q20 = q10 * r0  =>  r0 = q10^-1 * q20  =>  r0^-1 = q20^-1 * q10
 * where q10, q20 are the initial orientations of body 1 and 2 and r0 the initial rotation from body 1 to body 2
 */
Quatf get_inv_initial_orientation(Quatf rotation1, Quatf rotation2);

/**
 * inverse of the initial rotation from body 1 to body 2 in body 1 space, given reference axes x and y for
 * both bodies (a constraint specified in world space with explicit axes): r0^-1 = c2 * c1^-1, where c1, c2
 * are the rotation matrices built from the axes.
 */
Quatf get_inv_initial_orientation_xy(Vec3f axis_x1, Vec3f axis_y1, Vec3f axis_x2, Vec3f axis_y2);

/**
 * the 3x3 effective mass: for J = [0, -E, 0, E], K = J M^-1 J^T = I1^-1 + I2^-1, inverted, the inertia from the body
 * quaternions
 */
void calculate_constraint_properties(RotationEulerConstraintPart &part, RigidBody &body_a, RigidBody &body_b,
                                     uint32_t step_stamp);

/** warm start by applying the previous frame's impulse scaled by the warm start ratio */
void warm_start(RotationEulerConstraintPart &part, RigidBody &body_a, RigidBody &body_b,
                float warm_start_impulse_ratio);

/**
 * J v = w1 - w2, lambda = -K^-1 (J v). dynamic and kinematic bodies contribute velocity, only dynamic
 * bodies receive impulses. true if an impulse was applied
 */
bool solve_velocity_constraint(RotationEulerConstraintPart &part, RigidBody &body_a, RigidBody &body_b);

/**
 * baumgarte stabilization of the rotation error diff = q2 * inv_initial_orientation * q1^-1, which for
 * small angles is 2 * diff.xyz. true if a correction was applied
 */
bool solve_position_constraint(RotationEulerConstraintPart &part, RigidBody &body_a, RigidBody &body_b,
                               Quatf inv_initial_orientation, float baumgarte);

}  // namespace rotation_euler_constraint_part

TUMBLE_NAMESPACE_END
