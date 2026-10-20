#pragma once

#include <stdint.h>

#include "tumble/math/mat4f.h"
#include "tumble/math/types.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace rigid_body {
struct RigidBody;
}

// rotation about the two axes that leave only rotation about the hinge axis
namespace hinge_rotation_constraint_part {

/**
 * constrains rotation around 2 axes so that it only allows rotation around 1 axis (the hinge axis).
 *
 * based on: "constraints derivation for rigid body simulation in 3d" - daniel chappuis, section 2.4.1
 *
 * constraint equation (eq 87): C = [a1 . b2, a1 . c2]
 * where a1 = hinge axis on body a, b2, c2 = axes perpendicular to the hinge axis on body b (world space)
 *
 * jacobian (eq 90):
 * J = [0, -b2 x a1, 0, b2 x a1]
 *     [0, -c2 x a1, 0, c2 x a1]
 */
struct HingeRotationConstraintPart {
    /** world space hinge axis for body a */
    Vec3f a1;
    /** world space perpendicular to hinge axis for body b */
    Vec3f b2;
    /** world space perpendicular to hinge axis for body b (orthogonal to b2) */
    Vec3f c2;
    /** I1^-1 cached for velocity/position integration */
    Mat4f inv_i1;
    /** I2^-1 cached for velocity/position integration */
    Mat4f inv_i2;
    /** b2 x a1 (cached for the jacobian) */
    Vec3f b2_x_a1;
    /** c2 x a1 (cached for the jacobian) */
    Vec3f c2_x_a1;
    /** 2x2 effective mass matrix K^-1 = (J M^-1 J^T)^-1, stored as [m00, m01, m10, m11] */
    float effective_mass[4];
    /** accumulated 2d impulse (for warm starting) [lambda0, lambda1] */
    float total_lambda[2];
};

inline HingeRotationConstraintPart create() {
    HingeRotationConstraintPart part = {};
    part.inv_i1 = mat4f::identity();
    part.inv_i2 = mat4f::identity();
    return part;
}

/** deactivate this constraint part (zero out effective mass and lambda) */
inline void deactivate(HingeRotationConstraintPart &part) {
    part.effective_mass[0] = 0.0f;
    part.effective_mass[1] = 0.0f;
    part.effective_mass[2] = 0.0f;
    part.effective_mass[3] = 0.0f;
    part.total_lambda[0] = 0.0f;
    part.total_lambda[1] = 0.0f;
}

/** check if constraint is active (any element of the effective mass non-zero) */
inline bool is_active(const HingeRotationConstraintPart &part) {
    return part.effective_mass[0] != 0.0f || part.effective_mass[1] != 0.0f || part.effective_mass[2] != 0.0f ||
           part.effective_mass[3] != 0.0f;
}

/** sets up the effective mass and cached cross products from both bodies' world space hinge axes (normalized) */
void calculate_constraint_properties(HingeRotationConstraintPart &part, rigid_body::RigidBody &body_a,
                                     rigid_body::RigidBody &body_b, Vec3f world_space_hinge_axis1,
                                     Vec3f world_space_hinge_axis2, uint32_t step_stamp);

/** apply warm start impulse from previous frame */
void warm_start(HingeRotationConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                float warm_start_impulse_ratio);

/** enforces d/dt C = 0. true if an impulse was applied */
bool solve_velocity_constraint(HingeRotationConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b);

/** baumgarte stabilization, enforces C = 0. true if a correction was applied */
bool solve_position_constraint(HingeRotationConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                               float baumgarte);

}  // namespace hinge_rotation_constraint_part

TUMBLE_NAMESPACE_END
