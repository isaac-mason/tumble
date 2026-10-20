#pragma once

#include <stdint.h>

#include "tumble/math/mat2f.h"
#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/math/vec2f.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace rigid_body {
struct RigidBody;
}

// movement on the two axes perpendicular to a sliding axis
namespace dual_axis_constraint_part {

/**
 * constrains movement on 2 axes perpendicular to a sliding axis.
 *
 * based on: "constraints derivation for rigid body simulation in 3d" - daniel chappuis, section 2.3.1
 *
 * constraint equation (eq 51): C = [(p2 - p1) . n1, (p2 - p1) . n2]
 *
 * jacobian (transposed) (eq 55):
 * J^T = [
 *   -n1               -n2
 *   -(r1 + u) x n1    -(r1 + u) x n2
 *   n1                n2
 *   r2 x n1           r2 x n2
 * ]
 *
 * n1, n2 = constraint axes perpendicular to slider axis (normalized), p1, p2 = constraint points,
 * r1 = p1 - x1, r2 = p2 - x2, u = x2 + r2 - x1 - r1 = p2 - p1, x1, x2 = center of mass for the bodies
 */
struct DualAxisConstraintPart {
    /** (r1 + u) x n1 */
    Vec3f r1_plus_u_x_n1;
    /** (r1 + u) x n2 */
    Vec3f r1_plus_u_x_n2;
    /** r2 x n1 */
    Vec3f r2_x_n1;
    /** r2 x n2 */
    Vec3f r2_x_n2;

    /** I1^-1 x ((r1 + u) x n1) */
    Vec3f inv_i1_r1_plus_u_x_n1;
    /** I1^-1 x ((r1 + u) x n2) */
    Vec3f inv_i1_r1_plus_u_x_n2;
    /** I2^-1 x (r2 x n1) */
    Vec3f inv_i2_r2_x_n1;
    /** I2^-1 x (r2 x n2) */
    Vec3f inv_i2_r2_x_n2;

    /** 2x2 effective mass matrix K^-1 = (J M^-1 J^T)^-1 */
    Mat2f effective_mass;

    /** accumulated 2d impulse (lambda) for warm starting */
    Vec2f total_lambda;
};

inline DualAxisConstraintPart create() {
    DualAxisConstraintPart part = {};
    return part;
}

/** deactivates the constraint part (resets state) */
inline void deactivate(DualAxisConstraintPart &part) {
    vec2f::zero(part.total_lambda);
    mat2f::set(part.effective_mass, 0.0f, 0.0f, 0.0f, 0.0f);
}

/** checks if the constraint part is active: a zero effective mass is degenerate/inactive */
inline bool is_active(const DualAxisConstraintPart &part) {
    return part.effective_mass.m[0] != 0.0f || part.effective_mass.m[1] != 0.0f || part.effective_mass.m[2] != 0.0f ||
           part.effective_mass.m[3] != 0.0f;
}

/**
 * all inputs in world space. r1_plus_u is body a's moment arm (attachment point - com), r2 body b's;
 * n1 and n2 the constraint axes (normalized, perpendicular to the slider axis and each other).
 */
void calculate_constraint_properties(DualAxisConstraintPart &part, rigid_body::RigidBody &body_a, Vec3f r1_plus_u,
                                     rigid_body::RigidBody &body_b, Vec3f r2, Vec3f n1, Vec3f n2,
                                     uint32_t step_stamp);

/** warm start by applying the previous frame's impulses scaled by the warm start ratio */
void warm_start(DualAxisConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b, Vec3f n1,
                Vec3f n2, float warm_start_impulse_ratio);

/**
 * iteratively make d/dt C = 0. dynamic and kinematic bodies contribute velocity to the jacobian, only
 * dynamic bodies receive impulses. true if any impulse was applied
 */
bool solve_velocity_constraint(DualAxisConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                               Vec3f n1, Vec3f n2);

/** iteratively make C = 0. u is the separation p2 - p1 in world space. true if any correction was applied */
bool solve_position_constraint(DualAxisConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                               Vec3f u, Vec3f n1, Vec3f n2, float baumgarte);

}  // namespace dual_axis_constraint_part

TUMBLE_NAMESPACE_END
