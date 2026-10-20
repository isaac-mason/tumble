#pragma once

// crashcat's point-constraint-part.ts: removes the three translational degrees of freedom between two
// points, the 3-axis version of the axis part with a 3x3 effective mass. used by the point and hinge
// constraints for the attachment.

#include <stdint.h>

#include "math/mat4f.h"
#include "math/types.h"
#include "math/vec3f.h"

namespace rigid_body {
struct RigidBody;
}

namespace point_constraint_part {

struct PointConstraintPart {
    /** r1 in world space (moment arm from body 1 com to the constraint point) */
    Vec3f r1;
    /** r2 in world space (moment arm from body 2 com to the constraint point) */
    Vec3f r2;
    /** I1^-1 [r1]x, cached for impulse application */
    Mat4f inv_i1_r1x;
    /** I2^-1 [r2]x, cached for impulse application */
    Mat4f inv_i2_r2x;
    /** the 3x3 effective mass K^-1 = (J M^-1 J^T)^-1; m[15] is 0 when inactive */
    Mat4f effective_mass;
    /** accumulated impulse (for warm starting) */
    Vec3f total_lambda;
};

inline PointConstraintPart create() {
    PointConstraintPart part;
    part.r1 = vec3f::zero();
    part.r2 = vec3f::zero();
    part.inv_i1_r1x = mat4f::identity();
    part.inv_i2_r2x = mat4f::identity();
    part.effective_mass = mat4f::identity();
    part.total_lambda = vec3f::zero();
    return part;
}

inline void deactivate(PointConstraintPart &part) {
    part.effective_mass = mat4f::zero();
    part.total_lambda = vec3f::zero();
}

inline bool is_active(const PointConstraintPart &part) { return v4f::lane<3>(part.effective_mass.c[3]) != 0.0f; }

/**
 * the 3x3 effective mass and cached angular terms. rotation_a / rotation_b are the bodies' rotations,
 * r1_local / r2_local the attachment points relative to each center of mass in body space. step_stamp
 * is the step's inverse inertia memo stamp (motion_properties::STEP_STAMP_NONE computes fresh).
 *
 * K^-1 = m1^-1 I + [r1]x I1^-1 [r1]x^T + m2^-1 I + [r2]x I2^-1 [r2]x^T, inverted for K.
 */
void calculate_constraint_properties(PointConstraintPart &part, rigid_body::RigidBody &body_a, const Mat4f &rotation_a,
                                     Vec3f r1_local, rigid_body::RigidBody &body_b, const Mat4f &rotation_b, Vec3f r2_local,
                                     uint32_t step_stamp);

/** warm start by applying the cached impulse scaled by warm_start_impulse_ratio */
void warm_start(PointConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                float warm_start_impulse_ratio);

/**
 * one velocity iteration: lambda = K (v1 - r1 x w1 - v2 + r2 x w2), accumulated and applied. true if an
 * impulse was applied.
 */
bool solve_velocity_constraint(PointConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b);

/**
 * baumgarte stabilization of the separation C = (x2 + r2) - (x1 + r1), x the centers of mass:
 * lambda = -K beta C, applied to the positions and rotations directly. true if a correction was applied.
 */
bool solve_position_constraint(PointConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                               float baumgarte);

}  // namespace point_constraint_part
