#include "tumble/physics/constraints/constraint_part/point_constraint_part.h"

#include "tumble/math/quatf.h"
#include "tumble/physics/body/motion_properties.h"
#include "tumble/physics/body/rigid_body.h"
#include "tumble/physics/body/rigid_body_step.h"

TUMBLE_NAMESPACE_BEGIN

namespace point_constraint_part {

void calculate_constraint_properties(PointConstraintPart &part, RigidBody &body_a, const Mat4f &rotation_a,
                                     Vec3f r1_local, RigidBody &body_b, const Mat4f &rotation_b, Vec3f r2_local,
                                     uint32_t step_stamp) {
    // transform local space moment arms to world space
    part.r1 = mat4f::multiply3x3_vec(rotation_a, r1_local);
    part.r2 = mat4f::multiply3x3_vec(rotation_b, r2_local);

    // build inverse effective mass matrix
    Mat4f inv_effective_mass = mat4f::zero();

    float summed_inv_mass = 0.0f;

    // body a contribution
    if (body_a.motion_type == MotionType::DYNAMIC) {
        MotionProperties &mp1 = body_a.motion_properties;
        summed_inv_mass += mp1.inv_mass;

        // inverse inertia in world space
        const Mat4f inv_inertia1 = motion_properties::get_world_inverse_inertia(mp1, quatf::load(body_a.quaternion), step_stamp);

        // [r1]x cross product matrix
        const Mat4f r_cross = mat4f::cross_product_matrix(part.r1);

        // cache I1^-1 [r1]x for impulse application
        part.inv_i1_r1x = mat4f::multiply3x3(inv_inertia1, r_cross);

        // add [r1]x I1^-1 [r1]x^T to the inverse effective mass
        inv_effective_mass = mat4f::add(inv_effective_mass, mat4f::multiply3x3_right_transposed(mat4f::multiply3x3(r_cross, inv_inertia1), r_cross));
    } else {
        // static body - zero contribution
        part.inv_i1_r1x = mat4f::zero();
    }

    // body b contribution
    if (body_b.motion_type == MotionType::DYNAMIC) {
        MotionProperties &mp2 = body_b.motion_properties;
        summed_inv_mass += mp2.inv_mass;

        // inverse inertia in world space
        const Mat4f inv_inertia2 = motion_properties::get_world_inverse_inertia(mp2, quatf::load(body_b.quaternion), step_stamp);

        // [r2]x cross product matrix
        const Mat4f r_cross = mat4f::cross_product_matrix(part.r2);

        // cache I2^-1 [r2]x for impulse application
        part.inv_i2_r2x = mat4f::multiply3x3(inv_inertia2, r_cross);

        // add [r2]x I2^-1 [r2]x^T to the inverse effective mass
        inv_effective_mass = mat4f::add(inv_effective_mass, mat4f::multiply3x3_right_transposed(mat4f::multiply3x3(r_cross, inv_inertia2), r_cross));
    } else {
        // static body - zero contribution
        part.inv_i2_r2x = mat4f::zero();
    }

    // add linear mass term (diagonal)
    inv_effective_mass.c[0] = v4f::set_lane<0>(inv_effective_mass.c[0], v4f::lane<0>(inv_effective_mass.c[0]) + summed_inv_mass);
    inv_effective_mass.c[1] = v4f::set_lane<1>(inv_effective_mass.c[1], v4f::lane<1>(inv_effective_mass.c[1]) + summed_inv_mass);
    inv_effective_mass.c[2] = v4f::set_lane<2>(inv_effective_mass.c[2], v4f::lane<2>(inv_effective_mass.c[2]) + summed_inv_mass);

    // invert to get effective mass; a singular matrix is a degenerate constraint
    Mat4f effective_mass;
    if (mat4f::invert3x3(effective_mass, inv_effective_mass)) {
        part.effective_mass = effective_mass;
    } else {
        deactivate(part);
    }
}

/**
 * apply an impulse to both bodies.
 *
 * linear impulse: dP = lambda
 * angular impulse: dL = [r]x lambda, as I^-1 [r]x lambda (precomputed)
 */
static bool apply_velocity_step(const PointConstraintPart &part, RigidBody &body_a, RigidBody &body_b,
                                Vec3f lambda) {
    // check for zero impulse
    if (vec3f::squared_length(lambda) == 0.0f) return false;

    // body a: subtract impulse (constraint reaction)
    if (body_a.motion_type == MotionType::DYNAMIC) {
        MotionProperties &mp1 = body_a.motion_properties;

        // linear: v -= lambda * inv_mass (locked translation axes stay zero)
        motion_properties::sub_linear_velocity_step(mp1, vec3f::scale(lambda, mp1.inv_mass));

        // angular: w -= I^-1 [r1]x lambda
        motion_properties::sub_angular_velocity_step(mp1, mat4f::multiply3x3_vec(part.inv_i1_r1x, lambda));
    }

    // body b: add impulse
    if (body_b.motion_type == MotionType::DYNAMIC) {
        MotionProperties &mp2 = body_b.motion_properties;

        // linear: v += lambda * inv_mass (locked translation axes stay zero)
        motion_properties::add_linear_velocity_step(mp2, vec3f::scale(lambda, mp2.inv_mass));

        // angular: w += I^-1 [r2]x lambda
        motion_properties::add_angular_velocity_step(mp2, mat4f::multiply3x3_vec(part.inv_i2_r2x, lambda));
    }

    return true;
}

void warm_start(PointConstraintPart &part, RigidBody &body_a, RigidBody &body_b,
                float warm_start_impulse_ratio) {
    if (!is_active(part)) return;

    // scale cached impulse (no negation - apply accumulated lambda directly)
    part.total_lambda = vec3f::scale(part.total_lambda, warm_start_impulse_ratio);

    apply_velocity_step(part, body_a, body_b, part.total_lambda);
}

bool solve_velocity_constraint(PointConstraintPart &part, RigidBody &body_a, RigidBody &body_b) {
    if (!is_active(part)) return false;

    // jacobian velocity: J v = v1 - [r1]x w1 - v2 + [r2]x w2, where [r]x w = r x w
    const MotionProperties &mp1 = body_a.motion_properties;
    const MotionProperties &mp2 = body_b.motion_properties;
    Vec3f jv = vec3f::subtract(vec3f::load(mp1.linear_velocity), vec3f::cross(part.r1, vec3f::load(mp1.angular_velocity)));
    jv = vec3f::subtract(jv, vec3f::load(mp2.linear_velocity));
    jv = vec3f::add(jv, vec3f::cross(part.r2, vec3f::load(mp2.angular_velocity)));

    // lambda = K (v1 - r1 x w1 - v2 + r2 x w2); this computes -J v, so no negation needed
    const Vec3f lambda = mat4f::multiply3x3_vec(part.effective_mass, jv);

    // accumulate impulse
    part.total_lambda = vec3f::add(part.total_lambda, lambda);

    return apply_velocity_step(part, body_a, body_b, lambda);
}

bool solve_position_constraint(PointConstraintPart &part, RigidBody &body_a, RigidBody &body_b,
                               float baumgarte) {
    if (!is_active(part)) return false;

    // separation vector: C = (x2 + r2) - (x1 + r1), x the center of mass position
    Vec3f separation = vec3f::add(vec3f::load(body_b.center_of_mass_position), part.r2);
    separation = vec3f::subtract(separation, vec3f::load(body_a.center_of_mass_position));
    separation = vec3f::subtract(separation, part.r1);

    // check for zero separation (small epsilon for floating point)
    if (vec3f::squared_length(separation) < 1e-12f) return false;

    // lambda = -K beta C
    const Vec3f lambda = vec3f::scale(mat4f::multiply3x3_vec(part.effective_mass, separation), -baumgarte);

    // the body step functions handle the dof mask and the quaternion integration
    // body a: subtract correction
    if (body_a.motion_type == MotionType::DYNAMIC) {
        // linear: position -= lambda * inv_mass
        rigid_body_step::sub_position_step(&body_a, vec3f::scale(lambda, body_a.motion_properties.inv_mass));

        // angular: rotation -= I^-1 [r1]x lambda
        rigid_body_step::sub_rotation_step(&body_a, mat4f::multiply3x3_vec(part.inv_i1_r1x, lambda));
    }

    // body b: add correction
    if (body_b.motion_type == MotionType::DYNAMIC) {
        // linear: position += lambda * inv_mass
        rigid_body_step::add_position_step(&body_b, vec3f::scale(lambda, body_b.motion_properties.inv_mass));

        // angular: rotation += I^-1 [r2]x lambda
        rigid_body_step::add_rotation_step(&body_b, mat4f::multiply3x3_vec(part.inv_i2_r2x, lambda));
    }

    return true;
}

}  // namespace point_constraint_part

TUMBLE_NAMESPACE_END
