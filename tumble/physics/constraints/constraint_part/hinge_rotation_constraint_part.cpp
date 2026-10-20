#include "physics/constraints/constraint_part/hinge_rotation_constraint_part.h"

#include "math/mat4f.h"
#include "math/quatf.h"
#include "math/scalar.h"
#include "math/vec3f.h"
#include "physics/body/motion_properties.h"
#include "physics/body/motion_type.h"
#include "physics/body/rigid_body.h"
#include "physics/body/rigid_body_step.h"

namespace hinge_rotation_constraint_part {

void calculate_constraint_properties(HingeRotationConstraintPart &part, rigid_body::RigidBody &body_a,
                                     rigid_body::RigidBody &body_b, Vec3f world_space_hinge_axis1, Vec3f world_space_hinge_axis2,
                                     uint32_t step_stamp) {
    part.a1 = world_space_hinge_axis1;

    // a2, handling the case where the axes are more than 90 degrees apart
    Vec3f a2 = world_space_hinge_axis2;
    const float dot = vec3f::dot(part.a1, a2);

    if (dot <= 1.0e-3f) {
        // get a perpendicular vector in the plane formed by a1 and a2
        Vec3f perp = vec3f::scale_and_add(a2, part.a1, -dot);

        // a1 ~ -a2, take any perpendicular
        if (vec3f::squared_length(perp) < 1.0e-6f) perp = vec3f::normalized_perpendicular(part.a1);

        // blend in a little bit from a1 so we're less than 90 degrees apart: 0.01 * a1 + 0.99 * perp (normalized)
        perp = vec3f::normalize(perp);
        a2 = vec3f::normalize(vec3f::scale_and_add(vec3f::scale(part.a1, 0.01f), perp, 0.99f));
    }

    // b2 and c2, perpendicular to a2
    part.b2 = vec3f::normalized_perpendicular(a2);
    part.c2 = vec3f::cross(a2, part.b2);

    part.inv_i1 = body_a.motion_type == motion_type::MOTION_DYNAMIC
                      ? motion_properties::get_world_inverse_inertia(body_a.motion_properties, quatf::load(body_a.quaternion), step_stamp)
                      : mat4f::zero();
    part.inv_i2 = body_b.motion_type == motion_type::MOTION_DYNAMIC
                      ? motion_properties::get_world_inverse_inertia(body_b.motion_properties, quatf::load(body_b.quaternion), step_stamp)
                      : mat4f::zero();

    // cross products for the jacobian
    part.b2_x_a1 = vec3f::cross(part.b2, part.a1);
    part.c2_x_a1 = vec3f::cross(part.c2, part.a1);

    const Mat4f summed_inv_inertia = mat4f::add(part.inv_i1, part.inv_i2);

    // K = J M^-1 J^T (2x2), K[i,j] = row_i . (I_sum^-1 . row_j) with rows b2 x a1 and c2 x a1
    float inv_eff_mass[4];

    Vec3f temp = mat4f::multiply3x3_vec(summed_inv_inertia, part.b2_x_a1);
    inv_eff_mass[0] = vec3f::dot(part.b2_x_a1, temp);  // K[0,0]
    inv_eff_mass[2] = vec3f::dot(part.c2_x_a1, temp);  // K[1,0]

    temp = mat4f::multiply3x3_vec(summed_inv_inertia, part.c2_x_a1);
    inv_eff_mass[1] = vec3f::dot(part.b2_x_a1, temp);  // K[0,1]
    inv_eff_mass[3] = vec3f::dot(part.c2_x_a1, temp);  // K[1,1]

    // invert the 2x2 matrix to get the effective mass
    const float det = inv_eff_mass[0] * inv_eff_mass[3] - inv_eff_mass[1] * inv_eff_mass[2];
    if (scalar::abs(det) < 1e-10f) {
        deactivate(part);
        return;
    }

    const float inv_det = 1.0f / det;
    part.effective_mass[0] = inv_eff_mass[3] * inv_det;
    part.effective_mass[1] = -inv_eff_mass[1] * inv_det;
    part.effective_mass[2] = -inv_eff_mass[2] * inv_det;
    part.effective_mass[3] = inv_eff_mass[0] * inv_det;
}

/** P = J^T lambda = b2_x_a1 * lambda0 + c2_x_a1 * lambda1 */
static Vec3f impulse_of(const HingeRotationConstraintPart &part, float lambda0, float lambda1) {
    return vec3f::scale_and_add(vec3f::scale(part.b2_x_a1, lambda0), part.c2_x_a1, lambda1);
}

static bool apply_velocity_step(HingeRotationConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                                float lambda0, float lambda1) {
    if (lambda0 == 0.0f && lambda1 == 0.0f) return false;

    const Vec3f impulse = impulse_of(part, lambda0, lambda1);

    // body a: w -= I1^-1 * impulse
    if (body_a.motion_type == motion_type::MOTION_DYNAMIC) {
        motion_properties::sub_angular_velocity_step(body_a.motion_properties, mat4f::multiply3x3_vec(part.inv_i1, impulse));
    }

    // body b: w += I2^-1 * impulse
    if (body_b.motion_type == motion_type::MOTION_DYNAMIC) {
        motion_properties::add_angular_velocity_step(body_b.motion_properties, mat4f::multiply3x3_vec(part.inv_i2, impulse));
    }

    return true;
}

void warm_start(HingeRotationConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                float warm_start_impulse_ratio) {
    part.total_lambda[0] *= warm_start_impulse_ratio;
    part.total_lambda[1] *= warm_start_impulse_ratio;

    apply_velocity_step(part, body_a, body_b, part.total_lambda[0], part.total_lambda[1]);
}

bool solve_velocity_constraint(HingeRotationConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b) {
    if (!is_active(part)) return false;

    // jv[0] = b2_x_a1 . (w1 - w2), jv[1] = c2_x_a1 . (w1 - w2)
    float jv0 = 0.0f;
    float jv1 = 0.0f;

    if (body_a.motion_type != motion_type::MOTION_STATIC) {
        const Vec3f w1 = vec3f::load(body_a.motion_properties.angular_velocity);
        jv0 += vec3f::dot(part.b2_x_a1, w1);
        jv1 += vec3f::dot(part.c2_x_a1, w1);
    }
    if (body_b.motion_type != motion_type::MOTION_STATIC) {
        const Vec3f w2 = vec3f::load(body_b.motion_properties.angular_velocity);
        jv0 -= vec3f::dot(part.b2_x_a1, w2);
        jv1 -= vec3f::dot(part.c2_x_a1, w2);
    }

    // lambda = -K^-1 * (J * v)
    const float lambda0 = part.effective_mass[0] * jv0 + part.effective_mass[1] * jv1;
    const float lambda1 = part.effective_mass[2] * jv0 + part.effective_mass[3] * jv1;

    part.total_lambda[0] += lambda0;
    part.total_lambda[1] += lambda1;

    return apply_velocity_step(part, body_a, body_b, lambda0, lambda1);
}

bool solve_position_constraint(HingeRotationConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b,
                               float baumgarte) {
    if (!is_active(part)) return false;

    // C[0] = a1 . b2, C[1] = a1 . c2
    const float c0 = vec3f::dot(part.a1, part.b2);
    const float c1 = vec3f::dot(part.a1, part.c2);

    if (scalar::abs(c0) < 1e-10f && scalar::abs(c1) < 1e-10f) return false;

    // lambda = -K^-1 * beta * C. no division by dt: the position integration multiplies by it again
    const float lambda0 = -baumgarte * (part.effective_mass[0] * c0 + part.effective_mass[1] * c1);
    const float lambda1 = -baumgarte * (part.effective_mass[2] * c0 + part.effective_mass[3] * c1);

    const Vec3f impulse = impulse_of(part, lambda0, lambda1);
    if (body_a.motion_type == motion_type::MOTION_DYNAMIC) rigid_body_step::sub_rotation_step(&body_a, mat4f::multiply3x3_vec(part.inv_i1, impulse));
    if (body_b.motion_type == motion_type::MOTION_DYNAMIC) rigid_body_step::add_rotation_step(&body_b, mat4f::multiply3x3_vec(part.inv_i2, impulse));

    return true;
}

}  // namespace hinge_rotation_constraint_part
