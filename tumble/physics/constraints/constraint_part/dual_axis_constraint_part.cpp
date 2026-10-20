#include "physics/constraints/constraint_part/dual_axis_constraint_part.h"

#include "math/mat2f.h"
#include "math/mat4f.h"
#include "math/quatf.h"
#include "math/vec2f.h"
#include "math/vec3f.h"
#include "physics/body/motion_properties.h"
#include "physics/body/motion_type.h"
#include "physics/body/rigid_body.h"
#include "physics/body/rigid_body_step.h"

namespace dual_axis_constraint_part {

void calculate_constraint_properties(DualAxisConstraintPart &part, rigid_body::RigidBody &body_a, Vec3f r1_plus_u,
                                     rigid_body::RigidBody &body_b, Vec3f r2, Vec3f n1, Vec3f n2, uint32_t step_stamp) {
    // the cross products are needed for kinematic bodies too (their angular velocity enters the jacobian),
    // the inverse inertia terms only for dynamic ones
    part.r1_plus_u_x_n1 = vec3f::cross(r1_plus_u, n1);
    part.r1_plus_u_x_n2 = vec3f::cross(r1_plus_u, n2);
    part.r2_x_n1 = vec3f::cross(r2, n1);
    part.r2_x_n2 = vec3f::cross(r2, n2);

    // column-major: [m00, m10, m01, m11]
    Mat2f inv_effective_mass;
    mat2f::set(inv_effective_mass, 0.0f, 0.0f, 0.0f, 0.0f);

    // contributions from body a
    if (body_a.motion_type == motion_type::MOTION_DYNAMIC) {
        motion_properties::MotionProperties &mp_a = body_a.motion_properties;

        const Mat4f inv_i_a = motion_properties::get_world_inverse_inertia(mp_a, quatf::load(body_a.quaternion), step_stamp);

        // I^-1 x (r x n) (3x3 only)
        const Vec3f inv_i_cross_n1 = mat4f::multiply3x3_vec(inv_i_a, part.r1_plus_u_x_n1);
        const Vec3f inv_i_cross_n2 = mat4f::multiply3x3_vec(inv_i_a, part.r1_plus_u_x_n2);
        part.inv_i1_r1_plus_u_x_n1 = inv_i_cross_n1;
        part.inv_i1_r1_plus_u_x_n2 = inv_i_cross_n2;

        // K^-1 = J M^-1 J^T
        inv_effective_mass.m[0] += mp_a.inv_mass + vec3f::dot(part.r1_plus_u_x_n1, inv_i_cross_n1);  // m00
        inv_effective_mass.m[1] += vec3f::dot(part.r1_plus_u_x_n2, inv_i_cross_n1);                  // m10
        inv_effective_mass.m[2] += vec3f::dot(part.r1_plus_u_x_n1, inv_i_cross_n2);                  // m01
        inv_effective_mass.m[3] += mp_a.inv_mass + vec3f::dot(part.r1_plus_u_x_n2, inv_i_cross_n2);  // m11
    }

    // contributions from body b
    if (body_b.motion_type == motion_type::MOTION_DYNAMIC) {
        motion_properties::MotionProperties &mp_b = body_b.motion_properties;

        const Mat4f inv_i_b = motion_properties::get_world_inverse_inertia(mp_b, quatf::load(body_b.quaternion), step_stamp);

        const Vec3f inv_i_cross_n1 = mat4f::multiply3x3_vec(inv_i_b, part.r2_x_n1);
        const Vec3f inv_i_cross_n2 = mat4f::multiply3x3_vec(inv_i_b, part.r2_x_n2);
        part.inv_i2_r2_x_n1 = inv_i_cross_n1;
        part.inv_i2_r2_x_n2 = inv_i_cross_n2;

        inv_effective_mass.m[0] += mp_b.inv_mass + vec3f::dot(part.r2_x_n1, inv_i_cross_n1);  // m00
        inv_effective_mass.m[1] += vec3f::dot(part.r2_x_n2, inv_i_cross_n1);                  // m10
        inv_effective_mass.m[2] += vec3f::dot(part.r2_x_n1, inv_i_cross_n2);                  // m01
        inv_effective_mass.m[3] += mp_b.inv_mass + vec3f::dot(part.r2_x_n2, inv_i_cross_n2);  // m11
    }

    if (!mat2f::invert(part.effective_mass, inv_effective_mass)) deactivate(part);
}

/** v1 * lambda.x + v2 * lambda.y */
static Vec3f combine(Vec3f v1, Vec3f v2, Vec2f lambda) { return vec3f::scale_and_add(vec3f::scale(v1, lambda.x), v2, lambda.y); }

/** the impulse along n1 and n2, lambda a velocity or a position step, through each dynamic body */
template <bool POSITION>
static void apply(const DualAxisConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b, Vec3f n1,
                  Vec3f n2, Vec2f lambda) {
    // P = n1 * lambda[0] + n2 * lambda[1]
    const Vec3f impulse = combine(n1, n2, lambda);

    // body a (subtract)
    if (body_a.motion_type == motion_type::MOTION_DYNAMIC) {
        const Vec3f linear = vec3f::scale(impulse, body_a.motion_properties.inv_mass);
        // w -= lambda[0] * inv_i1_r1_plus_u_x_n1 + lambda[1] * inv_i1_r1_plus_u_x_n2
        const Vec3f angular = combine(part.inv_i1_r1_plus_u_x_n1, part.inv_i1_r1_plus_u_x_n2, lambda);
        if (POSITION) {
            rigid_body_step::sub_position_step(&body_a, linear);
            rigid_body_step::sub_rotation_step(&body_a, angular);
        } else {
            motion_properties::sub_linear_velocity_step(body_a.motion_properties, linear);
            motion_properties::sub_angular_velocity_step(body_a.motion_properties, angular);
        }
    }

    // body b (add)
    if (body_b.motion_type == motion_type::MOTION_DYNAMIC) {
        const Vec3f linear = vec3f::scale(impulse, body_b.motion_properties.inv_mass);
        // w += lambda[0] * inv_i2_r2_x_n1 + lambda[1] * inv_i2_r2_x_n2
        const Vec3f angular = combine(part.inv_i2_r2_x_n1, part.inv_i2_r2_x_n2, lambda);
        if (POSITION) {
            rigid_body_step::add_position_step(&body_b, linear);
            rigid_body_step::add_rotation_step(&body_b, angular);
        } else {
            motion_properties::add_linear_velocity_step(body_b.motion_properties, linear);
            motion_properties::add_angular_velocity_step(body_b.motion_properties, angular);
        }
    }
}

void warm_start(DualAxisConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b, Vec3f n1, Vec3f n2,
                float warm_start_impulse_ratio) {
    if (!is_active(part)) return;

    part.total_lambda.x *= warm_start_impulse_ratio;
    part.total_lambda.y *= warm_start_impulse_ratio;

    if (part.total_lambda.x == 0.0f && part.total_lambda.y == 0.0f) return;

    apply<false>(part, body_a, body_b, n1, n2, part.total_lambda);
}

bool solve_velocity_constraint(DualAxisConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b, Vec3f n1,
                               Vec3f n2) {
    if (!is_active(part)) return false;

    // jv[0] = n1 . (v1 - v2) + (r1 + u) x n1 . w1 - r2 x n1 . w2
    // jv[1] = n2 . (v1 - v2) + (r1 + u) x n2 . w1 - r2 x n2 . w2

    // velocity: dynamic and kinematic bodies
    const bool moving_a = body_a.motion_type != motion_type::MOTION_STATIC;
    const bool moving_b = body_b.motion_type != motion_type::MOTION_STATIC;
    const motion_properties::MotionProperties &mp_a = body_a.motion_properties;
    const motion_properties::MotionProperties &mp_b = body_b.motion_properties;

    // linear contribution
    Vec2f jv = {0.0f, 0.0f};

    if (moving_a && moving_b) {
        const Vec3f va = vec3f::load(mp_a.linear_velocity);
        const Vec3f vb = vec3f::load(mp_b.linear_velocity);
        jv.x = vec3f::dot(n1, va) - vec3f::dot(n1, vb);
        jv.y = vec3f::dot(n2, va) - vec3f::dot(n2, vb);
    } else if (moving_a) {
        const Vec3f va = vec3f::load(mp_a.linear_velocity);
        jv.x = vec3f::dot(n1, va);
        jv.y = vec3f::dot(n2, va);
    } else if (moving_b) {
        const Vec3f vb = vec3f::load(mp_b.linear_velocity);
        jv.x = -vec3f::dot(n1, vb);
        jv.y = -vec3f::dot(n2, vb);
    }

    // angular contribution
    if (moving_a) {
        const Vec3f wa = vec3f::load(mp_a.angular_velocity);
        jv.x += vec3f::dot(part.r1_plus_u_x_n1, wa);
        jv.y += vec3f::dot(part.r1_plus_u_x_n2, wa);
    }
    if (moving_b) {
        const Vec3f wb = vec3f::load(mp_b.angular_velocity);
        jv.x -= vec3f::dot(part.r2_x_n1, wb);
        jv.y -= vec3f::dot(part.r2_x_n2, wb);
    }

    // lambda = K * jv; jv already has the velocity error in the right direction
    Vec2f lambda;
    vec2f::transform_mat2(lambda, jv, part.effective_mass);

    part.total_lambda.x += lambda.x;
    part.total_lambda.y += lambda.y;

    if (lambda.x == 0.0f && lambda.y == 0.0f) return false;

    apply<false>(part, body_a, body_b, n1, n2, lambda);
    return true;
}

bool solve_position_constraint(DualAxisConstraintPart &part, rigid_body::RigidBody &body_a, rigid_body::RigidBody &body_b, Vec3f u,
                               Vec3f n1, Vec3f n2, float baumgarte) {
    if (!is_active(part)) return false;

    // C = [u . n1, u . n2]
    const Vec2f c = {vec3f::dot(u, n1), vec3f::dot(u, n2)};

    if (c.x == 0.0f && c.y == 0.0f) return false;

    // lambda = -baumgarte * K^-1 * C
    Vec2f lambda;
    vec2f::transform_mat2(lambda, c, part.effective_mass);
    vec2f::scale(lambda, lambda, -baumgarte);

    if (lambda.x == 0.0f && lambda.y == 0.0f) return false;

    apply<true>(part, body_a, body_b, n1, n2, lambda);
    return true;
}

}  // namespace dual_axis_constraint_part
