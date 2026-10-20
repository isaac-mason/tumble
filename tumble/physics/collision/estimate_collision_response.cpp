#include "tumble/physics/collision/estimate_collision_response.h"

#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/motion_properties.h"
#include "tumble/physics/body/motion_type.h"

TUMBLE_NAMESPACE_BEGIN

namespace estimate_collision_response {

/** lightweight axis constraint for collision estimation (mirrors AxisConstraintPart) */
struct MiniAxisConstraint {
    Float3 r1_plus_u_x_axis;
    Float3 r2_x_axis;
    Float3 inv_i1_r1_plus_u_x_axis;
    Float3 inv_i2_r2_x_axis;
    float effective_mass;
    float bias;
};

/** lightweight angular-only constraint for the angular friction part */
struct MiniAngularConstraint {
    Float3 inv_i1_axis;
    Float3 inv_i2_axis;
    float effective_mass;
    float bias;
};

/** initialize a mini axis constraint */
static void init_constraint(MiniAxisConstraint &constraint, float inv_mass_1, float inv_mass_2, const Mat4f &inv_inertia_1,
                            const Mat4f &inv_inertia_2, Vec3f r1, Vec3f r2, Vec3f axis) {
    const Vec3f r1_cross_axis = vec3f::cross(r1, axis);
    const Vec3f r2_cross_axis = vec3f::cross(r2, axis);

    vec3f::store(constraint.r1_plus_u_x_axis, r1_cross_axis);
    vec3f::store(constraint.r2_x_axis, r2_cross_axis);

    vec3f::store(constraint.inv_i1_r1_plus_u_x_axis, mat4f::multiply3x3_vec(inv_inertia_1, r1_cross_axis));
    vec3f::store(constraint.inv_i2_r2_x_axis, mat4f::multiply3x3_vec(inv_inertia_2, r2_cross_axis));

    float inv_effective_mass = inv_mass_1 + inv_mass_2;
    inv_effective_mass += vec3f::dot(r1_cross_axis, vec3f::load(constraint.inv_i1_r1_plus_u_x_axis));
    inv_effective_mass += vec3f::dot(r2_cross_axis, vec3f::load(constraint.inv_i2_r2_x_axis));

    constraint.effective_mass = inv_effective_mass > 0.0f ? 1.0f / inv_effective_mass : 0.0f;
}

/** initialize the mini angular friction constraint (axis is the contact normal) */
static void init_angular_constraint(MiniAngularConstraint &constraint, const Mat4f &inv_inertia_1, const Mat4f &inv_inertia_2,
                                    Vec3f axis) {
    vec3f::store(constraint.inv_i1_axis, mat4f::multiply3x3_vec(inv_inertia_1, axis));
    vec3f::store(constraint.inv_i2_axis, mat4f::multiply3x3_vec(inv_inertia_2, axis));

    const float inv_effective_mass = vec3f::dot(axis, vec3f::load(constraint.inv_i1_axis)) + vec3f::dot(axis, vec3f::load(constraint.inv_i2_axis));
    constraint.effective_mass = inv_effective_mass > 0.0f ? 1.0f / inv_effective_mass : 0.0f;
}

/** relative velocity along the constraint axis */
static float get_relative_velocity(const MiniAxisConstraint &constraint, Vec3f lin_vel_1, Vec3f ang_vel_1, Vec3f lin_vel_2,
                                   Vec3f ang_vel_2, Vec3f axis) {
    // the linear and angular terms summed per component, reduced once
    Vec3f acc = vec3f::multiply(axis, vec3f::subtract(lin_vel_1, lin_vel_2));
    acc = vec3f::add(acc, vec3f::multiply(vec3f::load(constraint.r1_plus_u_x_axis), ang_vel_1));
    acc = vec3f::subtract(acc, vec3f::multiply(vec3f::load(constraint.r2_x_axis), ang_vel_2));
    return vec3f::x(acc) + vec3f::y(acc) + vec3f::z(acc);
}

/** the unclamped lambda increment */
static float solve_get_lambda(const MiniAxisConstraint &constraint, Vec3f lin_vel_1, Vec3f ang_vel_1, Vec3f lin_vel_2,
                              Vec3f ang_vel_2, Vec3f axis) {
    const float jv = get_relative_velocity(constraint, lin_vel_1, ang_vel_1, lin_vel_2, ang_vel_2, axis);
    return constraint.effective_mass * (jv - constraint.bias);
}

/** apply a delta lambda to the velocities */
static void apply_lambda(const MiniAxisConstraint &constraint, float lambda, float inv_mass_1, float inv_mass_2, Vec3f &lin_vel_1, Vec3f &ang_vel_1,
                         Vec3f &lin_vel_2, Vec3f &ang_vel_2, Vec3f axis) {
    lin_vel_1 = vec3f::scale_and_add(lin_vel_1, axis, -lambda * inv_mass_1);
    ang_vel_1 = vec3f::scale_and_add(ang_vel_1, vec3f::load(constraint.inv_i1_r1_plus_u_x_axis), -lambda);
    lin_vel_2 = vec3f::scale_and_add(lin_vel_2, axis, lambda * inv_mass_2);
    ang_vel_2 = vec3f::scale_and_add(ang_vel_2, vec3f::load(constraint.inv_i2_r2_x_axis), lambda);
}

/** solve with clamping, returning the new total lambda */
static float solve(const MiniAxisConstraint &constraint, float current_total_lambda, float min_lambda, float max_lambda, float inv_mass_1,
                   float inv_mass_2, Vec3f &lin_vel_1, Vec3f &ang_vel_1, Vec3f &lin_vel_2, Vec3f &ang_vel_2, Vec3f axis) {
    const float lambda = solve_get_lambda(constraint, lin_vel_1, ang_vel_1, lin_vel_2, ang_vel_2, axis);
    const float new_total_lambda = scalar::max(min_lambda, scalar::min(max_lambda, current_total_lambda + lambda));
    const float delta_lambda = new_total_lambda - current_total_lambda;
    if (delta_lambda != 0.0f) {
        apply_lambda(constraint, delta_lambda, inv_mass_1, inv_mass_2, lin_vel_1, ang_vel_1, lin_vel_2, ang_vel_2, axis);
    }
    return new_total_lambda;
}

/** angular friction: the unclamped lambda increment (jv = axis . (w1 - w2)) */
static float solve_get_angular_lambda(const MiniAngularConstraint &constraint, Vec3f ang_vel_1, Vec3f ang_vel_2, Vec3f axis) {
    const float jv = vec3f::dot(axis, ang_vel_1) - vec3f::dot(axis, ang_vel_2);
    return constraint.effective_mass * (jv - constraint.bias);
}

/** angular friction: apply delta lambda (no linear terms) */
static void apply_angular_lambda(const MiniAngularConstraint &constraint, float lambda, Vec3f &ang_vel_1, Vec3f &ang_vel_2) {
    ang_vel_1 = vec3f::scale_and_add(ang_vel_1, vec3f::load(constraint.inv_i1_axis), -lambda);
    ang_vel_2 = vec3f::scale_and_add(ang_vel_2, vec3f::load(constraint.inv_i2_axis), lambda);
}

void estimate_collision_response(CollisionEstimationResult &result, const rigid_body::RigidBody &body_1, const rigid_body::RigidBody &body_2,
                                 const manifold::ContactManifold &manifold, float combined_friction, float combined_restitution,
                                 float min_velocity_for_restitution, uint32_t num_iterations) {
    MiniAxisConstraint normal_constraints[manifold::MAX_CLIPPING_CONTACT_POINTS];
    float distance_to_friction_center[manifold::MAX_CLIPPING_CONTACT_POINTS];
    MiniAxisConstraint friction_constraint_1;
    MiniAxisConstraint friction_constraint_2;
    MiniAngularConstraint angular_friction_constraint = {};

    const uint32_t num_contact_points = manifold.num_contact_points;
    result.num_impulses = num_contact_points;

    for (uint32_t i = 0; i < num_contact_points; i++) result.contact_impulse[i] = 0.0f;
    result.friction_impulse = vec3f::zero();
    result.angular_friction_impulse = 0.0f;

    // friction basis: tangent_1 perpendicular to normal, tangent_2 = normal x tangent_1
    const Vec3f normal = manifold.world_space_normal;
    const Vec3f tangent_1 = vec3f::normalize(vec3f::perpendicular(normal));
    const Vec3f tangent_2 = vec3f::cross(normal, tangent_1);

    // copy body velocities (zero only for static)
    if (body_1.motion_type != motion_type::MOTION_STATIC) {
        result.linear_velocity_1 = vec3f::load(body_1.motion_properties.linear_velocity);
        result.angular_velocity_1 = vec3f::load(body_1.motion_properties.angular_velocity);
    } else {
        result.linear_velocity_1 = vec3f::zero();
        result.angular_velocity_1 = vec3f::zero();
    }

    if (body_2.motion_type != motion_type::MOTION_STATIC) {
        result.linear_velocity_2 = vec3f::load(body_2.motion_properties.linear_velocity);
        result.angular_velocity_2 = vec3f::load(body_2.motion_properties.angular_velocity);
    } else {
        result.linear_velocity_2 = vec3f::zero();
        result.angular_velocity_2 = vec3f::zero();
    }

    // inverse mass / inertia: only dynamic bodies contribute
    const bool is_dynamic_1 = body_1.motion_type == motion_type::MOTION_DYNAMIC;
    const bool is_dynamic_2 = body_2.motion_type == motion_type::MOTION_DYNAMIC;
    const float inv_mass_1 = is_dynamic_1 ? body_1.motion_properties.inv_mass : 0.0f;
    const float inv_mass_2 = is_dynamic_2 ? body_2.motion_properties.inv_mass : 0.0f;

    const Mat4f inv_inertia_1 =
        is_dynamic_1 ? motion_properties::get_inverse_inertia_for_rotation(body_1.motion_properties, mat4f::from_quat(quatf::load(body_1.quaternion)))
                     : mat4f::zero();
    const Mat4f inv_inertia_2 =
        is_dynamic_2 ? motion_properties::get_inverse_inertia_for_rotation(body_2.motion_properties, mat4f::from_quat(quatf::load(body_2.quaternion)))
                     : mat4f::zero();

    // com relative to manifold.base_offset
    const Vec3f com_1 = vec3f::subtract(vec3f::load(body_1.center_of_mass_position), manifold.base_offset);
    const Vec3f com_2 = vec3f::subtract(vec3f::load(body_2.center_of_mass_position), manifold.base_offset);

    // per-contact-point setup: each body's own contact point gives its moment arm. each body's friction point accumulates
    // its own points, so the friction lever arms are not perturbed by the separation along the normal of a speculative contact
    Vec3f friction_point_1 = vec3f::zero();
    Vec3f friction_point_2 = vec3f::zero();

    Vec3f r1, r2;
    for (uint32_t c = 0; c < num_contact_points; c++) {
        const Vec3f a = manifold.relative_contact_points_on_a[c];
        const Vec3f b = manifold.relative_contact_points_on_b[c];
        friction_point_1 = vec3f::add(friction_point_1, a);
        friction_point_2 = vec3f::add(friction_point_2, b);

        r1 = vec3f::subtract(a, com_1);
        r2 = vec3f::subtract(b, com_2);

        MiniAxisConstraint &normal_constraint = normal_constraints[c];
        init_constraint(normal_constraint, inv_mass_1, inv_mass_2, inv_inertia_1, inv_inertia_2, r1, r2, normal);

        normal_constraint.bias = 0.0f;
        if (combined_restitution > 0.0f) {
            Vec3f vel_1_at_point = vec3f::cross(result.angular_velocity_1, r1);
            Vec3f vel_2_at_point, relative_velocity;
            vel_1_at_point = vec3f::add(result.linear_velocity_1, vel_1_at_point);
            vel_2_at_point = vec3f::cross(result.angular_velocity_2, r2);
            vel_2_at_point = vec3f::add(result.linear_velocity_2, vel_2_at_point);
            relative_velocity = vec3f::subtract(vel_2_at_point, vel_1_at_point);

            const float vn = vec3f::dot(relative_velocity, normal);
            if (vn < -min_velocity_for_restitution) {
                normal_constraint.bias = combined_restitution * vn;
            }
        }
    }

    if (num_contact_points == 0) return;

    // each body's friction point = the mean of its contact points; friction is reported halfway between them
    const float count = (float)num_contact_points;
    friction_point_1 = vec3f::divide(friction_point_1, vec3f::splat(count));
    friction_point_2 = vec3f::divide(friction_point_2, vec3f::splat(count));
    result.friction_point = vec3f::scale(vec3f::add(friction_point_1, friction_point_2), 0.5f);

    // per-point moment arm to body 1's friction point, projected onto the contact plane
    for (uint32_t c = 0; c < num_contact_points; c++) {
        Vec3f delta = vec3f::subtract(manifold.relative_contact_points_on_a[c], friction_point_1);
        delta = vec3f::scale_and_add(delta, normal, -vec3f::dot(delta, normal));
        distance_to_friction_center[c] = vec3f::length(delta);
    }

    // manifold-level friction constraints anchored at each body's friction point
    if (combined_friction > 0.0f) {
        r1 = vec3f::subtract(friction_point_1, com_1);
        r2 = vec3f::subtract(friction_point_2, com_2);

        init_constraint(friction_constraint_1, inv_mass_1, inv_mass_2, inv_inertia_1, inv_inertia_2, r1, r2, tangent_1);
        friction_constraint_1.bias = 0.0f;

        init_constraint(friction_constraint_2, inv_mass_1, inv_mass_2, inv_inertia_1, inv_inertia_2, r1, r2, tangent_2);
        friction_constraint_2.bias = 0.0f;

        if (num_contact_points > 1) {
            init_angular_constraint(angular_friction_constraint, inv_inertia_1, inv_inertia_2, normal);
            angular_friction_constraint.bias = 0.0f;
        } else {
            angular_friction_constraint.effective_mass = 0.0f;
        }
    }

    // iterative pgs solve. caps are derived from the previous iteration's accumulated
    // contact impulse (matches the real solver, no special "skip iter 0" branch)
    float friction_lambda_1 = 0.0f;
    float friction_lambda_2 = 0.0f;
    const uint32_t iterations = num_contact_points == 1 && combined_friction == 0.0f ? 1 : num_iterations;

    for (uint32_t iteration = 0; iteration < iterations; iteration++) {
        if (combined_friction > 0.0f) {
            // friction caps from current contact impulses
            float sum_normal = 0.0f;
            float sum_distance_weighted = 0.0f;
            for (uint32_t c = 0; c < num_contact_points; c++) {
                const float ln = result.contact_impulse[c];
                sum_normal += ln;
                sum_distance_weighted += distance_to_friction_center[c] * ln;
            }

            // joint cone clamp on the two linear friction parts
            float lambda_1 = friction_lambda_1 + solve_get_lambda(friction_constraint_1, result.linear_velocity_1, result.angular_velocity_1,
                                                                          result.linear_velocity_2, result.angular_velocity_2, tangent_1);
            float lambda_2 = friction_lambda_2 + solve_get_lambda(friction_constraint_2, result.linear_velocity_1, result.angular_velocity_1,
                                                                          result.linear_velocity_2, result.angular_velocity_2, tangent_2);

            const float max_linear = combined_friction * sum_normal;
            const float mag_sq = lambda_1 * lambda_1 + lambda_2 * lambda_2;
            // the + FLOAT_MIN_NORMAL keeps a mag_sq too small to divide by from being scaled
            if (mag_sq > max_linear * max_linear + scalar::FLOAT_MIN_NORMAL) {
                const float scale = max_linear / scalar::sqrt(mag_sq);
                lambda_1 *= scale;
                lambda_2 *= scale;
            }

            apply_lambda(friction_constraint_1, lambda_1 - friction_lambda_1, inv_mass_1, inv_mass_2, result.linear_velocity_1,
                         result.angular_velocity_1, result.linear_velocity_2, result.angular_velocity_2, tangent_1);
            apply_lambda(friction_constraint_2, lambda_2 - friction_lambda_2, inv_mass_1, inv_mass_2, result.linear_velocity_1,
                         result.angular_velocity_1, result.linear_velocity_2, result.angular_velocity_2, tangent_2);
            friction_lambda_1 = lambda_1;
            friction_lambda_2 = lambda_2;

            // angular friction with symmetric clamp
            if (angular_friction_constraint.effective_mass != 0.0f) {
                const float lambda_angular_increment =
                    solve_get_angular_lambda(angular_friction_constraint, result.angular_velocity_1, result.angular_velocity_2, normal);
                const float max_angular = combined_friction * sum_distance_weighted;
                const float candidate = result.angular_friction_impulse + lambda_angular_increment;
                const float clamped = scalar::max(-max_angular, scalar::min(max_angular, candidate));
                const float delta_angular = clamped - result.angular_friction_impulse;
                if (delta_angular != 0.0f) {
                    apply_angular_lambda(angular_friction_constraint, delta_angular, result.angular_velocity_1, result.angular_velocity_2);
                }
                result.angular_friction_impulse = clamped;
            }
        }

        // normal pass
        for (uint32_t c = 0; c < num_contact_points; c++) {
            result.contact_impulse[c] =
                solve(normal_constraints[c], result.contact_impulse[c], 0.0f, scalar::FLOAT_MAX, inv_mass_1, inv_mass_2, result.linear_velocity_1,
                      result.angular_velocity_1, result.linear_velocity_2, result.angular_velocity_2, normal);
        }
    }

    result.friction_impulse = vec3f::scale(tangent_1, friction_lambda_1);
    result.friction_impulse = vec3f::scale_and_add(result.friction_impulse, tangent_2, friction_lambda_2);
}

}  // namespace estimate_collision_response

TUMBLE_NAMESPACE_END
