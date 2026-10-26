#include "tumble/physics/collision/estimate_collision_response.h"

#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/body/motion_properties.h"
#include "tumble/physics/body/motion_type.h"
#include "tumble/physics/constraints/constraint_part/angular_friction_constraint_part.h"
#include "tumble/physics/constraints/constraint_part/contact_constraint_part.h"

TUMBLE_NAMESPACE_BEGIN

namespace estimate_collision_response {


// the solver's own parts, both sides dynamic: a static or kinematic body brings no inverse mass or inertia
typedef ContactConstraintPart ContactPart;
typedef AngularFrictionConstraintPart AngularPart;

/** the body's world inverse inertia, scaled; zero unless it is dynamic */
static Mat4f inverse_inertia_of(const RigidBody &body, float scale) {
    if (body.motion_type != MotionType::DYNAMIC) return mat4f::zero();
    const Mat4f inverse_inertia =
        motion_properties::get_inverse_inertia_for_rotation(body.motion_properties, mat4f::from_quat(quatf::load(body.quaternion)));
    return mat4f::multiply_scalar(inverse_inertia, scale);
}

void estimate_collision_response(CollisionEstimationResult &result, const RigidBody &body_1, const RigidBody &body_2,
                                 const ContactManifold &manifold, const ContactSettings &settings,
                                 float min_velocity_for_restitution, uint32_t num_iterations) {
    const uint32_t num_points = manifold.num_contact_points;
    const Vec3f normal = manifold.world_space_normal;

    // the friction directions
    const Vec3f tangent_1 = vec3f::normalized_perpendicular(normal);
    const Vec3f tangent_2 = vec3f::cross(normal, tangent_1);

    // the bodies' velocities, zero for a static body
    if (body_1.motion_type != MotionType::STATIC) {
        result.linear_velocity_1 = vec3f::load(body_1.motion_properties.linear_velocity);
        result.angular_velocity_1 = vec3f::load(body_1.motion_properties.angular_velocity);
    } else {
        result.linear_velocity_1 = vec3f::zero();
        result.angular_velocity_1 = vec3f::zero();
    }
    if (body_2.motion_type != MotionType::STATIC) {
        result.linear_velocity_2 = vec3f::load(body_2.motion_properties.linear_velocity);
        result.angular_velocity_2 = vec3f::load(body_2.motion_properties.angular_velocity);
    } else {
        result.linear_velocity_2 = vec3f::zero();
        result.angular_velocity_2 = vec3f::zero();
    }

    result.num_impulses = num_points;
    for (uint32_t c = 0; c < num_points; c++) result.contact_impulse[c] = 0.0f;
    result.friction_point = vec3f::zero();
    result.friction_impulse = vec3f::zero();
    result.angular_friction_impulse = 0.0f;

    // sensors have no collision response, and a manifold without points nothing to respond to
    if (settings.is_sensor || num_points == 0) return;

    // the inverse masses and inertias, scaled for this contact
    const float inv_m1 = body_1.motion_type == MotionType::DYNAMIC ? settings.inv_mass_scale_1 * body_1.motion_properties.inv_mass : 0.0f;
    const float inv_m2 = body_2.motion_type == MotionType::DYNAMIC ? settings.inv_mass_scale_2 * body_2.motion_properties.inv_mass : 0.0f;
    const Mat4f inv_i1 = inverse_inertia_of(body_1, settings.inv_inertia_scale_1);
    const Mat4f inv_i2 = inverse_inertia_of(body_2, settings.inv_inertia_scale_2);

    // the centres of mass relative to the base offset
    const Vec3f com_1 = vec3f::subtract(vec3f::load(body_1.center_of_mass_position), manifold.base_offset);
    const Vec3f com_2 = vec3f::subtract(vec3f::load(body_2.center_of_mass_position), manifold.base_offset);

    // the non-penetration parts, and each body's friction point as the sum of its own contact points
    ContactPart contact_constraints[manifold::MAX_CLIPPING_CONTACT_POINTS];
    Vec3f friction_point_1 = vec3f::zero();
    Vec3f friction_point_2 = vec3f::zero();
    for (uint32_t c = 0; c < num_points; c++) {
        const Vec3f p1 = manifold.relative_contact_points_on_a[c];
        const Vec3f p2 = manifold.relative_contact_points_on_b[c];

        // per body, so the friction lever arm is not perturbed by the separation along the normal of a speculative contact
        friction_point_1 = vec3f::add(friction_point_1, p1);
        friction_point_2 = vec3f::add(friction_point_2, p2);

        // the contact point relative to each centre of mass
        const Vec3f r1 = vec3f::subtract(p1, com_1);
        const Vec3f r2 = vec3f::subtract(p2, com_2);

        // elastic collisions: restitution when the approach is fast enough
        float bias = 0.0f;
        if (settings.combined_restitution > 0.0f) {
            const Vec3f velocity_1 = vec3f::add(result.linear_velocity_1, vec3f::cross(result.angular_velocity_1, r1));
            const Vec3f velocity_2 = vec3f::add(result.linear_velocity_2, vec3f::cross(result.angular_velocity_2, r2));
            const float normal_velocity = vec3f::dot(vec3f::subtract(velocity_2, velocity_1), normal);
            if (normal_velocity < -min_velocity_for_restitution) bias = settings.combined_restitution * normal_velocity;
        }

        ContactPart &constraint = contact_constraints[c];
        constraint.total_lambda = 0.0f;
        contact_constraint_part::calculate_constraint_properties<MotionType::DYNAMIC, MotionType::DYNAMIC>(constraint, inv_m1, inv_i1, r1, inv_m2, inv_i2,
                                                                                               r2, normal, bias);
    }

    // each point's distance to body 1's friction point in the contact plane
    const Vec3f count = vec3f::splat((float)num_points);
    friction_point_1 = vec3f::divide(friction_point_1, count);
    friction_point_2 = vec3f::divide(friction_point_2, count);
    float distance_to_friction_center[manifold::MAX_CLIPPING_CONTACT_POINTS];
    for (uint32_t c = 0; c < num_points; c++) {
        const Vec3f delta = vec3f::subtract(manifold.relative_contact_points_on_a[c], friction_point_1);
        distance_to_friction_center[c] = vec3f::length(vec3f::scale_and_add(delta, normal, -vec3f::dot(delta, normal)));
    }
    result.friction_point = vec3f::scale(vec3f::add(friction_point_1, friction_point_2), 0.5f);

    // the friction parts, their bias the surface velocity along each tangent
    ContactPart friction_1 = {};
    ContactPart friction_2 = {};
    AngularPart angular_friction = {};
    if (settings.combined_friction > 0.0f) {
        const Vec3f r1 = vec3f::subtract(friction_point_1, com_1);
        const Vec3f r2 = vec3f::subtract(friction_point_2, com_2);
        const Vec3f relative_angular_surface_velocity = vec3f::load(settings.relative_angular_surface_velocity);
        const Vec3f surface_velocity = vec3f::add(vec3f::load(settings.relative_linear_surface_velocity),
                                                  vec3f::cross(relative_angular_surface_velocity, friction_point_1));
        contact_constraint_part::calculate_constraint_properties<MotionType::DYNAMIC, MotionType::DYNAMIC>(friction_1, inv_m1, inv_i1, r1, inv_m2, inv_i2, r2,
                                                                                               tangent_1, vec3f::dot(tangent_1, surface_velocity));
        contact_constraint_part::calculate_constraint_properties<MotionType::DYNAMIC, MotionType::DYNAMIC>(friction_2, inv_m1, inv_i1, r1, inv_m2, inv_i2, r2,
                                                                                               tangent_2, vec3f::dot(tangent_2, surface_velocity));
        if (num_points > 1) {
            angular_friction_constraint_part::calculate_constraint_properties<MotionType::DYNAMIC, MotionType::DYNAMIC>(
                angular_friction, inv_i1, inv_i2, normal, vec3f::dot(relative_angular_surface_velocity, normal));
        }
    }

    // a single point without friction needs only one iteration
    const uint32_t iterations = settings.combined_friction <= 0.0f && num_points == 1 ? 1 : num_iterations;
    for (uint32_t iteration = 0; iteration < iterations; iteration++) {
        // friction first, capped by the non-penetration impulses so far
        if (settings.combined_friction > 0.0f) {
            float max_linear_lambda = 0.0f;
            float max_angular_lambda = 0.0f;
            for (uint32_t c = 0; c < num_points; c++) {
                const float lambda = contact_constraints[c].total_lambda;
                max_linear_lambda += lambda;
                max_angular_lambda += distance_to_friction_center[c] * lambda;
            }
            max_linear_lambda *= settings.combined_friction;
            max_angular_lambda *= settings.combined_friction;

            // the impulse to stop the motion along the tangents, scaled back onto the friction cone
            float lambda_1 = contact_constraint_part::solve_velocity_constraint_get_total_lambda<MotionType::DYNAMIC, MotionType::DYNAMIC>(
                friction_1, result.linear_velocity_1, result.angular_velocity_1, result.linear_velocity_2, result.angular_velocity_2, tangent_1);
            float lambda_2 = contact_constraint_part::solve_velocity_constraint_get_total_lambda<MotionType::DYNAMIC, MotionType::DYNAMIC>(
                friction_2, result.linear_velocity_1, result.angular_velocity_1, result.linear_velocity_2, result.angular_velocity_2, tangent_2);
            // the + FLOAT_MIN_NORMAL keeps a total_lambda_sq too small to divide by from being scaled
            const float total_lambda_sq = lambda_1 * lambda_1 + lambda_2 * lambda_2;
            if (total_lambda_sq > max_linear_lambda * max_linear_lambda + scalar::FLOAT_MIN_NORMAL) {
                const float scale = max_linear_lambda / scalar::sqrt(total_lambda_sq);
                lambda_1 *= scale;
                lambda_2 *= scale;
            }
            contact_constraint_part::solve_velocity_constraint_apply_lambda<MotionType::DYNAMIC, MotionType::DYNAMIC>(
                friction_1, result.linear_velocity_1, result.angular_velocity_1, result.linear_velocity_2, result.angular_velocity_2, inv_m1, inv_m2,
                tangent_1, lambda_1);
            contact_constraint_part::solve_velocity_constraint_apply_lambda<MotionType::DYNAMIC, MotionType::DYNAMIC>(
                friction_2, result.linear_velocity_1, result.angular_velocity_1, result.linear_velocity_2, result.angular_velocity_2, inv_m1, inv_m2,
                tangent_2, lambda_2);

            if (num_points > 1) {
                angular_friction_constraint_part::solve_velocity_constraint<MotionType::DYNAMIC, MotionType::DYNAMIC>(
                    angular_friction, result.angular_velocity_1, result.angular_velocity_2, normal, -max_angular_lambda, max_angular_lambda);
            }
        }

        // non-penetration last: contacts only push
        for (uint32_t c = 0; c < num_points; c++) {
            contact_constraint_part::solve_velocity_constraint<MotionType::DYNAMIC, MotionType::DYNAMIC>(
                contact_constraints[c], result.linear_velocity_1, result.angular_velocity_1, result.linear_velocity_2, result.angular_velocity_2,
                inv_m1, inv_m2, normal, 0.0f, scalar::FLOAT_MAX);
        }
    }

    for (uint32_t c = 0; c < num_points; c++) result.contact_impulse[c] = contact_constraints[c].total_lambda;
    result.friction_impulse = vec3f::scale_and_add(vec3f::scale(tangent_1, friction_1.total_lambda), tangent_2, friction_2.total_lambda);
    result.angular_friction_impulse = angular_friction.total_lambda;
}

}  // namespace estimate_collision_response

TUMBLE_NAMESPACE_END
