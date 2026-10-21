#include "tumble/physics/shapes/convex.h"

#include "tumble/math/aabox.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/quatf.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/collision/gjk.h"
#include "tumble/physics/collision/penetration.h"
#include "tumble/physics/collision/simplex.h"
#include "tumble/physics/collision/support.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

namespace convex {


int32_t get_convex_material(const Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    TUMBLE_ASSERT(sub_shape::is_empty(sub_shape_id));
    (void)sub_shape_id;
    return shape->material_id;
}

/* cast ray */

static support::Support cast_ray_support;

void cast_ray_vs_convex(cast_ray::CastRayCollector &collector, const cast_ray::CastRaySettings &settings, const cast_ray::Rayf &ray,
                        const Shape *shape, const sub_shape::SubShapeIdBuilder &sub_shape_id, const Transformf &transform) {
    shapes::set_shape_support(cast_ray_support, shape, support::INCLUDE_CONVEX_RADIUS, vec3f::load(transform.scale));

    // transform ray from world space to shape local space
    // first, compute inverse quaternion
    const Quatf inv_quat = quatf::conjugate(quatf::load(transform.quaternion));

    // transform ray origin: (origin - position) rotated by inverse quaternion
    Vec3f ray_origin_local = vec3f::subtract(ray.origin, vec3f::load(transform.position));
    ray_origin_local = vec3f::transform_quat(ray_origin_local, inv_quat);

    // transform ray direction: direction rotated by inverse quaternion
    Vec3f ray_direction_local = vec3f::transform_quat(ray.direction, inv_quat);

    // scale direction by ray length for gjk
    ray_direction_local = vec3f::scale(ray_direction_local, ray.length);

    // cast ray using gjk
    gjk::GjkCastRayResult gjk_result = gjk::create_gjk_cast_ray_result();
    gjk::cast_ray(gjk_result, ray_origin_local, ray_direction_local, 1e-3f, cast_ray_support, collector.early_out_fraction);

    if (!gjk_result.is_hit_found) {
        cast_ray::add_miss(collector);
        return;
    }

    const float fraction = gjk_result.lambda;
    cast_ray::CastRayHit hit = cast_ray::create_hit();
    hit.status = cast_ray::COLLIDING;
    hit.sub_shape_id = sub_shape_id.value;
    hit.body_id_b = collector.body_id_b;
    bool hit_added = false;

    // a ray starting inside (fraction 0) only hits when the convex is treated as solid
    if (settings.treat_convex_as_solid || fraction > 0.0f) {
        hit.fraction = scalar::max(0.0f, fraction);
        cast_ray::add_hit(collector, hit);
        hit_added = true;
    }

    // back faces: cast the inverted ray from the early-out fraction back to the forward hit to find the exit
    if (settings.collide_with_backfaces_convex && !cast_ray::should_early_out(collector)) {
        const float start_fraction = scalar::min(1.0f, collector.early_out_fraction);
        const float delta_fraction = fraction - start_fraction;
        if (delta_fraction < 0.0f) {
            const Vec3f inverted_origin = vec3f::scale_and_add(ray_origin_local, ray_direction_local, start_fraction);
            const Vec3f inverted_direction = vec3f::scale(ray_direction_local, delta_fraction);
            gjk::cast_ray(gjk_result, inverted_origin, inverted_direction, 1e-3f, cast_ray_support, 1.0f);

            // a fraction of 0 means the ray ends inside the shape, which is no back face
            if (gjk_result.is_hit_found && gjk_result.lambda > 0.0f) {
                hit.fraction = fraction + (gjk_result.lambda - 1.0f) * delta_fraction;
                cast_ray::add_hit(collector, hit);
                hit_added = true;
            }
        }
    }

    if (!hit_added) cast_ray::add_miss(collector);
}

/* collide point */

static support::Support collide_point_point_support;
static support::Support collide_point_shape_support;

void collide_point_vs_convex(collide_point::CollidePointCollector &collector, const collide_point::CollidePointSettings &settings,
                             Vec3f point, const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b,
                             const Transformf &transform_b) {
    // transform point to shape's local space: apply inverse rotation
    const Quatf inv_quat_b = quatf::conjugate(quatf::load(transform_b.quaternion));
    Vec3f local_point = vec3f::subtract(point, vec3f::load(transform_b.position));
    local_point = vec3f::transform_quat(local_point, inv_quat_b);

    // early-out: aabb test
    const AABox scaled_aabb = aabox::scale(aabox::load(shape_b->aabb), vec3f::load(transform_b.scale));
    if (!aabox::contains_point(scaled_aabb, local_point)) {
        return;
    }

    // point support (always returns the same point)
    support::set_point_support(collide_point_point_support, local_point);

    // get shape support function with scale applied
    shapes::set_shape_support(collide_point_shape_support, shape_b, support::INCLUDE_CONVEX_RADIUS, vec3f::load(transform_b.scale));

    // use point position as initial separating axis guess
    Vec3f initial_direction = local_point;
    if (vec3f::squared_length(initial_direction) < 1e-10f) {
        // if point is at origin, use arbitrary direction
        initial_direction = vec3f::make(0.0f, 1.0f, 0.0f);
    }

    // run gjk to test intersection
    gjk::GjkClosestPoints gjk_result = gjk::create_gjk_closest_points();
    gjk::closest_points(gjk_result, collide_point_shape_support, collide_point_point_support, settings.collision_tolerance,
                        initial_direction,
                        0.0f);  // max_distance_squared = 0 (only care about intersection)

    // if squared distance is very small, point is inside
    const float tolerance_sq = settings.collision_tolerance * settings.collision_tolerance;
    if (gjk_result.squared_distance <= tolerance_sq) {
        collide_point::CollidePointHit hit = collide_point::create_hit();
        hit.sub_shape_id_b = sub_shape_id_b.value;
        hit.body_id_b = collector.body_id_b;
        collide_point::add_hit(collector, hit);
    }
}

/* collide convex vs convex */

static support::Support collide_support_a;
static support::Support collide_support_b;
// zeroed is an empty depth; epa's hull initialisation resets the builder before use
static penetration::PenetrationDepth collide_penetration_depth;

void collide_convex_vs_convex(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                              const Shape *shape_a, const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a,
                              const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    // to transform shape b into a's local space, we need to compute:
    // 1. the relative rotation: inv(quat_a) * quat_b
    // 2. the relative position: inv(quat_a) * (pos_b - pos_a)

    // compute the conjugate (inverse) of quat_a
    const Quatf inv_quat_a = quatf::conjugate(quatf::load(transform_a.quaternion));

    // compute relative rotation: inv(quat_a) * quat_b
    const Quatf relative_rot = quatf::multiply(inv_quat_a, quatf::load(transform_b.quaternion));

    // compute the relative position vector: pos_b - pos_a, rotated by inv(quat_a) to bring it into a's local space
    const Vec3f relative_pos = vec3f::subtract(vec3f::load(transform_b.position), vec3f::load(transform_a.position));
    const Vec3f rotated_relative_pos = vec3f::transform_quat(relative_pos, inv_quat_a);

    // the transformation matrix for a in world space (rotation + translation only, no scale)
    Mat4f transform_a_in_world = mat4f::from_rotation_translation(quatf::load(transform_a.quaternion), vec3f::load(transform_a.position));

    // the transformation matrix for b relative to a's local space
    Mat4f transform_b_in_a = mat4f::from_rotation_translation(relative_rot, rotated_relative_pos);

    // delegate to local-space collision function
    collide_convex_vs_convex_local(collector, settings, shape_a, sub_shape_id_a.value, shape_b, sub_shape_id_b.value, transform_b_in_a,
                                   transform_a_in_world, vec3f::load(transform_a.scale), vec3f::load(transform_b.scale));
}

void collide_convex_vs_convex_local(collide_shape::CollideShapeCollector &collector, const collide_shape::CollideShapeSettings &settings,
                                    const Shape *shape_a, sub_shape::SubShapeId sub_shape_id_a, const Shape *shape_b,
                                    sub_shape::SubShapeId sub_shape_id_b, const Mat4f &transform_b_in_a, const Mat4f &transform_a_in_world,
                                    Vec3f scale_a, Vec3f scale_b) {
    // TODO: OBB3 early out?

    // get supports (shrunk cores; convex radius reported and applied by gjk)
    support::Support &support_a = collide_support_a;
    support::Support &support_b = collide_support_b;
    shapes::set_shape_support(support_a, shape_a, support::EXCLUDE_CONVEX_RADIUS, scale_a);
    shapes::set_shape_support(support_b, shape_b, support::EXCLUDE_CONVEX_RADIUS, scale_b);

    // fold shape b's transform into a's local space onto its support struct
    support_b.has_transform = true;
    support_b.transform = transform_b_in_a;

    // initial search direction (vector from a to b in a's local space)
    // note: As we don't remember the penetration axis from the last iteration, and it is likely that shape2 is pushed out of
    // collision relative to shape1 by comparing their COM's, we use that as an initial penetration axis: shape2.com - shape1.com
    // This has been seen to improve performance by approx. 1% over using a fixed axis like (1, 0, 0).
    Vec3f penetration_axis = mat4f::get_translation(transform_b_in_a);

    // ensure that we do not pass in a near zero penetration axis
    if (vec3f::squared_length(penetration_axis) <= 1e-12f) {
        penetration_axis = vec3f::make(1.0f, 0.0f, 0.0f);
    }

    penetration::PenetrationDepth &penetration_depth = collide_penetration_depth;
    simplex::Simplex simplex;
    simplex.size = 0;

    float max_separation_distance_to_use = settings.max_separation_distance;

    penetration::penetration_depth_step_gjk(penetration_depth, simplex, support_a, support_b,
                                            support_a.convex_radius + max_separation_distance_to_use, support_b.convex_radius,
                                            penetration_axis, settings.collision_tolerance);

    // check result of collision detection
    switch (penetration_depth.status) {
        case penetration::NOT_COLLIDING: {
            // definitive no collision
            return;
        }

        case penetration::COLLIDING: {
            // gjk found shallow penetration - use those results
            break;
        }

        case penetration::INDETERMINATE: {
            // need to run expensive epa algorithm

            // we know we're overlapping at this point, so we can set the max separation distance to 0.
            // numerically it is possible that gjk finds that the shapes are overlapping but epa finds that they're separated.
            // in order to avoid this, we clamp the max separation distance to 1 so that we don't excessively inflate the shape,
            // but we still inflate it enough to avoid the case where epa misses the collision.
            max_separation_distance_to_use = scalar::min(max_separation_distance_to_use, 1.0f);

            // re-fill supports to include convex radius for epa (resets transform/add_radius)
            shapes::set_shape_support(support_a, shape_a, support::INCLUDE_CONVEX_RADIUS, scale_a);
            shapes::set_shape_support(support_b, shape_b, support::INCLUDE_CONVEX_RADIUS, scale_b);

            // add separation distance along a's direction
            support_a.add_radius = max_separation_distance_to_use;

            // shape b in a's local space
            support_b.has_transform = true;
            support_b.transform = transform_b_in_a;

            // perform epa step
            if (!penetration::penetration_depth_step_epa(penetration_depth, support_a, support_b, settings.penetration_tolerance,
                                                         simplex)) {
                return;
            }
            break;
        }
    }

    // check if the penetration is bigger than the early out fraction
    const Vec3f delta = vec3f::subtract(penetration_depth.point_a, penetration_depth.point_b);
    const float penetration = vec3f::length(delta) - max_separation_distance_to_use;
    if (-penetration >= collector.early_out_fraction) {
        return;
    }

    // correct point_a for the added separation distance
    const float penetration_axis_len = vec3f::length(penetration_depth.penetration_axis);
    if (penetration_axis_len > 0.0f) {
        const float correction = max_separation_distance_to_use / penetration_axis_len;
        penetration_depth.point_a = vec3f::scale_and_add(penetration_depth.point_a, penetration_depth.penetration_axis, -correction);
    }

    // convert to world space using transform_a_in_world
    collide_shape::CollideShapeHit hit = collide_shape::create_hit();
    hit.point_a = vec3f::transform_mat4(penetration_depth.point_a, transform_a_in_world);
    hit.point_b = vec3f::transform_mat4(penetration_depth.point_b, transform_a_in_world);
    hit.penetration_axis = mat4f::multiply3x3_vec(transform_a_in_world, penetration_depth.penetration_axis);

    // populate hit info
    hit.penetration = penetration;
    hit.sub_shape_id_a = sub_shape_id_a;
    hit.sub_shape_id_b = sub_shape_id_b;
    hit.body_id_b = collector.body_id_b;

    // gather faces
    if (settings.collect_faces) {
        // get supporting face of shape a (negate penetration axis)
        const Vec3f face_dir_a = vec3f::negate(penetration_depth.penetration_axis);
        shapes::get_shape_supporting_face(hit.face_a, shape_a, sub_shape_id_a, face_dir_a, transform_a_in_world, scale_a);

        // get supporting face of shape b
        // transform penetration axis from a's local to b's local using inverse rotation of transform_b_in_a
        // (transposed 3x3 portion = inverse rotation for orthonormal matrices)
        const Vec3f face_dir_b = mat4f::multiply3x3_transposed_vec(transform_b_in_a, penetration_depth.penetration_axis);

        // compute b's world transform: b_to_world = transform_a_in_world * transform_b_in_a
        Mat4f b_to_world = mat4f::multiply(transform_a_in_world, transform_b_in_a);

        shapes::get_shape_supporting_face(hit.face_b, shape_b, sub_shape_id_b, face_dir_b, b_to_world, scale_b);
    }

    // notify the collector
    collide_shape::add_hit(collector, hit);
}

/* cast convex vs convex */

static support::Support cast_support_a;
static support::Support cast_support_b;

void cast_convex_vs_convex(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings, const Shape *shape_a,
                           const sub_shape::SubShapeIdBuilder &sub_shape_id_a, const Transformf &transform_a, Vec3f displacement_a,
                           const Shape *shape_b, const sub_shape::SubShapeIdBuilder &sub_shape_id_b, const Transformf &transform_b) {
    // compute inverse of b's quaternion for transforming displacement
    const Quatf inverse_quaternion_b = quatf::conjugate(quatf::load(transform_b.quaternion));

    // transform a into b's local space using matrices
    // cast_transform = inverse(target_transform) * transform_a
    const Mat4f transform_a_mat = mat4f::from_rotation_translation(quatf::load(transform_a.quaternion), vec3f::load(transform_a.position));
    Mat4f target_transform = mat4f::from_rotation_translation(quatf::load(transform_b.quaternion), vec3f::load(transform_b.position));
    Mat4f inv_target_transform, cast_transform;
    mat4f::invert(inv_target_transform, target_transform);

    // a's start transform in b's space
    cast_transform = mat4f::multiply(inv_target_transform, transform_a_mat);

    // transform displacement to b's space
    const Vec3f displacement_in_b = vec3f::transform_quat(displacement_a, inverse_quaternion_b);

    // delegate to local-space function with world space info for contact points
    cast_convex_vs_convex_local(collector, settings, shape_a, sub_shape_id_a.value, shape_b, sub_shape_id_b.value, cast_transform,
                                vec3f::load(transform_a.scale), displacement_in_b, vec3f::load(transform_b.scale), target_transform);
}

void cast_convex_vs_convex_local(cast_shape::CastShapeCollector &collector, const cast_shape::CastShapeSettings &settings,
                                 const Shape *shape_a, sub_shape::SubShapeId sub_shape_id_a, const Shape *shape_b,
                                 sub_shape::SubShapeId sub_shape_id_b, const Mat4f &cast_transform, Vec3f scale_a,
                                 Vec3f displacement_in_b, Vec3f scale_b, const Mat4f &target_transform) {
    // the actual shapes, or the shapes shrunk by their convex radius with the radius added back by the cast
    const support::SupportFunctionMode support_mode =
        settings.use_shrunken_shape_and_convex_radius ? support::EXCLUDE_CONVEX_RADIUS : support::DEFAULT;
    support::Support &support_a = cast_support_a;
    support::Support &support_b = cast_support_b;
    shapes::set_shape_support(support_a, shape_a, support_mode, scale_a);
    shapes::set_shape_support(support_b, shape_b, support_mode, scale_b);

    // run gjk casting with epa fallback for deep penetration in b's local space
    gjk::GjkCastShapeResult gjk_result = gjk::create_gjk_cast_shape_result();
    gjk_result.lambda = collector.early_out_fraction;
    penetration::cast_shape(gjk_result, cast_transform, support_a, support_b, displacement_in_b, settings.collision_tolerance,
                            settings.penetration_tolerance, support_a.convex_radius + settings.extra_convex_radius, support_b.convex_radius,
                            collector.early_out_fraction, settings.return_deepest_point);

    // check if hit found
    if (!gjk_result.hit) {
        cast_shape::add_miss(collector);
        return;
    }

    // lambda must be within valid sweep range
    if (gjk_result.lambda > 1.0f) {
        cast_shape::add_miss(collector);
        return;
    }

    // back-face culling: reject if shapes are separating (moving apart); a normal perpendicular to the cast is not a back face
    if (!settings.collide_with_backfaces_convex) {
        if (vec3f::dot(gjk_result.separating_axis, displacement_in_b) < 0.0f) {
            cast_shape::add_miss(collector);
            return;
        }
    }

    cast_shape::CastShapeHit hit = cast_shape::create_hit();

    // transform contact points from b's local space to world space using target_transform
    hit.point_a = vec3f::transform_mat4(gjk_result.point_a, target_transform);
    hit.point_b = vec3f::transform_mat4(gjk_result.point_b, target_transform);
    hit.penetration_depth = vec3f::distance(hit.point_a, hit.point_b);

    // reject a starting overlap no deeper than the collector's bound; hits at fraction > 0 past it were already
    // rejected by the gjk cast
    if (gjk_result.lambda == 0.0f && -hit.penetration_depth >= collector.early_out_fraction) {
        cast_shape::add_miss(collector);
        return;
    }

    // transform penetration_axis from b's local space to world space (keep unnormalized)
    hit.penetration_axis = mat4f::multiply3x3_vec(target_transform, gjk_result.separating_axis);

    // compute contact normal as -penetration_axis.normalized()
    hit.normal = vec3f::normalize(hit.penetration_axis);
    hit.normal = vec3f::negate(hit.normal);

    // update result object
    hit.status = cast_shape::COLLIDING;
    hit.fraction = gjk_result.lambda;
    hit.sub_shape_id_a = sub_shape_id_a;
    hit.sub_shape_id_b = sub_shape_id_b;
    hit.body_id_b = collector.body_id_b;

    // extract supporting faces if requested
    if (settings.collect_faces) {
        // calculate transform for shape a at contact point
        // transform_1_to_2 = cast_transform with translation += fraction * displacement_in_b
        Mat4f a_at_contact = cast_transform;
        a_at_contact.c[3] = v4f::add(a_at_contact.c[3], v4f::set_lane<3>(vec3f::scale(displacement_in_b, gjk_result.lambda).v, 0.0f));

        // shape a's world transform at contact = target_transform * a_at_contact
        const Mat4f a_to_world_at_contact = mat4f::multiply(target_transform, a_at_contact);

        // shape a: contact_normal is in b's local space, transform to a's local using transposed rotation
        Vec3f convex_query_normal = mat4f::multiply3x3_transposed_vec(cast_transform, gjk_result.separating_axis);
        convex_query_normal = vec3f::negate(convex_query_normal);
        shapes::get_shape_supporting_face(hit.face_a, shape_a, sub_shape_id_a, convex_query_normal, a_to_world_at_contact, scale_a);

        // shape b: contact_normal (separating_axis) is already in b's local space
        shapes::get_shape_supporting_face(hit.face_b, shape_b, sub_shape_id_b, gjk_result.separating_axis, target_transform, scale_b);
    } else {
        face::clear(hit.face_a);
        face::clear(hit.face_b);
    }

    cast_shape::add_hit(collector, hit);
}

}  // namespace convex

TUMBLE_NAMESPACE_END
