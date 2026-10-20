#include "tumble/physics/collision/penetration.h"

#include "tumble/math/scalar.h"
#include "tumble/math/simd.h"
#include "tumble/math/vec3f.h"
#include "tumble/util/assert.h"

TUMBLE_NAMESPACE_BEGIN

namespace penetration {

namespace hull = epa_convex_hull_builder;

// rotation matrix constants for 120 degree rotation
static constexpr float COS_120_DEG = -0.5f;
static constexpr float SIN_120_DEG = 0.866025403784438646763f;
static constexpr float ONE_MINUS_COS_120_DEG = 1.0f - COS_120_DEG;

void reset(PenetrationDepth &depth) {
    depth.status = NOT_COLLIDING;
    depth.penetration_axis = vec3f::zero();
    depth.point_a = vec3f::zero();
    depth.point_b = vec3f::zero();
}

void penetration_depth_step_gjk(PenetrationDepth &out_penetration_depth, simplex::Simplex &out_simplex, support::Support &support_a,
                                support::Support &support_b, float convex_radius_a, float convex_radius_b, Vec3f direction,
                                float tolerance) {
    const float combined_radius = convex_radius_a + convex_radius_b;
    const float combined_radius_squared = combined_radius * combined_radius;

    // run gjk to find the closest points between shapes
    gjk::GjkClosestPoints closest = gjk::create_gjk_closest_points();
    gjk::closest_points(closest, support_a, support_b, tolerance, direction, combined_radius_squared);

    // check if shapes are separated by more than the combined convex radius
    if (closest.squared_distance > combined_radius_squared) {
        // shapes are separated - no collision
        // explicitly clear contact data to avoid stale data from previous collisions
        out_penetration_depth.point_a = vec3f::zero();
        out_penetration_depth.point_b = vec3f::zero();
        out_penetration_depth.penetration_axis = vec3f::zero();
        out_penetration_depth.status = NOT_COLLIDING;
        return;
    }

    // copy results to output
    out_penetration_depth.point_a = closest.point_a;
    out_penetration_depth.point_b = closest.point_b;
    out_penetration_depth.penetration_axis = closest.penetration_axis;

    simplex::copy(out_simplex, closest.simplex);

    if (closest.squared_distance > 0.0f) {
        // collision within convex radius - adjust contact points based on convex radii
        const float v_length = scalar::sqrt(closest.squared_distance);

        // move point_a along penetration axis by convex_radius_a
        out_penetration_depth.point_a =
            vec3f::scale_and_add(out_penetration_depth.point_a, out_penetration_depth.penetration_axis, convex_radius_a / v_length);

        // move point_b along negative penetration axis by convex_radius_b
        out_penetration_depth.point_b =
            vec3f::scale_and_add(out_penetration_depth.point_b, out_penetration_depth.penetration_axis, -(convex_radius_b / v_length));

        out_penetration_depth.status = COLLIDING;
        return;
    }

    // distance is zero or very small - need epa to determine penetration depth
    out_penetration_depth.status = INDETERMINATE;
}

/** adds a support point in the given direction; its index, or -1 when the store is full. */
static int32_t add_epa_support_point(EpaSupportPoints &points, support::Support &support_a, support::Support &support_b, Vec3f direction) {
    // the epa loops stop at EPA_MAX_POINTS, so a full store is a broken invariant
    TUMBLE_ASSERT(points.size < EPA_MAX_POINTS);
    if (points.size >= EPA_MAX_POINTS) return -1;

    const Vec3f p = support_a.get_support(support_a, direction);
    const Vec3f q = support_b.get_support(support_b, vec3f::negate(direction));

    // store new point, y = p - q (minkowski difference)
    const uint32_t idx = points.size++;
    points.y[idx] = vec3f::subtract(p, q);
    points.p[idx] = p;
    points.q[idx] = q;
    return static_cast<int32_t>(idx);
}

/** whether any of the triangles faces the origin (dot(normal, centroid) < 0), four at a time. */
static bool any_triangle_facing_origin(const hull::EpaConvexHullBuilderState &state, const hull::NewTriangles &triangles) {
    for (uint32_t base = 0; base < triangles.size; base += 4) {
        const uint32_t count = triangles.size - base < 4 ? triangles.size - base : 4;
        const hull::Triangle *tris[4];
        for (uint32_t lane = 0; lane < 4; lane++) tris[lane] = &state.triangles[triangles.values[base + (lane < count ? lane : 0)]];

        v128_t nx, ny, nz, cx, cy, cz;
        v4f::transpose3x4(tris[0]->normal.v, tris[1]->normal.v, tris[2]->normal.v, tris[3]->normal.v, nx, ny, nz);
        v4f::transpose3x4(tris[0]->centroid.v, tris[1]->centroid.v, tris[2]->centroid.v, tris[3]->centroid.v, cx, cy, cz);

        const v128_t dot = v4f::add(v4f::add(v4f::mul(nx, cx), v4f::mul(ny, cy)), v4f::mul(nz, cz));
        // the padding lanes repeat the first triangle of the group, so they cannot change the answer
        if (v4f::any_true(v4f::lt(dot, v4f::zero()))) return true;
    }
    return false;
}

/** the barycentric combination of three support points: v0 + (v1 - v0) * lambda0 + (v2 - v0) * lambda1 */
static Vec3f barycentric_point(Vec3f v0, Vec3f v1, Vec3f v2, float lambda0, float lambda1) {
    return vec3f::add(vec3f::add(v0, vec3f::scale(vec3f::subtract(v1, v0), lambda0)), vec3f::scale(vec3f::subtract(v2, v0), lambda1));
}

bool penetration_depth_step_epa(PenetrationDepth &out, support::Support &support_a_including_radius,
                                support::Support &support_b_including_radius, float tolerance, const simplex::Simplex &simplex) {
    // fetch the simplex from gjk and convert to support points
    EpaSupportPoints &support_points = out.support_points;
    for (uint32_t i = 0; i < simplex.size; i++) {
        support_points.y[i] = simplex.y[i];
        support_points.p[i] = simplex.p[i];
        support_points.q[i] = simplex.q[i];
    }
    support_points.size = simplex.size;

    // fill up to 4 support points if needed
    switch (simplex.size) {
        case 1: {
            // 1 vertex at origin - add 4 new points
            support_points.size--;
            add_epa_support_point(support_points, support_a_including_radius, support_b_including_radius, vec3f::make(0.0f, 1.0f, 0.0f));
            add_epa_support_point(support_points, support_a_including_radius, support_b_including_radius, vec3f::make(-1.0f, -1.0f, -1.0f));
            add_epa_support_point(support_points, support_a_including_radius, support_b_including_radius, vec3f::make(1.0f, -1.0f, -1.0f));
            add_epa_support_point(support_points, support_a_including_radius, support_b_including_radius, vec3f::make(0.0f, -1.0f, 1.0f));
            break;
        }

        case 2: {
            // two vertices - create 3 extra by rotating perpendicular axis
            // axis = normalize(y1 - y0)
            const Vec3f axis = vec3f::subtract(support_points.y[1], support_points.y[0]);
            const float axisx = vec3f::x(axis);
            const float axisy = vec3f::y(axis);
            const float axisz = vec3f::z(axis);
            const float axis_len = scalar::sqrt(axisx * axisx + axisy * axisy + axisz * axisz);
            const float axis_normx = axisx / axis_len;
            const float axis_normy = axisy / axis_len;
            const float axis_normz = axisz / axis_len;

            // compute normalized perpendicular to axis
            const Vec3f dir1 = vec3f::normalized_perpendicular(vec3f::make(axis_normx, axis_normy, axis_normz));
            const float dir1x = vec3f::x(dir1), dir1y = vec3f::y(dir1), dir1z = vec3f::z(dir1);

            // rotation matrix for 120 degrees about axis: R = cos(t)I + sin(t)[k]x + (1-cos(t))kk^T
            const float r00 = COS_120_DEG + axis_normx * axis_normx * ONE_MINUS_COS_120_DEG;
            const float r01 = axis_normx * axis_normy * ONE_MINUS_COS_120_DEG - axis_normz * SIN_120_DEG;
            const float r02 = axis_normx * axis_normz * ONE_MINUS_COS_120_DEG + axis_normy * SIN_120_DEG;
            const float r10 = axis_normy * axis_normx * ONE_MINUS_COS_120_DEG + axis_normz * SIN_120_DEG;
            const float r11 = COS_120_DEG + axis_normy * axis_normy * ONE_MINUS_COS_120_DEG;
            const float r12 = axis_normy * axis_normz * ONE_MINUS_COS_120_DEG - axis_normx * SIN_120_DEG;
            const float r20 = axis_normz * axis_normx * ONE_MINUS_COS_120_DEG - axis_normy * SIN_120_DEG;
            const float r21 = axis_normz * axis_normy * ONE_MINUS_COS_120_DEG + axis_normx * SIN_120_DEG;
            const float r22 = COS_120_DEG + axis_normz * axis_normz * ONE_MINUS_COS_120_DEG;

            // dir2 = R * dir1
            const float dir2x = r00 * dir1x + r01 * dir1y + r02 * dir1z;
            const float dir2y = r10 * dir1x + r11 * dir1y + r12 * dir1z;
            const float dir2z = r20 * dir1x + r21 * dir1y + r22 * dir1z;
            // dir3 = R * dir2
            const float dir3x = r00 * dir2x + r01 * dir2y + r02 * dir2z;
            const float dir3y = r10 * dir2x + r11 * dir2y + r12 * dir2z;
            const float dir3z = r20 * dir2x + r21 * dir2y + r22 * dir2z;

            add_epa_support_point(support_points, support_a_including_radius, support_b_including_radius, vec3f::make(dir1x, dir1y, dir1z));
            add_epa_support_point(support_points, support_a_including_radius, support_b_including_radius, vec3f::make(dir2x, dir2y, dir2z));
            add_epa_support_point(support_points, support_a_including_radius, support_b_including_radius, vec3f::make(dir3x, dir3y, dir3z));
            break;
        }

        default: {
            // already have enough points
            break;
        }
    }

    // a point that could not be stored leaves fewer than the hull needs
    if (support_points.size < 3) {
        out.status = NOT_COLLIDING;
        return false;
    }

    // create hull from initial points
    hull::EpaConvexHullBuilderState &hull_state = out.hull;
    hull_state.positions = support_points.y;
    hull::NewTriangles &new_triangles = out.new_triangles;
    hull::initialize(hull_state, 0, 1, 2);

    // add remaining points to hull
    const uint32_t initial_points = support_points.size;
    for (uint32_t i = 3; i < initial_points; i++) {
        float best_dist_sq = -1.0f;
        const int32_t t = hull::find_facing_triangle(hull_state, support_points.y[i], best_dist_sq);
        if (t != -1) {
            new_triangles.size = 0;
            if (!hull::add_point(hull_state, t, static_cast<int32_t>(i), scalar::FLOAT_MAX, new_triangles)) {
                out.status = NOT_COLLIDING;
                return false;
            }
        }
    }

    // loop until origin is inside hull
    while (true) {
        const int32_t triangle = hull::peek_closest_triangle_in_queue(hull_state);

        if (triangle == -1) {
            out.status = NOT_COLLIDING;
            return false;
        }

        // don't process removed triangles, just free them (they stay in the heap since removing them would mean resorting it)
        if ((hull_state.triangles[triangle].flags & hull::TRI_FLAG_REMOVED) != 0) {
            hull::pop_closest_triangle_from_queue(hull_state);

            if (!hull::has_next_triangle(hull_state)) {
                out.status = NOT_COLLIDING;
                return false;
            }

            hull::free_triangle(hull_state, triangle);
            continue;
        }

        // if closest distance is >= 0, origin is in hull
        if (hull_state.triangles[triangle].closest_length_sq >= 0.0f) break;

        hull::pop_closest_triangle_from_queue(hull_state);

        // add support point to get origin inside hull
        const int32_t new_index =
            add_epa_support_point(support_points, support_a_including_radius, support_b_including_radius, hull_state.triangles[triangle].normal);

        new_triangles.size = 0;

        if (new_index == -1 || !hull::triangle_is_facing(hull_state, triangle, support_points.y[new_index]) ||
            !hull::add_point(hull_state, triangle, new_index, scalar::FLOAT_MAX, new_triangles)) {
            out.status = NOT_COLLIDING;
            return false;
        }

        hull::free_triangle(hull_state, triangle);

        if (!hull::has_next_triangle(hull_state) || support_points.size >= EPA_MAX_POINTS_TO_INCLUDE_ORIGIN_IN_HULL) {
            out.status = NOT_COLLIDING;
            return false;
        }
    }

    // main epa loop - find closest point
    float closest_dist_sq = scalar::FLOAT_MAX;
    int32_t last = -1;
    bool flip_v_sign = false;

    do {
        const int32_t triangle = hull::pop_closest_triangle_from_queue(hull_state);

        if (triangle == -1) {
            out.status = NOT_COLLIDING;
            return false;
        }

        // skip removed triangles
        if ((hull_state.triangles[triangle].flags & hull::TRI_FLAG_REMOVED) != 0) {
            hull::free_triangle(hull_state, triangle);
            continue;
        }

        const float closest_length_sq = hull_state.triangles[triangle].closest_length_sq;

        // check if we found the closest point
        if (closest_length_sq >= closest_dist_sq) break;

        // replace last good triangle
        if (last != -1) hull::free_triangle(hull_state, last);
        last = triangle;

        // add support point in direction of normal
        const Vec3f normal = hull_state.triangles[triangle].normal;
        const int32_t new_index = add_epa_support_point(support_points, support_a_including_radius, support_b_including_radius, normal);
        if (new_index == -1) break;
        const Vec3f w = support_points.y[new_index];

        // project w onto triangle normal
        const float dot = vec3f::dot(normal, w);

        // check for separating axis
        if (dot < 0.0f) {
            out.status = NOT_COLLIDING;
            return false;
        }

        // get distance squared along normal
        const float normal_len_sq = vec3f::dot(normal, normal);
        const float dist_sq = (dot * dot) / normal_len_sq;

        // check for convergence
        if (dist_sq - closest_length_sq < closest_length_sq * tolerance) break;

        closest_dist_sq = scalar::min(closest_dist_sq, dist_sq);

        // check if triangle thinks point is not front facing
        if (!hull::triangle_is_facing(hull_state, triangle, w)) break;

        // add point to hull
        new_triangles.size = 0;

        if (!hull::add_point(hull_state, triangle, new_index, closest_dist_sq, new_triangles)) break;

        // check for hull defects
        if (any_triangle_facing_origin(hull_state, new_triangles)) {
            // check if we need to flip penetration sign
            const Vec3f negated_normal = vec3f::negate(normal);
            const Vec3f p2 = support_a_including_radius.get_support(support_a_including_radius, negated_normal);
            const Vec3f q2 = support_b_including_radius.get_support(support_b_including_radius, normal);
            const float dot2 = vec3f::dot(negated_normal, vec3f::subtract(p2, q2));
            if (dot2 < dot) flip_v_sign = true;
            break;
        }
    } while (hull::has_next_triangle(hull_state) && support_points.size < EPA_MAX_POINTS);

    // calculate results
    if (last == -1) {
        out.status = NOT_COLLIDING;
        return false;
    }

    const hull::Triangle &last_triangle = hull_state.triangles[last];
    const Vec3f last_normal = last_triangle.normal;
    const Vec3f last_centroid = last_triangle.centroid;

    // calculate penetration normal and depth
    const float normal_length_sq = vec3f::dot(last_normal, last_normal);
    const float centroid_dot_normal = vec3f::dot(last_centroid, last_normal);

    Vec3f penetration_normal = vec3f::scale(last_normal, centroid_dot_normal / normal_length_sq);

    // check for near-zero penetration
    if (vec3f::dot(penetration_normal, penetration_normal) < 1e-10f) {
        out.status = NOT_COLLIDING;
        return false;
    }

    if (flip_v_sign) penetration_normal = vec3f::negate(penetration_normal);

    // calculate contact points using barycentric coordinates
    const int32_t s0 = last_triangle.edges[0].start_index;
    const int32_t s1 = last_triangle.edges[1].start_index;
    const int32_t s2 = last_triangle.edges[2].start_index;
    const float lambda0 = last_triangle.lambda[0];
    const float lambda1 = last_triangle.lambda[1];
    const Vec3f *p = support_points.p;
    const Vec3f *q = support_points.q;

    if ((last_triangle.flags & hull::TRI_FLAG_LAMBDA_RELATIVE_TO_0) != 0) {
        // point = x0 + (x1 - x0) * lambda0 + (x2 - x0) * lambda1
        out.point_a = barycentric_point(p[s0], p[s1], p[s2], lambda0, lambda1);
        out.point_b = barycentric_point(q[s0], q[s1], q[s2], lambda0, lambda1);
    } else {
        // point = x1 + (x0 - x1) * lambda0 + (x2 - x1) * lambda1
        out.point_a = barycentric_point(p[s1], p[s0], p[s2], lambda0, lambda1);
        out.point_b = barycentric_point(q[s1], q[s0], q[s2], lambda0, lambda1);
    }

    out.penetration_axis = penetration_normal;
    out.status = COLLIDING;

    return true;
}

static PenetrationDepth cast_shape_penetration_depth;

void cast_shape(gjk::GjkCastShapeResult &out, const Mat4f &transform_a_to_b, support::Support &shape_a_support,
                support::Support &shape_b_support, Vec3f displacement, float collision_tolerance, float penetration_tolerance,
                float convex_radius_a, float convex_radius_b, float max_lambda, bool return_deepest_point) {
    // first determine if there's a collision at all
    gjk::cast_shape(out, transform_a_to_b, shape_a_support, shape_b_support, displacement, collision_tolerance, convex_radius_a,
                    convex_radius_b, max_lambda);

    if (!out.hit) return;

    // when our contact normal is too small, we don't have an accurate result
    const float squared_tolerance = collision_tolerance * collision_tolerance;
    const bool contact_normal_invalid = vec3f::dot(out.separating_axis, out.separating_axis) < squared_tolerance;

    const float combined_radius = convex_radius_a + convex_radius_b;
    // only when lambda = 0 we can have the bodies overlap
    // when no convex radius was provided we can never trust contact points at lambda = 0
    const bool should_do_epa = return_deepest_point && out.lambda == 0.0f && (combined_radius == 0.0f || contact_normal_invalid);

    if (should_do_epa) {
        // if we're initially intersecting, we need to run the epa algorithm in order to find the deepest contact point.
        // fold the convex radii onto the support structs; a's a->b transform was already folded by gjk::cast_shape above.
        shape_a_support.add_radius = convex_radius_a;
        shape_b_support.add_radius = convex_radius_b;

        PenetrationDepth &penetration_depth = cast_shape_penetration_depth;
        const bool collided =
            penetration_depth_step_epa(penetration_depth, shape_a_support, shape_b_support, penetration_tolerance, out.simplex);
        if (collided) {
            out.separating_axis = penetration_depth.penetration_axis;
            out.point_a = penetration_depth.point_a;
            out.point_b = penetration_depth.point_b;
        } else {
            out.hit = false;
        }
    } else if (contact_normal_invalid) {
        // if we weren't able to calculate a contact normal, use the cast direction instead
        out.separating_axis = displacement;
    }
}

}  // namespace penetration

TUMBLE_NAMESPACE_END
