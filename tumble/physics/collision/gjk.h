#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/collision/simplex.h"
#include "tumble/physics/collision/support.h"

TUMBLE_NAMESPACE_BEGIN
  // namespace gjk

struct GjkCastRayResult {
    bool is_hit_found;
    float lambda;
    Simplex simplex;
};

namespace gjk {

inline GjkCastRayResult create_gjk_cast_ray_result() {
    GjkCastRayResult result;
    result.is_hit_found = false;
    result.lambda = 0.0f;
    result.simplex.size = 0;
    return result;
}

/**
 * casts a ray from `ray_origin` along `ray_direction` (not normalised: lambda is a fraction of it)
 * against a convex support. a hit at or beyond `max_lambda` is a miss.
 */
void cast_ray(GjkCastRayResult &out, Vec3f ray_origin, Vec3f ray_direction, float tolerance, Support &support,
              float max_lambda = 1.0f);

}  // namespace gjk

struct GjkCastShapeResult {
    bool hit;
    float lambda;
    Vec3f point_a;
    Vec3f point_b;
    Vec3f separating_axis;
    Simplex simplex;
};

namespace gjk {

inline GjkCastShapeResult create_gjk_cast_shape_result() {
    GjkCastShapeResult result;
    result.hit = false;
    result.lambda = 0.0f;
    result.point_a = vec3f::zero();
    result.point_b = vec3f::zero();
    result.separating_axis = vec3f::zero();
    result.simplex.size = 0;
    return result;
}

/**
 * casts convex a along `displacement` against stationary convex b, in b's space. a's support gets
 * `transform_a_to_b` installed and its add_radius cleared; the convex radii are the supports'
 * reported ones, kept apart so the separating axis stays good.
 */
void cast_shape(GjkCastShapeResult &out, const Mat4f &transform_a_to_b, Support &shape_a_support,
                Support &shape_b_support, Vec3f displacement, float tolerance, float convex_radius_a,
                float convex_radius_b, float max_lambda);

}  // namespace gjk

struct GjkClosestPoints {
    // closest points, ray casts and shape casts between convex supports (van den bergen, a fast and robust gjk
    // implementation for collision detection of convex objects)

    /** the closest point of a simplex to the origin, and which of its points (bit i = point i) span it. */
    struct Result {
        Vec3f point;
        uint32_t point_set;
    };

    float squared_distance;
    Vec3f penetration_axis;
    Vec3f point_a;
    Vec3f point_b;
    Simplex simplex;
};

namespace gjk {

/** the closest point on segment ab to the origin. */
void compute_closest_point_on_line(GjkClosestPoints::Result &out, Vec3f a, Vec3f b, float squared_tolerance);

/** the closest point on triangle abc to the origin; with must_include_c, only features that contain c. */
void compute_closest_point_on_triangle(GjkClosestPoints::Result &out, Vec3f in_a, Vec3f in_b, Vec3f in_c, bool must_include_c,
                                       float squared_tolerance);

/** the closest point on tetrahedron abcd to the origin, 0b1111 and the origin when inside; with must_include_d, only features that contain d. */
void compute_closest_point_on_tetrahedron(GjkClosestPoints::Result &out, Vec3f in_a, Vec3f in_b, Vec3f in_c, Vec3f in_d,
                                          bool must_include_d, float tolerance);

}
namespace gjk {

inline GjkClosestPoints create_gjk_closest_points() {
    GjkClosestPoints result;
    result.squared_distance = 0.0f;
    result.penetration_axis = vec3f::zero();
    result.point_a = vec3f::zero();
    result.point_b = vec3f::zero();
    result.simplex.size = 0;
    return result;
}

/**
 * the closest points between convex a and b. on output squared_distance is 0 when they collide,
 * between 0 and FLT_MAX when separated (penetration_axis is then the separating axis), and FLT_MAX
 * when further apart than max_distance_squared. `tolerance` is how close they may get before they
 * count as colliding; `direction` is the first guess at the separating axis.
 */
void closest_points(GjkClosestPoints &out, Support &support_a, Support &support_b, float tolerance,
                    Vec3f direction, float max_distance_squared);

}  // namespace gjk

TUMBLE_NAMESPACE_END
