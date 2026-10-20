#pragma once

// crashcat's cast-utils.ts, the types: the ray vs triangle result. the tests themselves (ray vs
// triangle, ray vs box in three forms) are cast_utils.cpp.

#include <stdint.h>

#include "tumble/math/aabox.h"
#include "tumble/math/types.h"
#include "tumble/physics/collision/cast_ray_vs_shape.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace cast_utils {

struct RayIntersectsTriangleResult {
    bool hit;
    /** along the ray: 0 at the origin, 1 at length; 0 when no hit */
    float fraction;
    /** the triangle's front face was hit */
    bool front_facing;
};

inline RayIntersectsTriangleResult create_result() { return {false, 0.0f, false}; }

/**
 * ray vs triangle, as GteIntrRay3Triangle3. `max_t` is the largest hit distance still of interest (ray
 * units, at most `length`): a closest-hit traversal passes its current best so triangles behind it are
 * rejected before the divide.
 */
void ray_intersects_triangle(RayIntersectsTriangleResult &out, const cast_ray::Rayf &ray, float max_t, Vec3f a, Vec3f b,
                             Vec3f c, bool backface_culling);

/** 1 / d, or 0 when d is too small to invert: a parallel ray axis, which the slab tests check against the origin */
float safe_reciprocal(float d);

/**
 * entry fraction of a ray segment into a box, in [0, 1] of the segment, or FLT_MAX when the segment misses
 * the box or the box lies entirely behind the origin. `inv_displacement` is the reciprocals (safe_reciprocal)
 * of `direction * length`, computed once per query, so a box costs six multiplies and the compares.
 */
float ray_fraction_to_box3(Vec3f origin, Vec3f inv_displacement, const AABox &box);

/** normalised distance (0 to 1) along the ray to the box's entry point, or FLT_MAX when it misses or the box is behind */
float ray_distance_to_box3(const cast_ray::Rayf &ray, const AABox &box);

/** whether the ray segment touches the box at all */
bool ray_hits_box3(const cast_ray::Rayf &ray, const AABox &box);

}  // namespace cast_utils

TUMBLE_NAMESPACE_END
