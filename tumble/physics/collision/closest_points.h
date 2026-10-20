#pragma once

// crashcat's closest-points.ts: barycentric coordinates of the point on a segment or triangle
// closest to the origin.

#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace closest_points {

struct BarycentricCoordinatesResult {
    float u;
    float v;
    float w;
    bool is_valid;
};

inline BarycentricCoordinatesResult create_barycentric_coordinates_result() { return {0.0f, 0.0f, 0.0f, false}; }

/**
 * u, v of the point on segment ab closest to the origin (u * a + v * b). a segment shorter than
 * sqrt(squared_tolerance) falls back to the nearer end and reports invalid.
 */
void compute_barycentric_coordinates_2d(BarycentricCoordinatesResult &out, Vec3f a, Vec3f b, float squared_tolerance);

/**
 * u, v, w of the origin projected onto the plane of triangle abc (u * a + v * b + w * c). a
 * degenerate triangle falls back to its longest edge and reports invalid.
 */
void compute_barycentric_coordinates_3d(BarycentricCoordinatesResult &out, Vec3f a, Vec3f b, Vec3f c,
                                        float squared_tolerance);

}  // namespace closest_points

TUMBLE_NAMESPACE_END
