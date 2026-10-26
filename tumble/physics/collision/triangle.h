#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"

TUMBLE_NAMESPACE_BEGIN

// the closest point on a triangle to the origin, and which feature holds it

namespace triangle {

// feature bits for get_closest_point_on_triangle: which part of the triangle contains the closest point

/** closest to vertex A (v0) */
constexpr uint32_t VERTEX_A = 0b001;
/** closest to vertex B (v1) */
constexpr uint32_t VERTEX_B = 0b010;
/** closest to vertex C (v2) */
constexpr uint32_t VERTEX_C = 0b100;
/** closest to edge AB (v0-v1) */
constexpr uint32_t EDGE_AB = 0b011;
/** closest to edge AC (v0-v2) */
constexpr uint32_t EDGE_AC = 0b101;
/** closest to edge BC (v1-v2) */
constexpr uint32_t EDGE_BC = 0b110;
/** closest to interior face */
constexpr uint32_t FACE = 0b111;

}  // namespace triangle

struct ClosestPointOnTriangleResult {
    /** closest point on triangle (in same coordinate space as input vertices) */
    Vec3f point;
    /** squared distance from origin to closest point */
    float distance_sq;
    /** feature bitmask indicating which part of triangle is closest (triangle::VERTEX_A and the rest) */
    uint32_t feature;
};

namespace triangle {

inline ClosestPointOnTriangleResult create_closest_point_on_triangle_result() { return {vec3f::zero(), 0.0f, 0}; }

/**
 * the closest point on a triangle to the origin (ericson, real-time collision detection).
 * a, b, c are relative to the query point, i.e. the query point is at the origin.
 */
void get_closest_point_on_triangle(ClosestPointOnTriangleResult &out, Vec3f a, Vec3f b, Vec3f c);

}  // namespace triangle

TUMBLE_NAMESPACE_END
