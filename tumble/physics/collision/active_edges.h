#pragma once

// crashcat's active-edges.ts: which triangle mesh edges generate collisions.
//
// an active edge is an edge that either has no neighbouring edge or if the angle between the two
// connecting faces is too large. active edges should generate collisions, while inactive edges
// (coplanar or nearly-coplanar interior edges) should not cause "ghost collisions".
//
// a triangle's active edges are a 3-bit mask: bit 0 = edge v0-v1, bit 1 = edge v1-v2, bit 2 = edge v2-v0.

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace active_edges {

/** cos(179 degrees): back-to-back triangle threshold */
constexpr float COS_179_DEGREES = -0.999848f;
/** cos(1 degree): parallel normal threshold */
constexpr float COS_1_DEGREE = 0.999848f;
constexpr float BARYCENTRIC_EPSILON = 1.0e-4f;
constexpr float BARYCENTRIC_ONE_MINUS_EPSILON = 1.0f - BARYCENTRIC_EPSILON;

/**
 * whether an edge between two triangles should be treated as collidable: active when it is a mesh
 * boundary or a sharp convex crease; inactive when nearly coplanar or concave (to prevent snagging).
 * normal1 is the left triangle's (looking along the edge from above), normal2 the right one's;
 * cos_threshold_angle is typically cos(10 degrees).
 */
bool is_edge_active(Vec3f normal1, Vec3f normal2, Vec3f edge_direction, float cos_threshold_angle);

/**
 * the triangle normal in place of the collision normal when the contact is on an inactive edge,
 * which prevents ghost collisions at interior mesh edges. returns `normal` or `triangle_normal`
 * itself. vertices, point and normals in the triangle's local space; the normals need not be unit;
 * `movement_direction` resolves the ambiguous cases and may be zero.
 */
Vec3f fix_normal(Vec3f v0, Vec3f v1, Vec3f v2, Vec3f triangle_normal, uint32_t active_edges,
                  Vec3f point, Vec3f normal, Vec3f movement_direction);

}  // namespace active_edges

TUMBLE_NAMESPACE_END
