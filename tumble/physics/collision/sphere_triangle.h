#pragma once

// crashcat's sphere-triangle.ts: the ray tests a swept sphere makes against a triangle's edges and
// vertices, and the feature-to-active-edge table.

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace sphere_triangle {

/**
 * ray against a finite cylinder's side, returning the smallest fraction in [0, 1], or infinity on a miss.
 * based on "real time collision detection" by christer ericson, chapter 5.3.7.
 * the ray starts at the origin and `direction`'s length is its extent; the caps are not tested.
 */
float ray_cylinder(Vec3f direction, Vec3f cylinder_a, Vec3f cylinder_b, float radius);

/** ray against sphere: the smallest fraction in [0, 1], 0 if the ray starts inside, infinity on a miss. */
float ray_sphere(Vec3f origin, Vec3f direction, Vec3f sphere_center, float sphere_radius);

/** ray_sphere with the ray starting at the origin. */
float ray_sphere_from_origin(Vec3f direction, Vec3f sphere_center, float sphere_radius);

/**
 * maps a triangle feature (from triangle::get_closest_point_on_triangle) to the active edges it needs.
 *
 * feature bits: 0b001 = vertex A, 0b010 = vertex B, 0b100 = vertex C
 * edge bits: 0b001 = edge AB, 0b010 = edge BC, 0b100 = edge CA
 *
 * a vertex needs either adjacent edge active, an edge needs itself active, and the face interior
 * (0b111) needs no edge check.
 */
inline constexpr uint32_t FEATURE_TO_ACTIVE_EDGES[8] = {
    0b000,  // 0: invalid
    0b101,  // 1 (0b001): vertex A -> needs edge AB (0) or edge CA (2) active
    0b011,  // 2 (0b010): vertex B -> needs edge AB (0) or edge BC (1) active
    0b001,  // 3 (0b011): edge AB -> needs edge AB (0) active
    0b110,  // 4 (0b100): vertex C -> needs edge BC (1) or edge CA (2) active
    0b100,  // 5 (0b101): edge CA -> needs edge CA (2) active
    0b010,  // 6 (0b110): edge BC -> needs edge BC (1) active
    0b000,  // 7 (0b111): face interior -> no edge check needed
};

}  // namespace sphere_triangle

TUMBLE_NAMESPACE_END
