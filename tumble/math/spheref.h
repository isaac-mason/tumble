#pragma once

#include <stdint.h>

#include "tumble/math/scalar.h"
#include "tumble/math/types.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace spheref {

inline Spheref create() { return {{0.0f, 0.0f, 0.0f}, 1.0f}; }

/** centred on the bounding box of n xyz points, just large enough to hold them all */
inline Spheref &set_from_points(Spheref &out, const float *points, uint32_t n) {
    if (n == 0) {
        out.center = {0.0f, 0.0f, 0.0f};
        out.radius = 0.0f;
        return out;
    }
    float min_x = __builtin_inff(), min_y = min_x, min_z = min_x;
    float max_x = -min_x, max_y = -min_x, max_z = -min_x;
    for (uint32_t i = 0; i < n; i++) {
        const float x = points[i * 3];
        const float y = points[i * 3 + 1];
        const float z = points[i * 3 + 2];
        if (x < min_x) min_x = x;
        if (x > max_x) max_x = x;
        if (y < min_y) min_y = y;
        if (y > max_y) max_y = y;
        if (z < min_z) min_z = z;
        if (z > max_z) max_z = z;
    }
    const float cx = (min_x + max_x) * 0.5f;
    const float cy = (min_y + max_y) * 0.5f;
    const float cz = (min_z + max_z) * 0.5f;
    float max_sq = 0.0f;
    for (uint32_t i = 0; i < n; i++) {
        const float dx = points[i * 3] - cx;
        const float dy = points[i * 3 + 1] - cy;
        const float dz = points[i * 3 + 2] - cz;
        const float sq = dx * dx + dy * dy + dz * dz;
        if (sq > max_sq) max_sq = sq;
    }
    out.center = {cx, cy, cz};
    const float radius = scalar::sqrt(max_sq);
    // a rounded-down root would leave the farthest point outside; nudge up by an ulp
    out.radius = radius * radius < max_sq ? radius * (1.0f + 1.1920929e-7f) : radius;
    return out;
}

inline bool contains_point(const Spheref &sphere, const Float3 &point) {
    const float dx = point.x - sphere.center.x;
    const float dy = point.y - sphere.center.y;
    const float dz = point.z - sphere.center.z;
    return dx * dx + dy * dy + dz * dz <= sphere.radius * sphere.radius;
}

}  // namespace spheref

TUMBLE_NAMESPACE_END
