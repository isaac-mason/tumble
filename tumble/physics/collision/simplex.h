#pragma once

// the gjk/epa simplex as jolt's GJKClosestPoint keeps it: up to four support pairs, y = p - q at the
// same index describing one pair.

#include <stdint.h>

#include "math/types.h"

namespace simplex {

struct Simplex {
    /** minkowski difference p - q per point */
    Vec3f y[4];
    /** support point on shape a per point */
    Vec3f p[4];
    /** support point on shape b per point */
    Vec3f q[4];
    /** points in the simplex, 0..4 */
    uint32_t size;
};

inline Simplex &copy(Simplex &out, const Simplex &input) {
    out.size = input.size;
    for (uint32_t i = 0; i < input.size; i++) {
        out.y[i] = input.y[i];
        out.p[i] = input.p[i];
        out.q[i] = input.q[i];
    }
    return out;
}

}  // namespace simplex
