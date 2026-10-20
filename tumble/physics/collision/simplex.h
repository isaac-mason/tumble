#pragma once

#include <stdint.h>

#include "tumble/math/types.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

// up to four support pairs, y = p - q at one index describing one pair
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

TUMBLE_NAMESPACE_END
