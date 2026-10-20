#pragma once

// crashcat's combine-material.ts: how two bodies' friction or restitution become one value.

#include <stdint.h>

#include "tumble/math/scalar.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace combine_material {

enum MaterialCombineMode : uint32_t {
    AVERAGE = 0,
    MULTIPLY = 1,
    MIN = 2,
    MAX = 3,
    GEOMETRIC_MEAN = 4,
};

/** when the modes differ the more restrictive wins: max > min > geometric mean > multiply > average. */
inline uint32_t priority(MaterialCombineMode mode) {
    switch (mode) {
        case MAX:
            return 4;
        case MIN:
            return 3;
        case GEOMETRIC_MEAN:
            return 2;
        case MULTIPLY:
            return 1;
        default:
            return 0;
    }
}

inline float combine(float value_a, float value_b, MaterialCombineMode mode_a, MaterialCombineMode mode_b) {
    const MaterialCombineMode mode = priority(mode_a) >= priority(mode_b) ? mode_a : mode_b;
    switch (mode) {
        case MULTIPLY:
            return value_a * value_b;
        case MIN:
            return scalar::min(value_a, value_b);
        case MAX:
            return scalar::max(value_a, value_b);
        case GEOMETRIC_MEAN:
            return scalar::sqrt(value_a * value_b);
        default:
            return (value_a + value_b) * 0.5f;
    }
}

}  // namespace combine_material

TUMBLE_NAMESPACE_END
