#pragma once

#include <stdint.h>

#include "tumble/config.h"
#include "tumble/math/scalar.h"

TUMBLE_NAMESPACE_BEGIN

// how two bodies' friction or restitution become one value

enum class MaterialCombineMode : uint32_t {
    AVERAGE = 0,
    MULTIPLY = 1,
    MIN = 2,
    MAX = 3,
    GEOMETRIC_MEAN = 4,
};

namespace combine_material {

/** when the modes differ the more restrictive wins: max > min > geometric mean > multiply > average. */
inline uint32_t priority(MaterialCombineMode mode) {
    switch (mode) {
        case MaterialCombineMode::MAX:
            return 4;
        case MaterialCombineMode::MIN:
            return 3;
        case MaterialCombineMode::GEOMETRIC_MEAN:
            return 2;
        case MaterialCombineMode::MULTIPLY:
            return 1;
        default:
            return 0;
    }
}

inline float combine(float value_a, float value_b, MaterialCombineMode mode_a, MaterialCombineMode mode_b) {
    const MaterialCombineMode mode = priority(mode_a) >= priority(mode_b) ? mode_a : mode_b;
    switch (mode) {
        case MaterialCombineMode::MULTIPLY:
            return value_a * value_b;
        case MaterialCombineMode::MIN:
            return scalar::min(value_a, value_b);
        case MaterialCombineMode::MAX:
            return scalar::max(value_a, value_b);
        case MaterialCombineMode::GEOMETRIC_MEAN:
            return scalar::sqrt(value_a * value_b);
        default:
            return (value_a + value_b) * 0.5f;
    }
}

}  // namespace combine_material

TUMBLE_NAMESPACE_END
