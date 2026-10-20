#pragma once

#include <stdint.h>
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

// degrees of freedom as a 6-bit mask, translation in the low three bits
namespace dof {

constexpr uint32_t TRANSLATION_X = 1u << 0;
constexpr uint32_t TRANSLATION_Y = 1u << 1;
constexpr uint32_t TRANSLATION_Z = 1u << 2;
constexpr uint32_t ROTATION_X = 1u << 3;
constexpr uint32_t ROTATION_Y = 1u << 4;
constexpr uint32_t ROTATION_Z = 1u << 5;

inline uint32_t dof(bool tx, bool ty, bool tz, bool rx, bool ry, bool rz) {
    uint32_t mask = 0;
    if (tx) mask |= TRANSLATION_X;
    if (ty) mask |= TRANSLATION_Y;
    if (tz) mask |= TRANSLATION_Z;
    if (rx) mask |= ROTATION_X;
    if (ry) mask |= ROTATION_Y;
    if (rz) mask |= ROTATION_Z;
    return mask;
}

constexpr uint32_t DOF_ALL = 0x3f;
constexpr uint32_t DOF_TRANSLATION_ONLY = 0x07;
constexpr uint32_t DOF_ROTATION_ONLY = 0x38;

}  // namespace dof

TUMBLE_NAMESPACE_END
