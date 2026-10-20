#pragma once

#include <stdint.h>
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace motion_type {

/** crashcat's motion-type.ts. */
enum MotionType : uint32_t {
    MOTION_STATIC = 0,
    MOTION_KINEMATIC = 1,
    MOTION_DYNAMIC = 2,
};

}  // namespace motion_type

TUMBLE_NAMESPACE_END
