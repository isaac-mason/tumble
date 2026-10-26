#pragma once

#include <stdint.h>
#include "tumble/config.h"

TUMBLE_NAMESPACE_BEGIN

enum class MotionType : uint32_t {
    STATIC = 0,
    KINEMATIC = 1,
    DYNAMIC = 2,
};

TUMBLE_NAMESPACE_END
