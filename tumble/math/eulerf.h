#pragma once

#include "tumble/config.h"
#include "tumble/math/types.h"

TUMBLE_NAMESPACE_BEGIN

namespace eulerf {

inline Eulerf &set(Eulerf &out, float x, float y, float z, EulerOrder order) {
    out.x = x;
    out.y = y;
    out.z = z;
    out.order = order;
    return out;
}

}  // namespace eulerf

TUMBLE_NAMESPACE_END
