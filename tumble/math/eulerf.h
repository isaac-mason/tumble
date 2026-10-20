#pragma once

#include "tumble/math/types.h"
#include "tumble/tumble.h"

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
