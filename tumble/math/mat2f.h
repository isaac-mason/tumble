#pragma once

#include "tumble/math/types.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace mat2f {

inline Mat2f &set(Mat2f &out, float m00, float m01, float m10, float m11) {
    out.m[0] = m00;
    out.m[1] = m01;
    out.m[2] = m10;
    out.m[3] = m11;
    return out;
}

/** false, out untouched, for a singular matrix */
inline bool invert(Mat2f &out, const Mat2f &a) {
    const float a0 = a.m[0], a1 = a.m[1], a2 = a.m[2], a3 = a.m[3];
    float det = a0 * a3 - a2 * a1;
    if (det == 0.0f) return false;
    det = 1.0f / det;
    set(out, a3 * det, -a1 * det, -a2 * det, a0 * det);
    return true;
}

}  // namespace mat2f

TUMBLE_NAMESPACE_END
