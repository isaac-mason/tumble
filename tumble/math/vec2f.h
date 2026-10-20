#pragma once

#include "tumble/math/types.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace vec2f {

inline Vec2f &copy(Vec2f &out, const Vec2f &a) {
    out = a;
    return out;
}

inline Vec2f &set(Vec2f &out, float x, float y) {
    out.x = x;
    out.y = y;
    return out;
}

inline Vec2f &zero(Vec2f &out) { return set(out, 0.0f, 0.0f); }

inline Vec2f &scale(Vec2f &out, const Vec2f &a, float b) { return set(out, a.x * b, a.y * b); }

inline Vec2f &transform_mat2(Vec2f &out, const Vec2f &a, const Mat2f &m) {
    return set(out, m.m[0] * a.x + m.m[2] * a.y, m.m[1] * a.x + m.m[3] * a.y);
}

}  // namespace vec2f

TUMBLE_NAMESPACE_END
