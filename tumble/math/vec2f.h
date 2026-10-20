#pragma once

// the math package's vec2.ts over Vec2f. two lanes don't earn a v128, so this one is scalar.

#include "math/scalar.h"
#include "math/types.h"

namespace vec2f {

inline Vec2f create() { return {0.0f, 0.0f}; }
inline Vec2f clone(const Vec2f &a) { return a; }
inline Vec2f from_values(float x, float y) { return {x, y}; }

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

inline Vec2f &add(Vec2f &out, const Vec2f &a, const Vec2f &b) { return set(out, a.x + b.x, a.y + b.y); }
inline Vec2f &subtract(Vec2f &out, const Vec2f &a, const Vec2f &b) { return set(out, a.x - b.x, a.y - b.y); }
inline Vec2f &multiply(Vec2f &out, const Vec2f &a, const Vec2f &b) { return set(out, a.x * b.x, a.y * b.y); }
inline Vec2f &scale(Vec2f &out, const Vec2f &a, float b) { return set(out, a.x * b, a.y * b); }
inline Vec2f &scale_and_add(Vec2f &out, const Vec2f &a, const Vec2f &b, float s) { return set(out, a.x + b.x * s, a.y + b.y * s); }
inline Vec2f &negate(Vec2f &out, const Vec2f &a) { return set(out, -a.x, -a.y); }

inline float dot(const Vec2f &a, const Vec2f &b) { return a.x * b.x + a.y * b.y; }
inline float squared_length(const Vec2f &a) { return dot(a, a); }
inline float length(const Vec2f &a) { return scalar::sqrt(squared_length(a)); }

inline float distance(const Vec2f &a, const Vec2f &b) {
    const float x = b.x - a.x;
    const float y = b.y - a.y;
    return scalar::sqrt(x * x + y * y);
}

/** the zero vector stays zero, as the js does. */
inline Vec2f &normalize(Vec2f &out, const Vec2f &a) {
    float len = squared_length(a);
    if (len > 0.0f) len = 1.0f / scalar::sqrt(len);
    return scale(out, a, len);
}

/** the 2d cross product as a z-only vec3. */
inline Float3 &cross(Float3 &out, const Vec2f &a, const Vec2f &b) {
    out.x = 0.0f;
    out.y = 0.0f;
    out.z = a.x * b.y - a.y * b.x;
    return out;
}

inline Vec2f &lerp(Vec2f &out, const Vec2f &a, const Vec2f &b, float t) {
    return set(out, a.x + t * (b.x - a.x), a.y + t * (b.y - a.y));
}

inline Vec2f &transform_mat2(Vec2f &out, const Vec2f &a, const Mat2f &m) {
    return set(out, m.m[0] * a.x + m.m[2] * a.y, m.m[1] * a.x + m.m[3] * a.y);
}

/** rotates a about b by rad. */
inline Vec2f &rotate(Vec2f &out, const Vec2f &a, const Vec2f &b, float rad) {
    const float p0 = a.x - b.x;
    const float p1 = a.y - b.y;
    const float s = scalar::sin(rad);
    const float c = scalar::cos(rad);
    return set(out, p0 * c - p1 * s + b.x, p0 * s + p1 * c + b.y);
}

/** the unsigned angle between a and b; zero when either is zero. */
inline float angle(const Vec2f &a, const Vec2f &b) {
    const float mag = scalar::sqrt(squared_length(a) * squared_length(b));
    const float cosine = mag != 0.0f ? dot(a, b) / mag : 0.0f;
    return scalar::acos(scalar::min(scalar::max(cosine, -1.0f), 1.0f));
}

inline bool exact_equals(const Vec2f &a, const Vec2f &b) { return a.x == b.x && a.y == b.y; }
inline bool equals(const Vec2f &a, const Vec2f &b) { return scalar::equals(a.x, b.x) && scalar::equals(a.y, b.y); }

inline Vec2f &mul(Vec2f &out, const Vec2f &a, const Vec2f &b) { return multiply(out, a, b); }
inline float len(const Vec2f &a) { return length(a); }
inline float sqr_len(const Vec2f &a) { return squared_length(a); }

}  // namespace vec2f
