#pragma once

// the scalar functions over the storage types, retired from src when the register types took over: the reference the
// register functions are checked against, bit for bit.

#include "tumble/math/vec3f.h"

TUMBLE_NAMESPACE_BEGIN

namespace vec3f {

inline Float3 create() { return {0.0f, 0.0f, 0.0f}; }
inline Float3 clone(const Float3 &a) { return a; }
inline Float3 from_values(float x, float y, float z) { return {x, y, z}; }

inline Float3 &copy(Float3 &out, const Float3 &a) {
    out = a;
    return out;
}

inline Float3 &set(Float3 &out, float x, float y, float z) {
    out.x = x;
    out.y = y;
    out.z = z;
    return out;
}

inline Float3 &set_scalar(Float3 &out, float s) { return set(out, s, s, s); }

inline Float3 &zero(Float3 &out) { return set(out, 0.0f, 0.0f, 0.0f); }

inline Float3 &from_buffer(Float3 &out, const float *buffer, uint32_t start) {
    return set(out, buffer[start], buffer[start + 1], buffer[start + 2]);
}

inline void to_buffer(float *buffer, const Float3 &v, uint32_t start) {
    buffer[start] = v.x;
    buffer[start + 1] = v.y;
    buffer[start + 2] = v.z;
}

inline Float3 &store(Float3 &out, v128_t v) {
    v4f::store3(&out.x, v);
    return out;
}

inline Float3 &add(Float3 &out, const Float3 &a, const Float3 &b) { return store(out, v4f::add(v4f::load3(&a.x), v4f::load3(&b.x))); }
inline Float3 &add_scalar(Float3 &out, const Float3 &a, float b) { return store(out, v4f::add(v4f::load3(&a.x), v4f::splat(b))); }
inline Float3 &subtract(Float3 &out, const Float3 &a, const Float3 &b) { return store(out, v4f::sub(v4f::load3(&a.x), v4f::load3(&b.x))); }
inline Float3 &subtract_scalar(Float3 &out, const Float3 &a, float b) { return store(out, v4f::sub(v4f::load3(&a.x), v4f::splat(b))); }
inline Float3 &multiply(Float3 &out, const Float3 &a, const Float3 &b) { return store(out, v4f::mul(v4f::load3(&a.x), v4f::load3(&b.x))); }
inline Float3 &divide(Float3 &out, const Float3 &a, const Float3 &b) { return store(out, v4f::div(v4f::load3(&a.x), v4f::load3(&b.x))); }
inline Float3 &min(Float3 &out, const Float3 &a, const Float3 &b) { return store(out, v4f::min(v4f::load3(&a.x), v4f::load3(&b.x))); }
inline Float3 &max(Float3 &out, const Float3 &a, const Float3 &b) { return store(out, v4f::max(v4f::load3(&a.x), v4f::load3(&b.x))); }
inline Float3 &negate(Float3 &out, const Float3 &a) { return store(out, v4f::neg(v4f::load3(&a.x))); }
inline Float3 &scale(Float3 &out, const Float3 &a, float b) { return store(out, v4f::mul(v4f::load3(&a.x), v4f::splat(b))); }
inline Float3 &scale_and_add(Float3 &out, const Float3 &a, const Float3 &b, float s) {
    return store(out, v4f::mul_add(v4f::load3(&b.x), v4f::splat(s), v4f::load3(&a.x)));
}

inline Float3 &ceil(Float3 &out, const Float3 &a) { return store(out, wasm_f32x4_ceil(v4f::load3(&a.x))); }
inline Float3 &floor(Float3 &out, const Float3 &a) { return store(out, wasm_f32x4_floor(v4f::load3(&a.x))); }
/** symmetric round per component, as scalar::round. */
inline Float3 &round(Float3 &out, const Float3 &a) {
    return set(out, scalar::round(a.x), scalar::round(a.y), scalar::round(a.z));
}

inline Float3 &inverse(Float3 &out, const Float3 &a) { return set(out, 1.0f / a.x, 1.0f / a.y, 1.0f / a.z); }

inline float dot(const Float3 &a, const Float3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline float squared_length(const Float3 &a) { return dot(a, a); }
inline float length(const Float3 &a) { return scalar::sqrt(squared_length(a)); }

inline float squared_distance(const Float3 &a, const Float3 &b) {
    const float x = b.x - a.x;
    const float y = b.y - a.y;
    const float z = b.z - a.z;
    return x * x + y * y + z * z;
}
inline float distance(const Float3 &a, const Float3 &b) { return scalar::sqrt(squared_distance(a, b)); }

/** the zero vector stays zero, as the js does. */
inline Float3 &normalize(Float3 &out, const Float3 &a) {
    float len = squared_length(a);
    if (len > 0.0f) len = 1.0f / scalar::sqrt(len);
    return scale(out, a, len);
}

inline Float3 &cross(Float3 &out, const Float3 &a, const Float3 &b) { return store(out, v4f::cross(v4f::load3(&a.x), v4f::load3(&b.x))); }

/** a unit vector perpendicular to a; +x for the zero vector, since every direction is. */
inline Float3 &perpendicular(Float3 &out, const Float3 &a) {
    if (scalar::abs(a.x) > scalar::abs(a.y)) {
        const float inv_len = 1.0f / scalar::sqrt(a.x * a.x + a.z * a.z);
        return set(out, a.z * inv_len, 0.0f, -a.x * inv_len);
    }
    const float len = scalar::sqrt(a.y * a.y + a.z * a.z);
    if (len == 0.0f) return set(out, 1.0f, 0.0f, 0.0f);
    const float inv_len = 1.0f / len;
    return set(out, 0.0f, a.z * inv_len, -a.y * inv_len);
}

inline Float3 &lerp(Float3 &out, const Float3 &a, const Float3 &b, float t) {
    const v128_t va = v4f::load3(&a.x);
    return store(out, v4f::mul_add(v4f::sub(v4f::load3(&b.x), va), v4f::splat(t), va));
}

/** quadratic through a at t=0, b at t=0.5, c at t=1. */
inline Float3 &lagrange(Float3 &out, const Float3 &a, const Float3 &b, const Float3 &c, float t) {
    const float c0 = 2.0f * (t - 1.0f) * (t - 0.5f);
    const float c1 = -4.0f * (t - 1.0f) * t;
    const float c2 = 2.0f * (t - 0.5f) * t;
    v128_t r = v4f::mul(v4f::load3(&a.x), v4f::splat(c0));
    r = v4f::mul_add(v4f::load3(&b.x), v4f::splat(c1), r);
    r = v4f::mul_add(v4f::load3(&c.x), v4f::splat(c2), r);
    return store(out, r);
}

/** spherical interpolation between unit vectors. */
inline Float3 &slerp(Float3 &out, const Float3 &a, const Float3 &b, float t) {
    const float angle = scalar::acos(scalar::min(scalar::max(dot(a, b), -1.0f), 1.0f));
    const float sin_total = scalar::sin(angle);
    const float ratio_a = scalar::sin((1.0f - t) * angle) / sin_total;
    const float ratio_b = scalar::sin(t * angle) / sin_total;
    return store(out, v4f::mul_add(v4f::load3(&b.x), v4f::splat(ratio_b), v4f::mul(v4f::load3(&a.x), v4f::splat(ratio_a))));
}

inline Float3 &hermite(Float3 &out, const Float3 &a, const Float3 &b, const Float3 &c, const Float3 &d, float t) {
    const float t2 = t * t;
    const float f1 = t2 * (2.0f * t - 3.0f) + 1.0f;
    const float f2 = t2 * (t - 2.0f) + t;
    const float f3 = t2 * (t - 1.0f);
    const float f4 = t2 * (3.0f - 2.0f * t);
    v128_t r = v4f::mul(v4f::load3(&a.x), v4f::splat(f1));
    r = v4f::mul_add(v4f::load3(&b.x), v4f::splat(f2), r);
    r = v4f::mul_add(v4f::load3(&c.x), v4f::splat(f3), r);
    r = v4f::mul_add(v4f::load3(&d.x), v4f::splat(f4), r);
    return store(out, r);
}

inline Float3 &bezier(Float3 &out, const Float3 &a, const Float3 &b, const Float3 &c, const Float3 &d, float t) {
    const float inv = 1.0f - t;
    const float inv2 = inv * inv;
    const float t2 = t * t;
    const float f1 = inv2 * inv;
    const float f2 = 3.0f * t * inv2;
    const float f3 = 3.0f * t2 * inv;
    const float f4 = t2 * t;
    v128_t r = v4f::mul(v4f::load3(&a.x), v4f::splat(f1));
    r = v4f::mul_add(v4f::load3(&b.x), v4f::splat(f2), r);
    r = v4f::mul_add(v4f::load3(&c.x), v4f::splat(f3), r);
    r = v4f::mul_add(v4f::load3(&d.x), v4f::splat(f4), r);
    return store(out, r);
}

/** a as a point through m, with the perspective divide; a zero (or nan) w divides by one, as the js `w || 1`. */
inline Float3 &transform_mat4(Float3 &out, const Float3 &a, const Float4x4 &m) {
    v128_t r = v4f::mul(v4f::load4(&m.m[0]), v4f::splat(a.x));
    r = v4f::mul_add(v4f::load4(&m.m[4]), v4f::splat(a.y), r);
    r = v4f::mul_add(v4f::load4(&m.m[8]), v4f::splat(a.z), r);
    r = v4f::add(r, v4f::load4(&m.m[12]));
    float w = v4f::lane<3>(r);
    if (w == 0.0f || w != w) w = 1.0f;
    return store(out, v4f::div(r, v4f::splat(w)));
}

inline Float3 &transform_mat3(Float3 &out, const Float3 &a, const Mat3f &m) {
    return set(out, a.x * m.m[0] + a.y * m.m[3] + a.z * m.m[6], a.x * m.m[1] + a.y * m.m[4] + a.z * m.m[7],
               a.x * m.m[2] + a.y * m.m[5] + a.z * m.m[8]);
}

/** rotates a by q: a + 2w (q x a) + 2 (q x (q x a)). */
inline Float3 &transform_quat(Float3 &out, const Float3 &a, const Float4 &q) {
    const v128_t qv = v4f::load4(&q.x);
    const v128_t va = v4f::load3(&a.x);
    const v128_t uv = v4f::cross(qv, va);
    const v128_t uuv = v4f::cross(qv, uv);
    v128_t r = v4f::mul_add(uv, v4f::splat(q.w * 2.0f), va);
    r = v4f::mul_add(uuv, v4f::splat(2.0f), r);
    return store(out, r);
}

/** rotates a about the x axis through b by rad. */
inline Float3 &rotate_x(Float3 &out, const Float3 &a, const Float3 &b, float rad) {
    const float p1 = a.y - b.y;
    const float p2 = a.z - b.z;
    const float c = scalar::cos(rad);
    const float s = scalar::sin(rad);
    return set(out, a.x, p1 * c - p2 * s + b.y, p1 * s + p2 * c + b.z);
}

inline Float3 &rotate_y(Float3 &out, const Float3 &a, const Float3 &b, float rad) {
    const float p0 = a.x - b.x;
    const float p2 = a.z - b.z;
    const float c = scalar::cos(rad);
    const float s = scalar::sin(rad);
    return set(out, p2 * s + p0 * c + b.x, a.y, p2 * c - p0 * s + b.z);
}

inline Float3 &rotate_z(Float3 &out, const Float3 &a, const Float3 &b, float rad) {
    const float p0 = a.x - b.x;
    const float p1 = a.y - b.y;
    const float c = scalar::cos(rad);
    const float s = scalar::sin(rad);
    return set(out, p0 * c - p1 * s + b.x, p0 * s + p1 * c + b.y, a.z);
}

/** the unsigned angle between a and b in [0, pi]; zero when either is zero. */
inline float angle(const Float3 &a, const Float3 &b) {
    const float mag = scalar::sqrt(squared_length(a) * squared_length(b));
    const float cosine = mag != 0.0f ? dot(a, b) / mag : 0.0f;
    return scalar::acos(scalar::min(scalar::max(cosine, -1.0f), 1.0f));
}

/** the signed angle from a to b about axis, in (-pi, pi]; the components along the axis carry no angle. */
inline float signed_angle(const Float3 &a, const Float3 &b, const Float3 &axis) {
    const float ad = dot(a, axis);
    const Float3 pa = {a.x - axis.x * ad, a.y - axis.y * ad, a.z - axis.z * ad};
    const float bd = dot(b, axis);
    const Float3 pb = {b.x - axis.x * bd, b.y - axis.y * bd, b.z - axis.z * bd};
    Float3 c;
    cross(c, pa, pb);
    return scalar::atan2(dot(c, axis), dot(pa, pb));
}

/** turns the unit vector `from` toward the unit vector `to` by at most max_angle (rodrigues). */
inline Float3 &rotate_towards(Float3 &out, const Float3 &from, const Float3 &to, float max_angle) {
    const float cosine = scalar::min(scalar::max(dot(from, to), -1.0f), 1.0f);
    const float limit = max_angle > 0.0f ? max_angle : 0.0f;
    if (scalar::acos(cosine) <= limit) return copy(out, to);

    Float3 axis;
    cross(axis, from, to);
    const float axis_length = length(axis);
    if (axis_length < 1e-8f) {
        // antiparallel: every plane containing `from` is equally valid, so pick one
        perpendicular(axis, from);
    } else {
        scale(axis, axis, 1.0f / axis_length);
    }
    const float c = scalar::cos(limit);
    const float s = scalar::sin(limit);
    const float d = dot(axis, from) * (1.0f - c);
    return set(out, from.x * c + (axis.y * from.z - axis.z * from.y) * s + axis.x * d,
               from.y * c + (axis.z * from.x - axis.x * from.z) * s + axis.y * d,
               from.z * c + (axis.x * from.y - axis.y * from.x) * s + axis.z * d);
}

inline bool exact_equals(const Float3 &a, const Float3 &b) { return a.x == b.x && a.y == b.y && a.z == b.z; }

inline bool equals(const Float3 &a, const Float3 &b) {
    return scalar::equals(a.x, b.x) && scalar::equals(a.y, b.y) && scalar::equals(a.z, b.z);
}

inline bool finite(const Float3 &a) { return scalar::is_finite(a.x) && scalar::is_finite(a.y) && scalar::is_finite(a.z); }

/** an odd number of negative components flips handedness. */
inline bool is_scale_inside_out(const Float3 &scale) {
    const uint32_t mask = (scale.x < 0.0f ? 1u : 0u) | (scale.y < 0.0f ? 2u : 0u) | (scale.z < 0.0f ? 4u : 0u);
    return (__builtin_popcount(mask) & 1u) != 0u;
}

// the math package's short names, kept so a ported file reads like its source
inline Float3 &sub(Float3 &out, const Float3 &a, const Float3 &b) { return subtract(out, a, b); }
inline Float3 &mul(Float3 &out, const Float3 &a, const Float3 &b) { return multiply(out, a, b); }
inline Float3 &div(Float3 &out, const Float3 &a, const Float3 &b) { return divide(out, a, b); }
inline float dist(const Float3 &a, const Float3 &b) { return distance(a, b); }
inline float sqr_dist(const Float3 &a, const Float3 &b) { return squared_distance(a, b); }
inline float len(const Float3 &a) { return length(a); }
inline float sqr_len(const Float3 &a) { return squared_length(a); }

}  // namespace vec3f

TUMBLE_NAMESPACE_END
