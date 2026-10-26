#pragma once

#include "tumble/config.h"
#include "tumble/math/scalar.h"
#include "tumble/math/simd.h"
#include "tumble/math/types.h"
#include "tumble/util/bits.h"

TUMBLE_NAMESPACE_BEGIN

namespace vec3f {

/** lanes (x, y, z, z) from a value whose lanes 0..2 are x, y, z */
inline V128 fix_w(V128 v) { return v4f::shuffle<0, 1, 2, 2>(v); }

inline Vec3f load(const Float3 &a) {
    return {v4f::load3_xyzz(&a.x)};
}
inline Float3 &store(Float3 &out, Vec3f a) {
    v4f::store3(&out.x, a.v);
    return out;
}
inline Vec3f load(const float *buffer, uint32_t start) { return load(*reinterpret_cast<const Float3 *>(buffer + start)); }
inline void store(float *buffer, Vec3f a, uint32_t start) { v4f::store3(buffer + start, a.v); }

inline Vec3f make(float x, float y, float z) { return {v4f::make(x, y, z, z)}; }
inline Vec3f splat(float s) { return {v4f::splat(s)}; }
inline Vec3f zero() { return {v4f::zero()}; }
inline Vec3f axis_x() { return make(1.0f, 0.0f, 0.0f); }
inline Vec3f axis_y() { return make(0.0f, 1.0f, 0.0f); }
inline Vec3f axis_z() { return make(0.0f, 0.0f, 1.0f); }

inline float x(Vec3f a) { return v4f::lane<0>(a.v); }
inline float y(Vec3f a) { return v4f::lane<1>(a.v); }
inline float z(Vec3f a) { return v4f::lane<2>(a.v); }
inline Vec3f with_x(Vec3f a, float s) { return {v4f::set_lane<0>(a.v, s)}; }
inline Vec3f with_y(Vec3f a, float s) { return {v4f::set_lane<1>(a.v, s)}; }
inline Vec3f with_z(Vec3f a, float s) { return {v4f::shuffle2<0, 1, 4, 4>(a.v, v4f::splat(s))}; }

inline Vec3f add(Vec3f a, Vec3f b) { return {v4f::add(a.v, b.v)}; }
inline Vec3f subtract(Vec3f a, Vec3f b) { return {v4f::sub(a.v, b.v)}; }
inline Vec3f multiply(Vec3f a, Vec3f b) { return {v4f::mul(a.v, b.v)}; }
inline Vec3f divide(Vec3f a, Vec3f b) { return {v4f::div(a.v, b.v)}; }
/** b < a ? b : a per lane */
inline Vec3f min(Vec3f a, Vec3f b) { return {v4f::min(a.v, b.v)}; }
/** a < b ? b : a per lane */
inline Vec3f max(Vec3f a, Vec3f b) { return {v4f::max(a.v, b.v)}; }
/** scalar::min per lane */
inline Vec3f fmin(Vec3f a, Vec3f b) { return {v4f::fmin(a.v, b.v)}; }
/** scalar::max per lane */
inline Vec3f fmax(Vec3f a, Vec3f b) { return {v4f::fmax(a.v, b.v)}; }
inline Vec3f negate(Vec3f a) { return {v4f::neg(a.v)}; }
inline Vec3f abs(Vec3f a) { return {v4f::abs(a.v)}; }
inline Vec3f scale(Vec3f a, float b) { return {v4f::mul(a.v, v4f::splat(b))}; }
/** a + b * s */
inline Vec3f scale_and_add(Vec3f a, Vec3f b, float s) { return {v4f::mul_add(b.v, v4f::splat(s), a.v)}; }

inline Vec3f inverse(Vec3f a) { return {v4f::div(v4f::splat(1.0f), a.v)}; }

inline float dot(Vec3f a, Vec3f b) {
    const V128 m = v4f::mul(a.v, b.v);
    return v4f::lane<0>(m) + v4f::lane<1>(m) + v4f::lane<2>(m);
}
/** (x + y) + z */
inline float reduce_sum(Vec3f a) { return x(a) + y(a) + z(a); }
inline float squared_length(Vec3f a) { return dot(a, a); }
inline float length(Vec3f a) { return scalar::sqrt(squared_length(a)); }
inline float squared_distance(Vec3f a, Vec3f b) {
    const Vec3f d = subtract(b, a);
    return dot(d, d);
}
inline float distance(Vec3f a, Vec3f b) { return scalar::sqrt(squared_distance(a, b)); }

/** the zero vector stays zero */
inline Vec3f normalize(Vec3f a) {
    float len = squared_length(a);
    if (len > 0.0f) len = 1.0f / scalar::sqrt(len);
    return scale(a, len);
}

/** lane 3 a copy of z */
inline V128 cross_lanes(V128 a, V128 b) {
    return v4f::sub(v4f::mul(v4f::shuffle<1, 2, 0, 0>(a), v4f::shuffle<2, 0, 1, 1>(b)),
                    v4f::mul(v4f::shuffle<2, 0, 1, 1>(a), v4f::shuffle<1, 2, 0, 0>(b)));
}
inline Vec3f cross(Vec3f a, Vec3f b) { return {cross_lanes(a.v, b.v)}; }

/** a unit vector perpendicular to a; +x for the zero vector, since every direction is */
inline Vec3f perpendicular(Vec3f a) {
    const float ax = x(a), ay = y(a), az = z(a);
    if (scalar::abs(ax) > scalar::abs(ay)) {
        const float inv_len = 1.0f / scalar::sqrt(ax * ax + az * az);
        return make(az * inv_len, 0.0f, -ax * inv_len);
    }
    const float len = scalar::sqrt(ay * ay + az * az);
    if (len == 0.0f) return axis_x();
    const float inv_len = 1.0f / len;
    return make(0.0f, az * inv_len, -ay * inv_len);
}

/** a unit vector perpendicular to a, which must not be zero */
inline Vec3f normalized_perpendicular(Vec3f a) {
    const float ax = x(a), ay = y(a), az = z(a);
    if (scalar::abs(ax) > scalar::abs(ay)) {
        const float len = scalar::sqrt(ax * ax + az * az);
        return make(az / len, 0.0f, -ax / len);
    }
    const float len = scalar::sqrt(ay * ay + az * az);
    return make(0.0f, az / len, -ay / len);
}

inline Vec3f lerp(Vec3f a, Vec3f b, float t) { return {v4f::mul_add(v4f::sub(b.v, a.v), v4f::splat(t), a.v)}; }

/** a as a point through m, with the perspective divide; a zero or nan w divides by one */
inline Vec3f transform_mat4(Vec3f a, const Mat4f &m) {
    V128 r = v4f::mul(m.c[0], v4f::splat(x(a)));
    r = v4f::mul_add(m.c[1], v4f::splat(y(a)), r);
    r = v4f::mul_add(m.c[2], v4f::splat(z(a)), r);
    r = v4f::add(r, m.c[3]);
    float w = v4f::lane<3>(r);
    if (w == 0.0f || w != w) w = 1.0f;
    return {fix_w(v4f::div(r, v4f::splat(w)))};
}

/** a + 2w (q x a) + 2 (q x (q x a)) */
inline Vec3f transform_quat(Vec3f a, Quatf q) {
    const V128 uv = cross_lanes(q.v, a.v);
    const V128 uuv = cross_lanes(q.v, uv);
    V128 r = v4f::mul_add(uv, v4f::splat(v4f::lane<3>(q.v) * 2.0f), a.v);
    r = v4f::mul_add(uuv, v4f::splat(2.0f), r);
    return {r};
}

inline bool exact_equals(Vec3f a, Vec3f b) { return v4f::all_true(v4f::eq(a.v, b.v)); }

/** an odd number of negative components flips handedness */
inline bool is_scale_inside_out(Vec3f scale) {
    return (bits::count_ones(v4f::bitmask(v4f::lt(scale.v, v4f::zero())) & 7u) & 1u) != 0u;
}

}  // namespace vec3f

TUMBLE_NAMESPACE_END
