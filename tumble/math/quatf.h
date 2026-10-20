#pragma once

#include "tumble/math/scalar.h"
#include "tumble/math/simd.h"
#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/math/vec4f.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace quatf {

inline Quatf load(const Float4 &a) { return {v4f::load4(&a.x)}; }
inline Float4 &store(Float4 &out, Quatf a) {
    v4f::store4(&out.x, a.v);
    return out;
}

inline Quatf make(float x, float y, float z, float w) { return {v4f::make(x, y, z, w)}; }
inline Quatf identity() { return make(0.0f, 0.0f, 0.0f, 1.0f); }

inline float x(Quatf q) { return v4f::lane<0>(q.v); }
inline float y(Quatf q) { return v4f::lane<1>(q.v); }
inline float z(Quatf q) { return v4f::lane<2>(q.v); }
inline float w(Quatf q) { return v4f::lane<3>(q.v); }
inline Vec3f xyz(Quatf q) { return {v4f::shuffle<0, 1, 2, 2>(q.v)}; }

inline Quatf scale(Quatf a, float b) { return {v4f::mul(a.v, v4f::splat(b))}; }
inline float dot(Quatf a, Quatf b) { return v4f::dot4(a.v, b.v); }
inline Quatf normalize(Quatf a) { return {vec4f::normalize({a.v}).v}; }

/** rotation of rad about the unit axis */
inline Quatf set_axis_angle(Vec3f axis, float rad) {
    rad *= 0.5f;
    const float s = scalar::sin(rad);
    return make(s * vec3f::x(axis), s * vec3f::y(axis), s * vec3f::z(axis), scalar::cos(rad));
}

/** the hamilton product */
inline Quatf multiply(Quatf a, Quatf b) {
    const V128 flip_w = v4f::make(1.0f, 1.0f, 1.0f, -1.0f);
    const V128 t0 = v4f::mul(v4f::shuffle<3, 3, 3, 3>(a.v), b.v);
    const V128 t1 = v4f::mul(v4f::mul(v4f::shuffle<0, 1, 2, 0>(a.v), flip_w), v4f::shuffle<3, 3, 3, 0>(b.v));
    const V128 t2 = v4f::mul(v4f::mul(v4f::shuffle<1, 2, 0, 1>(a.v), flip_w), v4f::shuffle<2, 0, 1, 1>(b.v));
    const V128 t3 = v4f::mul(v4f::shuffle<2, 0, 1, 2>(a.v), v4f::shuffle<1, 2, 0, 2>(b.v));
    return {v4f::sub(v4f::add(v4f::add(t0, t1), t2), t3)};
}

/** zero for the zero quaternion */
inline Quatf invert(Quatf a) {
    const float d = dot(a, a);
    if (d == 0.0f) return {v4f::zero()};
    const float inv = 1.0f / d;
    return {v4f::mul(v4f::neg_xyz(a.v), v4f::splat(inv))};
}

inline Quatf conjugate(Quatf a) { return {v4f::neg_xyz(a.v)}; }

/** the diagonal picks the stable branch */
inline Quatf from_mat3(const Mat3f &m3) {
    const float *m = m3.m;
    float o[4];
    const float trace = m[0] + m[4] + m[8];
    float root;
    if (trace > 0.0f) {
        root = scalar::sqrt(trace + 1.0f);  // 2w
        o[3] = 0.5f * root;
        root = 0.5f / root;  // 1/(4w)
        o[0] = (m[5] - m[7]) * root;
        o[1] = (m[6] - m[2]) * root;
        o[2] = (m[1] - m[3]) * root;
    } else {
        int i = 0;
        if (m[4] > m[0]) i = 1;
        if (m[8] > m[i * 3 + i]) i = 2;
        const int j = (i + 1) % 3;
        const int k = (i + 2) % 3;
        root = scalar::sqrt(m[i * 3 + i] - m[j * 3 + j] - m[k * 3 + k] + 1.0f);
        o[i] = 0.5f * root;
        root = 0.5f / root;
        o[3] = (m[j * 3 + k] - m[k * 3 + j]) * root;
        o[j] = (m[j * 3 + i] + m[i * 3 + j]) * root;
        o[k] = (m[k * 3 + i] + m[i * 3 + k]) * root;
    }
    return make(o[0], o[1], o[2], o[3]);
}

/** from the rotation part */
inline Quatf from_mat4(const Mat4f &m) {
    const V128 c0 = m.c[0], c1 = m.c[1], c2 = m.c[2];
    const Mat3f m3 = {{v4f::lane<0>(c0), v4f::lane<1>(c0), v4f::lane<2>(c0), v4f::lane<0>(c1), v4f::lane<1>(c1),
                       v4f::lane<2>(c1), v4f::lane<0>(c2), v4f::lane<1>(c2), v4f::lane<2>(c2)}};
    return from_mat3(m3);
}

inline Quatf from_euler(const Eulerf &euler) {
    const float c1 = scalar::cos(euler.x / 2.0f);
    const float c2 = scalar::cos(euler.y / 2.0f);
    const float c3 = scalar::cos(euler.z / 2.0f);
    const float s1 = scalar::sin(euler.x / 2.0f);
    const float s2 = scalar::sin(euler.y / 2.0f);
    const float s3 = scalar::sin(euler.z / 2.0f);
    switch (euler.order) {
        case EULER_XYZ:
            return make(s1 * c2 * c3 + c1 * s2 * s3, c1 * s2 * c3 - s1 * c2 * s3, c1 * c2 * s3 + s1 * s2 * c3,
                        c1 * c2 * c3 - s1 * s2 * s3);
        case EULER_YXZ:
            return make(s1 * c2 * c3 + c1 * s2 * s3, c1 * s2 * c3 - s1 * c2 * s3, c1 * c2 * s3 - s1 * s2 * c3,
                        c1 * c2 * c3 + s1 * s2 * s3);
        case EULER_ZXY:
            return make(s1 * c2 * c3 - c1 * s2 * s3, c1 * s2 * c3 + s1 * c2 * s3, c1 * c2 * s3 + s1 * s2 * c3,
                        c1 * c2 * c3 - s1 * s2 * s3);
        case EULER_ZYX:
            return make(s1 * c2 * c3 - c1 * s2 * s3, c1 * s2 * c3 + s1 * c2 * s3, c1 * c2 * s3 - s1 * s2 * c3,
                        c1 * c2 * c3 + s1 * s2 * s3);
        case EULER_YZX:
            return make(s1 * c2 * c3 + c1 * s2 * s3, c1 * s2 * c3 + s1 * c2 * s3, c1 * c2 * s3 - s1 * s2 * c3,
                        c1 * c2 * c3 - s1 * s2 * s3);
        case EULER_XZY:
            return make(s1 * c2 * c3 - c1 * s2 * s3, c1 * s2 * c3 - s1 * c2 * s3, c1 * c2 * s3 + s1 * s2 * c3,
                        c1 * c2 * c3 + s1 * s2 * s3);
    }
    return identity();
}

}  // namespace quatf

TUMBLE_NAMESPACE_END
