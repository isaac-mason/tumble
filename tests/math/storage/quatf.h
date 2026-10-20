#pragma once

#include "tumble/math/quatf.h"
#include "math/storage/vec3f.h"
#include "math/storage/vec4f.h"

TUMBLE_NAMESPACE_BEGIN

namespace quatf {

inline Float4 &set(Float4 &out, float x, float y, float z, float w) {
    out.x = x;
    out.y = y;
    out.z = z;
    out.w = w;
    return out;
}

inline Float4 &identity(Float4 &out) { return set(out, 0.0f, 0.0f, 0.0f, 1.0f); }

inline Float4 &store(Float4 &out, v128_t v) {
    v4f::store4(&out.x, v);
    return out;
}

// a quaternion is stored as a Float4, so the vec4 arithmetic applies as it is
inline Float4 &scale(Float4 &out, const Float4 &a, float b) { return vec4f::scale(out, a, b); }
inline float dot(const Float4 &a, const Float4 &b) { return vec4f::dot(a, b); }
inline Float4 &normalize(Float4 &out, const Float4 &a) { return vec4f::normalize(out, a); }
/** rotation of rad about the unit axis. */
inline Float4 &set_axis_angle(Float4 &out, const Float3 &axis, float rad) {
    rad *= 0.5f;
    const float s = scalar::sin(rad);
    return set(out, s * axis.x, s * axis.y, s * axis.z, scalar::cos(rad));
}

/** a * b: the hamilton product, one lane. */
inline Float4 &multiply(Float4 &out, const Float4 &a, const Float4 &b) {
    const v128_t va = v4f::load4(&a.x);
    const v128_t vb = v4f::load4(&b.x);
    const v128_t flip_w = v4f::make(1.0f, 1.0f, 1.0f, -1.0f);
    // lane i of each term, written out: x: aw bx + ax bw + ay bz - az by, and so on round to
    // w: aw bw - ax bx - ay by - az bz
    const v128_t t0 = v4f::mul(v4f::splat(a.w), vb);
    const v128_t t1 = v4f::mul(v4f::mul(v4f::shuffle<0, 1, 2, 0>(va), flip_w), v4f::shuffle<3, 3, 3, 0>(vb));
    const v128_t t2 = v4f::mul(v4f::mul(v4f::shuffle<1, 2, 0, 1>(va), flip_w), v4f::shuffle<2, 0, 1, 1>(vb));
    const v128_t t3 = v4f::mul(v4f::shuffle<2, 0, 1, 2>(va), v4f::shuffle<1, 2, 0, 2>(vb));
    return store(out, v4f::sub(v4f::add(v4f::add(t0, t1), t2), t3));
}

/** the inverse; zero for the zero quaternion. */
inline Float4 &invert(Float4 &out, const Float4 &a) {
    const float d = dot(a, a);
    if (d == 0.0f) return set(out, 0.0f, 0.0f, 0.0f, 0.0f);
    const float inv = 1.0f / d;
    return set(out, -a.x * inv, -a.y * inv, -a.z * inv, a.w * inv);
}

inline Float4 &conjugate(Float4 &out, const Float4 &a) { return set(out, -a.x, -a.y, -a.z, a.w); }

/** from a rotation matrix; the diagonal picks the stable branch. */
inline Float4 &from_mat3(Float4 &out, const Mat3f &m3) {
    const float *m = m3.m;
    const float trace = m[0] + m[4] + m[8];
    float root;
    if (trace > 0.0f) {
        root = scalar::sqrt(trace + 1.0f);  // 2w
        out.w = 0.5f * root;
        root = 0.5f / root;  // 1/(4w)
        out.x = (m[5] - m[7]) * root;
        out.y = (m[6] - m[2]) * root;
        out.z = (m[1] - m[3]) * root;
    } else {
        int i = 0;
        if (m[4] > m[0]) i = 1;
        if (m[8] > m[i * 3 + i]) i = 2;
        const int j = (i + 1) % 3;
        const int k = (i + 2) % 3;
        float *o = &out.x;
        root = scalar::sqrt(m[i * 3 + i] - m[j * 3 + j] - m[k * 3 + k] + 1.0f);
        o[i] = 0.5f * root;
        root = 0.5f / root;
        out.w = (m[j * 3 + k] - m[k * 3 + j]) * root;
        o[j] = (m[j * 3 + i] + m[i * 3 + j]) * root;
        o[k] = (m[k * 3 + i] + m[i * 3 + k]) * root;
    }
    return out;
}

/** from the rotation part of a mat4. */
inline Float4 &from_mat4(Float4 &out, const Float4x4 &m) {
    const Mat3f m3 = {{m.m[0], m.m[1], m.m[2], m.m[4], m.m[5], m.m[6], m.m[8], m.m[9], m.m[10]}};
    return from_mat3(out, m3);
}

inline Float4 &from_euler(Float4 &out, const Eulerf &euler) {
    const float c1 = scalar::cos(euler.x / 2.0f);
    const float c2 = scalar::cos(euler.y / 2.0f);
    const float c3 = scalar::cos(euler.z / 2.0f);
    const float s1 = scalar::sin(euler.x / 2.0f);
    const float s2 = scalar::sin(euler.y / 2.0f);
    const float s3 = scalar::sin(euler.z / 2.0f);
    switch (euler.order) {
        case EULER_XYZ:
            return set(out, s1 * c2 * c3 + c1 * s2 * s3, c1 * s2 * c3 - s1 * c2 * s3, c1 * c2 * s3 + s1 * s2 * c3,
                       c1 * c2 * c3 - s1 * s2 * s3);
        case EULER_YXZ:
            return set(out, s1 * c2 * c3 + c1 * s2 * s3, c1 * s2 * c3 - s1 * c2 * s3, c1 * c2 * s3 - s1 * s2 * c3,
                       c1 * c2 * c3 + s1 * s2 * s3);
        case EULER_ZXY:
            return set(out, s1 * c2 * c3 - c1 * s2 * s3, c1 * s2 * c3 + s1 * c2 * s3, c1 * c2 * s3 + s1 * s2 * c3,
                       c1 * c2 * c3 - s1 * s2 * s3);
        case EULER_ZYX:
            return set(out, s1 * c2 * c3 - c1 * s2 * s3, c1 * s2 * c3 + s1 * c2 * s3, c1 * c2 * s3 - s1 * s2 * c3,
                       c1 * c2 * c3 + s1 * s2 * s3);
        case EULER_YZX:
            return set(out, s1 * c2 * c3 + c1 * s2 * s3, c1 * s2 * c3 + s1 * c2 * s3, c1 * c2 * s3 - s1 * s2 * c3,
                       c1 * c2 * c3 - s1 * s2 * s3);
        case EULER_XZY:
            return set(out, s1 * c2 * c3 - c1 * s2 * s3, c1 * s2 * c3 - s1 * c2 * s3, c1 * c2 * s3 + s1 * s2 * c3,
                       c1 * c2 * c3 + s1 * s2 * s3);
    }
    return out;
}

}  // namespace quatf

TUMBLE_NAMESPACE_END
