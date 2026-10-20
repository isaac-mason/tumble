#pragma once

#include "tumble/math/scalar.h"
#include "tumble/math/types.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace mat2f {

inline Mat2f create() { return {{1, 0, 0, 1}}; }
inline Mat2f clone(const Mat2f &a) { return a; }
inline Mat2f from_values(float m00, float m01, float m10, float m11) { return {{m00, m01, m10, m11}}; }

inline Mat2f &copy(Mat2f &out, const Mat2f &a) {
    out = a;
    return out;
}

inline Mat2f &set(Mat2f &out, float m00, float m01, float m10, float m11) {
    out.m[0] = m00;
    out.m[1] = m01;
    out.m[2] = m10;
    out.m[3] = m11;
    return out;
}

inline Mat2f &identity(Mat2f &out) { return set(out, 1, 0, 0, 1); }

inline Mat2f &transpose(Mat2f &out, const Mat2f &a) {
    if (&out == &a) {
        const float a1 = a.m[1];
        out.m[1] = a.m[2];
        out.m[2] = a1;
        return out;
    }
    return set(out, a.m[0], a.m[2], a.m[1], a.m[3]);
}

/** false, out untouched, for a singular matrix. */
inline bool invert(Mat2f &out, const Mat2f &a) {
    const float a0 = a.m[0], a1 = a.m[1], a2 = a.m[2], a3 = a.m[3];
    float det = a0 * a3 - a2 * a1;
    if (det == 0.0f) return false;
    det = 1.0f / det;
    set(out, a3 * det, -a1 * det, -a2 * det, a0 * det);
    return true;
}

inline Mat2f &adjoint(Mat2f &out, const Mat2f &a) {
    const float a0 = a.m[0];
    return set(out, a.m[3], -a.m[1], -a.m[2], a0);
}

inline float determinant(const Mat2f &a) { return a.m[0] * a.m[3] - a.m[2] * a.m[1]; }

inline Mat2f &multiply(Mat2f &out, const Mat2f &a, const Mat2f &b) {
    const float a0 = a.m[0], a1 = a.m[1], a2 = a.m[2], a3 = a.m[3];
    const float b0 = b.m[0], b1 = b.m[1], b2 = b.m[2], b3 = b.m[3];
    return set(out, a0 * b0 + a2 * b1, a1 * b0 + a3 * b1, a0 * b2 + a2 * b3, a1 * b2 + a3 * b3);
}

inline Mat2f &rotate(Mat2f &out, const Mat2f &a, float rad) {
    const float a0 = a.m[0], a1 = a.m[1], a2 = a.m[2], a3 = a.m[3];
    const float s = scalar::sin(rad);
    const float c = scalar::cos(rad);
    return set(out, a0 * c + a2 * s, a1 * c + a3 * s, a0 * -s + a2 * c, a1 * -s + a3 * c);
}

inline Mat2f &scale(Mat2f &out, const Mat2f &a, const Vec2f &v) {
    return set(out, a.m[0] * v.x, a.m[1] * v.x, a.m[2] * v.y, a.m[3] * v.y);
}

inline Mat2f &from_rotation(Mat2f &out, float rad) {
    const float s = scalar::sin(rad);
    const float c = scalar::cos(rad);
    return set(out, c, s, -s, c);
}

inline Mat2f &from_scaling(Mat2f &out, const Vec2f &v) { return set(out, v.x, 0, 0, v.y); }

inline float frob(const Mat2f &a) {
    return scalar::sqrt(a.m[0] * a.m[0] + a.m[1] * a.m[1] + a.m[2] * a.m[2] + a.m[3] * a.m[3]);
}

/** the ldu decomposition: a = L * D * U with D the identity. */
inline void ldu(Mat2f &L, Mat2f &D, Mat2f &U, const Mat2f &a) {
    const float l = a.m[2] / a.m[0];
    set(L, 1, 0, l, 1);
    set(D, 1, 0, 0, 1);
    set(U, a.m[0], a.m[1], 0, a.m[3] - l * a.m[1]);
}

inline Mat2f &add(Mat2f &out, const Mat2f &a, const Mat2f &b) {
    return set(out, a.m[0] + b.m[0], a.m[1] + b.m[1], a.m[2] + b.m[2], a.m[3] + b.m[3]);
}

inline Mat2f &subtract(Mat2f &out, const Mat2f &a, const Mat2f &b) {
    return set(out, a.m[0] - b.m[0], a.m[1] - b.m[1], a.m[2] - b.m[2], a.m[3] - b.m[3]);
}

inline Mat2f &multiply_scalar(Mat2f &out, const Mat2f &a, float b) {
    return set(out, a.m[0] * b, a.m[1] * b, a.m[2] * b, a.m[3] * b);
}

inline Mat2f &multiply_scalar_and_add(Mat2f &out, const Mat2f &a, const Mat2f &b, float s) {
    return set(out, a.m[0] + b.m[0] * s, a.m[1] + b.m[1] * s, a.m[2] + b.m[2] * s, a.m[3] + b.m[3] * s);
}

inline bool exact_equals(const Mat2f &a, const Mat2f &b) {
    return a.m[0] == b.m[0] && a.m[1] == b.m[1] && a.m[2] == b.m[2] && a.m[3] == b.m[3];
}

inline bool equals(const Mat2f &a, const Mat2f &b) {
    return scalar::equals(a.m[0], b.m[0]) && scalar::equals(a.m[1], b.m[1]) && scalar::equals(a.m[2], b.m[2]) &&
           scalar::equals(a.m[3], b.m[3]);
}

inline Mat2f &mul(Mat2f &out, const Mat2f &a, const Mat2f &b) { return multiply(out, a, b); }

}  // namespace mat2f

TUMBLE_NAMESPACE_END
