#pragma once

// the math package's mat3.ts over Mat3f, column-major. `invert` and `normal_from_mat4` return
// false where the js returns null (a singular matrix leaves out untouched).

#include "math/scalar.h"
#include "math/types.h"

namespace mat3f {

inline Mat3f create() { return {{1, 0, 0, 0, 1, 0, 0, 0, 1}}; }
inline Mat3f clone(const Mat3f &a) { return a; }

inline Mat3f from_values(float m00, float m01, float m02, float m10, float m11, float m12, float m20, float m21, float m22) {
    return {{m00, m01, m02, m10, m11, m12, m20, m21, m22}};
}

inline Mat3f &copy(Mat3f &out, const Mat3f &a) {
    out = a;
    return out;
}

inline Mat3f &set(Mat3f &out, float m00, float m01, float m02, float m10, float m11, float m12, float m20, float m21,
                  float m22) {
    out.m[0] = m00;
    out.m[1] = m01;
    out.m[2] = m02;
    out.m[3] = m10;
    out.m[4] = m11;
    out.m[5] = m12;
    out.m[6] = m20;
    out.m[7] = m21;
    out.m[8] = m22;
    return out;
}

inline Mat3f &identity(Mat3f &out) { return set(out, 1, 0, 0, 0, 1, 0, 0, 0, 1); }
inline Mat3f &zero(Mat3f &out) { return set(out, 0, 0, 0, 0, 0, 0, 0, 0, 0); }

/** the upper-left 3x3 of a mat4. */
inline Mat3f &from_mat4(Mat3f &out, const Float4x4 &a) {
    return set(out, a.m[0], a.m[1], a.m[2], a.m[4], a.m[5], a.m[6], a.m[8], a.m[9], a.m[10]);
}

inline Mat3f &transpose(Mat3f &out, const Mat3f &a) {
    if (&out == &a) {
        const float a01 = a.m[1];
        const float a02 = a.m[2];
        const float a12 = a.m[5];
        out.m[1] = a.m[3];
        out.m[2] = a.m[6];
        out.m[3] = a01;
        out.m[5] = a.m[7];
        out.m[6] = a02;
        out.m[7] = a12;
        return out;
    }
    return set(out, a.m[0], a.m[3], a.m[6], a.m[1], a.m[4], a.m[7], a.m[2], a.m[5], a.m[8]);
}

/** false, out untouched, for a singular matrix. */
inline bool invert(Mat3f &out, const Mat3f &a) {
    const float a00 = a.m[0], a01 = a.m[1], a02 = a.m[2];
    const float a10 = a.m[3], a11 = a.m[4], a12 = a.m[5];
    const float a20 = a.m[6], a21 = a.m[7], a22 = a.m[8];
    const float b01 = a22 * a11 - a12 * a21;
    const float b11 = -a22 * a10 + a12 * a20;
    const float b21 = a21 * a10 - a11 * a20;
    float det = a00 * b01 + a01 * b11 + a02 * b21;
    if (det == 0.0f) return false;
    det = 1.0f / det;
    out.m[0] = b01 * det;
    out.m[1] = (-a22 * a01 + a02 * a21) * det;
    out.m[2] = (a12 * a01 - a02 * a11) * det;
    out.m[3] = b11 * det;
    out.m[4] = (a22 * a00 - a02 * a20) * det;
    out.m[5] = (-a12 * a00 + a02 * a10) * det;
    out.m[6] = b21 * det;
    out.m[7] = (-a21 * a00 + a01 * a20) * det;
    out.m[8] = (a11 * a00 - a01 * a10) * det;
    return true;
}

inline Mat3f &adjoint(Mat3f &out, const Mat3f &a) {
    const float a00 = a.m[0], a01 = a.m[1], a02 = a.m[2];
    const float a10 = a.m[3], a11 = a.m[4], a12 = a.m[5];
    const float a20 = a.m[6], a21 = a.m[7], a22 = a.m[8];
    return set(out, a11 * a22 - a12 * a21, a02 * a21 - a01 * a22, a01 * a12 - a02 * a11, a12 * a20 - a10 * a22,
               a00 * a22 - a02 * a20, a02 * a10 - a00 * a12, a10 * a21 - a11 * a20, a01 * a20 - a00 * a21,
               a00 * a11 - a01 * a10);
}

inline float determinant(const Mat3f &a) {
    const float a00 = a.m[0], a01 = a.m[1], a02 = a.m[2];
    const float a10 = a.m[3], a11 = a.m[4], a12 = a.m[5];
    const float a20 = a.m[6], a21 = a.m[7], a22 = a.m[8];
    return a00 * (a22 * a11 - a12 * a21) + a01 * (-a22 * a10 + a12 * a20) + a02 * (a21 * a10 - a11 * a20);
}

/** a * b. */
inline Mat3f &multiply(Mat3f &out, const Mat3f &a, const Mat3f &b) {
    const float a00 = a.m[0], a01 = a.m[1], a02 = a.m[2];
    const float a10 = a.m[3], a11 = a.m[4], a12 = a.m[5];
    const float a20 = a.m[6], a21 = a.m[7], a22 = a.m[8];
    const float b00 = b.m[0], b01 = b.m[1], b02 = b.m[2];
    const float b10 = b.m[3], b11 = b.m[4], b12 = b.m[5];
    const float b20 = b.m[6], b21 = b.m[7], b22 = b.m[8];
    return set(out, b00 * a00 + b01 * a10 + b02 * a20, b00 * a01 + b01 * a11 + b02 * a21, b00 * a02 + b01 * a12 + b02 * a22,
               b10 * a00 + b11 * a10 + b12 * a20, b10 * a01 + b11 * a11 + b12 * a21, b10 * a02 + b11 * a12 + b12 * a22,
               b20 * a00 + b21 * a10 + b22 * a20, b20 * a01 + b21 * a11 + b22 * a21, b20 * a02 + b21 * a12 + b22 * a22);
}

/** a 2d translation applied after a. */
inline Mat3f &translate(Mat3f &out, const Mat3f &a, const Vec2f &v) {
    const float a00 = a.m[0], a01 = a.m[1], a02 = a.m[2];
    const float a10 = a.m[3], a11 = a.m[4], a12 = a.m[5];
    const float a20 = a.m[6], a21 = a.m[7], a22 = a.m[8];
    return set(out, a00, a01, a02, a10, a11, a12, v.x * a00 + v.y * a10 + a20, v.x * a01 + v.y * a11 + a21,
               v.x * a02 + v.y * a12 + a22);
}

inline Mat3f &rotate(Mat3f &out, const Mat3f &a, float rad) {
    const float a00 = a.m[0], a01 = a.m[1], a02 = a.m[2];
    const float a10 = a.m[3], a11 = a.m[4], a12 = a.m[5];
    const float s = scalar::sin(rad);
    const float c = scalar::cos(rad);
    return set(out, c * a00 + s * a10, c * a01 + s * a11, c * a02 + s * a12, c * a10 - s * a00, c * a11 - s * a01,
               c * a12 - s * a02, a.m[6], a.m[7], a.m[8]);
}

inline Mat3f &scale(Mat3f &out, const Mat3f &a, const Vec2f &v) {
    return set(out, v.x * a.m[0], v.x * a.m[1], v.x * a.m[2], v.y * a.m[3], v.y * a.m[4], v.y * a.m[5], a.m[6], a.m[7],
               a.m[8]);
}

inline Mat3f &from_translation(Mat3f &out, const Vec2f &v) { return set(out, 1, 0, 0, 0, 1, 0, v.x, v.y, 1); }

inline Mat3f &from_rotation(Mat3f &out, float rad) {
    const float s = scalar::sin(rad);
    const float c = scalar::cos(rad);
    return set(out, c, s, 0, -s, c, 0, 0, 0, 1);
}

inline Mat3f &from_scaling(Mat3f &out, const Vec2f &v) { return set(out, v.x, 0, 0, 0, v.y, 0, 0, 0, 1); }

inline Mat3f &from_quat(Mat3f &out, const Float4 &q) {
    const float x2 = q.x + q.x;
    const float y2 = q.y + q.y;
    const float z2 = q.z + q.z;
    const float xx = q.x * x2;
    const float yx = q.y * x2;
    const float yy = q.y * y2;
    const float zx = q.z * x2;
    const float zy = q.z * y2;
    const float zz = q.z * z2;
    const float wx = q.w * x2;
    const float wy = q.w * y2;
    const float wz = q.w * z2;
    out.m[0] = 1.0f - yy - zz;
    out.m[3] = yx - wz;
    out.m[6] = zx + wy;
    out.m[1] = yx + wz;
    out.m[4] = 1.0f - xx - zz;
    out.m[7] = zy - wx;
    out.m[2] = zx - wy;
    out.m[5] = zy + wx;
    out.m[8] = 1.0f - xx - yy;
    return out;
}

/** the inverse transpose of a mat4's upper 3x3, for normals; false, out untouched, when singular. */
inline bool normal_from_mat4(Mat3f &out, const Float4x4 &a) {
    const float a00 = a.m[0], a01 = a.m[1], a02 = a.m[2], a03 = a.m[3];
    const float a10 = a.m[4], a11 = a.m[5], a12 = a.m[6], a13 = a.m[7];
    const float a20 = a.m[8], a21 = a.m[9], a22 = a.m[10], a23 = a.m[11];
    const float a30 = a.m[12], a31 = a.m[13], a32 = a.m[14], a33 = a.m[15];
    const float b00 = a00 * a11 - a01 * a10;
    const float b01 = a00 * a12 - a02 * a10;
    const float b02 = a00 * a13 - a03 * a10;
    const float b03 = a01 * a12 - a02 * a11;
    const float b04 = a01 * a13 - a03 * a11;
    const float b05 = a02 * a13 - a03 * a12;
    const float b06 = a20 * a31 - a21 * a30;
    const float b07 = a20 * a32 - a22 * a30;
    const float b08 = a20 * a33 - a23 * a30;
    const float b09 = a21 * a32 - a22 * a31;
    const float b10 = a21 * a33 - a23 * a31;
    const float b11 = a22 * a33 - a23 * a32;
    float det = b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;
    if (det == 0.0f) return false;
    det = 1.0f / det;
    out.m[0] = (a11 * b11 - a12 * b10 + a13 * b09) * det;
    out.m[1] = (a12 * b08 - a10 * b11 - a13 * b07) * det;
    out.m[2] = (a10 * b10 - a11 * b08 + a13 * b06) * det;
    out.m[3] = (a02 * b10 - a01 * b11 - a03 * b09) * det;
    out.m[4] = (a00 * b11 - a02 * b08 + a03 * b07) * det;
    out.m[5] = (a01 * b08 - a00 * b10 - a03 * b06) * det;
    out.m[6] = (a31 * b05 - a32 * b04 + a33 * b03) * det;
    out.m[7] = (a32 * b02 - a30 * b05 - a33 * b01) * det;
    out.m[8] = (a30 * b04 - a31 * b02 + a33 * b00) * det;
    return true;
}

/** a 2d projection mapping pixels to clip space, y down. */
inline Mat3f &projection(Mat3f &out, float width, float height) {
    return set(out, 2.0f / width, 0, 0, 0, -2.0f / height, 0, -1, 1, 1);
}

inline float frob(const Mat3f &a) {
    float sum = 0.0f;
    for (int i = 0; i < 9; i++) sum += a.m[i] * a.m[i];
    return scalar::sqrt(sum);
}

inline Mat3f &add(Mat3f &out, const Mat3f &a, const Mat3f &b) {
    for (int i = 0; i < 9; i++) out.m[i] = a.m[i] + b.m[i];
    return out;
}

inline Mat3f &subtract(Mat3f &out, const Mat3f &a, const Mat3f &b) {
    for (int i = 0; i < 9; i++) out.m[i] = a.m[i] - b.m[i];
    return out;
}

inline Mat3f &multiply_scalar(Mat3f &out, const Mat3f &a, float b) {
    for (int i = 0; i < 9; i++) out.m[i] = a.m[i] * b;
    return out;
}

inline Mat3f &multiply_scalar_and_add(Mat3f &out, const Mat3f &a, const Mat3f &b, float s) {
    for (int i = 0; i < 9; i++) out.m[i] = a.m[i] + b.m[i] * s;
    return out;
}

inline bool exact_equals(const Mat3f &a, const Mat3f &b) {
    for (int i = 0; i < 9; i++) {
        if (a.m[i] != b.m[i]) return false;
    }
    return true;
}

inline bool equals(const Mat3f &a, const Mat3f &b) {
    for (int i = 0; i < 9; i++) {
        if (!scalar::equals(a.m[i], b.m[i])) return false;
    }
    return true;
}

inline Mat3f &mul(Mat3f &out, const Mat3f &a, const Mat3f &b) { return multiply(out, a, b); }

}  // namespace mat3f
