#pragma once

// the scalar functions over the storage types, retired from src when the register types took over: the reference the
// register functions are checked against, bit for bit.

#include "math/mat4f.h"
#include "math/storage/vec3f.h"
#include "math/storage/vec4f.h"
#include "math/storage/quatf.h"

namespace mat4f {

inline Float4x4 create() { return {{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}}; }
inline Float4x4 clone(const Float4x4 &a) { return a; }

inline Float4x4 from_values(float m00, float m01, float m02, float m03, float m10, float m11, float m12, float m13, float m20,
                         float m21, float m22, float m23, float m30, float m31, float m32, float m33) {
    return {{m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33}};
}

inline Float4x4 &copy(Float4x4 &out, const Float4x4 &a) {
    out = a;
    return out;
}

inline Float4x4 &set(Float4x4 &out, float m00, float m01, float m02, float m03, float m10, float m11, float m12, float m13,
                  float m20, float m21, float m22, float m23, float m30, float m31, float m32, float m33) {
    out.m[0] = m00;
    out.m[1] = m01;
    out.m[2] = m02;
    out.m[3] = m03;
    out.m[4] = m10;
    out.m[5] = m11;
    out.m[6] = m12;
    out.m[7] = m13;
    out.m[8] = m20;
    out.m[9] = m21;
    out.m[10] = m22;
    out.m[11] = m23;
    out.m[12] = m30;
    out.m[13] = m31;
    out.m[14] = m32;
    out.m[15] = m33;
    return out;
}

inline Float4x4 &identity(Float4x4 &out) { return set(out, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1); }
inline Float4x4 &zero(Float4x4 &out) { return set(out, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0); }

/** column i as a lane. */
inline v128_t col(const Float4x4 &a, int i) { return v4f::load4(&a.m[i * 4]); }
inline void set_col(Float4x4 &out, int i, v128_t v) { v4f::store4(&out.m[i * 4], v); }

inline Float4x4 &transpose(Float4x4 &out, const Float4x4 &a) {
    if (&out == &a) {
        const float a01 = a.m[1], a02 = a.m[2], a03 = a.m[3];
        const float a12 = a.m[6], a13 = a.m[7], a23 = a.m[11];
        out.m[1] = a.m[4];
        out.m[2] = a.m[8];
        out.m[3] = a.m[12];
        out.m[4] = a01;
        out.m[6] = a.m[9];
        out.m[7] = a.m[13];
        out.m[8] = a02;
        out.m[9] = a12;
        out.m[11] = a.m[14];
        out.m[12] = a03;
        out.m[13] = a13;
        out.m[14] = a23;
        return out;
    }
    return set(out, a.m[0], a.m[4], a.m[8], a.m[12], a.m[1], a.m[5], a.m[9], a.m[13], a.m[2], a.m[6], a.m[10], a.m[14],
               a.m[3], a.m[7], a.m[11], a.m[15]);
}

/** false, out untouched, for a singular matrix. */
inline bool invert(Float4x4 &out, const Float4x4 &a) {
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
    out.m[1] = (a02 * b10 - a01 * b11 - a03 * b09) * det;
    out.m[2] = (a31 * b05 - a32 * b04 + a33 * b03) * det;
    out.m[3] = (a22 * b04 - a21 * b05 - a23 * b03) * det;
    out.m[4] = (a12 * b08 - a10 * b11 - a13 * b07) * det;
    out.m[5] = (a00 * b11 - a02 * b08 + a03 * b07) * det;
    out.m[6] = (a32 * b02 - a30 * b05 - a33 * b01) * det;
    out.m[7] = (a20 * b05 - a22 * b02 + a23 * b01) * det;
    out.m[8] = (a10 * b10 - a11 * b08 + a13 * b06) * det;
    out.m[9] = (a01 * b08 - a00 * b10 - a03 * b06) * det;
    out.m[10] = (a30 * b04 - a31 * b02 + a33 * b00) * det;
    out.m[11] = (a21 * b02 - a20 * b04 - a23 * b00) * det;
    out.m[12] = (a11 * b07 - a10 * b09 - a12 * b06) * det;
    out.m[13] = (a00 * b09 - a01 * b07 + a02 * b06) * det;
    out.m[14] = (a31 * b01 - a30 * b03 - a32 * b00) * det;
    out.m[15] = (a20 * b03 - a21 * b01 + a22 * b00) * det;
    return true;
}

/** inverts the upper 3x3 and clears the rest to identity; false, out untouched, when singular. */
inline bool invert3x3(Float4x4 &out, const Float4x4 &a) {
    const float a00 = a.m[0], a01 = a.m[1], a02 = a.m[2];
    const float a10 = a.m[4], a11 = a.m[5], a12 = a.m[6];
    const float a20 = a.m[8], a21 = a.m[9], a22 = a.m[10];
    const float b01 = a22 * a11 - a12 * a21;
    const float b11 = -a22 * a10 + a12 * a20;
    const float b21 = a21 * a10 - a11 * a20;
    float det = a00 * b01 + a01 * b11 + a02 * b21;
    if (det == 0.0f) return false;
    det = 1.0f / det;
    set(out, b01 * det, (-a22 * a01 + a02 * a21) * det, (a12 * a01 - a02 * a11) * det, 0, b11 * det,
        (a22 * a00 - a02 * a20) * det, (-a12 * a00 + a02 * a10) * det, 0, b21 * det, (-a21 * a00 + a01 * a20) * det,
        (a11 * a00 - a01 * a10) * det, 0, 0, 0, 0, 1);
    return true;
}

inline Float4x4 &adjoint(Float4x4 &out, const Float4x4 &a) {
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
    return set(out, a11 * b11 - a12 * b10 + a13 * b09, a02 * b10 - a01 * b11 - a03 * b09, a31 * b05 - a32 * b04 + a33 * b03,
               a22 * b04 - a21 * b05 - a23 * b03, a12 * b08 - a10 * b11 - a13 * b07, a00 * b11 - a02 * b08 + a03 * b07,
               a32 * b02 - a30 * b05 - a33 * b01, a20 * b05 - a22 * b02 + a23 * b01, a10 * b10 - a11 * b08 + a13 * b06,
               a01 * b08 - a00 * b10 - a03 * b06, a30 * b04 - a31 * b02 + a33 * b00, a21 * b02 - a20 * b04 - a23 * b00,
               a11 * b07 - a10 * b09 - a12 * b06, a00 * b09 - a01 * b07 + a02 * b06, a31 * b01 - a30 * b03 - a32 * b00,
               a20 * b03 - a21 * b01 + a22 * b00);
}

inline float determinant(const Float4x4 &a) {
    const float a00 = a.m[0], a01 = a.m[1], a02 = a.m[2], a03 = a.m[3];
    const float a10 = a.m[4], a11 = a.m[5], a12 = a.m[6], a13 = a.m[7];
    const float a20 = a.m[8], a21 = a.m[9], a22 = a.m[10], a23 = a.m[11];
    const float a30 = a.m[12], a31 = a.m[13], a32 = a.m[14], a33 = a.m[15];
    const float b0 = a00 * a11 - a01 * a10;
    const float b1 = a00 * a12 - a02 * a10;
    const float b2 = a01 * a12 - a02 * a11;
    const float b3 = a20 * a31 - a21 * a30;
    const float b4 = a20 * a32 - a22 * a30;
    const float b5 = a21 * a32 - a22 * a31;
    const float b6 = a00 * b5 - a01 * b4 + a02 * b3;
    const float b7 = a10 * b5 - a11 * b4 + a12 * b3;
    const float b8 = a20 * b2 - a21 * b1 + a22 * b0;
    const float b9 = a30 * b2 - a31 * b1 + a32 * b0;
    return a13 * b6 - a03 * b7 + a33 * b8 - a23 * b9;
}

/** a * b; out may alias either. */
inline Float4x4 &multiply(Float4x4 &out, const Float4x4 &a, const Float4x4 &b) {
    const v128_t a0 = col(a, 0), a1 = col(a, 1), a2 = col(a, 2), a3 = col(a, 3);
    for (int j = 0; j < 4; j++) {
        const float *bj = &b.m[j * 4];
        v128_t r = v4f::mul(a0, v4f::splat(bj[0]));
        r = v4f::mul_add(a1, v4f::splat(bj[1]), r);
        r = v4f::mul_add(a2, v4f::splat(bj[2]), r);
        r = v4f::mul_add(a3, v4f::splat(bj[3]), r);
        set_col(out, j, r);
    }
    return out;
}

/** the upper 3x3 of a times the upper 3x3 of b, the rest identity. */
inline Float4x4 &multiply3x3(Float4x4 &out, const Float4x4 &a, const Float4x4 &b) {
    const v128_t a0 = col(a, 0), a1 = col(a, 1), a2 = col(a, 2);
    for (int j = 0; j < 3; j++) {
        const float *bj = &b.m[j * 4];
        v128_t r = v4f::mul(a0, v4f::splat(bj[0]));
        r = v4f::mul_add(a1, v4f::splat(bj[1]), r);
        r = v4f::mul_add(a2, v4f::splat(bj[2]), r);
        set_col(out, j, v4f::set_lane<3>(r, 0.0f));
    }
    set_col(out, 3, v4f::make(0, 0, 0, 1));
    return out;
}

/** the upper 3x3 of a times the transpose of the upper 3x3 of b, the rest identity. */
inline Float4x4 &multiply3x3_right_transposed(Float4x4 &out, const Float4x4 &a, const Float4x4 &b) {
    const v128_t a0 = col(a, 0), a1 = col(a, 1), a2 = col(a, 2);
    for (int j = 0; j < 3; j++) {
        v128_t r = v4f::mul(a0, v4f::splat(b.m[j]));
        r = v4f::mul_add(a1, v4f::splat(b.m[4 + j]), r);
        r = v4f::mul_add(a2, v4f::splat(b.m[8 + j]), r);
        set_col(out, j, v4f::set_lane<3>(r, 0.0f));
    }
    set_col(out, 3, v4f::make(0, 0, 0, 1));
    return out;
}

/** the transpose of the upper 3x3 of mat times vec. */
inline Float3 &multiply3x3_transposed_vec(Float3 &out, const Float4x4 &mat, const Float3 &vec) {
    out.x = mat.m[0] * vec.x + mat.m[1] * vec.y + mat.m[2] * vec.z;
    out.y = mat.m[4] * vec.x + mat.m[5] * vec.y + mat.m[6] * vec.z;
    out.z = mat.m[8] * vec.x + mat.m[9] * vec.y + mat.m[10] * vec.z;
    return out;
}

/** the upper 3x3 of mat times vec. */
inline Float3 &multiply3x3_vec(Float3 &out, const Float4x4 &mat, const Float3 &vec) {
    v128_t r = v4f::mul(col(mat, 0), v4f::splat(vec.x));
    r = v4f::mul_add(col(mat, 1), v4f::splat(vec.y), r);
    r = v4f::mul_add(col(mat, 2), v4f::splat(vec.z), r);
    v4f::store3(&out.x, r);
    return out;
}

/** the matrix whose product with u is v x u. */
inline Float4x4 &cross_product_matrix(Float4x4 &out, const Float3 &v) {
    return set(out, 0, v.z, -v.y, 0, -v.z, 0, v.x, 0, v.y, -v.x, 0, 0, 0, 0, 0, 1);
}

/** a translated by v (a * T(v)). */
inline Float4x4 &translate(Float4x4 &out, const Float4x4 &a, const Float3 &v) {
    const v128_t a0 = col(a, 0), a1 = col(a, 1), a2 = col(a, 2), a3 = col(a, 3);
    if (&a != &out) {
        set_col(out, 0, a0);
        set_col(out, 1, a1);
        set_col(out, 2, a2);
    }
    v128_t r = v4f::mul_add(a0, v4f::splat(v.x), a3);
    r = v4f::mul_add(a1, v4f::splat(v.y), r);
    r = v4f::mul_add(a2, v4f::splat(v.z), r);
    set_col(out, 3, r);
    return out;
}

/** a scaled by v (a * S(v)). */
inline Float4x4 &scale(Float4x4 &out, const Float4x4 &a, const Float3 &v) {
    set_col(out, 0, v4f::mul(col(a, 0), v4f::splat(v.x)));
    set_col(out, 1, v4f::mul(col(a, 1), v4f::splat(v.y)));
    set_col(out, 2, v4f::mul(col(a, 2), v4f::splat(v.z)));
    if (&a != &out) set_col(out, 3, col(a, 3));
    return out;
}

/** a rotated by rad about axis; false, out untouched, for a zero axis. */
inline bool rotate(Float4x4 &out, const Float4x4 &a, float rad, const Float3 &axis) {
    float x = axis.x, y = axis.y, z = axis.z;
    float len = scalar::sqrt(x * x + y * y + z * z);
    if (len < scalar::EPSILON) return false;
    len = 1.0f / len;
    x *= len;
    y *= len;
    z *= len;
    const float s = scalar::sin(rad);
    const float c = scalar::cos(rad);
    const float t = 1.0f - c;
    const v128_t a0 = col(a, 0), a1 = col(a, 1), a2 = col(a, 2);
    const float b00 = x * x * t + c, b01 = y * x * t + z * s, b02 = z * x * t - y * s;
    const float b10 = x * y * t - z * s, b11 = y * y * t + c, b12 = z * y * t + x * s;
    const float b20 = x * z * t + y * s, b21 = y * z * t - x * s, b22 = z * z * t + c;
    set_col(out, 0, v4f::mul_add(a2, v4f::splat(b02), v4f::mul_add(a1, v4f::splat(b01), v4f::mul(a0, v4f::splat(b00)))));
    set_col(out, 1, v4f::mul_add(a2, v4f::splat(b12), v4f::mul_add(a1, v4f::splat(b11), v4f::mul(a0, v4f::splat(b10)))));
    set_col(out, 2, v4f::mul_add(a2, v4f::splat(b22), v4f::mul_add(a1, v4f::splat(b21), v4f::mul(a0, v4f::splat(b20)))));
    if (&a != &out) set_col(out, 3, col(a, 3));
    return true;
}

inline Float4x4 &rotate_x(Float4x4 &out, const Float4x4 &a, float rad) {
    const float s = scalar::sin(rad);
    const float c = scalar::cos(rad);
    const v128_t a1 = col(a, 1), a2 = col(a, 2);
    if (&a != &out) {
        set_col(out, 0, col(a, 0));
        set_col(out, 3, col(a, 3));
    }
    set_col(out, 1, v4f::mul_add(a2, v4f::splat(s), v4f::mul(a1, v4f::splat(c))));
    set_col(out, 2, v4f::sub(v4f::mul(a2, v4f::splat(c)), v4f::mul(a1, v4f::splat(s))));
    return out;
}

inline Float4x4 &rotate_y(Float4x4 &out, const Float4x4 &a, float rad) {
    const float s = scalar::sin(rad);
    const float c = scalar::cos(rad);
    const v128_t a0 = col(a, 0), a2 = col(a, 2);
    if (&a != &out) {
        set_col(out, 1, col(a, 1));
        set_col(out, 3, col(a, 3));
    }
    set_col(out, 0, v4f::sub(v4f::mul(a0, v4f::splat(c)), v4f::mul(a2, v4f::splat(s))));
    set_col(out, 2, v4f::mul_add(a0, v4f::splat(s), v4f::mul(a2, v4f::splat(c))));
    return out;
}

inline Float4x4 &rotate_z(Float4x4 &out, const Float4x4 &a, float rad) {
    const float s = scalar::sin(rad);
    const float c = scalar::cos(rad);
    const v128_t a0 = col(a, 0), a1 = col(a, 1);
    if (&a != &out) {
        set_col(out, 2, col(a, 2));
        set_col(out, 3, col(a, 3));
    }
    set_col(out, 0, v4f::mul_add(a1, v4f::splat(s), v4f::mul(a0, v4f::splat(c))));
    set_col(out, 1, v4f::sub(v4f::mul(a1, v4f::splat(c)), v4f::mul(a0, v4f::splat(s))));
    return out;
}

inline Float4x4 &from_translation(Float4x4 &out, const Float3 &v) {
    return set(out, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, v.x, v.y, v.z, 1);
}

inline Float4x4 &from_scaling(Float4x4 &out, const Float3 &v) {
    return set(out, v.x, 0, 0, 0, 0, v.y, 0, 0, 0, 0, v.z, 0, 0, 0, 0, 1);
}

/** false, out untouched, for a zero axis. */
inline bool from_rotation(Float4x4 &out, float rad, const Float3 &axis) {
    float x = axis.x, y = axis.y, z = axis.z;
    float len = scalar::sqrt(x * x + y * y + z * z);
    if (len < scalar::EPSILON) return false;
    len = 1.0f / len;
    x *= len;
    y *= len;
    z *= len;
    const float s = scalar::sin(rad);
    const float c = scalar::cos(rad);
    const float t = 1.0f - c;
    set(out, x * x * t + c, y * x * t + z * s, z * x * t - y * s, 0, x * y * t - z * s, y * y * t + c, z * y * t + x * s, 0,
        x * z * t + y * s, y * z * t - x * s, z * z * t + c, 0, 0, 0, 0, 1);
    return true;
}

inline Float4x4 &from_x_rotation(Float4x4 &out, float rad) {
    const float s = scalar::sin(rad);
    const float c = scalar::cos(rad);
    return set(out, 1, 0, 0, 0, 0, c, s, 0, 0, -s, c, 0, 0, 0, 0, 1);
}

inline Float4x4 &from_y_rotation(Float4x4 &out, float rad) {
    const float s = scalar::sin(rad);
    const float c = scalar::cos(rad);
    return set(out, c, 0, -s, 0, 0, 1, 0, 0, s, 0, c, 0, 0, 0, 0, 1);
}

inline Float4x4 &from_z_rotation(Float4x4 &out, float rad) {
    const float s = scalar::sin(rad);
    const float c = scalar::cos(rad);
    return set(out, c, s, 0, 0, -s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
}

inline Float4x4 &from_rotation_translation(Float4x4 &out, const Float4 &q, const Float3 &v) {
    const float x2 = q.x + q.x, y2 = q.y + q.y, z2 = q.z + q.z;
    const float xx = q.x * x2, xy = q.x * y2, xz = q.x * z2;
    const float yy = q.y * y2, yz = q.y * z2, zz = q.z * z2;
    const float wx = q.w * x2, wy = q.w * y2, wz = q.w * z2;
    return set(out, 1.0f - (yy + zz), xy + wz, xz - wy, 0, xy - wz, 1.0f - (xx + zz), yz + wx, 0, xz + wy, yz - wx,
               1.0f - (xx + yy), 0, v.x, v.y, v.z, 1);
}

inline Float3 &get_translation(Float3 &out, const Float4x4 &mat) {
    out.x = mat.m[12];
    out.y = mat.m[13];
    out.z = mat.m[14];
    return out;
}

/** the length of each of the three axis columns. */
inline Float3 &get_scaling(Float3 &out, const Float4x4 &mat) {
    out.x = scalar::sqrt(mat.m[0] * mat.m[0] + mat.m[1] * mat.m[1] + mat.m[2] * mat.m[2]);
    out.y = scalar::sqrt(mat.m[4] * mat.m[4] + mat.m[5] * mat.m[5] + mat.m[6] * mat.m[6]);
    out.z = scalar::sqrt(mat.m[8] * mat.m[8] + mat.m[9] * mat.m[9] + mat.m[10] * mat.m[10]);
    return out;
}

/** the rotation of a matrix with the given scaling divided out. */
inline Float4 &rotation_with_scaling(Float4 &out, const Float4x4 &mat, const Float3 &scaling) {
    const float is1 = 1.0f / scaling.x;
    const float is2 = 1.0f / scaling.y;
    const float is3 = 1.0f / scaling.z;
    const float sm11 = mat.m[0] * is1, sm12 = mat.m[1] * is2, sm13 = mat.m[2] * is3;
    const float sm21 = mat.m[4] * is1, sm22 = mat.m[5] * is2, sm23 = mat.m[6] * is3;
    const float sm31 = mat.m[8] * is1, sm32 = mat.m[9] * is2, sm33 = mat.m[10] * is3;
    const float trace = sm11 + sm22 + sm33;
    float S;
    if (trace > 0.0f) {
        S = scalar::sqrt(trace + 1.0f) * 2.0f;
        out.w = 0.25f * S;
        out.x = (sm23 - sm32) / S;
        out.y = (sm31 - sm13) / S;
        out.z = (sm12 - sm21) / S;
    } else if (sm11 > sm22 && sm11 > sm33) {
        S = scalar::sqrt(1.0f + sm11 - sm22 - sm33) * 2.0f;
        out.w = (sm23 - sm32) / S;
        out.x = 0.25f * S;
        out.y = (sm12 + sm21) / S;
        out.z = (sm31 + sm13) / S;
    } else if (sm22 > sm33) {
        S = scalar::sqrt(1.0f + sm22 - sm11 - sm33) * 2.0f;
        out.w = (sm31 - sm13) / S;
        out.x = (sm12 + sm21) / S;
        out.y = 0.25f * S;
        out.z = (sm23 + sm32) / S;
    } else {
        S = scalar::sqrt(1.0f + sm33 - sm11 - sm22) * 2.0f;
        out.w = (sm12 - sm21) / S;
        out.x = (sm31 + sm13) / S;
        out.y = (sm23 + sm32) / S;
        out.z = 0.25f * S;
    }
    return out;
}

inline Float4 &get_rotation(Float4 &out, const Float4x4 &mat) {
    Float3 scaling;
    get_scaling(scaling, mat);
    return rotation_with_scaling(out, mat, scaling);
}

/** rotation, translation and scale of a trs matrix. */
inline Float4 &decompose(Float4 &out_r, Float3 &out_t, Float3 &out_s, const Float4x4 &mat) {
    get_translation(out_t, mat);
    get_scaling(out_s, mat);
    return rotation_with_scaling(out_r, mat, out_s);
}

inline Float4x4 &from_rotation_translation_scale(Float4x4 &out, const Float4 &q, const Float3 &v, const Float3 &s) {
    const float x2 = q.x + q.x, y2 = q.y + q.y, z2 = q.z + q.z;
    const float xx = q.x * x2, xy = q.x * y2, xz = q.x * z2;
    const float yy = q.y * y2, yz = q.y * z2, zz = q.z * z2;
    const float wx = q.w * x2, wy = q.w * y2, wz = q.w * z2;
    set_col(out, 0, v4f::mul(v4f::make(1.0f - (yy + zz), xy + wz, xz - wy, 0.0f), v4f::splat(s.x)));
    set_col(out, 1, v4f::mul(v4f::make(xy - wz, 1.0f - (xx + zz), yz + wx, 0.0f), v4f::splat(s.y)));
    set_col(out, 2, v4f::mul(v4f::make(xz + wy, yz - wx, 1.0f - (xx + yy), 0.0f), v4f::splat(s.z)));
    set_col(out, 3, v4f::make(v.x, v.y, v.z, 1.0f));
    return out;
}

/** as from_rotation_translation_scale, rotating and scaling about origin o. */
inline Float4x4 &from_rotation_translation_scale_origin(Float4x4 &out, const Float4 &q, const Float3 &v, const Float3 &s,
                                                     const Float3 &o) {
    from_rotation_translation_scale(out, q, v, s);
    const float *m = out.m;
    out.m[12] = v.x + o.x - (m[0] * o.x + m[4] * o.y + m[8] * o.z);
    out.m[13] = v.y + o.y - (m[1] * o.x + m[5] * o.y + m[9] * o.z);
    out.m[14] = v.z + o.z - (m[2] * o.x + m[6] * o.y + m[10] * o.z);
    return out;
}

inline Float4x4 &from_quat(Float4x4 &out, const Float4 &q) {
    const float x2 = q.x + q.x, y2 = q.y + q.y, z2 = q.z + q.z;
    const float xx = q.x * x2, yx = q.y * x2, yy = q.y * y2;
    const float zx = q.z * x2, zy = q.z * y2, zz = q.z * z2;
    const float wx = q.w * x2, wy = q.w * y2, wz = q.w * z2;
    return set(out, 1.0f - yy - zz, yx + wz, zx - wy, 0, yx - wz, 1.0f - xx - zz, zy + wx, 0, zx + wy, zy - wx,
               1.0f - xx - yy, 0, 0, 0, 0, 1);
}

/** a view matrix looking from eye at center; identity when they coincide. */
inline Float4x4 &look_at(Float4x4 &out, const Float3 &eye, const Float3 &center, const Float3 &up) {
    if (scalar::abs(eye.x - center.x) < scalar::EPSILON && scalar::abs(eye.y - center.y) < scalar::EPSILON &&
        scalar::abs(eye.z - center.z) < scalar::EPSILON) {
        return identity(out);
    }
    float z0 = eye.x - center.x;
    float z1 = eye.y - center.y;
    float z2 = eye.z - center.z;
    float len = 1.0f / scalar::sqrt(z0 * z0 + z1 * z1 + z2 * z2);
    z0 *= len;
    z1 *= len;
    z2 *= len;
    float x0 = up.y * z2 - up.z * z1;
    float x1 = up.z * z0 - up.x * z2;
    float x2 = up.x * z1 - up.y * z0;
    len = scalar::sqrt(x0 * x0 + x1 * x1 + x2 * x2);
    if (len == 0.0f) {
        x0 = 0.0f;
        x1 = 0.0f;
        x2 = 0.0f;
    } else {
        len = 1.0f / len;
        x0 *= len;
        x1 *= len;
        x2 *= len;
    }
    float y0 = z1 * x2 - z2 * x1;
    float y1 = z2 * x0 - z0 * x2;
    float y2 = z0 * x1 - z1 * x0;
    len = scalar::sqrt(y0 * y0 + y1 * y1 + y2 * y2);
    if (len == 0.0f) {
        y0 = 0.0f;
        y1 = 0.0f;
        y2 = 0.0f;
    } else {
        len = 1.0f / len;
        y0 *= len;
        y1 *= len;
        y2 *= len;
    }
    return set(out, x0, y0, z0, 0, x1, y1, z1, 0, x2, y2, z2, 0, -(x0 * eye.x + x1 * eye.y + x2 * eye.z),
               -(y0 * eye.x + y1 * eye.y + y2 * eye.z), -(z0 * eye.x + z1 * eye.y + z2 * eye.z), 1);
}

/** the model matrix of something at eye facing target, the inverse of look_at's view. */
inline Float4x4 &target_to(Float4x4 &out, const Float3 &eye, const Float3 &target, const Float3 &up) {
    float z0 = eye.x - target.x;
    float z1 = eye.y - target.y;
    float z2 = eye.z - target.z;
    float len = z0 * z0 + z1 * z1 + z2 * z2;
    if (len > 0.0f) {
        len = 1.0f / scalar::sqrt(len);
        z0 *= len;
        z1 *= len;
        z2 *= len;
    }
    float x0 = up.y * z2 - up.z * z1;
    float x1 = up.z * z0 - up.x * z2;
    float x2 = up.x * z1 - up.y * z0;
    len = x0 * x0 + x1 * x1 + x2 * x2;
    if (len > 0.0f) {
        len = 1.0f / scalar::sqrt(len);
        x0 *= len;
        x1 *= len;
        x2 *= len;
    }
    return set(out, x0, x1, x2, 0, z1 * x2 - z2 * x1, z2 * x0 - z0 * x2, z0 * x1 - z1 * x0, 0, z0, z1, z2, 0, eye.x, eye.y,
               eye.z, 1);
}

inline float frob(const Float4x4 &a) {
    v128_t sum = v4f::mul(col(a, 0), col(a, 0));
    for (int i = 1; i < 4; i++) sum = v4f::mul_add(col(a, i), col(a, i), sum);
    return scalar::sqrt(v4f::hsum4(sum));
}

inline Float4x4 &add(Float4x4 &out, const Float4x4 &a, const Float4x4 &b) {
    for (int i = 0; i < 4; i++) set_col(out, i, v4f::add(col(a, i), col(b, i)));
    return out;
}

inline Float4x4 &subtract(Float4x4 &out, const Float4x4 &a, const Float4x4 &b) {
    for (int i = 0; i < 4; i++) set_col(out, i, v4f::sub(col(a, i), col(b, i)));
    return out;
}

inline Float4x4 &multiply_scalar(Float4x4 &out, const Float4x4 &a, float b) {
    const v128_t s = v4f::splat(b);
    for (int i = 0; i < 4; i++) set_col(out, i, v4f::mul(col(a, i), s));
    return out;
}

inline Float4x4 &multiply_scalar_and_add(Float4x4 &out, const Float4x4 &a, const Float4x4 &b, float s) {
    const v128_t vs = v4f::splat(s);
    for (int i = 0; i < 4; i++) set_col(out, i, v4f::mul_add(col(b, i), vs, col(a, i)));
    return out;
}

inline bool exact_equals(const Float4x4 &a, const Float4x4 &b) {
    for (int i = 0; i < 16; i++) {
        if (a.m[i] != b.m[i]) return false;
    }
    return true;
}

inline bool equals(const Float4x4 &a, const Float4x4 &b) {
    for (int i = 0; i < 16; i++) {
        if (!scalar::equals(a.m[i], b.m[i])) return false;
    }
    return true;
}

inline Float4x4 &mul(Float4x4 &out, const Float4x4 &a, const Float4x4 &b) { return multiply(out, a, b); }
inline Float4x4 &sub(Float4x4 &out, const Float4x4 &a, const Float4x4 &b) { return subtract(out, a, b); }

}  // namespace mat4f
