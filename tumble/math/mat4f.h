#pragma once

// the math package's mat4.ts on Mat4f, four column lanes, column-major: everything but the projection builders
// (frustum / perspective / ortho), which the renderer makes on its side, and fromQuat2. a matrix's storage is a
// Float4x4. each function gives the bits of its scalar reference (test/math/storage/mat4f.h): multiply is four
// column passes; the scalar-shaped ones (invert, adjoint, the decompositions, the view builders) read their entries
// out and keep the scalar expressions. `invert`, `invert3x3`, `rotate` and `from_rotation` return false where the js
// returns null, leaving out untouched.

#include "tumble/math/scalar.h"
#include "tumble/math/simd.h"
#include "tumble/math/types.h"
#include "tumble/math/vec3f.h"
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace mat4f {

inline Mat4f load(const Float4x4 &a) {
    return {{v4f::load4(&a.m[0]), v4f::load4(&a.m[4]), v4f::load4(&a.m[8]), v4f::load4(&a.m[12])}};
}
inline Float4x4 &store(Float4x4 &out, const Mat4f &a) {
    for (int i = 0; i < 4; i++) v4f::store4(&out.m[i * 4], a.c[i]);
    return out;
}

inline Mat4f make_columns(Vec4f c0, Vec4f c1, Vec4f c2, Vec4f c3) { return {{c0.v, c1.v, c2.v, c3.v}}; }
inline Mat4f identity() {
    return {{v4f::make(1, 0, 0, 0), v4f::make(0, 1, 0, 0), v4f::make(0, 0, 1, 0), v4f::make(0, 0, 0, 1)}};
}
inline Mat4f zero() { return {{v4f::zero(), v4f::zero(), v4f::zero(), v4f::zero()}}; }

inline Vec4f column(const Mat4f &m, int i) { return {m.c[i]}; }
/** the first three lanes of column i. */
inline Vec3f axis(const Mat4f &m, int i) { return {vec3f::fix_w(m.c[i])}; }

inline Mat4f transpose(const Mat4f &a) {
    const v128_t t0 = v4f::shuffle2<0, 4, 1, 5>(a.c[0], a.c[1]);
    const v128_t t1 = v4f::shuffle2<0, 4, 1, 5>(a.c[2], a.c[3]);
    const v128_t t2 = v4f::shuffle2<2, 6, 3, 7>(a.c[0], a.c[1]);
    const v128_t t3 = v4f::shuffle2<2, 6, 3, 7>(a.c[2], a.c[3]);
    return {{v4f::shuffle2<0, 1, 4, 5>(t0, t1), v4f::shuffle2<2, 3, 6, 7>(t0, t1), v4f::shuffle2<0, 1, 4, 5>(t2, t3),
             v4f::shuffle2<2, 3, 6, 7>(t2, t3)}};
}

/** false, out untouched, for a singular matrix. */
inline bool invert(Mat4f &out, const Mat4f &m) {
    Float4x4 a;
    store(a, m);
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
    out.c[0] = v4f::make((a11 * b11 - a12 * b10 + a13 * b09) * det, (a02 * b10 - a01 * b11 - a03 * b09) * det,
                         (a31 * b05 - a32 * b04 + a33 * b03) * det, (a22 * b04 - a21 * b05 - a23 * b03) * det);
    out.c[1] = v4f::make((a12 * b08 - a10 * b11 - a13 * b07) * det, (a00 * b11 - a02 * b08 + a03 * b07) * det,
                         (a32 * b02 - a30 * b05 - a33 * b01) * det, (a20 * b05 - a22 * b02 + a23 * b01) * det);
    out.c[2] = v4f::make((a10 * b10 - a11 * b08 + a13 * b06) * det, (a01 * b08 - a00 * b10 - a03 * b06) * det,
                         (a30 * b04 - a31 * b02 + a33 * b00) * det, (a21 * b02 - a20 * b04 - a23 * b00) * det);
    out.c[3] = v4f::make((a11 * b07 - a10 * b09 - a12 * b06) * det, (a00 * b09 - a01 * b07 + a02 * b06) * det,
                         (a31 * b01 - a30 * b03 - a32 * b00) * det, (a20 * b03 - a21 * b01 + a22 * b00) * det);
    return true;
}

/** inverts the upper 3x3 and clears the rest to identity; false, out untouched, when singular. */
inline bool invert3x3(Mat4f &out, const Mat4f &m) {
    const float a00 = v4f::lane<0>(m.c[0]), a01 = v4f::lane<1>(m.c[0]), a02 = v4f::lane<2>(m.c[0]);
    const float a10 = v4f::lane<0>(m.c[1]), a11 = v4f::lane<1>(m.c[1]), a12 = v4f::lane<2>(m.c[1]);
    const float a20 = v4f::lane<0>(m.c[2]), a21 = v4f::lane<1>(m.c[2]), a22 = v4f::lane<2>(m.c[2]);
    const float b01 = a22 * a11 - a12 * a21;
    const float b11 = -a22 * a10 + a12 * a20;
    const float b21 = a21 * a10 - a11 * a20;
    float det = a00 * b01 + a01 * b11 + a02 * b21;
    if (det == 0.0f) return false;
    det = 1.0f / det;
    out.c[0] = v4f::make(b01 * det, (-a22 * a01 + a02 * a21) * det, (a12 * a01 - a02 * a11) * det, 0.0f);
    out.c[1] = v4f::make(b11 * det, (a22 * a00 - a02 * a20) * det, (-a12 * a00 + a02 * a10) * det, 0.0f);
    out.c[2] = v4f::make(b21 * det, (-a21 * a00 + a01 * a20) * det, (a11 * a00 - a01 * a10) * det, 0.0f);
    out.c[3] = v4f::make(0.0f, 0.0f, 0.0f, 1.0f);
    return true;
}

inline Mat4f adjoint(const Mat4f &m) {
    Float4x4 a;
    store(a, m);
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
    return {{v4f::make(a11 * b11 - a12 * b10 + a13 * b09, a02 * b10 - a01 * b11 - a03 * b09, a31 * b05 - a32 * b04 + a33 * b03,
                       a22 * b04 - a21 * b05 - a23 * b03),
             v4f::make(a12 * b08 - a10 * b11 - a13 * b07, a00 * b11 - a02 * b08 + a03 * b07, a32 * b02 - a30 * b05 - a33 * b01,
                       a20 * b05 - a22 * b02 + a23 * b01),
             v4f::make(a10 * b10 - a11 * b08 + a13 * b06, a01 * b08 - a00 * b10 - a03 * b06, a30 * b04 - a31 * b02 + a33 * b00,
                       a21 * b02 - a20 * b04 - a23 * b00),
             v4f::make(a11 * b07 - a10 * b09 - a12 * b06, a00 * b09 - a01 * b07 + a02 * b06, a31 * b01 - a30 * b03 - a32 * b00,
                       a20 * b03 - a21 * b01 + a22 * b00)}};
}

inline float determinant(const Mat4f &m) {
    Float4x4 a;
    store(a, m);
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

/** a's columns weighted by the lanes of v: a0 v.x + a1 v.y + a2 v.z + a3 v.w, summed in that order. */
inline v128_t combine4(const Mat4f &a, v128_t v) {
    v128_t r = v4f::mul(a.c[0], v4f::shuffle<0, 0, 0, 0>(v));
    r = v4f::mul_add(a.c[1], v4f::shuffle<1, 1, 1, 1>(v), r);
    r = v4f::mul_add(a.c[2], v4f::shuffle<2, 2, 2, 2>(v), r);
    return v4f::mul_add(a.c[3], v4f::shuffle<3, 3, 3, 3>(v), r);
}

/** a0 v.x + a1 v.y + a2 v.z, summed in that order. */
inline v128_t combine3(const Mat4f &a, v128_t v) {
    v128_t r = v4f::mul(a.c[0], v4f::shuffle<0, 0, 0, 0>(v));
    r = v4f::mul_add(a.c[1], v4f::shuffle<1, 1, 1, 1>(v), r);
    return v4f::mul_add(a.c[2], v4f::shuffle<2, 2, 2, 2>(v), r);
}

/** a * b. */
inline Mat4f multiply(const Mat4f &a, const Mat4f &b) {
    return {{combine4(a, b.c[0]), combine4(a, b.c[1]), combine4(a, b.c[2]), combine4(a, b.c[3])}};
}

/** the upper 3x3 of a times the upper 3x3 of b, the rest identity. */
inline Mat4f multiply3x3(const Mat4f &a, const Mat4f &b) {
    return {{v4f::set_lane<3>(combine3(a, b.c[0]), 0.0f), v4f::set_lane<3>(combine3(a, b.c[1]), 0.0f),
             v4f::set_lane<3>(combine3(a, b.c[2]), 0.0f), v4f::make(0, 0, 0, 1)}};
}

/** the upper 3x3 of a times the transpose of the upper 3x3 of b, the rest identity. */
inline Mat4f multiply3x3_right_transposed(const Mat4f &a, const Mat4f &b) {
    const Mat4f bt = transpose(b);
    return {{v4f::set_lane<3>(combine3(a, bt.c[0]), 0.0f), v4f::set_lane<3>(combine3(a, bt.c[1]), 0.0f),
             v4f::set_lane<3>(combine3(a, bt.c[2]), 0.0f), v4f::make(0, 0, 0, 1)}};
}

/** the transpose of the upper 3x3 of mat times vec. */
inline Vec3f multiply3x3_transposed_vec(const Mat4f &mat, Vec3f vec) {
    return {vec3f::fix_w(combine3(transpose(mat), vec.v))};
}

/** the upper 3x3 of mat times vec. */
inline Vec3f multiply3x3_vec(const Mat4f &mat, Vec3f vec) { return {vec3f::fix_w(combine3(mat, vec.v))}; }

/** vec as a point through mat, without the perspective divide (jolt's Mat44 * Vec3). */
inline Vec3f multiply_vec3(const Mat4f &mat, Vec3f vec) { return {vec3f::fix_w(v4f::add(combine3(mat, vec.v), mat.c[3]))}; }

/** the matrix whose product with u is v x u. */
inline Mat4f cross_product_matrix(Vec3f v) {
    const float x = vec3f::x(v), y = vec3f::y(v), z = vec3f::z(v);
    return {{v4f::make(0, z, -y, 0), v4f::make(-z, 0, x, 0), v4f::make(y, -x, 0, 0), v4f::make(0, 0, 0, 1)}};
}

/** a translated by v (a * T(v)). */
inline Mat4f translate(const Mat4f &a, Vec3f v) {
    v128_t r = v4f::mul_add(a.c[0], v4f::splat(vec3f::x(v)), a.c[3]);
    r = v4f::mul_add(a.c[1], v4f::splat(vec3f::y(v)), r);
    r = v4f::mul_add(a.c[2], v4f::splat(vec3f::z(v)), r);
    return {{a.c[0], a.c[1], a.c[2], r}};
}

/** a scaled by v (a * S(v)). */
inline Mat4f scale(const Mat4f &a, Vec3f v) {
    return {{v4f::mul(a.c[0], v4f::splat(vec3f::x(v))), v4f::mul(a.c[1], v4f::splat(vec3f::y(v))),
             v4f::mul(a.c[2], v4f::splat(vec3f::z(v))), a.c[3]}};
}

/** a rotated by rad about axis; false, out untouched, for a zero axis. */
inline bool rotate(Mat4f &out, const Mat4f &a, float rad, Vec3f axis) {
    float x = vec3f::x(axis), y = vec3f::y(axis), z = vec3f::z(axis);
    float len = scalar::sqrt(x * x + y * y + z * z);
    if (len < scalar::EPSILON) return false;
    len = 1.0f / len;
    x *= len;
    y *= len;
    z *= len;
    const float s = scalar::sin(rad);
    const float c = scalar::cos(rad);
    const float t = 1.0f - c;
    const v128_t a0 = a.c[0], a1 = a.c[1], a2 = a.c[2];
    const float b00 = x * x * t + c, b01 = y * x * t + z * s, b02 = z * x * t - y * s;
    const float b10 = x * y * t - z * s, b11 = y * y * t + c, b12 = z * y * t + x * s;
    const float b20 = x * z * t + y * s, b21 = y * z * t - x * s, b22 = z * z * t + c;
    out.c[0] = v4f::mul_add(a2, v4f::splat(b02), v4f::mul_add(a1, v4f::splat(b01), v4f::mul(a0, v4f::splat(b00))));
    out.c[1] = v4f::mul_add(a2, v4f::splat(b12), v4f::mul_add(a1, v4f::splat(b11), v4f::mul(a0, v4f::splat(b10))));
    out.c[2] = v4f::mul_add(a2, v4f::splat(b22), v4f::mul_add(a1, v4f::splat(b21), v4f::mul(a0, v4f::splat(b20))));
    out.c[3] = a.c[3];
    return true;
}

inline Mat4f rotate_x(const Mat4f &a, float rad) {
    const v128_t s = v4f::splat(scalar::sin(rad));
    const v128_t c = v4f::splat(scalar::cos(rad));
    return {{a.c[0], v4f::mul_add(a.c[2], s, v4f::mul(a.c[1], c)), v4f::sub(v4f::mul(a.c[2], c), v4f::mul(a.c[1], s)), a.c[3]}};
}

inline Mat4f rotate_y(const Mat4f &a, float rad) {
    const v128_t s = v4f::splat(scalar::sin(rad));
    const v128_t c = v4f::splat(scalar::cos(rad));
    return {{v4f::sub(v4f::mul(a.c[0], c), v4f::mul(a.c[2], s)), a.c[1], v4f::mul_add(a.c[0], s, v4f::mul(a.c[2], c)), a.c[3]}};
}

inline Mat4f rotate_z(const Mat4f &a, float rad) {
    const v128_t s = v4f::splat(scalar::sin(rad));
    const v128_t c = v4f::splat(scalar::cos(rad));
    return {{v4f::mul_add(a.c[1], s, v4f::mul(a.c[0], c)), v4f::sub(v4f::mul(a.c[1], c), v4f::mul(a.c[0], s)), a.c[2], a.c[3]}};
}

inline Mat4f from_translation(Vec3f v) {
    return {{v4f::make(1, 0, 0, 0), v4f::make(0, 1, 0, 0), v4f::make(0, 0, 1, 0), v4f::set_lane<3>(v.v, 1.0f)}};
}

inline Mat4f from_scaling(Vec3f v) {
    return {{v4f::make(vec3f::x(v), 0, 0, 0), v4f::make(0, vec3f::y(v), 0, 0), v4f::make(0, 0, vec3f::z(v), 0),
             v4f::make(0, 0, 0, 1)}};
}

/** false, out untouched, for a zero axis. */
inline bool from_rotation(Mat4f &out, float rad, Vec3f axis) {
    float x = vec3f::x(axis), y = vec3f::y(axis), z = vec3f::z(axis);
    float len = scalar::sqrt(x * x + y * y + z * z);
    if (len < scalar::EPSILON) return false;
    len = 1.0f / len;
    x *= len;
    y *= len;
    z *= len;
    const float s = scalar::sin(rad);
    const float c = scalar::cos(rad);
    const float t = 1.0f - c;
    out.c[0] = v4f::make(x * x * t + c, y * x * t + z * s, z * x * t - y * s, 0);
    out.c[1] = v4f::make(x * y * t - z * s, y * y * t + c, z * y * t + x * s, 0);
    out.c[2] = v4f::make(x * z * t + y * s, y * z * t - x * s, z * z * t + c, 0);
    out.c[3] = v4f::make(0, 0, 0, 1);
    return true;
}

inline Mat4f from_x_rotation(float rad) {
    const float s = scalar::sin(rad);
    const float c = scalar::cos(rad);
    return {{v4f::make(1, 0, 0, 0), v4f::make(0, c, s, 0), v4f::make(0, -s, c, 0), v4f::make(0, 0, 0, 1)}};
}

inline Mat4f from_y_rotation(float rad) {
    const float s = scalar::sin(rad);
    const float c = scalar::cos(rad);
    return {{v4f::make(c, 0, -s, 0), v4f::make(0, 1, 0, 0), v4f::make(s, 0, c, 0), v4f::make(0, 0, 0, 1)}};
}

inline Mat4f from_z_rotation(float rad) {
    const float s = scalar::sin(rad);
    const float c = scalar::cos(rad);
    return {{v4f::make(c, s, 0, 0), v4f::make(-s, c, 0, 0), v4f::make(0, 0, 1, 0), v4f::make(0, 0, 0, 1)}};
}

/** the three rotation columns of q, lane 3 zero. */
inline void rotation_columns(v128_t &c0, v128_t &c1, v128_t &c2, Quatf q) {
    const float qx = v4f::lane<0>(q.v), qy = v4f::lane<1>(q.v), qz = v4f::lane<2>(q.v), qw = v4f::lane<3>(q.v);
    const float x2 = qx + qx, y2 = qy + qy, z2 = qz + qz;
    const float xx = qx * x2, xy = qx * y2, xz = qx * z2;
    const float yy = qy * y2, yz = qy * z2, zz = qz * z2;
    const float wx = qw * x2, wy = qw * y2, wz = qw * z2;
    c0 = v4f::make(1.0f - (yy + zz), xy + wz, xz - wy, 0.0f);
    c1 = v4f::make(xy - wz, 1.0f - (xx + zz), yz + wx, 0.0f);
    c2 = v4f::make(xz + wy, yz - wx, 1.0f - (xx + yy), 0.0f);
}

inline Mat4f from_rotation_translation(Quatf q, Vec3f v) {
    Mat4f out;
    rotation_columns(out.c[0], out.c[1], out.c[2], q);
    out.c[3] = v4f::set_lane<3>(v.v, 1.0f);
    return out;
}

inline Vec3f get_translation(const Mat4f &mat) { return {vec3f::fix_w(mat.c[3])}; }

/** the length of each of the three axis columns. */
inline Vec3f get_scaling(const Mat4f &mat) {
    const Mat4f t = transpose(mat);
    v128_t sq = v4f::mul(t.c[0], t.c[0]);
    sq = v4f::add(sq, v4f::mul(t.c[1], t.c[1]));
    sq = v4f::add(sq, v4f::mul(t.c[2], t.c[2]));
    return {vec3f::fix_w(v4f::sqrt(sq))};
}

/** the rotation of a matrix with the given scaling divided out. */
inline Quatf rotation_with_scaling(const Mat4f &mat, Vec3f scaling) {
    const float is1 = 1.0f / vec3f::x(scaling);
    const float is2 = 1.0f / vec3f::y(scaling);
    const float is3 = 1.0f / vec3f::z(scaling);
    const float sm11 = v4f::lane<0>(mat.c[0]) * is1, sm12 = v4f::lane<1>(mat.c[0]) * is2, sm13 = v4f::lane<2>(mat.c[0]) * is3;
    const float sm21 = v4f::lane<0>(mat.c[1]) * is1, sm22 = v4f::lane<1>(mat.c[1]) * is2, sm23 = v4f::lane<2>(mat.c[1]) * is3;
    const float sm31 = v4f::lane<0>(mat.c[2]) * is1, sm32 = v4f::lane<1>(mat.c[2]) * is2, sm33 = v4f::lane<2>(mat.c[2]) * is3;
    const float trace = sm11 + sm22 + sm33;
    float S;
    if (trace > 0.0f) {
        S = scalar::sqrt(trace + 1.0f) * 2.0f;
        return {v4f::make((sm23 - sm32) / S, (sm31 - sm13) / S, (sm12 - sm21) / S, 0.25f * S)};
    }
    if (sm11 > sm22 && sm11 > sm33) {
        S = scalar::sqrt(1.0f + sm11 - sm22 - sm33) * 2.0f;
        return {v4f::make(0.25f * S, (sm12 + sm21) / S, (sm31 + sm13) / S, (sm23 - sm32) / S)};
    }
    if (sm22 > sm33) {
        S = scalar::sqrt(1.0f + sm22 - sm11 - sm33) * 2.0f;
        return {v4f::make((sm12 + sm21) / S, 0.25f * S, (sm23 + sm32) / S, (sm31 - sm13) / S)};
    }
    S = scalar::sqrt(1.0f + sm33 - sm11 - sm22) * 2.0f;
    return {v4f::make((sm31 + sm13) / S, (sm23 + sm32) / S, 0.25f * S, (sm12 - sm21) / S)};
}

inline Quatf get_rotation(const Mat4f &mat) { return rotation_with_scaling(mat, get_scaling(mat)); }

/** rotation, translation and scale of a trs matrix. */
inline Quatf decompose(Vec3f &out_t, Vec3f &out_s, const Mat4f &mat) {
    out_t = get_translation(mat);
    out_s = get_scaling(mat);
    return rotation_with_scaling(mat, out_s);
}

inline Mat4f from_rotation_translation_scale(Quatf q, Vec3f v, Vec3f s) {
    Mat4f out;
    rotation_columns(out.c[0], out.c[1], out.c[2], q);
    out.c[0] = v4f::mul(out.c[0], v4f::splat(vec3f::x(s)));
    out.c[1] = v4f::mul(out.c[1], v4f::splat(vec3f::y(s)));
    out.c[2] = v4f::mul(out.c[2], v4f::splat(vec3f::z(s)));
    out.c[3] = v4f::set_lane<3>(v.v, 1.0f);
    return out;
}

/** as from_rotation_translation_scale, rotating and scaling about origin o. */
inline Mat4f from_rotation_translation_scale_origin(Quatf q, Vec3f v, Vec3f s, Vec3f o) {
    Mat4f out = from_rotation_translation_scale(q, v, s);
    const v128_t moved = combine3(out, o.v);
    out.c[3] = v4f::set_lane<3>(v4f::sub(v4f::add(v.v, o.v), moved), 1.0f);
    return out;
}

inline Mat4f from_quat(Quatf q) {
    const float qx = v4f::lane<0>(q.v), qy = v4f::lane<1>(q.v), qz = v4f::lane<2>(q.v), qw = v4f::lane<3>(q.v);
    const float x2 = qx + qx, y2 = qy + qy, z2 = qz + qz;
    const float xx = qx * x2, yx = qy * x2, yy = qy * y2;
    const float zx = qz * x2, zy = qz * y2, zz = qz * z2;
    const float wx = qw * x2, wy = qw * y2, wz = qw * z2;
    return {{v4f::make(1.0f - yy - zz, yx + wz, zx - wy, 0), v4f::make(yx - wz, 1.0f - xx - zz, zy + wx, 0),
             v4f::make(zx + wy, zy - wx, 1.0f - xx - yy, 0), v4f::make(0, 0, 0, 1)}};
}

/** a view matrix looking from eye at center; identity when they coincide. */
inline Mat4f look_at(Vec3f eye, Vec3f center, Vec3f up) {
    const float ex = vec3f::x(eye), ey = vec3f::y(eye), ez = vec3f::z(eye);
    const float ux = vec3f::x(up), uy = vec3f::y(up), uz = vec3f::z(up);
    if (scalar::abs(ex - vec3f::x(center)) < scalar::EPSILON && scalar::abs(ey - vec3f::y(center)) < scalar::EPSILON &&
        scalar::abs(ez - vec3f::z(center)) < scalar::EPSILON) {
        return identity();
    }
    float z0 = ex - vec3f::x(center);
    float z1 = ey - vec3f::y(center);
    float z2 = ez - vec3f::z(center);
    float len = 1.0f / scalar::sqrt(z0 * z0 + z1 * z1 + z2 * z2);
    z0 *= len;
    z1 *= len;
    z2 *= len;
    float x0 = uy * z2 - uz * z1;
    float x1 = uz * z0 - ux * z2;
    float x2 = ux * z1 - uy * z0;
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
    return {{v4f::make(x0, y0, z0, 0), v4f::make(x1, y1, z1, 0), v4f::make(x2, y2, z2, 0),
             v4f::make(-(x0 * ex + x1 * ey + x2 * ez), -(y0 * ex + y1 * ey + y2 * ez), -(z0 * ex + z1 * ey + z2 * ez), 1)}};
}

/** the model matrix of something at eye facing target, the inverse of look_at's view. */
inline Mat4f target_to(Vec3f eye, Vec3f target, Vec3f up) {
    const float ux = vec3f::x(up), uy = vec3f::y(up), uz = vec3f::z(up);
    float z0 = vec3f::x(eye) - vec3f::x(target);
    float z1 = vec3f::y(eye) - vec3f::y(target);
    float z2 = vec3f::z(eye) - vec3f::z(target);
    float len = z0 * z0 + z1 * z1 + z2 * z2;
    if (len > 0.0f) {
        len = 1.0f / scalar::sqrt(len);
        z0 *= len;
        z1 *= len;
        z2 *= len;
    }
    float x0 = uy * z2 - uz * z1;
    float x1 = uz * z0 - ux * z2;
    float x2 = ux * z1 - uy * z0;
    len = x0 * x0 + x1 * x1 + x2 * x2;
    if (len > 0.0f) {
        len = 1.0f / scalar::sqrt(len);
        x0 *= len;
        x1 *= len;
        x2 *= len;
    }
    return {{v4f::make(x0, x1, x2, 0), v4f::make(z1 * x2 - z2 * x1, z2 * x0 - z0 * x2, z0 * x1 - z1 * x0, 0),
             v4f::make(z0, z1, z2, 0), v4f::set_lane<3>(eye.v, 1.0f)}};
}

inline float frob(const Mat4f &a) {
    v128_t sum = v4f::mul(a.c[0], a.c[0]);
    for (int i = 1; i < 4; i++) sum = v4f::mul_add(a.c[i], a.c[i], sum);
    return scalar::sqrt(v4f::hsum4(sum));
}

inline Mat4f add(const Mat4f &a, const Mat4f &b) {
    return {{v4f::add(a.c[0], b.c[0]), v4f::add(a.c[1], b.c[1]), v4f::add(a.c[2], b.c[2]), v4f::add(a.c[3], b.c[3])}};
}

inline Mat4f subtract(const Mat4f &a, const Mat4f &b) {
    return {{v4f::sub(a.c[0], b.c[0]), v4f::sub(a.c[1], b.c[1]), v4f::sub(a.c[2], b.c[2]), v4f::sub(a.c[3], b.c[3])}};
}

inline Mat4f multiply_scalar(const Mat4f &a, float b) {
    const v128_t s = v4f::splat(b);
    return {{v4f::mul(a.c[0], s), v4f::mul(a.c[1], s), v4f::mul(a.c[2], s), v4f::mul(a.c[3], s)}};
}

inline Mat4f multiply_scalar_and_add(const Mat4f &a, const Mat4f &b, float s) {
    const v128_t vs = v4f::splat(s);
    return {{v4f::mul_add(b.c[0], vs, a.c[0]), v4f::mul_add(b.c[1], vs, a.c[1]), v4f::mul_add(b.c[2], vs, a.c[2]),
             v4f::mul_add(b.c[3], vs, a.c[3])}};
}

inline bool exact_equals(const Mat4f &a, const Mat4f &b) {
    return v4f::all_true(v4f::eq(a.c[0], b.c[0])) && v4f::all_true(v4f::eq(a.c[1], b.c[1])) &&
           v4f::all_true(v4f::eq(a.c[2], b.c[2])) && v4f::all_true(v4f::eq(a.c[3], b.c[3]));
}

inline bool equals(const Mat4f &a, const Mat4f &b) {
    Float4x4 fa, fb;
    store(fa, a);
    store(fb, b);
    for (int i = 0; i < 16; i++) {
        if (!scalar::equals(fa.m[i], fb.m[i])) return false;
    }
    return true;
}

}  // namespace mat4f

TUMBLE_NAMESPACE_END
