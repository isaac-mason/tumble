#pragma once

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

/** the first three lanes of column i */
inline Vec3f axis(const Mat4f &m, int i) { return {vec3f::fix_w(m.c[i])}; }

inline Mat4f transpose(const Mat4f &a) {
    const V128 t0 = v4f::shuffle2<0, 4, 1, 5>(a.c[0], a.c[1]);
    const V128 t1 = v4f::shuffle2<0, 4, 1, 5>(a.c[2], a.c[3]);
    const V128 t2 = v4f::shuffle2<2, 6, 3, 7>(a.c[0], a.c[1]);
    const V128 t3 = v4f::shuffle2<2, 6, 3, 7>(a.c[2], a.c[3]);
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
inline V128 combine4(const Mat4f &a, V128 v) {
    V128 r = v4f::mul(a.c[0], v4f::shuffle<0, 0, 0, 0>(v));
    r = v4f::mul_add(a.c[1], v4f::shuffle<1, 1, 1, 1>(v), r);
    r = v4f::mul_add(a.c[2], v4f::shuffle<2, 2, 2, 2>(v), r);
    return v4f::mul_add(a.c[3], v4f::shuffle<3, 3, 3, 3>(v), r);
}

/** a0 v.x + a1 v.y + a2 v.z, summed in that order. */
inline V128 combine3(const Mat4f &a, V128 v) {
    V128 r = v4f::mul(a.c[0], v4f::shuffle<0, 0, 0, 0>(v));
    r = v4f::mul_add(a.c[1], v4f::shuffle<1, 1, 1, 1>(v), r);
    return v4f::mul_add(a.c[2], v4f::shuffle<2, 2, 2, 2>(v), r);
}

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

/** vec as a point through mat, without the perspective divide */
inline Vec3f multiply_vec3(const Mat4f &mat, Vec3f vec) { return {vec3f::fix_w(v4f::add(combine3(mat, vec.v), mat.c[3]))}; }

/** the matrix whose product with u is v x u. */
inline Mat4f cross_product_matrix(Vec3f v) {
    const float x = vec3f::x(v), y = vec3f::y(v), z = vec3f::z(v);
    return {{v4f::make(0, z, -y, 0), v4f::make(-z, 0, x, 0), v4f::make(y, -x, 0, 0), v4f::make(0, 0, 0, 1)}};
}

/** a translated by v (a * T(v)). */
inline Mat4f translate(const Mat4f &a, Vec3f v) {
    V128 r = v4f::mul_add(a.c[0], v4f::splat(vec3f::x(v)), a.c[3]);
    r = v4f::mul_add(a.c[1], v4f::splat(vec3f::y(v)), r);
    r = v4f::mul_add(a.c[2], v4f::splat(vec3f::z(v)), r);
    return {{a.c[0], a.c[1], a.c[2], r}};
}

/** a scaled by v (a * S(v)). */
inline Mat4f scale(const Mat4f &a, Vec3f v) {
    return {{v4f::mul(a.c[0], v4f::splat(vec3f::x(v))), v4f::mul(a.c[1], v4f::splat(vec3f::y(v))),
             v4f::mul(a.c[2], v4f::splat(vec3f::z(v))), a.c[3]}};
}

inline Mat4f from_scaling(Vec3f v) {
    return {{v4f::make(vec3f::x(v), 0, 0, 0), v4f::make(0, vec3f::y(v), 0, 0), v4f::make(0, 0, vec3f::z(v), 0),
             v4f::make(0, 0, 0, 1)}};
}

/** the three rotation columns of q, lane 3 zero */
inline void rotation_columns(V128 &c0, V128 &c1, V128 &c2, Quatf q) {
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

inline Mat4f from_rotation_translation_scale(Quatf q, Vec3f v, Vec3f s) {
    Mat4f out;
    rotation_columns(out.c[0], out.c[1], out.c[2], q);
    out.c[0] = v4f::mul(out.c[0], v4f::splat(vec3f::x(s)));
    out.c[1] = v4f::mul(out.c[1], v4f::splat(vec3f::y(s)));
    out.c[2] = v4f::mul(out.c[2], v4f::splat(vec3f::z(s)));
    out.c[3] = v4f::set_lane<3>(v.v, 1.0f);
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

inline Mat4f add(const Mat4f &a, const Mat4f &b) {
    return {{v4f::add(a.c[0], b.c[0]), v4f::add(a.c[1], b.c[1]), v4f::add(a.c[2], b.c[2]), v4f::add(a.c[3], b.c[3])}};
}

inline Mat4f subtract(const Mat4f &a, const Mat4f &b) {
    return {{v4f::sub(a.c[0], b.c[0]), v4f::sub(a.c[1], b.c[1]), v4f::sub(a.c[2], b.c[2]), v4f::sub(a.c[3], b.c[3])}};
}

inline Mat4f multiply_scalar(const Mat4f &a, float b) {
    const V128 s = v4f::splat(b);
    return {{v4f::mul(a.c[0], s), v4f::mul(a.c[1], s), v4f::mul(a.c[2], s), v4f::mul(a.c[3], s)}};
}

}  // namespace mat4f

TUMBLE_NAMESPACE_END
