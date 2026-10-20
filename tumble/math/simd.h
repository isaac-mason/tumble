#pragma once

#include <stdint.h>
#include "tumble/tumble.h"

#ifdef TUMBLE_USE_WASM_SIMD
#include <wasm_simd128.h>
#else
#include "tumble/math/scalar.h"
#endif

TUMBLE_NAMESPACE_BEGIN

// four 32-bit lanes, read as floats (v4f), as integers (v4i) or as masks (a compare's lanes, all ones or all zeros)
#ifdef TUMBLE_USE_WASM_SIMD
using V128 = v128_t;
#else
struct alignas(16) V128 {
    uint32_t lanes[4];
};
#endif

// lane 3 of a three-wide value is not data
namespace v4f {

#ifdef TUMBLE_USE_WASM_SIMD

/** lane 3 zero; never reads past the 12 bytes */
inline V128 load3(const float *p) {
    V128 v = wasm_v128_load64_zero(p);
    return wasm_v128_load32_lane(p + 2, v, 2);
}

/** lanes (x, y, z, z); never reads past the 12 bytes */
inline V128 load3_xyzz(const float *p) {
    const V128 xy = wasm_v128_load64_zero(p);
    const V128 zz = wasm_v128_load32_splat(p + 2);
    return wasm_i32x4_shuffle(xy, zz, 0, 1, 4, 4);
}

/** never writes past the 12 bytes */
inline void store3(float *p, V128 v) {
    wasm_v128_store64_lane(p, v, 0);
    wasm_v128_store32_lane(p + 2, v, 2);
}

inline V128 load4(const float *p) { return wasm_v128_load(p); }
inline void store4(float *p, V128 v) { wasm_v128_store(p, v); }

inline V128 make(float x, float y, float z, float w) { return wasm_f32x4_make(x, y, z, w); }
inline V128 splat(float s) { return wasm_f32x4_splat(s); }
inline V128 zero() { return wasm_f32x4_const_splat(0.0f); }

inline V128 add(V128 a, V128 b) { return wasm_f32x4_add(a, b); }
inline V128 sub(V128 a, V128 b) { return wasm_f32x4_sub(a, b); }
inline V128 mul(V128 a, V128 b) { return wasm_f32x4_mul(a, b); }
inline V128 div(V128 a, V128 b) { return wasm_f32x4_div(a, b); }
/** b < a ? b : a */
inline V128 min(V128 a, V128 b) { return wasm_f32x4_pmin(a, b); }
/** a < b ? b : a */
inline V128 max(V128 a, V128 b) { return wasm_f32x4_pmax(a, b); }
/** as scalar::min: nan propagates, -0 below +0 */
inline V128 fmin(V128 a, V128 b) { return wasm_f32x4_min(a, b); }
/** as scalar::max */
inline V128 fmax(V128 a, V128 b) { return wasm_f32x4_max(a, b); }
inline V128 neg(V128 a) { return wasm_f32x4_neg(a); }
inline V128 abs(V128 a) { return wasm_f32x4_abs(a); }
inline V128 sqrt(V128 a) { return wasm_f32x4_sqrt(a); }

template <int I>
inline float lane(V128 v) {
    return wasm_f32x4_extract_lane(v, I);
}

template <int I>
inline V128 set_lane(V128 v, float s) {
    return wasm_f32x4_replace_lane(v, I, s);
}

/** lanes (a[X], a[Y], a[Z], a[W]); X..W index into a alone. */
template <int X, int Y, int Z, int W>
inline V128 shuffle(V128 a) {
    return wasm_i32x4_shuffle(a, a, X, Y, Z, W);
}

/** lanes drawn from both: 0..3 index a, 4..7 index b. */
template <int X, int Y, int Z, int W>
inline V128 shuffle2(V128 a, V128 b) {
    return wasm_i32x4_shuffle(a, b, X, Y, Z, W);
}

inline V128 lt(V128 a, V128 b) { return wasm_f32x4_lt(a, b); }
inline V128 le(V128 a, V128 b) { return wasm_f32x4_le(a, b); }
inline V128 gt(V128 a, V128 b) { return wasm_f32x4_gt(a, b); }
inline V128 ge(V128 a, V128 b) { return wasm_f32x4_ge(a, b); }
inline V128 eq(V128 a, V128 b) { return wasm_f32x4_eq(a, b); }

inline V128 bit_and(V128 a, V128 b) { return wasm_v128_and(a, b); }
inline V128 bit_or(V128 a, V128 b) { return wasm_v128_or(a, b); }
inline V128 bit_xor(V128 a, V128 b) { return wasm_v128_xor(a, b); }

/** a where mask is set, else b */
inline V128 select(V128 a, V128 b, V128 mask) { return wasm_v128_bitselect(a, b, mask); }

/** every lane of the mask set */
inline bool all_true(V128 mask) { return wasm_i32x4_all_true(mask); }
/** any bit of the mask set */
inline bool any_true(V128 mask) { return wasm_v128_any_true(mask); }

/** a mask's lanes as the low four bits */
inline uint32_t bitmask(V128 mask) { return wasm_i32x4_bitmask(mask); }

#else

inline float get(V128 v, int i) { return __builtin_bit_cast(float, v.lanes[i]); }
inline uint32_t bits_of(float s) { return __builtin_bit_cast(uint32_t, s); }
inline uint32_t mask_of(bool set) { return set ? 0xffffffffu : 0u; }

inline V128 make(float x, float y, float z, float w) { return {{bits_of(x), bits_of(y), bits_of(z), bits_of(w)}}; }
inline V128 splat(float s) { return make(s, s, s, s); }
inline V128 zero() { return {{0, 0, 0, 0}}; }

inline V128 load3(const float *p) { return make(p[0], p[1], p[2], 0.0f); }
inline V128 load3_xyzz(const float *p) { return make(p[0], p[1], p[2], p[2]); }
inline void store3(float *p, V128 v) {
    p[0] = get(v, 0);
    p[1] = get(v, 1);
    p[2] = get(v, 2);
}
inline V128 load4(const float *p) { return make(p[0], p[1], p[2], p[3]); }
inline void store4(float *p, V128 v) {
    for (int i = 0; i < 4; i++) p[i] = get(v, i);
}

#define TUMBLE_LANES_F(expression)                               \
    V128 r;                                                      \
    for (int i = 0; i < 4; i++) r.lanes[i] = bits_of(expression); \
    return r
#define TUMBLE_LANES_U(expression)                       \
    V128 r;                                              \
    for (int i = 0; i < 4; i++) r.lanes[i] = (expression); \
    return r

inline V128 add(V128 a, V128 b) { TUMBLE_LANES_F(get(a, i) + get(b, i)); }
inline V128 sub(V128 a, V128 b) { TUMBLE_LANES_F(get(a, i) - get(b, i)); }
inline V128 mul(V128 a, V128 b) { TUMBLE_LANES_F(get(a, i) * get(b, i)); }
inline V128 div(V128 a, V128 b) { TUMBLE_LANES_F(get(a, i) / get(b, i)); }
inline V128 min(V128 a, V128 b) { TUMBLE_LANES_U(get(b, i) < get(a, i) ? b.lanes[i] : a.lanes[i]); }
inline V128 max(V128 a, V128 b) { TUMBLE_LANES_U(get(a, i) < get(b, i) ? b.lanes[i] : a.lanes[i]); }
inline V128 fmin(V128 a, V128 b) { TUMBLE_LANES_F(scalar::min(get(a, i), get(b, i))); }
inline V128 fmax(V128 a, V128 b) { TUMBLE_LANES_F(scalar::max(get(a, i), get(b, i))); }
inline V128 neg(V128 a) { TUMBLE_LANES_U(a.lanes[i] ^ 0x80000000u); }
inline V128 abs(V128 a) { TUMBLE_LANES_U(a.lanes[i] & 0x7fffffffu); }
inline V128 sqrt(V128 a) { TUMBLE_LANES_F(scalar::sqrt(get(a, i))); }

template <int I>
inline float lane(V128 v) {
    return get(v, I);
}

template <int I>
inline V128 set_lane(V128 v, float s) {
    v.lanes[I] = bits_of(s);
    return v;
}

template <int X, int Y, int Z, int W>
inline V128 shuffle(V128 a) {
    return {{a.lanes[X], a.lanes[Y], a.lanes[Z], a.lanes[W]}};
}

template <int X, int Y, int Z, int W>
inline V128 shuffle2(V128 a, V128 b) {
    const uint32_t both[8] = {a.lanes[0], a.lanes[1], a.lanes[2], a.lanes[3], b.lanes[0], b.lanes[1], b.lanes[2], b.lanes[3]};
    return {{both[X], both[Y], both[Z], both[W]}};
}

inline V128 lt(V128 a, V128 b) { TUMBLE_LANES_U(mask_of(get(a, i) < get(b, i))); }
inline V128 le(V128 a, V128 b) { TUMBLE_LANES_U(mask_of(get(a, i) <= get(b, i))); }
inline V128 gt(V128 a, V128 b) { TUMBLE_LANES_U(mask_of(get(a, i) > get(b, i))); }
inline V128 ge(V128 a, V128 b) { TUMBLE_LANES_U(mask_of(get(a, i) >= get(b, i))); }
inline V128 eq(V128 a, V128 b) { TUMBLE_LANES_U(mask_of(get(a, i) == get(b, i))); }

inline V128 bit_and(V128 a, V128 b) { TUMBLE_LANES_U(a.lanes[i] & b.lanes[i]); }
inline V128 bit_or(V128 a, V128 b) { TUMBLE_LANES_U(a.lanes[i] | b.lanes[i]); }
inline V128 bit_xor(V128 a, V128 b) { TUMBLE_LANES_U(a.lanes[i] ^ b.lanes[i]); }

inline V128 select(V128 a, V128 b, V128 mask) { TUMBLE_LANES_U((a.lanes[i] & mask.lanes[i]) | (b.lanes[i] & ~mask.lanes[i])); }

inline bool all_true(V128 mask) { return mask.lanes[0] != 0 && mask.lanes[1] != 0 && mask.lanes[2] != 0 && mask.lanes[3] != 0; }
inline bool any_true(V128 mask) { return (mask.lanes[0] | mask.lanes[1] | mask.lanes[2] | mask.lanes[3]) != 0; }

inline uint32_t bitmask(V128 mask) {
    return (mask.lanes[0] >> 31) | (mask.lanes[1] >> 31) << 1 | (mask.lanes[2] >> 31) << 2 | (mask.lanes[3] >> 31) << 3;
}

#undef TUMBLE_LANES_F
#undef TUMBLE_LANES_U

#endif

/** lane 3 kept */
inline V128 neg_xyz(V128 a) { return bit_xor(a, make(-0.0f, -0.0f, -0.0f, 0.0f)); }

/** not fused */
inline V128 mul_add(V128 a, V128 b, V128 c) { return add(mul(a, b), c); }

/** four packed xyz points into x, y and z lanes; never reads past the 48 bytes */
inline void load3x4(const float *p, V128 &x, V128 &y, V128 &z) {
    const V128 a = load4(p);      // x0 y0 z0 x1
    const V128 b = load4(p + 4);  // y1 z1 x2 y2
    const V128 c = load4(p + 8);  // z2 x3 y3 z3
    x = shuffle2<0, 1, 2, 5>(shuffle2<0, 3, 6, 0>(a, b), c);
    y = shuffle2<0, 1, 2, 6>(shuffle2<1, 4, 7, 0>(a, b), c);
    z = shuffle2<0, 1, 4, 7>(shuffle2<2, 5, 0, 0>(a, b), c);
}

/** four points into x, y and z lanes */
inline void transpose3x4(V128 r0, V128 r1, V128 r2, V128 r3, V128 &x, V128 &y, V128 &z) {
    const V128 xy01 = shuffle2<0, 4, 1, 5>(r0, r1);  // x0 x1 y0 y1
    const V128 xy23 = shuffle2<0, 4, 1, 5>(r2, r3);  // x2 x3 y2 y3
    const V128 zw01 = shuffle2<2, 6, 3, 7>(r0, r1);  // z0 z1 w0 w1
    const V128 zw23 = shuffle2<2, 6, 3, 7>(r2, r3);  // z2 z3 w2 w3
    x = shuffle2<0, 1, 4, 5>(xy01, xy23);
    y = shuffle2<2, 3, 6, 7>(xy01, xy23);
    z = shuffle2<0, 1, 4, 5>(zw01, zw23);
}

inline float hsum3(V128 v) { return lane<0>(v) + lane<1>(v) + lane<2>(v); }

inline float hsum4(V128 v) {
    const V128 s = add(v, shuffle<2, 3, 0, 1>(v));
    return lane<0>(add(s, shuffle<1, 0, 3, 2>(s)));
}

inline float dot3(V128 a, V128 b) { return hsum3(mul(a, b)); }
inline float dot4(V128 a, V128 b) { return hsum4(mul(a, b)); }

}  // namespace v4f

namespace v4i {

#ifdef TUMBLE_USE_WASM_SIMD
inline V128 make(int32_t x, int32_t y, int32_t z, int32_t w) { return wasm_i32x4_make(x, y, z, w); }
inline V128 splat(int32_t s) { return wasm_i32x4_splat(s); }
/** wraps */
inline V128 add(V128 a, V128 b) { return wasm_i32x4_add(a, b); }
template <int I>
inline int32_t lane(V128 v) {
    return wasm_i32x4_extract_lane(v, I);
}
#else
inline V128 make(int32_t x, int32_t y, int32_t z, int32_t w) { return {{(uint32_t)x, (uint32_t)y, (uint32_t)z, (uint32_t)w}}; }
inline V128 splat(int32_t s) { return make(s, s, s, s); }
inline V128 add(V128 a, V128 b) { return {{a.lanes[0] + b.lanes[0], a.lanes[1] + b.lanes[1], a.lanes[2] + b.lanes[2], a.lanes[3] + b.lanes[3]}}; }
template <int I>
inline int32_t lane(V128 v) {
    return (int32_t)v.lanes[I];
}
#endif

}  // namespace v4i

TUMBLE_NAMESPACE_END
