#pragma once

#include <stdint.h>
#include "tumble/config.h"

#ifdef TUMBLE_USE_WASM_SIMD
#include <wasm_simd128.h>
#elif defined(TUMBLE_USE_SSE2)
#include <emmintrin.h>
#elif defined(TUMBLE_USE_NEON)
#include <arm_neon.h>
#else
#include "tumble/math/scalar.h"
#endif

#if defined(__clang__) || (defined(__GNUC__) && __GNUC__ >= 12)
#define TUMBLE_HAS_SHUFFLEVECTOR
#endif

TUMBLE_NAMESPACE_BEGIN

// four 32-bit lanes, read as floats (v4f), as integers (v4i) or as masks (a compare's lanes, all ones or all zeros)
#ifdef TUMBLE_USE_WASM_SIMD
using V128 = v128_t;
#elif defined(TUMBLE_USE_SSE2)
using V128 = __m128;
#elif defined(TUMBLE_USE_NEON)
using V128 = float32x4_t;
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

#elif defined(TUMBLE_USE_SSE2)

inline V128 load3(const float *p) {
    const V128 xy = _mm_loadl_pi(_mm_setzero_ps(), reinterpret_cast<const __m64 *>(p));
    return _mm_movelh_ps(xy, _mm_load_ss(p + 2));
}

inline V128 load3_xyzz(const float *p) {
    const V128 xy = _mm_loadl_pi(_mm_setzero_ps(), reinterpret_cast<const __m64 *>(p));
    return _mm_movelh_ps(xy, _mm_load1_ps(p + 2));
}

inline void store3(float *p, V128 v) {
    _mm_storel_pi(reinterpret_cast<__m64 *>(p), v);
    _mm_store_ss(p + 2, _mm_movehl_ps(v, v));
}

inline V128 load4(const float *p) { return _mm_loadu_ps(p); }
inline void store4(float *p, V128 v) { _mm_storeu_ps(p, v); }

inline V128 make(float x, float y, float z, float w) { return _mm_setr_ps(x, y, z, w); }
inline V128 splat(float s) { return _mm_set1_ps(s); }
inline V128 zero() { return _mm_setzero_ps(); }

inline V128 add(V128 a, V128 b) { return _mm_add_ps(a, b); }
inline V128 sub(V128 a, V128 b) { return _mm_sub_ps(a, b); }
inline V128 mul(V128 a, V128 b) { return _mm_mul_ps(a, b); }
inline V128 div(V128 a, V128 b) { return _mm_div_ps(a, b); }
inline V128 min(V128 a, V128 b) { return _mm_min_ps(b, a); }
inline V128 max(V128 a, V128 b) { return _mm_max_ps(b, a); }
inline V128 fmin(V128 a, V128 b) {
    const V128 either = _mm_or_ps(_mm_min_ps(a, b), _mm_min_ps(b, a));
    const V128 unordered = _mm_cmpunord_ps(either, either);
    const V128 quieted = _mm_or_ps(either, unordered);
    const V128 payload_mask = _mm_castsi128_ps(_mm_srli_epi32(_mm_castps_si128(unordered), 10));
    return _mm_andnot_ps(payload_mask, quieted);
}
inline V128 fmax(V128 a, V128 b) {
    const V128 high = _mm_max_ps(a, b);
    const V128 low = _mm_max_ps(b, a);
    const V128 differ = _mm_xor_ps(high, low);
    const V128 merged = _mm_sub_ps(_mm_or_ps(high, differ), differ);
    const V128 unordered = _mm_cmpunord_ps(merged, merged);
    const V128 payload_mask = _mm_castsi128_ps(_mm_srli_epi32(_mm_castps_si128(unordered), 10));
    return _mm_andnot_ps(payload_mask, merged);
}
inline V128 neg(V128 a) { return _mm_xor_ps(a, _mm_set1_ps(-0.0f)); }
inline V128 abs(V128 a) { return _mm_andnot_ps(_mm_set1_ps(-0.0f), a); }
inline V128 sqrt(V128 a) { return _mm_sqrt_ps(a); }

template <int I>
inline float lane(V128 v) {
    return _mm_cvtss_f32(_mm_shuffle_ps(v, v, _MM_SHUFFLE(I, I, I, I)));
}

template <int I>
inline V128 set_lane(V128 v, float s) {
    const V128 lane_mask = _mm_castsi128_ps(_mm_setr_epi32(I == 0 ? -1 : 0, I == 1 ? -1 : 0, I == 2 ? -1 : 0, I == 3 ? -1 : 0));
    return _mm_or_ps(_mm_andnot_ps(lane_mask, v), _mm_and_ps(lane_mask, _mm_set1_ps(s)));
}

template <int X, int Y, int Z, int W>
inline V128 shuffle(V128 a) {
    return _mm_shuffle_ps(a, a, _MM_SHUFFLE(W, Z, Y, X));
}

template <int X, int Y, int Z, int W>
inline V128 shuffle2(V128 a, V128 b) {
#ifdef TUMBLE_HAS_SHUFFLEVECTOR
    return __builtin_shufflevector(a, b, X, Y, Z, W);
#else
    const V128 from_a = shuffle<X & 3, Y & 3, Z & 3, W & 3>(a);
    const V128 from_b = shuffle<X & 3, Y & 3, Z & 3, W & 3>(b);
    const V128 take_a = _mm_castsi128_ps(_mm_setr_epi32(X < 4 ? -1 : 0, Y < 4 ? -1 : 0, Z < 4 ? -1 : 0, W < 4 ? -1 : 0));
    return _mm_or_ps(_mm_and_ps(take_a, from_a), _mm_andnot_ps(take_a, from_b));
#endif
}

inline V128 lt(V128 a, V128 b) { return _mm_cmplt_ps(a, b); }
inline V128 le(V128 a, V128 b) { return _mm_cmple_ps(a, b); }
inline V128 gt(V128 a, V128 b) { return _mm_cmpgt_ps(a, b); }
inline V128 ge(V128 a, V128 b) { return _mm_cmpge_ps(a, b); }
inline V128 eq(V128 a, V128 b) { return _mm_cmpeq_ps(a, b); }

inline V128 bit_and(V128 a, V128 b) { return _mm_and_ps(a, b); }
inline V128 bit_or(V128 a, V128 b) { return _mm_or_ps(a, b); }
inline V128 bit_xor(V128 a, V128 b) { return _mm_xor_ps(a, b); }

inline V128 select(V128 a, V128 b, V128 mask) { return _mm_or_ps(_mm_and_ps(mask, a), _mm_andnot_ps(mask, b)); }

inline bool all_true(V128 mask) {
    const __m128i lane_is_zero = _mm_cmpeq_epi32(_mm_castps_si128(mask), _mm_setzero_si128());
    return _mm_movemask_ps(_mm_castsi128_ps(lane_is_zero)) == 0;
}
inline bool any_true(V128 mask) {
    const __m128i byte_is_zero = _mm_cmpeq_epi8(_mm_castps_si128(mask), _mm_setzero_si128());
    return _mm_movemask_epi8(byte_is_zero) != 0xFFFF;
}

inline uint32_t bitmask(V128 mask) { return (uint32_t)_mm_movemask_ps(mask); }

#elif defined(TUMBLE_USE_NEON)

inline V128 load3(const float *p) {
    const V128 xy = vcombine_f32(vld1_f32(p), vdup_n_f32(0.0f));
    return vsetq_lane_f32(p[2], xy, 2);
}

inline V128 load3_xyzz(const float *p) {
    const V128 xy = vcombine_f32(vld1_f32(p), vdup_n_f32(0.0f));
    return vsetq_lane_f32(p[2], vsetq_lane_f32(p[2], xy, 2), 3);
}

inline void store3(float *p, V128 v) {
    vst1_f32(p, vget_low_f32(v));
    vst1q_lane_f32(p + 2, v, 2);
}

inline V128 load4(const float *p) { return vld1q_f32(p); }
inline void store4(float *p, V128 v) { vst1q_f32(p, v); }

inline V128 make(float x, float y, float z, float w) {
    const float lanes[4] = {x, y, z, w};
    return vld1q_f32(lanes);
}
inline V128 splat(float s) { return vdupq_n_f32(s); }
inline V128 zero() { return vdupq_n_f32(0.0f); }

inline V128 add(V128 a, V128 b) { return vaddq_f32(a, b); }
inline V128 sub(V128 a, V128 b) { return vsubq_f32(a, b); }
inline V128 mul(V128 a, V128 b) { return vmulq_f32(a, b); }
inline V128 div(V128 a, V128 b) { return vdivq_f32(a, b); }
inline V128 min(V128 a, V128 b) { return vbslq_f32(vcltq_f32(b, a), b, a); }
inline V128 max(V128 a, V128 b) { return vbslq_f32(vcltq_f32(a, b), b, a); }
inline V128 fmin(V128 a, V128 b) { return vminq_f32(a, b); }
inline V128 fmax(V128 a, V128 b) { return vmaxq_f32(a, b); }
inline V128 neg(V128 a) { return vnegq_f32(a); }
inline V128 abs(V128 a) { return vabsq_f32(a); }
inline V128 sqrt(V128 a) { return vsqrtq_f32(a); }

template <int I>
inline float lane(V128 v) {
    return vgetq_lane_f32(v, I);
}

template <int I>
inline V128 set_lane(V128 v, float s) {
    return vsetq_lane_f32(s, v, I);
}

template <int X, int Y, int Z, int W>
inline V128 shuffle(V128 a) {
#ifdef TUMBLE_HAS_SHUFFLEVECTOR
    return __builtin_shufflevector(a, a, X, Y, Z, W);
#else
    return vsetq_lane_f32(lane<W>(a), vsetq_lane_f32(lane<Z>(a), vsetq_lane_f32(lane<Y>(a), vdupq_n_f32(lane<X>(a)), 1), 2), 3);
#endif
}

template <int X, int Y, int Z, int W>
inline V128 shuffle2(V128 a, V128 b) {
#ifdef TUMBLE_HAS_SHUFFLEVECTOR
    return __builtin_shufflevector(a, b, X, Y, Z, W);
#else
    const float both[8] = {lane<0>(a), lane<1>(a), lane<2>(a), lane<3>(a), lane<0>(b), lane<1>(b), lane<2>(b), lane<3>(b)};
    return make(both[X], both[Y], both[Z], both[W]);
#endif
}

inline V128 lt(V128 a, V128 b) { return vreinterpretq_f32_u32(vcltq_f32(a, b)); }
inline V128 le(V128 a, V128 b) { return vreinterpretq_f32_u32(vcleq_f32(a, b)); }
inline V128 gt(V128 a, V128 b) { return vreinterpretq_f32_u32(vcgtq_f32(a, b)); }
inline V128 ge(V128 a, V128 b) { return vreinterpretq_f32_u32(vcgeq_f32(a, b)); }
inline V128 eq(V128 a, V128 b) { return vreinterpretq_f32_u32(vceqq_f32(a, b)); }

inline V128 bit_and(V128 a, V128 b) { return vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(a), vreinterpretq_u32_f32(b))); }
inline V128 bit_or(V128 a, V128 b) { return vreinterpretq_f32_u32(vorrq_u32(vreinterpretq_u32_f32(a), vreinterpretq_u32_f32(b))); }
inline V128 bit_xor(V128 a, V128 b) { return vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(a), vreinterpretq_u32_f32(b))); }

inline V128 select(V128 a, V128 b, V128 mask) { return vbslq_f32(vreinterpretq_u32_f32(mask), a, b); }

inline bool all_true(V128 mask) { return vminvq_u32(vreinterpretq_u32_f32(mask)) != 0; }
inline bool any_true(V128 mask) { return vmaxvq_u32(vreinterpretq_u32_f32(mask)) != 0; }

inline uint32_t bitmask(V128 mask) {
    const uint32_t weights[4] = {1, 2, 4, 8};
    const uint32x4_t sign_bits = vshrq_n_u32(vreinterpretq_u32_f32(mask), 31);
    return vaddvq_u32(vmulq_u32(sign_bits, vld1q_u32(weights)));
}

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
#elif defined(TUMBLE_USE_SSE2)
inline V128 make(int32_t x, int32_t y, int32_t z, int32_t w) { return _mm_castsi128_ps(_mm_setr_epi32(x, y, z, w)); }
inline V128 splat(int32_t s) { return _mm_castsi128_ps(_mm_set1_epi32(s)); }
inline V128 add(V128 a, V128 b) { return _mm_castsi128_ps(_mm_add_epi32(_mm_castps_si128(a), _mm_castps_si128(b))); }
template <int I>
inline int32_t lane(V128 v) {
    return _mm_cvtsi128_si32(_mm_shuffle_epi32(_mm_castps_si128(v), _MM_SHUFFLE(I, I, I, I)));
}
#elif defined(TUMBLE_USE_NEON)
inline V128 make(int32_t x, int32_t y, int32_t z, int32_t w) {
    const int32_t lanes[4] = {x, y, z, w};
    return vreinterpretq_f32_s32(vld1q_s32(lanes));
}
inline V128 splat(int32_t s) { return vreinterpretq_f32_s32(vdupq_n_s32(s)); }
inline V128 add(V128 a, V128 b) { return vreinterpretq_f32_s32(vaddq_s32(vreinterpretq_s32_f32(a), vreinterpretq_s32_f32(b))); }
template <int I>
inline int32_t lane(V128 v) {
    return vgetq_lane_s32(vreinterpretq_s32_f32(v), I);
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

#undef TUMBLE_HAS_SHUFFLEVECTOR
