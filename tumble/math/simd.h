#pragma once

// the thin names every kernel writes simd through. one build, always -msimd128, no scalar twin.
// lane 3 of a three-wide value is whatever the load left there; nothing may read it as data.

#include <wasm_simd128.h>

namespace v4f {

/** three floats from a 12-byte struct into lanes 0..2, lane 3 zero. two loads, no overread. */
inline v128_t load3(const float *p) {
    v128_t v = wasm_v128_load64_zero(p);
    return wasm_v128_load32_lane(p + 2, v, 2);
}

/** lanes 0..2 into a 12-byte struct. two stores, no overwrite. */
inline void store3(float *p, v128_t v) {
    wasm_v128_store64_lane(p, v, 0);
    wasm_v128_store32_lane(p + 2, v, 2);
}

inline v128_t load4(const float *p) { return wasm_v128_load(p); }
inline void store4(float *p, v128_t v) { wasm_v128_store(p, v); }

inline v128_t make(float x, float y, float z, float w) { return wasm_f32x4_make(x, y, z, w); }
inline v128_t splat(float s) { return wasm_f32x4_splat(s); }
inline v128_t zero() { return wasm_f32x4_const_splat(0.0f); }

inline v128_t add(v128_t a, v128_t b) { return wasm_f32x4_add(a, b); }
inline v128_t sub(v128_t a, v128_t b) { return wasm_f32x4_sub(a, b); }
inline v128_t mul(v128_t a, v128_t b) { return wasm_f32x4_mul(a, b); }
inline v128_t div(v128_t a, v128_t b) { return wasm_f32x4_div(a, b); }
inline v128_t min(v128_t a, v128_t b) { return wasm_f32x4_pmin(a, b); }
inline v128_t max(v128_t a, v128_t b) { return wasm_f32x4_pmax(a, b); }
/** lane by lane as scalar::min, f32.min: nan in either lane gives nan, -0 below +0. */
inline v128_t fmin(v128_t a, v128_t b) { return wasm_f32x4_min(a, b); }
/** lane by lane as scalar::max, f32.max. */
inline v128_t fmax(v128_t a, v128_t b) { return wasm_f32x4_max(a, b); }
inline v128_t neg(v128_t a) { return wasm_f32x4_neg(a); }
inline v128_t abs(v128_t a) { return wasm_f32x4_abs(a); }
/** lanes 0..2 negated, lane 3 kept. */
inline v128_t neg_xyz(v128_t a) { return wasm_v128_xor(a, wasm_f32x4_make(-0.0f, -0.0f, -0.0f, 0.0f)); }
inline v128_t sqrt(v128_t a) { return wasm_f32x4_sqrt(a); }

/** a * b + c as two instructions; wasm core simd has no fma. */
inline v128_t mul_add(v128_t a, v128_t b, v128_t c) { return wasm_f32x4_add(wasm_f32x4_mul(a, b), c); }

template <int I>
inline float lane(v128_t v) {
    return wasm_f32x4_extract_lane(v, I);
}

template <int I>
inline v128_t set_lane(v128_t v, float s) {
    return wasm_f32x4_replace_lane(v, I, s);
}

/** lanes (a[X], a[Y], a[Z], a[W]); X..W index into a alone. */
template <int X, int Y, int Z, int W>
inline v128_t shuffle(v128_t a) {
    return wasm_i32x4_shuffle(a, a, X, Y, Z, W);
}

/** lanes drawn from both: 0..3 index a, 4..7 index b. */
template <int X, int Y, int Z, int W>
inline v128_t shuffle2(v128_t a, v128_t b) {
    return wasm_i32x4_shuffle(a, b, X, Y, Z, W);
}

/** four packed xyz points (12 floats) into one lane each of x, y and z. three loads, no overread. */
inline void load3x4(const float *p, v128_t &x, v128_t &y, v128_t &z) {
    const v128_t a = load4(p);      // x0 y0 z0 x1
    const v128_t b = load4(p + 4);  // y1 z1 x2 y2
    const v128_t c = load4(p + 8);  // z2 x3 y3 z3
    x = shuffle2<0, 1, 2, 5>(shuffle2<0, 3, 6, 0>(a, b), c);
    y = shuffle2<0, 1, 2, 6>(shuffle2<1, 4, 7, 0>(a, b), c);
    z = shuffle2<0, 1, 4, 7>(shuffle2<2, 5, 0, 0>(a, b), c);
}

/** four three-wide values (one point each) into one lane each of x, y and z. */
inline void transpose3x4(v128_t r0, v128_t r1, v128_t r2, v128_t r3, v128_t &x, v128_t &y, v128_t &z) {
    const v128_t xy01 = shuffle2<0, 4, 1, 5>(r0, r1);  // x0 x1 y0 y1
    const v128_t xy23 = shuffle2<0, 4, 1, 5>(r2, r3);  // x2 x3 y2 y3
    const v128_t zw01 = shuffle2<2, 6, 3, 7>(r0, r1);  // z0 z1 w0 w1
    const v128_t zw23 = shuffle2<2, 6, 3, 7>(r2, r3);  // z2 z3 w2 w3
    x = shuffle2<0, 1, 4, 5>(xy01, xy23);
    y = shuffle2<2, 3, 6, 7>(xy01, xy23);
    z = shuffle2<0, 1, 4, 5>(zw01, zw23);
}

inline v128_t yzx(v128_t a) { return shuffle<1, 2, 0, 3>(a); }
inline v128_t zxy(v128_t a) { return shuffle<2, 0, 1, 3>(a); }

/** sum of lanes 0..2. */
inline float hsum3(v128_t v) { return lane<0>(v) + lane<1>(v) + lane<2>(v); }

/** sum of all four lanes. */
inline float hsum4(v128_t v) {
    const v128_t s = wasm_f32x4_add(v, shuffle<2, 3, 0, 1>(v));
    return lane<0>(wasm_f32x4_add(s, shuffle<1, 0, 3, 2>(s)));
}

inline float dot3(v128_t a, v128_t b) { return hsum3(wasm_f32x4_mul(a, b)); }
inline float dot4(v128_t a, v128_t b) { return hsum4(wasm_f32x4_mul(a, b)); }

/** a x b in lanes 0..2. */
inline v128_t cross(v128_t a, v128_t b) {
    return wasm_f32x4_sub(wasm_f32x4_mul(yzx(a), zxy(b)), wasm_f32x4_mul(zxy(a), yzx(b)));
}

inline v128_t lt(v128_t a, v128_t b) { return wasm_f32x4_lt(a, b); }
inline v128_t le(v128_t a, v128_t b) { return wasm_f32x4_le(a, b); }
inline v128_t gt(v128_t a, v128_t b) { return wasm_f32x4_gt(a, b); }
inline v128_t ge(v128_t a, v128_t b) { return wasm_f32x4_ge(a, b); }
inline v128_t eq(v128_t a, v128_t b) { return wasm_f32x4_eq(a, b); }

/** lanes of a where mask is set, else b. */
inline v128_t select(v128_t a, v128_t b, v128_t mask) { return wasm_v128_bitselect(a, b, mask); }

inline bool all_true(v128_t mask) { return wasm_i32x4_all_true(mask); }
inline bool any_true(v128_t mask) { return wasm_v128_any_true(mask); }

/** the lanes of a mask as the low four bits. */
inline uint32_t bitmask(v128_t mask) { return wasm_i32x4_bitmask(mask); }

}  // namespace v4f
