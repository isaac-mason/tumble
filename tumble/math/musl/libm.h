// stand-in for musl's src/internal/libm.h, trimmed to what the vendored float sources use.
// wasm has no fenv, so FORCE_EVAL is empty (as it is under __EMSCRIPTEN__ upstream).
#ifndef BONGLE_MUSL_LIBM_H
#define BONGLE_MUSL_LIBM_H

#include <float.h>
#include <stdint.h>

#include "features.h"
#include "math.h"

#define WANT_ROUNDING 1
#define WANT_SNAN 0
#define issignalingf_inline(x) 0
#define issignaling_inline(x) 0
#define TOINT_INTRINSICS 0

#define predict_true(x) __builtin_expect(!!(x), 1)
#define predict_false(x) __builtin_expect(x, 0)

static inline float eval_as_float(float x) {
    float y = x;
    return y;
}

static inline double eval_as_double(double x) {
    double y = x;
    return y;
}

static inline float fp_barrierf(float x) {
    volatile float y = x;
    return y;
}

static inline double fp_barrier(double x) {
    volatile double y = x;
    return y;
}

static inline void fp_force_evalf(float x) {
    volatile float y;
    y = x;
    (void)y;
}

static inline void fp_force_eval(double x) {
    volatile double y;
    y = x;
    (void)y;
}

#define FORCE_EVAL(x)

#define asuint(f) ((union { float _f; uint32_t _i; }){f})._i
#define asfloat(i) ((union { uint32_t _i; float _f; }){i})._f
#define asuint64(f) ((union { double _f; uint64_t _i; }){f})._i
#define asdouble(i) ((union { uint64_t _i; double _f; }){i})._f

#define EXTRACT_WORDS(hi, lo, d)      \
    do {                              \
        uint64_t __u = asuint64(d);   \
        (hi) = __u >> 32;             \
        (lo) = (uint32_t)__u;         \
    } while (0)

#define GET_HIGH_WORD(hi, d)          \
    do {                              \
        (hi) = asuint64(d) >> 32;     \
    } while (0)

#define GET_LOW_WORD(lo, d)           \
    do {                              \
        (lo) = (uint32_t)asuint64(d); \
    } while (0)

#define INSERT_WORDS(d, hi, lo)                                     \
    do {                                                            \
        (d) = asdouble(((uint64_t)(hi) << 32) | (uint32_t)(lo));    \
    } while (0)

#define SET_HIGH_WORD(d, hi) INSERT_WORDS(d, hi, (uint32_t)asuint64(d))
#define SET_LOW_WORD(d, lo) INSERT_WORDS(d, asuint64(d) >> 32, lo)

#define GET_FLOAT_WORD(w, d) \
    do {                     \
        (w) = asuint(d);     \
    } while (0)

#define SET_FLOAT_WORD(d, w) \
    do {                     \
        (d) = asfloat(w);    \
    } while (0)

hidden int __rem_pio2_large(double *, double *, int, int, int);
hidden int __rem_pio2f(float, double *);
hidden float __sindf(double);
hidden float __cosdf(double);
hidden float __tandf(double, int);

hidden float __math_xflowf(uint32_t, float);
hidden float __math_uflowf(uint32_t);
hidden float __math_oflowf(uint32_t);
hidden float __math_divzerof(uint32_t);
hidden float __math_invalidf(float);

#endif
