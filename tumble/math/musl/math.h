// stand-in for <math.h> for the vendored musl sources. there is no libc in the core: everything
// that is a wasm instruction is a builtin here, and the rest is declared and vendored.
#ifndef BONGLE_MUSL_MATH_H
#define BONGLE_MUSL_MATH_H

#include <stdint.h>

#define FLT_EVAL_METHOD 0
typedef float float_t;
typedef double double_t;

#define INFINITY (__builtin_inff())
#define NAN (__builtin_nanf(""))
#define M_PI 3.14159265358979323846
#define M_PI_2 1.57079632679489661923

#define isnan(x) __builtin_isnan(x)
#define isinf(x) __builtin_isinf(x)
#define isfinite(x) __builtin_isfinite(x)
#define signbit(x) __builtin_signbit(x)

static inline float fabsf(float x) { return __builtin_fabsf(x); }
static inline double fabs(double x) { return __builtin_fabs(x); }
static inline float sqrtf(float x) { return __builtin_sqrtf(x); }
static inline double sqrt(double x) { return __builtin_sqrt(x); }
static inline float floorf(float x) { return __builtin_floorf(x); }
static inline double floor(double x) { return __builtin_floor(x); }
static inline float ceilf(float x) { return __builtin_ceilf(x); }
static inline double ceil(double x) { return __builtin_ceil(x); }
static inline float rintf(float x) { return __builtin_rintf(x); }
static inline double rint(double x) { return __builtin_rint(x); }
static inline float truncf(float x) { return __builtin_truncf(x); }
static inline double trunc(double x) { return __builtin_trunc(x); }

double scalbn(double x, int n);

float sinf(float x);
float cosf(float x);
float tanf(float x);
float atanf(float x);
float atan2f(float y, float x);
float acosf(float x);
float asinf(float x);
float expf(float x);
float powf(float x, float y);
float logf(float x);
float fmodf(float x, float y);

#endif
