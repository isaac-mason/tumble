#pragma once

// the math package's scalar.ts and angle.ts, plus the transcendentals: wasm instructions through
// builtins, and musl's single-precision routines (src/math/musl) for the rest.

#include <stdint.h>

extern "C" {
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
}

namespace scalar {

constexpr float EPSILON = 0.000001f;
constexpr float PI = 3.14159265358979323846f;
constexpr float TAU = 6.28318530717958647692f;
constexpr float DEGREES_TO_RADIANS = PI / 180.0f;
constexpr float RADIANS_TO_DEGREES = 180.0f / PI;
constexpr float FLOAT_MAX = __FLT_MAX__;
constexpr float INF = __builtin_inff();
/** the smallest positive normal float (c's and jolt's FLT_MIN) */
constexpr float FLOAT_MIN_NORMAL = __FLT_MIN__;
constexpr float FLOAT_EPSILON = __FLT_EPSILON__;

inline float sqrt(float x) { return __builtin_sqrtf(x); }
inline float abs(float x) { return __builtin_fabsf(x); }
inline float floor(float x) { return __builtin_floorf(x); }
inline float ceil(float x) { return __builtin_ceilf(x); }
inline float trunc(float x) { return __builtin_truncf(x); }
/** symmetric round: halves go away from zero, as scalar.ts's `round` does. */
inline float round(float x) { return __builtin_roundf(x); }
// the f32.min / f32.max instructions: nan propagates and -0 orders below +0, exactly js Math.min / Math.max.
// (c's fminf / fmaxf drop nans, so clang would call a libm that isn't there.)
inline float min(float a, float b) { return __builtin_wasm_min_f32(a, b); }
inline float max(float a, float b) { return __builtin_wasm_max_f32(a, b); }
inline float sign(float x) { return x > 0.0f ? 1.0f : (x < 0.0f ? -1.0f : 0.0f); }
inline bool is_nan(float x) { return __builtin_isnan(x); }
inline bool is_finite(float x) { return __builtin_isfinite(x); }

inline float sin(float x) { return sinf(x); }
inline float cos(float x) { return cosf(x); }
inline float tan(float x) { return tanf(x); }
inline float atan(float x) { return atanf(x); }
inline float atan2(float y, float x) { return atan2f(y, x); }
inline float acos(float x) { return acosf(x); }
inline float asin(float x) { return asinf(x); }
inline float exp(float x) { return expf(x); }
inline float pow(float x, float y) { return powf(x, y); }
inline float log(float x) { return logf(x); }
/** truncated remainder, the sign of x, as js `%`. */
inline float fmod(float x, float y) { return fmodf(x, y); }

/** |a - b| within epsilon, absolute at or below 1 and relative above. */
inline bool equals(float a, float b, float epsilon = EPSILON) {
    return abs(a - b) <= epsilon * max(1.0f, max(abs(a), abs(b)));
}

/** ease-in-out that leaves [0, 1] outside it. */
inline float fade(float t) { return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f); }

inline float lerp(float v0, float v1, float t) { return v0 * (1.0f - t) + v1 * t; }

/** quadratic through v0 at t=0, v1 at t=0.5, v2 at t=1. */
inline float lagrange(float v0, float v1, float v2, float t) {
    return 2.0f * (t - 1.0f) * (t - 0.5f) * v0 - 4.0f * (t - 1.0f) * t * v1 + 2.0f * (t - 0.5f) * t * v2;
}

/** n choose k, 0 when k is out of range. */
inline float binomial(int32_t n, int32_t k) {
    if (k < 0 || k > n) return 0.0f;
    const int32_t j = k < n - k ? k : n - k;
    float coeff = 1.0f;
    for (int32_t i = 0; i < j; i++) coeff = (coeff * (float)(n - i)) / (float)(i + 1);
    return round(coeff);
}

inline float clamp(float value, float lo, float hi) { return max(lo, min(hi, value)); }

/** floored modulo: wraps t into [0, length), so repeat(-1, 3) = 2. */
inline float repeat(float t, float length) { return fmod(fmod(t, length) + length, length); }

inline float remap(float x, float in_low, float in_high, float out_low, float out_high) {
    const float scale = (x - in_low) / (in_high - in_low);
    return out_low + scale * (out_high - out_low);
}

inline float remap_clamp(float x, float in_low, float in_high, float out_low, float out_high) {
    const float scale = (x - in_low) / (in_high - in_low);
    const float remapped = out_low + scale * (out_high - out_low);
    return max(out_low, min(out_high, remapped));
}

/** glsl smoothstep; equal edges act as a step at edge0. */
inline float smoothstep(float edge0, float edge1, float x) {
    if (edge0 == edge1) return x <= edge0 ? 0.0f : 1.0f;
    const float t = clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

/** perlin's smootherstep: `fade`, clamped; equal edges act as a step at edge0. */
inline float smootherstep(float edge0, float edge1, float x) {
    if (edge0 == edge1) return x <= edge0 ? 0.0f : 1.0f;
    return fade(clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f));
}

inline float degrees_to_radians(float degrees) { return degrees * DEGREES_TO_RADIANS; }
inline float radians_to_degrees(float radians) { return radians * RADIANS_TO_DEGREES; }

/** wraps an angle into (-pi, pi]. */
inline float wrap_angle(float a) { return a - TAU * floor((a + PI) / TAU); }

/** the shortest signed difference from current to target, in (-pi, pi]. */
inline float delta_angle(float current, float target) {
    const float diff = target - current;
    const float delta = diff - TAU * floor(diff / TAU);
    return delta > PI ? delta - TAU : delta;
}

// ---- f64 ----

/** the f64.min / f64.max instructions: nan propagates and -0 orders below +0. */
inline double min64(double a, double b) { return __builtin_wasm_min_f64(a, b); }
inline double max64(double a, double b) { return __builtin_wasm_max_f64(a, b); }

}  // namespace scalar
