#pragma once

#include <stdint.h>
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace scalar {

constexpr float PI = 3.14159265358979323846f;
constexpr float DEGREES_TO_RADIANS = PI / 180.0f;
constexpr float FLOAT_MAX = __FLT_MAX__;
constexpr float INF = __builtin_inff();
constexpr float FLOAT_MIN_NORMAL = __FLT_MIN__;
constexpr float FLOAT_EPSILON = __FLT_EPSILON__;

inline float sqrt(float x) { return __builtin_sqrtf(x); }
inline float abs(float x) { return __builtin_fabsf(x); }
inline float floor(float x) { return __builtin_floorf(x); }
// nan propagates and -0 orders below +0; fminf / fmaxf would drop the nan
#ifdef __wasm__
inline float min(float a, float b) { return __builtin_wasm_min_f32(a, b); }
inline float max(float a, float b) { return __builtin_wasm_max_f32(a, b); }
#else
inline float min(float a, float b) {
    if (a != a || b != b) return a + b;
    if (a == b) return __builtin_bit_cast(uint32_t, a) >> 31 ? a : b;
    return a < b ? a : b;
}
inline float max(float a, float b) {
    if (a != a || b != b) return a + b;
    if (a == b) return __builtin_bit_cast(uint32_t, a) >> 31 ? b : a;
    return a < b ? b : a;
}
#endif
inline float sign(float x) { return x > 0.0f ? 1.0f : (x < 0.0f ? -1.0f : 0.0f); }

// musl's, the same bits on every host
float sin(float x);
float cos(float x);
float atan(float x);
float atan2(float y, float x);
float acos(float x);

inline float clamp(float value, float lo, float hi) { return max(lo, min(hi, value)); }

}  // namespace scalar

TUMBLE_NAMESPACE_END
