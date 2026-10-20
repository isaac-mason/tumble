#pragma once

#include <stdint.h>
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

extern "C" {
float sinf(float x);
float cosf(float x);
float atanf(float x);
float atan2f(float y, float x);
float acosf(float x);
}

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
inline float min(float a, float b) { return __builtin_wasm_min_f32(a, b); }
inline float max(float a, float b) { return __builtin_wasm_max_f32(a, b); }
inline float sign(float x) { return x > 0.0f ? 1.0f : (x < 0.0f ? -1.0f : 0.0f); }

inline float sin(float x) { return sinf(x); }
inline float cos(float x) { return cosf(x); }
inline float atan(float x) { return atanf(x); }
inline float atan2(float y, float x) { return atan2f(y, x); }
inline float acos(float x) { return acosf(x); }

inline float clamp(float value, float lo, float hi) { return max(lo, min(hi, value)); }

}  // namespace scalar

TUMBLE_NAMESPACE_END
