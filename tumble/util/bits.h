#pragma once

#include <stdint.h>
#include "tumble/tumble.h"

#if defined(_MSC_VER) && !defined(__clang__)
#include <intrin.h>
#endif

TUMBLE_NAMESPACE_BEGIN

namespace bits {

/** value is not 0 */
inline uint32_t count_leading_zeros(uint32_t value) {
#if defined(_MSC_VER) && !defined(__clang__)
    unsigned long index;
    _BitScanReverse(&index, value);
    return 31 - index;
#else
    return (uint32_t)__builtin_clz(value);
#endif
}

/** value is not 0 */
inline uint32_t count_trailing_zeros(uint32_t value) {
#if defined(_MSC_VER) && !defined(__clang__)
    unsigned long index;
    _BitScanForward(&index, value);
    return index;
#else
    return (uint32_t)__builtin_ctz(value);
#endif
}

inline uint32_t count_ones(uint32_t value) {
#if defined(_MSC_VER) && !defined(__clang__)
    // __popcnt needs a cpu with popcnt
    value = value - ((value >> 1) & 0x55555555u);
    value = (value & 0x33333333u) + ((value >> 2) & 0x33333333u);
    return (((value + (value >> 4)) & 0x0f0f0f0fu) * 0x01010101u) >> 24;
#else
    return (uint32_t)__builtin_popcount(value);
#endif
}

}  // namespace bits

TUMBLE_NAMESPACE_END
