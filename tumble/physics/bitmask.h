#pragma once

// crashcat's bitmask.ts: flag helpers over a u32.

#include <stdint.h>

namespace bitmask {

constexpr uint32_t NONE_FLAG = 0;
constexpr uint32_t ALL_FLAG = 0x7fffffffu;

inline uint32_t add_flag(uint32_t flag, uint32_t value) { return flag | value; }
inline uint32_t remove_flag(uint32_t flag, uint32_t value) { return flag & ~value; }
inline uint32_t toggle_flag(uint32_t flag, uint32_t value) { return flag ^ value; }
inline bool has_flag(uint32_t flag, uint32_t value) { return (flag & value) != 0; }
inline bool does_not_have_flag(uint32_t flag, uint32_t value) { return (flag & value) == 0; }

}  // namespace bitmask
