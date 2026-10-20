#pragma once

#include <stddef.h>
#include <stdint.h>
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace util {

inline void zero(void *p, size_t bytes) { __builtin_memset(p, 0, bytes); }
inline void copy(void *dst, const void *src, size_t bytes) { __builtin_memcpy(dst, src, bytes); }

/** alignment is a power of two */
inline size_t align_up(size_t value, size_t alignment) { return (value + alignment - 1) & ~(alignment - 1); }

}  // namespace util

TUMBLE_NAMESPACE_END
