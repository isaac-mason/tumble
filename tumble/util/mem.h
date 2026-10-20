#pragma once

// byte-level helpers over the one linear memory. the builtins lower to memory.copy / memory.fill.

#include <stdint.h>
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace util {

#ifdef WASM_NATIVE
// the native profiling build (build-native.sh): the linear memory is a reserved range at native_memory_base, and an
// address is an offset into it, 0 still null.
extern "C" uintptr_t native_memory_base;

inline void *at(uint32_t address) { return address == 0 ? nullptr : reinterpret_cast<void *>(native_memory_base + address); }

template <typename T>
inline T *ptr(uint32_t address) {
    return static_cast<T *>(at(address));
}

inline uint32_t address_of(const void *p) {
    return p == nullptr ? 0 : static_cast<uint32_t>(reinterpret_cast<uintptr_t>(p) - native_memory_base);
}
#else
inline void *at(uint32_t address) { return reinterpret_cast<void *>(static_cast<uintptr_t>(address)); }

template <typename T>
inline T *ptr(uint32_t address) {
    return reinterpret_cast<T *>(static_cast<uintptr_t>(address));
}

inline uint32_t address_of(const void *p) { return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(p)); }
#endif

inline void zero(void *p, uint32_t bytes) { __builtin_memset(p, 0, bytes); }
inline void copy(void *dst, const void *src, uint32_t bytes) { __builtin_memcpy(dst, src, bytes); }

inline uint32_t align_up(uint32_t value, uint32_t alignment) { return (value + alignment - 1) & ~(alignment - 1); }

}  // namespace util

TUMBLE_NAMESPACE_END
