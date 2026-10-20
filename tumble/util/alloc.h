#pragma once

#include <stddef.h>
#include <stdint.h>

#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace util {

/** a block of `size` bytes aligned to `alignment` (a power of two), or null */
using AllocateFunction = void *(*)(size_t size, size_t alignment, void *user);
/** a block allocate returned, with the size it was asked for; never null */
using FreeFunction = void (*)(void *block, size_t size, void *user);

struct Allocator {
    AllocateFunction allocate;
    FreeFunction free;
    void *user;
};

/**
 * before anything allocates. a hosted build starts with aligned malloc and free; a freestanding one starts with none,
 * and every allocation fails until one is set
 */
void set_allocator(const Allocator &allocator);

/** what the library holds */
size_t allocated_bytes();
size_t allocated_blocks();

/** every block is at least this aligned, for simd */
constexpr size_t MIN_ALIGNMENT = 16;

/** null (error set) when the allocator fails; null for zero bytes */
void *alloc_bytes(size_t bytes, size_t alignment = MIN_ALIGNMENT);
/** the size alloc_bytes was asked for; null is fine */
void free_bytes(void *block, size_t bytes);

/** `count` objects' worth, uninitialised; null (error set) when the allocator fails */
template <typename T>
inline T *alloc(size_t count = 1) {
    return static_cast<T *>(alloc_bytes(count * sizeof(T), alignof(T) > MIN_ALIGNMENT ? alignof(T) : MIN_ALIGNMENT));
}

/** the count alloc was asked for; null is fine */
template <typename T>
inline void free(T *block, size_t count = 1) {
    free_bytes(block, count * sizeof(T));
}

}  // namespace util

TUMBLE_NAMESPACE_END
