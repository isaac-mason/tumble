#pragma once

// the core's heap: Sebastian Aaltonen's offset allocator (util/offset_allocator.h) over the whole
// address range above the core's own data, with the wasm memory grown on demand to cover what it
// hands out. `alloc<T>(count)` and `free(p)` are the whole interface; a 16-byte header before
// each block carries what free needs, and every block is 16-aligned for simd. small blocks are
// cached per size class in front of the allocator (alloc.cpp).
//
// growth is the only way the memory's buffer changes on the js side, and only allocation grows:
// js re-derives its views after any call that can allocate.

#include <stdint.h>
#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

namespace util {

struct HeapStats {
    /** bytes callers hold, headers excluded. */
    uint32_t bytes_in_use;
    /** blocks held. */
    uint32_t allocations;
    /** the memory's size now. */
    uint32_t memory_bytes;
    /** times the memory grew. */
    uint32_t grows;
    /** the largest free block inside the memory as it is now; the free range past its end is not counted. */
    uint32_t largest_free;
};

/** starts the heap at heap_begin; false (error set) when the memory cannot hold its tables. */
bool heap_init(uint32_t heap_begin);

/** the memory's size in bytes, from the instruction. */
uint32_t memory_bytes();

void heap_stats(HeapStats *out);

/** `bytes` of 16-aligned memory, or null (error set) when the memory cannot grow to fit. */
void *alloc_bytes(uint32_t bytes);
/** gives a block from alloc_bytes back; null is fine. */
void free(void *p);

/** `count` objects' worth of memory, uninitialised, or null (error set). */
template <typename T>
inline T *alloc(uint32_t count = 1) {
    return static_cast<T *>(alloc_bytes(count * (uint32_t)sizeof(T)));
}

}  // namespace util

TUMBLE_NAMESPACE_END
