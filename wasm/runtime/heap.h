#pragma once

#include <stdint.h>

#include "tumble/tumble.h"

TUMBLE_NAMESPACE_BEGIN

// the wasm build's heap: the offset allocator over the address range above the module's own data, the memory grown on
// demand to cover what it hands out. growth is the only way the memory's buffer changes, so js re-derives its views
// after any call that can allocate
namespace runtime {

struct HeapStats {
    /** bytes callers hold, headers excluded */
    uint32_t bytes_in_use;
    /** blocks held */
    uint32_t allocations;
    /** the memory's size now */
    uint32_t memory_bytes;
    /** times the memory grew */
    uint32_t grows;
    /** the largest free block inside the memory as it is now; the free range past its end is not counted */
    uint32_t largest_free;
};

/** starts the heap at heap_begin; false (error set) when the memory cannot hold its tables */
bool heap_init(uint32_t heap_begin);

uint32_t memory_bytes();

void heap_stats(HeapStats *out);

/** 16-aligned, or null when the memory cannot grow to fit */
void *heap_allocate(uint32_t bytes);
/** null is fine */
void heap_free(void *block);

}  // namespace runtime

TUMBLE_NAMESPACE_END
