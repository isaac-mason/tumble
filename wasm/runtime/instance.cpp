#include "wasm/runtime/instance.h"

#include "tumble/math/simd.h"
#include "tumble/util/alloc.h"
#include "tumble/util/assert.h"
#include "tumble/util/error.h"
#include "wasm/runtime/heap.h"

TUMBLE_NAMESPACE_BEGIN

namespace runtime {

static void *allocate(size_t size, size_t alignment, void *) {
    TUMBLE_ASSERT(alignment <= util::MIN_ALIGNMENT);
    return heap_allocate((uint32_t)size);
}

static void free(void *block, size_t, void *) { heap_free(block); }

bool init(uint32_t heap_begin) {
    util::error_clear();
    if (!heap_init(heap_begin)) return false;
    util::set_allocator({allocate, free, nullptr});
    return true;
}

uint32_t abi_version() { return 1; }

float simd_check() {
    const float a[3] = {1.0f, 2.0f, 3.0f};
    const v128_t v = v4f::load3(a);
    return v4f::dot3(v, v);
}

}  // namespace runtime

TUMBLE_NAMESPACE_END
