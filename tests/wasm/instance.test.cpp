#include "test.h"
#include "tumble/util/alloc.h"
#include "tumble/util/error.h"
#include "wasm/runtime/heap.h"
#include "wasm/runtime/instance.h"
#include "wasm/heap_begin.h"

void test_instance(TestCtx &ctx) {
    // a heap that cannot fit its tables below the top of memory is refused, and the error says so
    CHECK(ctx, runtime::init(0xfffffff0u) == false);
    CHECK(ctx, util::error() == Error::OUT_OF_MEMORY);
    util::error_clear();
    CHECK(ctx, runtime::abi_version() == 1);
    CHECK(ctx, runtime::simd_check() == 14.0f);

    // init installs the heap as the library's allocator
    CHECK(ctx, runtime::init(heap_begin()));
    runtime::HeapStats stats;
    runtime::heap_stats(&stats);
    const uint32_t blocks_before = stats.allocations;
    float *floats = util::alloc<float>(10);
    CHECK(ctx, floats != nullptr && util::allocated_bytes() == 40 && util::allocated_blocks() == 1);
    runtime::heap_stats(&stats);
    CHECK(ctx, stats.allocations == blocks_before + 1 && stats.bytes_in_use >= 40);
    util::free(floats, 10);
    CHECK(ctx, util::allocated_bytes() == 0 && util::allocated_blocks() == 0);
}
