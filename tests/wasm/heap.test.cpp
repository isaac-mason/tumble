#include "test.h"
#include "tumble/util/error.h"
#include "wasm/heap_begin.h"
#include "wasm/runtime/heap.h"

static runtime::HeapStats stats() {
    runtime::HeapStats s;
    runtime::heap_stats(&s);
    return s;
}

static uint8_t *take(uint32_t bytes) { return static_cast<uint8_t *>(runtime::heap_allocate(bytes)); }

static uint32_t address_of(const void *p) { return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(p)); }

void test_heap(TestCtx &ctx) {
    CHECK(ctx, runtime::heap_init(heap_begin()));
    CHECK(ctx, util::error() == Error::NONE);
    CHECK(ctx, stats().bytes_in_use == 0 && stats().allocations == 0);

    // blocks are 16-aligned, distinct, and sized as asked
    uint8_t *a = take(100);
    uint8_t *b = take(100);
    CHECK(ctx, a != nullptr && b != nullptr && a != b);
    CHECK(ctx, (address_of(a) & 15) == 0 && (address_of(b) & 15) == 0);
    CHECK(ctx, b >= a + 100 || a >= b + 100);
    CHECK(ctx, stats().bytes_in_use == 200 && stats().allocations == 2);

    // a freed block is reused: the same request lands in the same place
    runtime::heap_free(a);
    CHECK(ctx, take(100) == a);
    // a small block serves any request of its size class (65 to 128 bytes), the last freed first
    runtime::heap_free(a);
    runtime::heap_free(b);
    CHECK(ctx, take(128) == b && take(65) == a);
    CHECK(ctx, stats().bytes_in_use == 193 && stats().allocations == 2);
    // and a request of another class never gets it
    runtime::heap_free(a);
    uint8_t *other = take(64);
    CHECK(ctx, other != a);
    runtime::heap_free(other);
    runtime::heap_free(b);
    CHECK(ctx, stats().bytes_in_use == 0 && stats().allocations == 0);

    // neighbours coalesce: three freed blocks, each past the small classes, make room for one as big as all of them
    uint8_t *x = take(2000);
    uint8_t *y = take(2000);
    uint8_t *z = take(2000);
    runtime::heap_free(x);
    runtime::heap_free(y);
    runtime::heap_free(z);
    uint8_t *big = take(6000);
    CHECK(ctx, big == x);
    runtime::heap_free(big);

    // new and delete run over the same heap
    uint32_t *words = new uint32_t[64];
    CHECK(ctx, words != nullptr && stats().allocations == 1);
    delete[] words;
    CHECK(ctx, stats().allocations == 0);

    // the memory grows to cover what is handed out, and keeps working after
    const uint32_t before = runtime::memory_bytes();
    const uint32_t grows_before = stats().grows;
    uint8_t *huge = take(before + 1024);
    CHECK(ctx, huge != nullptr);
    CHECK(ctx, runtime::memory_bytes() >= before + 1024);
    CHECK(ctx, stats().grows > grows_before);
    huge[before] = 7;
    CHECK(ctx, huge[before] == 7);
    runtime::heap_free(huge);

    // the node tables grow through the heap itself: far more live blocks than the first tables hold
    constexpr uint32_t MANY = 10000;
    uint8_t **blocks = static_cast<uint8_t **>(runtime::heap_allocate(MANY * sizeof(uint8_t *)));
    bool all = true;
    for (uint32_t i = 0; i < MANY; i++) {
        blocks[i] = take(16 + (i % 7) * 16);
        all = all && blocks[i] != nullptr;
    }
    CHECK(ctx, all);
    CHECK(ctx, stats().allocations >= MANY);
    // free every other one so the survivors are fenced by gaps, then the rest
    for (uint32_t i = 0; i < MANY; i += 2) runtime::heap_free(blocks[i]);
    for (uint32_t i = 1; i < MANY; i += 2) runtime::heap_free(blocks[i]);
    runtime::heap_free(blocks);
    CHECK(ctx, stats().allocations == 0 && stats().bytes_in_use == 0);
    CHECK(ctx, util::error() == Error::NONE);
}

void test_heap_largest_free(TestCtx &ctx) {
    CHECK(ctx, runtime::heap_init(heap_begin()));
    // a fresh heap is one free block from the tables to the end of the address space; only its part inside the memory counts
    const runtime::HeapStats fresh = stats();
    CHECK(ctx, fresh.largest_free > 0);
    CHECK(ctx, fresh.largest_free <= fresh.memory_bytes - fresh.bytes_in_use);

    // a block past the memory's end grows it, and the free block after it ends at the new end
    constexpr uint32_t HOLE_BYTES = 4u << 20;
    uint8_t *hole = take(HOLE_BYTES);
    CHECK(ctx, hole != nullptr);
    const runtime::HeapStats grown = stats();
    CHECK(ctx, grown.memory_bytes >= address_of(hole) + HOLE_BYTES);
    CHECK(ctx, grown.largest_free == grown.memory_bytes - (address_of(hole) + HOLE_BYTES));

    // a block filling the rest of the memory exactly (its header included) leaves nothing free inside it
    uint8_t *fence = take(grown.largest_free - 16);
    CHECK(ctx, fence == hole + HOLE_BYTES + 16);
    CHECK(ctx, stats().memory_bytes == grown.memory_bytes);
    CHECK(ctx, stats().largest_free == 0);

    // freeing the first block leaves a hole in the middle, block header and all, and it is the largest
    runtime::heap_free(hole);
    CHECK(ctx, stats().largest_free == HOLE_BYTES + 16);
    CHECK(ctx, stats().largest_free <= stats().memory_bytes - stats().bytes_in_use);

    runtime::heap_free(fence);
    CHECK(ctx, stats().largest_free <= stats().memory_bytes);
    CHECK(ctx, stats().allocations == 0 && util::error() == Error::NONE);
}
