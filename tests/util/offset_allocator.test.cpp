#include "tumble/instance.h"
#include "test.h"
#include "tumble/util/offset_allocator.h"

using OffsetAllocator::Allocation;
using OffsetAllocator::Allocator;

void test_offset_allocator_largest_free_region_below(TestCtx &ctx) {
    CHECK(ctx, util::init(test_heap_begin()));
    Allocator allocator(1000, 16);
    CHECK(ctx, allocator.largestFreeRegionBelow(1000) == 1000);
    CHECK(ctx, allocator.largestFreeRegionBelow(10) == 10);

    // [0, 100) used, [100, 400) free, [400, 500) used, [500, 1000) free
    const Allocation first = allocator.allocate(100);
    const Allocation middle = allocator.allocate(300);
    const Allocation last = allocator.allocate(100);
    CHECK(ctx, first.offset == 0 && middle.offset == 100 && last.offset == 400);
    allocator.free(middle);

    CHECK(ctx, allocator.largestFreeRegionBelow(1000) == 500);
    // the end region cut short is smaller than the hole
    CHECK(ctx, allocator.largestFreeRegionBelow(600) == 300);
    CHECK(ctx, allocator.largestFreeRegionBelow(800) == 300);
    CHECK(ctx, allocator.largestFreeRegionBelow(801) == 301);
    // an end inside the hole cuts the hole
    CHECK(ctx, allocator.largestFreeRegionBelow(250) == 150);
    // nothing free below the first block's end
    CHECK(ctx, allocator.largestFreeRegionBelow(100) == 0);
    CHECK(ctx, allocator.largestFreeRegionBelow(0) == 0);

    allocator.free(first);
    allocator.free(last);
    CHECK(ctx, allocator.largestFreeRegionBelow(1000) == 1000);
    allocator.release();
}
