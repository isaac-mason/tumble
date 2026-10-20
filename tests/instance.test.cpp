#include "tumble/instance.h"
#include "test.h"
#include "tumble/util/mem.h"

void test_instance(TestCtx &ctx) {
    // a heap that cannot fit its tables below the top of memory is refused, and the error says so
    CHECK(ctx, util::init(0xfffffff0u) == false);
    CHECK(ctx, util::error() == util::ERROR_OUT_OF_MEMORY);
    util::error_clear();
    CHECK(ctx, util::abi_version() == 1);
    CHECK(ctx, util::simd_check() == 14.0f);
}
