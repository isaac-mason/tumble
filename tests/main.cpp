#include "test.h"

#include "wasm/runtime/export.h"
#include "wasm/heap_begin.h"
#include "wasm/runtime/instance.h"

// every `void test_<name>(TestCtx &)` in test/*.test.cpp, listed by build-test.sh into the build
// dir's test_list.inc: once as a declaration, once in the table, once as a name.
#define TEST(fn) void fn(TestCtx &ctx);
#include "test_list.inc"
#undef TEST

static TestFailure failures[TEST_MAX_FAILURES];
static TestCtx ctx;

void test_record(TestCtx &c, const char *file, uint32_t line, const char *expr) {
    if (c.failures < TEST_MAX_FAILURES) failures[c.failures] = {file, line, expr, c.current};
    c.failures++;
}

using TestFn = void (*)(TestCtx &);

#define TEST(fn) fn,
static const TestFn TESTS[] = {
#include "test_list.inc"
};
#undef TEST

#define TEST(fn) #fn,
static const char *const TEST_NAMES[] = {
#include "test_list.inc"
};
#undef TEST

WASM_EXPORT uint32_t test_count() { return sizeof(TESTS) / sizeof(TESTS[0]); }

/** runs test `index` under its name less the test_ prefix, over a fresh heap; run.mjs gives each test an instance of its own. */
WASM_EXPORT uint32_t test_run_one(uint32_t index) {
    ctx = {TEST_NAMES[index] + 5, 0, 0};
    CHECK(ctx, runtime::init(heap_begin()));
    TESTS[index](ctx);
    return ctx.failures;
}

WASM_EXPORT uint32_t test_checks() { return ctx.checks; }
WASM_EXPORT uint32_t test_failure_count() { return ctx.failures < TEST_MAX_FAILURES ? ctx.failures : TEST_MAX_FAILURES; }
WASM_EXPORT const TestFailure *test_failures() { return failures; }
