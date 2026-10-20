#include "harness.h"

// every `void test_<name>(TestCtx &)` in tests/**/*.test.cpp, listed by the build into test_list.inc: once as a
// declaration, once in the table, once as a name.
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

uint32_t test_count() { return sizeof(TESTS) / sizeof(TESTS[0]); }

const char *test_name(uint32_t index) { return TEST_NAMES[index] + 5; }

const TestCtx &test_run(uint32_t index) {
    ctx = {test_name(index), 0, 0};
    TESTS[index](ctx);
    return ctx;
}

const TestFailure *test_failures() { return failures; }
