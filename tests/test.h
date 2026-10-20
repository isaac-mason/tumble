#pragma once

// the core's test harness: CHECK records a failure into a fixed table that run.mjs reads back.
// no registration machinery; main.cpp lists every test function by hand.

#include <stdint.h>

struct TestFailure {
    const char *file;
    uint32_t line;
    const char *expr;
    const char *test;
};

struct TestCtx {
    const char *current;
    uint32_t checks;
    uint32_t failures;
};

/** the first byte above the core's own data and stack, from the linker; the tests put their layout here. */
extern "C" unsigned char __heap_base[];
inline uint32_t test_heap_begin() { return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(__heap_base)); }

constexpr uint32_t TEST_MAX_FAILURES = 256;

void test_record(TestCtx &ctx, const char *file, uint32_t line, const char *expr);

inline void test_check(TestCtx &ctx, bool ok, const char *file, uint32_t line, const char *expr) {
    ctx.checks++;
    if (!ok) test_record(ctx, file, line, expr);
}

#define CHECK(ctx, cond) test_check(ctx, (cond), __FILE__, __LINE__, #cond)
#define CHECK_NEAR(ctx, a, b, eps) test_check(ctx, __builtin_fabsf((float)(a) - (float)(b)) <= (float)(eps), __FILE__, __LINE__, #a " ~= " #b)
