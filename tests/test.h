#pragma once

#include <stdint.h>

namespace tumble {}
using namespace tumble;

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

constexpr uint32_t TEST_MAX_FAILURES = 256;

void test_record(TestCtx &ctx, const char *file, uint32_t line, const char *expr);

inline void test_check(TestCtx &ctx, bool ok, const char *file, uint32_t line, const char *expr) {
    ctx.checks++;
    if (!ok) test_record(ctx, file, line, expr);
}

#define CHECK(ctx, cond) test_check(ctx, (cond), __FILE__, __LINE__, #cond)
#define CHECK_NEAR(ctx, a, b, eps) test_check(ctx, __builtin_fabsf((float)(a) - (float)(b)) <= (float)(eps), __FILE__, __LINE__, #a " ~= " #b)
