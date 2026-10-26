#include <stdio.h>
#include <string.h>

#include "digest.h"
#include "harness.h"
#include "util/out_of_memory.h"

// ctest runs each test in a process of its own: `tumble_tests <name>`. `--list` prints the names, `--digest` the digest,
// `--out-of-memory` traps (ctest expects it to)
int main(int argc, char **argv) {
    if (argc == 2 && strcmp(argv[1], "--out-of-memory") == 0) {
        out_of_memory_traps();
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "--list") == 0) {
        for (uint32_t i = 0; i < test_count(); i++) printf("%s\n", test_name(i));
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "--digest") == 0) {
        printf("digest: %08x\n", digest());
        return 0;
    }
    if (argc != 2) {
        fprintf(stderr, "usage: %s <test> | --list | --digest\n", argv[0]);
        return 2;
    }
    for (uint32_t i = 0; i < test_count(); i++) {
        if (strcmp(test_name(i), argv[1]) != 0) continue;
        const TestCtx &ctx = test_run(i);
        const uint32_t recorded = ctx.failures < TEST_MAX_FAILURES ? ctx.failures : TEST_MAX_FAILURES;
        for (uint32_t f = 0; f < recorded; f++) {
            const TestFailure &failure = test_failures()[f];
            fprintf(stderr, "FAIL %s: %s:%u: %s\n", failure.test, failure.file, failure.line, failure.expr);
        }
        printf("%s: %u checks, %u failures\n", ctx.current, ctx.checks, ctx.failures);
        return ctx.failures == 0 ? 0 : 1;
    }
    fprintf(stderr, "no test %s\n", argv[1]);
    return 2;
}
