#include "digest.h"
#include "harness.h"
#include "wasm/heap_begin.h"
#include "wasm/runtime/export.h"
#include "wasm/runtime/instance.h"

// what run.mjs drives, each call in an instance of its own

static TestCtx last = {};

WASM_EXPORT uint32_t tests_count() { return test_count(); }

/** over a fresh heap */
WASM_EXPORT uint32_t tests_run_one(uint32_t index) {
    if (!runtime::init(heap_begin())) return 1;
    last = test_run(index);
    return last.failures;
}

WASM_EXPORT uint32_t tests_checks() { return last.checks; }
WASM_EXPORT uint32_t tests_failure_count() { return last.failures < TEST_MAX_FAILURES ? last.failures : TEST_MAX_FAILURES; }
WASM_EXPORT const TestFailure *tests_failures() { return test_failures(); }

WASM_EXPORT uint32_t tests_digest() {
    if (!runtime::init(heap_begin())) return 0;
    return digest();
}
