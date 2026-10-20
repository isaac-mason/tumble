#pragma once

#include "test.h"

uint32_t test_count();
/** the test's name less its test_ prefix */
const char *test_name(uint32_t index);
/** runs one test; a process or instance runs one, since tests leave statics behind */
const TestCtx &test_run(uint32_t index);
/** the first TEST_MAX_FAILURES failures the run recorded */
const TestFailure *test_failures();
