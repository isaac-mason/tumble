# run as cmake -DTESTS=<tumble_tests> -P expect_trap.cmake: a failed allocation must stop the process, which ctest's
# WILL_FAIL does not count when it dies by a signal
execute_process(COMMAND "${TESTS}" --out-of-memory RESULT_VARIABLE result OUTPUT_QUIET ERROR_QUIET)
if(result EQUAL 0)
    message(FATAL_ERROR "a failed allocation returned instead of trapping")
endif()
