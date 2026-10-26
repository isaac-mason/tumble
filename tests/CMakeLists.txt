# the native runner: every test but the wasm runtime's, each as a ctest test in a process of its own

file(GLOB_RECURSE TEST_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.test.cpp")
list(FILTER TEST_FILES EXCLUDE REGEX "/tests/wasm/")
# a test added to an existing file needs the list rewritten
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${TEST_FILES})

set(TEST_NAMES "")
set(TEST_LIST "")
foreach(file ${TEST_FILES})
    file(STRINGS "${file}" lines REGEX "^void test_[a-z0-9_]*\\(TestCtx")
    foreach(line ${lines})
        string(REGEX REPLACE "^void (test_[a-z0-9_]*)\\(TestCtx.*" "\\1" function "${line}")
        string(APPEND TEST_LIST "TEST(${function})\n")
        string(REGEX REPLACE "^test_" "" name "${function}")
        list(APPEND TEST_NAMES "${name}")
    endforeach()
endforeach()

set(TEST_LIST_DIR "${CMAKE_CURRENT_BINARY_DIR}/test-list")
file(CONFIGURE OUTPUT "${TEST_LIST_DIR}/test_list.inc" CONTENT "${TEST_LIST}")

add_executable(tumble_tests ${TEST_FILES} harness.cpp native/main.cpp util/out_of_memory.cpp)
target_link_libraries(tumble_tests PRIVATE tumble::tumble)
target_include_directories(tumble_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${TEST_LIST_DIR}")
if(MSVC)
    target_compile_options(tumble_tests PRIVATE /fp:precise)
else()
    target_compile_options(tumble_tests PRIVATE -ffp-contract=off -fno-fast-math)
endif()

foreach(name ${TEST_NAMES})
    add_test(NAME ${name} COMMAND tumble_tests ${name})
endforeach()

# a failed allocation traps: the process dies, which is the pass
add_test(NAME alloc_out_of_memory_traps
         COMMAND ${CMAKE_COMMAND} -DTESTS=$<TARGET_FILE:tumble_tests> -P "${CMAKE_CURRENT_SOURCE_DIR}/native/expect_trap.cmake")
