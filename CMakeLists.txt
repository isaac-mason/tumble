cmake_minimum_required(VERSION 3.20)

project(tumble LANGUAGES CXX)

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
    set(TUMBLE_IS_TOP_LEVEL ON)
else()
    set(TUMBLE_IS_TOP_LEVEL OFF)
endif()

option(TUMBLE_BUILD_TESTS "Build the tests and register them with ctest" ${TUMBLE_IS_TOP_LEVEL})
option(TUMBLE_BUILD_EXAMPLES "Build the examples" ${TUMBLE_IS_TOP_LEVEL})
option(TUMBLE_BUILD_SAMPLES "Build the samples app, fetching sokol and imgui" OFF)
option(TUMBLE_DEBUG "Check the library's asserts" OFF)
option(TUMBLE_NO_SIMD "Use the scalar backend even where simd is available" OFF)

file(GLOB_RECURSE TUMBLE_SOURCES CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/tumble/*.cpp")
add_library(tumble ${TUMBLE_SOURCES})
add_library(tumble::tumble ALIAS tumble)

target_include_directories(tumble PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>)
target_compile_features(tumble PUBLIC cxx_std_20)

if(TUMBLE_DEBUG)
    target_compile_definitions(tumble PUBLIC TUMBLE_DEBUG)
endif()
if(TUMBLE_NO_SIMD)
    target_compile_definitions(tumble PUBLIC TUMBLE_NO_SIMD)
endif()

# the same bits on every host: no fused multiply-add (wasm has none), no fast math
if(MSVC)
    target_compile_options(tumble PRIVATE /fp:precise /W4)
else()
    target_compile_options(tumble PRIVATE -ffp-contract=off -fno-fast-math -Wall -Wextra)
endif()

if(TUMBLE_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
    add_subdirectory(docs)
endif()

if(TUMBLE_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(TUMBLE_BUILD_SAMPLES)
    add_subdirectory(samples)
endif()
