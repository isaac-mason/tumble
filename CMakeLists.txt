cmake_minimum_required(VERSION 3.20)

project(tumble VERSION 0.0.0 LANGUAGES CXX)

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
    set(TUMBLE_IS_TOP_LEVEL ON)
else()
    set(TUMBLE_IS_TOP_LEVEL OFF)
endif()

option(TUMBLE_BUILD_TESTS "Build the tests and register them with ctest" ${TUMBLE_IS_TOP_LEVEL})
option(TUMBLE_BUILD_EXAMPLES "Build the examples" ${TUMBLE_IS_TOP_LEVEL})
option(TUMBLE_BUILD_SAMPLES "Build the samples app, fetching sokol and imgui" OFF)
option(TUMBLE_DEBUG "Check the library's asserts" OFF)
option(TUMBLE_INSTALL "Install the library, its headers and its cmake package" ${TUMBLE_IS_TOP_LEVEL})
option(TUMBLE_DETERMINISM_FLAGS "Compile with the flags that keep results identical across hosts, in consumers too" ON)
option(TUMBLE_NO_SIMD "Use the scalar backend even where simd is available" OFF)

file(GLOB_RECURSE TUMBLE_SOURCES CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/tumble/*.cpp")
add_library(tumble ${TUMBLE_SOURCES})
add_library(tumble::tumble ALIAS tumble)

include(GNUInstallDirs)

target_include_directories(tumble PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(tumble PUBLIC cxx_std_20)

if(TUMBLE_DEBUG)
    target_compile_definitions(tumble PUBLIC TUMBLE_DEBUG)
endif()
if(TUMBLE_NO_SIMD)
    target_compile_definitions(tumble PUBLIC TUMBLE_NO_SIMD)
endif()

if(MSVC)
    target_compile_options(tumble PRIVATE /W4)
    if(TUMBLE_DETERMINISM_FLAGS)
        target_compile_options(tumble PUBLIC /fp:precise)
    endif()
else()
    target_compile_options(tumble PRIVATE -Wall -Wextra)
    if(TUMBLE_DETERMINISM_FLAGS)
        target_compile_options(tumble PUBLIC -ffp-contract=off -fno-fast-math)
    endif()
endif()

if(TUMBLE_INSTALL)
    include(CMakePackageConfigHelpers)
    set(TUMBLE_CMAKE_PACKAGE_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/tumble")

    install(TARGETS tumble EXPORT tumbleTargets
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    install(DIRECTORY tumble DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h")
    install(EXPORT tumbleTargets NAMESPACE tumble:: DESTINATION ${TUMBLE_CMAKE_PACKAGE_DIR})

    configure_package_config_file("${PROJECT_SOURCE_DIR}/cmake/tumbleConfig.cmake.in"
                                  "${PROJECT_BINARY_DIR}/tumbleConfig.cmake"
                                  INSTALL_DESTINATION ${TUMBLE_CMAKE_PACKAGE_DIR})
    write_basic_package_version_file("${PROJECT_BINARY_DIR}/tumbleConfigVersion.cmake"
                                     VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
    install(FILES "${PROJECT_BINARY_DIR}/tumbleConfig.cmake" "${PROJECT_BINARY_DIR}/tumbleConfigVersion.cmake"
            DESTINATION ${TUMBLE_CMAKE_PACKAGE_DIR})
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
