include(FetchContent)

FetchContent_Declare(sokol
    GIT_REPOSITORY https://github.com/floooh/sokol.git
    GIT_TAG 1847290135f95e57e6d220b0a41208306aafc0dd)
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.92.9)
FetchContent_Declare(cgltf
    GIT_REPOSITORY https://github.com/jkuhlmann/cgltf.git
    GIT_TAG v1.15)
FetchContent_MakeAvailable(sokol imgui cgltf)

add_library(tumble_labs_imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp)
target_include_directories(tumble_labs_imgui PUBLIC ${imgui_SOURCE_DIR})

# example_<name>.cpp names, without the prefix: a build of some examples only, as while porting one
set(TUMBLE_LABS_EXAMPLES "" CACHE STRING "the examples to build, e.g. \"cast_ray;sensor\"; empty builds them all")

file(GLOB TUMBLE_LABS_SOURCES CONFIGURE_DEPENDS app/*.cpp)
if(TUMBLE_LABS_EXAMPLES)
    foreach(name ${TUMBLE_LABS_EXAMPLES})
        list(APPEND TUMBLE_LABS_SOURCES examples/example_${name}.cpp)
    endforeach()
else()
    file(GLOB TUMBLE_LABS_EXAMPLE_SOURCES CONFIGURE_DEPENDS examples/*.cpp)
    list(APPEND TUMBLE_LABS_SOURCES ${TUMBLE_LABS_EXAMPLE_SOURCES})
endif()
add_executable(tumble_labs ${TUMBLE_LABS_SOURCES})
target_include_directories(tumble_labs PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${sokol_SOURCE_DIR} ${sokol_SOURCE_DIR}/util ${cgltf_SOURCE_DIR})
target_link_libraries(tumble_labs PRIVATE tumble::tumble tumble_labs_imgui)
if(NOT MSVC)
    target_compile_options(tumble_labs PRIVATE -Wall -Wextra)
endif()

if(EMSCRIPTEN)
    target_compile_definitions(tumble_labs PRIVATE SOKOL_GLES3)
    set_target_properties(tumble_labs PROPERTIES SUFFIX ".html")
    target_link_options(tumble_labs PRIVATE
        -sUSE_WEBGL2=1
        -sALLOW_MEMORY_GROWTH=1
        -sSTACK_SIZE=1048576
        --shell-file ${CMAKE_CURRENT_SOURCE_DIR}/app/shell.html
        --preload-file ${CMAKE_CURRENT_SOURCE_DIR}/assets@/assets)
    set_property(TARGET tumble_labs APPEND PROPERTY LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/app/shell.html)
else()
    target_compile_definitions(tumble_labs PRIVATE SOKOL_GLCORE TUMBLE_LABS_ASSETS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets")
    if(APPLE)
        set_source_files_properties(app/sokol.cpp PROPERTIES COMPILE_OPTIONS "-xobjective-c++;-Wno-deprecated-enum-enum-conversion")
        target_compile_definitions(tumble_labs PRIVATE GL_SILENCE_DEPRECATION)
        target_link_libraries(tumble_labs PRIVATE "-framework Cocoa" "-framework QuartzCore" "-framework OpenGL")
    elseif(UNIX)
        find_package(Threads REQUIRED)
        target_link_libraries(tumble_labs PRIVATE X11 Xi Xcursor GL dl m Threads::Threads)
    endif()
endif()
