# every snippet source in docs/cpp is a program over tumble::tumble that checks what it shows: ctest builds and runs
# each, and checks that README.md and js/README.md are what the templates and the snippets render

file(GLOB DOCS_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/cpp/*.cpp")
foreach(source ${DOCS_SOURCES})
    get_filename_component(name "${source}" NAME_WE)
    add_executable(docs_${name} "${source}")
    target_link_libraries(docs_${name} PRIVATE tumble::tumble)
    if(MSVC)
        target_compile_options(docs_${name} PRIVATE /W4 /WX)
    else()
        target_compile_options(docs_${name} PRIVATE -Wall -Wextra -Werror)
    endif()
    add_test(NAME docs_${name} COMMAND docs_${name})
endforeach()

# the getting-started project, built and run as a user would build it; its main is the quick start
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/cmake/fetch_content/CMakeLists.txt" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/fetch_content")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cpp/quick_start.cpp" "${CMAKE_CURRENT_BINARY_DIR}/fetch_content/main.cpp" COPYONLY)
if(CMAKE_GENERATOR STREQUAL "Ninja")
    add_test(NAME docs_fetch_content
             COMMAND "${CMAKE_COMMAND}" -DCONSUMER=${CMAKE_CURRENT_BINARY_DIR}/fetch_content -DTUMBLE=${PROJECT_SOURCE_DIR}
                     -DBUILD=${CMAKE_CURRENT_BINARY_DIR}/fetch_content/build -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/build_consumer.cmake")
endif()

find_program(NODE node)
if(NODE)
    add_test(NAME docs_rendered COMMAND "${NODE}" "${CMAKE_CURRENT_SOURCE_DIR}/build.mjs" --check)
endif()
