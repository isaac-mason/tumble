# SNIPPET_START: fetch-content
cmake_minimum_required(VERSION 3.20)
project(game LANGUAGES CXX)

include(FetchContent)
FetchContent_Declare(tumble GIT_REPOSITORY https://github.com/isaac-mason/tumble.git GIT_TAG main)
FetchContent_MakeAvailable(tumble)

add_executable(game main.cpp)
target_link_libraries(game PRIVATE tumble::tumble)
# SNIPPET_END: fetch-content
