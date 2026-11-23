cmake_minimum_required(VERSION 3.20)
project(installed_game LANGUAGES CXX)

find_package(tumble CONFIG REQUIRED)

add_executable(game main.cpp)
target_link_libraries(game PRIVATE tumble::tumble)
