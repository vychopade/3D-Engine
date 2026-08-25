cmake_minimum_required(VERSION 3.16)
project(physics_engine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_library(physics
    src/collision.cpp
    src/world.cpp
)
target_include_directories(physics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(physics PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra -Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

include(FetchContent)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(raylib)

add_executable(demo examples/demo.cpp)
target_link_libraries(demo PRIVATE physics raylib)

add_executable(test_physics tests/test_physics.cpp)
target_link_libraries(test_physics PRIVATE physics)

enable_testing()
add_test(NAME physics_unit COMMAND test_physics)
