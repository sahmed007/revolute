cmake_minimum_required(VERSION 3.21)

project(revolute VERSION 0.1.0 LANGUAGES CXX)

# Require standard C++20, without compiler-specific language extensions.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Let clangd use the same compilation settings as the build.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(revolute src/main.cpp)

if(MSVC)
    target_compile_options(revolute PRIVATE /W4 /WX)
else()
    target_compile_options(revolute PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
