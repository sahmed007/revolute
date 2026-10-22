cmake_minimum_required(VERSION 3.21)

project(revolute VERSION 0.1.0 LANGUAGES CXX)

# Require standard C++20, without compiler-specific language extensions.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Let clangd use the same compilation settings as the build.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
find_package(OpenGL REQUIRED)

# Only build GLFW itself, not its examples, tests, or documentation.
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG 3.4
  GIT_SHALLOW TRUE
)
FetchContent_Declare(imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.91.9b
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(glfw imgui)

# Dear ImGui ships source files rather than a CMake library target.
add_library(revolute_imgui STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/imgui_demo.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(revolute_imgui SYSTEM PUBLIC
  ${imgui_SOURCE_DIR}
  ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(revolute_imgui PUBLIC glfw OpenGL::GL)

add_library(revolute_kinematics src/kinematics.cpp)
target_include_directories(revolute_kinematics PUBLIC include)

add_executable(revolute src/main.cpp)
if(APPLE)
  target_compile_definitions(revolute PRIVATE GL_SILENCE_DEPRECATION)
endif()
target_link_libraries(revolute PRIVATE revolute_kinematics revolute_imgui)

function(enable_project_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /WX)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
endfunction()

enable_project_warnings(revolute_kinematics)
enable_project_warnings(revolute)

include(CTest)
if(BUILD_TESTING)
  FetchContent_Declare(doctest
    GIT_REPOSITORY https://github.com/doctest/doctest.git
    GIT_TAG v2.5.3
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(doctest)

  add_executable(revolute_tests tests/kinematics_test.cpp)
  target_link_libraries(revolute_tests PRIVATE revolute_kinematics doctest::doctest)
  target_include_directories(revolute_tests SYSTEM PRIVATE
    $<TARGET_PROPERTY:doctest::doctest,INTERFACE_INCLUDE_DIRECTORIES>
  )
  enable_project_warnings(revolute_tests)
  add_test(NAME kinematics COMMAND revolute_tests)
endif()
