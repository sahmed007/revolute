cmake_minimum_required(VERSION 3.21)

project(revolute VERSION 0.1.0 LANGUAGES CXX)

# Require standard C++20, without compiler-specific language extensions.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Let clangd use the same compilation settings as the build.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(revolute_kinematics src/kinematics.cpp)
target_include_directories(revolute_kinematics PUBLIC include)

add_executable(revolute src/main.cpp)
target_link_libraries(revolute PRIVATE revolute_kinematics)

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
  include(FetchContent)
  FetchContent_Declare(doctest
    GIT_REPOSITORY https://github.com/doctest/doctest.git
    GIT_TAG v2.4.11
    GIT_SHALLOW TRUE
  )
  # doctest 2.4.11 uses an older policy minimum; keep CMake 4 compatibility local.
  function(make_doctest_available)
    set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
    FetchContent_MakeAvailable(doctest)
  endfunction()
  make_doctest_available()

  add_executable(revolute_tests tests/kinematics_test.cpp)
  target_link_libraries(revolute_tests PRIVATE revolute_kinematics doctest::doctest)
  target_include_directories(revolute_tests SYSTEM PRIVATE
    $<TARGET_PROPERTY:doctest::doctest,INTERFACE_INCLUDE_DIRECTORIES>
  )
  enable_project_warnings(revolute_tests)
  add_test(NAME kinematics COMMAND revolute_tests)
endif()
