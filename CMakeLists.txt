cmake_minimum_required(VERSION 3.16)

project(virtual_automotive_ecu
  VERSION 0.1.0
  DESCRIPTION "Virtual Automotive ECU Platform (educational, hardware-free)"
  LANGUAGES CXX)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "This project requires Linux (SocketCAN and POSIX sockets). Use WSL2 on Windows.")
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

option(VECU_BUILD_TESTS "Build unit and integration tests" ON)
option(VECU_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(VECU_ENABLE_SANITIZERS "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------
# Project-wide compiler settings, applied to our own targets only (not to
# third-party code such as GoogleTest).
# ---------------------------------------------------------------------------
add_library(vecu_compile_options INTERFACE)
target_compile_options(vecu_compile_options INTERFACE
  -Wall -Wextra -Wpedantic
  -Wshadow -Wconversion -Wsign-conversion -Wold-style-cast -Wcast-align
  -Wnon-virtual-dtor -Woverloaded-virtual -Wnull-dereference
  -Wdouble-promotion -Wformat=2 -Wimplicit-fallthrough)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_compile_options(vecu_compile_options INTERFACE
    -Wduplicated-cond -Wduplicated-branches -Wlogical-op)
endif()
if(VECU_WARNINGS_AS_ERRORS)
  target_compile_options(vecu_compile_options INTERFACE -Werror)
endif()
if(VECU_ENABLE_SANITIZERS)
  target_compile_options(vecu_compile_options INTERFACE
    -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
  target_link_options(vecu_compile_options INTERFACE -fsanitize=address,undefined)
endif()

add_subdirectory(src)
add_subdirectory(apps)

if(VECU_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
