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
option(VECU_ENABLE_TSAN "Build with ThreadSanitizer (data-race detection)" OFF)
option(VECU_ENABLE_COVERAGE "Build with gcov instrumentation (line/function coverage, GCC)" OFF)
option(VECU_BUILD_FUZZERS "Build libFuzzer targets for the input parsers (Clang; includes ASan + UBSan)" OFF)

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
if(VECU_ENABLE_SANITIZERS AND VECU_ENABLE_TSAN)
  message(FATAL_ERROR "VECU_ENABLE_SANITIZERS (ASan) and VECU_ENABLE_TSAN cannot be combined")
endif()
if(VECU_ENABLE_COVERAGE)
  if(VECU_ENABLE_SANITIZERS OR VECU_ENABLE_TSAN)
    message(FATAL_ERROR "VECU_ENABLE_COVERAGE cannot be combined with sanitizers")
  endif()
  if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    message(FATAL_ERROR "VECU_ENABLE_COVERAGE needs GCC (gcov); see scripts/run_coverage.sh")
  endif()
  # -O0 keeps the line mapping exact; VECU_COVERAGE lets emergency exits flush the counters.
  target_compile_options(vecu_compile_options INTERFACE --coverage -O0 -g)
  target_compile_definitions(vecu_compile_options INTERFACE VECU_COVERAGE=1)
  target_link_options(vecu_compile_options INTERFACE --coverage)
endif()
if(VECU_BUILD_FUZZERS)
  if(VECU_ENABLE_SANITIZERS OR VECU_ENABLE_TSAN OR VECU_ENABLE_COVERAGE)
    message(FATAL_ERROR "VECU_BUILD_FUZZERS already includes ASan + UBSan; do not combine it with other instrumentation")
  endif()
  if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    message(FATAL_ERROR "VECU_BUILD_FUZZERS needs Clang (libFuzzer): CXX=clang++ cmake ...")
  endif()
  # Coverage feedback for the fuzzer in every library; the fuzz executables add -fsanitize=fuzzer.
  target_compile_options(vecu_compile_options INTERFACE
    -fsanitize=fuzzer-no-link,address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
  target_link_options(vecu_compile_options INTERFACE -fsanitize=address,undefined)
endif()
if(VECU_ENABLE_SANITIZERS)
  target_compile_options(vecu_compile_options INTERFACE
    -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
  target_link_options(vecu_compile_options INTERFACE -fsanitize=address,undefined)
endif()
set(VECU_TEST_LAUNCHER "")
if(VECU_ENABLE_TSAN)
  target_compile_options(vecu_compile_options INTERFACE -fsanitize=thread -fno-omit-frame-pointer)
  target_link_options(vecu_compile_options INTERFACE -fsanitize=thread)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 12)
    message(WARNING "GCC < 12 TSan does not intercept pthread_cond_clockwait (used by "
                    "condition_variable::wait_for) and reports false 'double lock' errors. "
                    "Prefer: CXX=clang++ cmake -DVECU_ENABLE_TSAN=ON ...")
  endif()
endif()

# Sanitizer runtimes of this toolchain generation (GCC 11 TSan, Clang 14 TSan/ASan) can fail at
# start-up on kernels with 32-bit mmap ASLR (Linux >= 6.5, including WSL2 6.6): measured for the
# fuzz targets, 18 of 60 starts crashed before running any input, 0 of 60 with ASLR disabled.
# Tests of these builds therefore run with ASLR disabled.
if(VECU_ENABLE_TSAN OR VECU_BUILD_FUZZERS)
  find_program(VECU_SETARCH setarch)
  if(VECU_SETARCH)
    set(VECU_TEST_LAUNCHER ${VECU_SETARCH} ${CMAKE_SYSTEM_PROCESSOR} -R)
    set(CMAKE_CROSSCOMPILING_EMULATOR ${VECU_TEST_LAUNCHER})  # used by gtest discovery and add_test
  endif()
endif()

add_subdirectory(src)
add_subdirectory(apps)

if(VECU_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
