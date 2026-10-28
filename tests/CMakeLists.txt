find_package(GTest REQUIRED)
include(GoogleTest)

# ---------------------------------------------------------------------------
# C++ unit tests: pure logic, deterministic, no sockets and no sleeping.
# ---------------------------------------------------------------------------
add_executable(vecu_unit_tests
  unit/test_ring_buffer.cpp
  unit/test_can_frame.cpp
  unit/test_can_signal.cpp
  unit/test_vehicle_signals.cpp
  unit/test_udp_wire_format.cpp
  unit/test_message_timeout_monitor.cpp
  unit/test_alive_counter.cpp
  unit/test_virtual_can_bus.cpp
  unit/test_can_log_writer.cpp
  unit/test_timing_stats.cpp
  unit/test_os_port.cpp
  unit/test_shared_signal.cpp
  unit/test_scheduler_config.cpp
  unit/test_can_receiver.cpp
  unit/test_pid_controller.cpp
  unit/test_cruise_controller.cpp
  unit/test_cruise_closed_loop.cpp
  unit/test_dtc_store.cpp
  unit/test_isotp.cpp
  unit/test_uds_server.cpp
  unit/test_fault_manager.cpp
  unit/test_nvm_and_aging.cpp
  unit/test_watchdog.cpp
  unit/test_firmware_image.cpp
  unit/test_boot_and_update.cpp
  unit/test_uds_programming.cpp)
target_link_libraries(vecu_unit_tests PRIVATE vecu_can vecu_control vecu_diagnostics vecu_watchdog vecu_firmware GTest::gtest_main vecu_compile_options)
target_include_directories(vecu_unit_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/unit)
target_compile_definitions(vecu_unit_tests PRIVATE
  VECU_DBC_PATH="${PROJECT_SOURCE_DIR}/config/vehicle_can.dbc")
gtest_discover_tests(vecu_unit_tests PROPERTIES LABELS unit)

# ---------------------------------------------------------------------------
# C++ integration tests: real OS transports (UDP multicast, SocketCAN if present).
# ---------------------------------------------------------------------------
add_executable(vecu_integration_tests
  integration/test_udp_can_channel.cpp
  integration/test_socketcan_channel.cpp
  integration/test_periodic_scheduler.cpp)
target_link_libraries(vecu_integration_tests PRIVATE vecu_can GTest::gtest_main vecu_compile_options)
gtest_discover_tests(vecu_integration_tests PROPERTIES LABELS integration)

# ---------------------------------------------------------------------------
# Python tests (pytest): codecs, vehicle model, simulator logic (unit); multi-process
# ECU, scheduler benchmark and closed-loop system tests (integration).
# ---------------------------------------------------------------------------
find_package(Python3 COMPONENTS Interpreter)
if(Python3_FOUND)
  execute_process(COMMAND ${Python3_EXECUTABLE} -c "import pytest"
                  RESULT_VARIABLE VECU_PYTEST_MISSING OUTPUT_QUIET ERROR_QUIET)
  if(VECU_PYTEST_MISSING)
    message(WARNING "pytest not found: Python tests are not registered (pip install pytest)")
  else()
    add_test(NAME python_unit
      COMMAND ${VECU_TEST_LAUNCHER} ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider ${CMAKE_CURRENT_SOURCE_DIR}/unit/python)
    set_tests_properties(python_unit PROPERTIES LABELS unit)

    # System tests: one CTest entry per area, so a regression report names the broken area
    # (ctest -L firmware, ctest -L system ...). They start real ECU processes and check
    # real-time behaviour, so each runs alone (RUN_SERIAL): with "ctest -j" a burst of
    # hundreds of unit-test process starts in parallel once stalled the whole host for ~12 ms
    # and made the closed-loop pace check fail.
    set(VECU_SYSTEM_TESTS
      "test_two_ecu_communication\;can"
      "test_can_latency\;can\;performance"
      "test_scheduler_bench\;scheduler\;performance"
      "test_closed_loop\;control"
      "test_diagnostics\;diagnostics"
      "test_fault_management\;faults\;watchdog"
      "test_firmware_update\;firmware")
    foreach(entry IN LISTS VECU_SYSTEM_TESTS)
      list(GET entry 0 file)
      list(REMOVE_AT entry 0)
      string(REPLACE "test_" "system_" name ${file})
      add_test(NAME ${name}
        COMMAND ${VECU_TEST_LAUNCHER} ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider
                ${CMAKE_CURRENT_SOURCE_DIR}/integration/${file}.py)
      set_tests_properties(${name} PROPERTIES
        LABELS "integration;system;${entry}"
        ENVIRONMENT "VECU_BIN_DIR=${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
        RUN_SERIAL TRUE
        TIMEOUT 240)
    endforeach()
  endif()
endif()

# ---------------------------------------------------------------------------
# Fuzz targets (Clang libFuzzer, -DVECU_BUILD_FUZZERS=ON): see tests/fuzz/.
# ---------------------------------------------------------------------------
if(VECU_BUILD_FUZZERS)
  add_subdirectory(fuzz)
endif()
