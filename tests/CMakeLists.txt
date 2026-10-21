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
  unit/test_cruise_closed_loop.cpp)
target_link_libraries(vecu_unit_tests PRIVATE vecu_can vecu_control GTest::gtest_main vecu_compile_options)
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

    add_test(NAME python_integration
      COMMAND ${VECU_TEST_LAUNCHER} ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider ${CMAKE_CURRENT_SOURCE_DIR}/integration)
    set_tests_properties(python_integration PROPERTIES
      LABELS integration
      ENVIRONMENT "VECU_BIN_DIR=${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
      TIMEOUT 300)
  endif()
endif()
