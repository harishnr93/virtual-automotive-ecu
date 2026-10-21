# ---------------------------------------------------------------------------
# vecu_core: platform services shared by every module: clock, logging, RAII helpers,
# the OS port (threads, absolute sleep, priority-inheritance mutex) and the periodic
# task scheduler with timing analysis.
# ---------------------------------------------------------------------------
add_library(vecu_core STATIC
  core/logger.cpp
  core/os_port_posix.cpp
  core/timing_stats.cpp
  core/periodic_scheduler.cpp
  core/timing_report.cpp)
target_include_directories(vecu_core PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_features(vecu_core PUBLIC cxx_std_17)
target_link_libraries(vecu_core
  PUBLIC Threads::Threads
  PRIVATE vecu_compile_options)

# ---------------------------------------------------------------------------
# vecu_can: CAN frame model, signal codec, transport backends, CAN logging,
# reception supervision (timeouts, alive counters).
# ---------------------------------------------------------------------------
add_library(vecu_can STATIC
  can/can_frame.cpp
  can/can_signal.cpp
  can/udp_wire_format.cpp
  can/socketcan_channel.cpp
  can/udp_can_channel.cpp
  can/virtual_can_bus.cpp
  can/can_channel_factory.cpp
  can/can_log_writer.cpp
  can/can_receiver.cpp)
target_include_directories(vecu_can
  PUBLIC ${PROJECT_SOURCE_DIR}/include
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(vecu_can
  PUBLIC vecu_core
  PRIVATE vecu_compile_options)

# ---------------------------------------------------------------------------
# vecu_control: control algorithms (PID, cruise control state machine).
# Depends only on the C++ standard library — no CAN, no threads, no clocks.
# ---------------------------------------------------------------------------
add_library(vecu_control STATIC
  control/pid_controller.cpp
  control/cruise_controller.cpp)
target_include_directories(vecu_control PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_features(vecu_control PUBLIC cxx_std_17)
target_link_libraries(vecu_control PRIVATE vecu_compile_options)
