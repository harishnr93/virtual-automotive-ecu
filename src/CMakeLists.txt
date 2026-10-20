# ---------------------------------------------------------------------------
# vecu_core: platform utilities shared by every module (clock, logging, RAII).
# ---------------------------------------------------------------------------
add_library(vecu_core STATIC
  core/logger.cpp)
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
  can/can_log_writer.cpp)
target_include_directories(vecu_can
  PUBLIC ${PROJECT_SOURCE_DIR}/include
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(vecu_can
  PUBLIC vecu_core
  PRIVATE vecu_compile_options)
