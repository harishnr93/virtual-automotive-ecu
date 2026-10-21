# Executables (composition roots). They wire the reusable libraries from src/ together;
# all logic worth testing lives in the libraries, not here.

add_library(vecu_app_common STATIC
  common/app_options.cpp
  common/shutdown_signal.cpp
  common/ecu_runtime.cpp)
target_include_directories(vecu_app_common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(vecu_app_common
  PUBLIC vecu_can
  PRIVATE vecu_compile_options)

add_executable(vecu_sensor_ecu sensor_ecu/main.cpp)
target_link_libraries(vecu_sensor_ecu PRIVATE vecu_app_common vecu_compile_options)

add_executable(vecu_cruise_ecu cruise_ecu/main.cpp)
target_link_libraries(vecu_cruise_ecu PRIVATE vecu_app_common vecu_compile_options)

add_executable(vecu_sched_bench sched_bench/main.cpp)
target_link_libraries(vecu_sched_bench PRIVATE vecu_app_common vecu_compile_options)
