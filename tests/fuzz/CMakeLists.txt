# libFuzzer targets for every parser that takes input from outside the ECU: CAN/UDP datagrams,
# ISO-TP frames, UDS requests, firmware images and flash content, the NVM file.
#
# CTest runs each target for VECU_FUZZ_SECONDS (a smoke run that also replays the seeds);
# longer campaigns: ./build-fuzz/bin/fuzz_uds_server -max_total_time=600 <corpus dirs>
# New inputs go to the build tree, so the committed seeds (corpus/) are never modified.
set(VECU_FUZZ_SECONDS 15 CACHE STRING "Run time of each fuzz target in the CTest smoke run [s]")

foreach(target isotp uds_server udp_wire firmware nvm)
  add_executable(fuzz_${target} fuzz_${target}.cpp)
  target_link_libraries(fuzz_${target} PRIVATE vecu_can vecu_diagnostics vecu_faults vecu_firmware vecu_compile_options)
  target_link_options(fuzz_${target} PRIVATE -fsanitize=fuzzer)
  set(work_corpus ${CMAKE_CURRENT_BINARY_DIR}/corpus/${target})
  file(MAKE_DIRECTORY ${work_corpus})
  add_test(NAME fuzz_${target}
    COMMAND ${VECU_TEST_LAUNCHER} $<TARGET_FILE:fuzz_${target}> ${work_corpus} ${CMAKE_CURRENT_SOURCE_DIR}/corpus/${target}
            -max_total_time=${VECU_FUZZ_SECONDS} -print_final_stats=1
            -artifact_prefix=${CMAKE_CURRENT_BINARY_DIR}/crash-${target}-)
  math(EXPR fuzz_timeout "${VECU_FUZZ_SECONDS} + 120")
  set_tests_properties(fuzz_${target} PROPERTIES LABELS fuzz TIMEOUT ${fuzz_timeout})
endforeach()
