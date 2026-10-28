#pragma once

// Invariant check for fuzz targets: unlike assert() it is active in every build type, and it
// stops the fuzzer with a crash report (and the input that caused it).

#include <cstdio>
#include <cstdlib>

#define FUZZ_CHECK(condition)                                                                       \
  do {                                                                                              \
    if (!(condition)) {                                                                             \
      std::fprintf(stderr, "FUZZ_CHECK failed: %s (%s:%d)\n", #condition, __FILE__, __LINE__);     \
      std::abort();                                                                                 \
    }                                                                                               \
  } while (false)
