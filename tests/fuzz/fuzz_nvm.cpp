// NVM file loader: the fault memory read at start-up from a file that may be truncated,
// corrupted or foreign. Invariant: whatever loads successfully is consistent and survives
// save -> load unchanged.

#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>

#include "faults/nvm_storage.hpp"
#include "fuzz_check.hpp"

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
  using namespace vecu::faults;
  static const std::string path = "/tmp/vecu_fuzz_nvm_" + std::to_string(::getpid()) + ".bin";
  if (std::FILE* file = std::fopen(path.c_str(), "wb")) {
    (void)std::fwrite(data, 1U, size, file);
    (void)std::fclose(file);
  }
  NvmData loaded;
  if (loadNvm(path, loaded) != NvmStatus::Ok) {
    return 0;
  }
  FUZZ_CHECK(loaded.record_count <= loaded.records.size());
  FUZZ_CHECK(saveNvm(path, loaded) == NvmStatus::Ok);
  NvmData again;
  FUZZ_CHECK(loadNvm(path, again) == NvmStatus::Ok);
  FUZZ_CHECK(again.record_count == loaded.record_count && again.reset_count == loaded.reset_count &&
             again.operation_cycles == loaded.operation_cycles && again.last_reset_reason == loaded.last_reset_reason);
  for (std::size_t i = 0; i < loaded.record_count; ++i) {
    FUZZ_CHECK(again.records[i].code == loaded.records[i].code && again.records[i].status == loaded.records[i].status);
  }
  return 0;
}
