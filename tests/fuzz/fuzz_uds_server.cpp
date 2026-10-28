// UDS server: arbitrary request sequences in every session, with a download handler, routines
// and DIDs registered as in the ECU.
//
// Input: [flags] then requests as [length byte][request bytes].
// Invariants: a response fits the buffer; it is either positive (SID + 0x40) or exactly a
// 3-byte negative response for the request's service; the server never leaves the programming
// session with an active download.

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "diagnostics/uds_server.hpp"
#include "fuzz_check.hpp"

namespace {

using namespace vecu;

class AcceptingDownload : public diag::IDownloadHandler {
 public:
  std::optional<diag::Nrc> requestDownload(std::uint32_t, std::uint32_t size) override {
    if (size == 0U) {
      return diag::Nrc::RequestOutOfRange;
    }
    return std::nullopt;
  }
  std::optional<diag::Nrc> transferData(std::uint8_t counter, const std::uint8_t*, std::size_t) override {
    if (counter == 0x13U) {
      return diag::Nrc::WrongBlockSequenceCounter;
    }
    return std::nullopt;
  }
  std::optional<diag::Nrc> requestTransferExit() override { return std::nullopt; }
  void abortDownload() override {}
};

}  // namespace

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
  if (size == 0U) {
    return 0;
  }
  faults::DtcStore dtcs;
  (void)dtcs.registerDtc(0xC12187U);
  (void)dtcs.registerDtc(0x403164U);
  diag::UdsServer server{diag::UdsServerConfig{}, dtcs};
  AcceptingDownload download;
  server.setDownloadHandler(&download);
  (void)server.addDid(0xF190U, 17U, diag::kSessionAll, [](std::uint8_t* out) {
    for (std::size_t i = 0; i < 17U; ++i) {
      out[i] = 'V';
    }
  });
  (void)server.addDid(0x0100U, 2U, diag::kSessionExtended, [](std::uint8_t* out) { out[0] = out[1] = 0; });
  (void)server.addRoutine(0xFF00U, diag::kSessionProgramming, [](const std::uint8_t*, std::size_t, std::uint8_t* s) {
    s[0] = 0;
    return diag::RoutineResult{std::nullopt, 1U};
  });
  (void)server.addRoutine(0x0201U, diag::kSessionAll, [](const std::uint8_t*, std::size_t n, std::uint8_t* s) {
    if (n > 3U) {
      return diag::RoutineResult{diag::Nrc::ConditionsNotCorrect, 0U};
    }
    s[0] = 1;
    s[1] = 2;
    return diag::RoutineResult{std::nullopt, 2U};
  });
  const bool functional_all = (data[0] & 0x01U) != 0U;
  core::TimePoint now{};
  std::array<std::uint8_t, 4096U> response{};
  for (std::size_t i = 1; i < size;) {
    const std::size_t length = data[i] % 64U;
    ++i;
    if (length == 0U || i + length > size) {
      now += std::chrono::milliseconds{1000};  // let time pass (S3 timeout path)
      server.poll(now);
      continue;
    }
    const bool functional = functional_all && (length <= 7U);
    const std::size_t n = server.handleRequest(data + i, length, functional, now, response.data(), response.size());
    FUZZ_CHECK(n <= response.size());
    if (n > 0U) {
      if (response[0] == 0x7FU) {
        FUZZ_CHECK(n == 3U && response[1] == data[i]);
      } else {
        FUZZ_CHECK(response[0] == static_cast<std::uint8_t>(data[i] + 0x40U));
      }
    }
    if (server.session() != diag::Session::Programming) {
      FUZZ_CHECK(!server.downloadActive());  // leaving programming always ends a download
    }
    (void)server.takePendingReset();
    now += std::chrono::milliseconds{10};
    i += length;
  }
  return 0;
}
