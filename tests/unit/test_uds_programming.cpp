// UDS programming support: session rules, routines (0x31) and the download services
// (0x34/0x36/0x37) against a fake download handler. No flash is involved here.

#include <gtest/gtest.h>

#include <vector>

#include "diagnostics/uds_server.hpp"

namespace vecu::diag {
namespace {

using std::chrono::milliseconds;
using Bytes = std::vector<std::uint8_t>;

core::TimePoint at(int ms) { return core::TimePoint{} + milliseconds{ms}; }

/// Records calls; NRCs to return can be scripted per call type.
class FakeDownload : public IDownloadHandler {
 public:
  std::optional<Nrc> requestDownload(std::uint32_t address, std::uint32_t size) override {
    address_ = address;
    size_ = size;
    return request_nrc;
  }
  std::optional<Nrc> transferData(std::uint8_t counter, const std::uint8_t* data, std::size_t length) override {
    counters.push_back(counter);
    received.insert(received.end(), data, data + length);
    return transfer_nrc;
  }
  std::optional<Nrc> requestTransferExit() override {
    ++exits;
    return exit_nrc;
  }
  void abortDownload() override { ++aborts; }

  std::optional<Nrc> request_nrc;
  std::optional<Nrc> transfer_nrc;
  std::optional<Nrc> exit_nrc;
  std::uint32_t address_{0U};
  std::uint32_t size_{0U};
  std::vector<std::uint8_t> counters;
  Bytes received;
  int exits{0};
  int aborts{0};
};

class UdsProgrammingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    server.setDownloadHandler(&download);
    server.setSessionChangeHandler([this](Session from, Session to) { changes.emplace_back(from, to); });
    ASSERT_TRUE(server.addRoutine(0xFF00U, kSessionProgramming, [this](const std::uint8_t* opt, std::size_t n,
                                                                      std::uint8_t* status) {
      last_option.assign(opt, opt + n);
      status[0] = 0x00U;  // "routine correct"
      return RoutineResult{std::nullopt, 1U};
    }));
    ASSERT_TRUE(server.addRoutine(0x0201U, kSessionExtended | kSessionProgramming,
                                  [](const std::uint8_t*, std::size_t, std::uint8_t*) {
                                    return RoutineResult{Nrc::ConditionsNotCorrect, 0U};
                                  }));
  }

  Bytes request(const Bytes& req, int now_ms = 0, bool functional = false) {
    std::array<std::uint8_t, 256U> buffer{};
    const std::size_t n =
        server.handleRequest(req.data(), req.size(), functional, at(now_ms), buffer.data(), buffer.size());
    return Bytes(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(n));
  }

  void enterProgramming() {
    ASSERT_EQ(request({0x10U, 0x03U}).at(0), 0x50U);
    ASSERT_EQ(request({0x10U, 0x02U}).at(0), 0x50U);
    ASSERT_EQ(server.session(), Session::Programming);
  }

  static Bytes nrc(std::uint8_t service, Nrc code) { return {0x7FU, service, static_cast<std::uint8_t>(code)}; }

  faults::DtcStore dtcs;
  UdsServer server{UdsServerConfig{}, dtcs};
  FakeDownload download;
  std::vector<std::pair<Session, Session>> changes;
  Bytes last_option;
};

// ------------------------------------------------------------------- session rules ----

TEST_F(UdsProgrammingTest, ProgrammingOnlyFromExtendedSession) {
  EXPECT_EQ(request({0x10U, 0x02U}), nrc(0x10U, Nrc::SubFunctionNotSupportedInActiveSession));
  enterProgramming();
  EXPECT_EQ(request({0x10U, 0x03U}), nrc(0x10U, Nrc::SubFunctionNotSupportedInActiveSession));  // no way back
  EXPECT_EQ(request({0x10U, 0x02U}).at(0), 0x50U);  // re-entering the same session is fine
  EXPECT_EQ(request({0x10U, 0x01U}).at(0), 0x50U);
  ASSERT_EQ(changes.size(), 3U);  // default->extended->programming->default
  EXPECT_EQ(changes[1], std::make_pair(Session::Extended, Session::Programming));
}

TEST_F(UdsProgrammingTest, SessionGuardCanRefuseProgramming) {
  bool vehicle_moving = true;
  server.setSessionGuard([&](Session s) -> std::optional<Nrc> {
    if ((s == Session::Programming) && vehicle_moving) {
      return Nrc::ConditionsNotCorrect;
    }
    return std::nullopt;
  });
  ASSERT_EQ(request({0x10U, 0x03U}).at(0), 0x50U);
  EXPECT_EQ(request({0x10U, 0x02U}), nrc(0x10U, Nrc::ConditionsNotCorrect));
  EXPECT_EQ(server.session(), Session::Extended);
  vehicle_moving = false;
  EXPECT_EQ(request({0x10U, 0x02U}).at(0), 0x50U);
}

TEST_F(UdsProgrammingTest, ServiceAvailabilityPerSession) {
  EXPECT_EQ(request({0x34U, 0x00U, 0x44U, 0, 0, 0, 0, 0, 0, 0x10U, 0}),
            nrc(0x34U, Nrc::ServiceNotSupportedInActiveSession));
  EXPECT_EQ(request({0x36U, 0x01U, 0xAAU}), nrc(0x36U, Nrc::ServiceNotSupportedInActiveSession));
  EXPECT_EQ(request({0x37U}), nrc(0x37U, Nrc::ServiceNotSupportedInActiveSession));
  enterProgramming();
  EXPECT_EQ(request({0x19U, 0x0AU}), nrc(0x19U, Nrc::ServiceNotSupportedInActiveSession));
  EXPECT_EQ(request({0x14U, 0xFFU, 0xFFU, 0xFFU}), nrc(0x14U, Nrc::ServiceNotSupportedInActiveSession));
  EXPECT_TRUE(request({0x19U, 0x0AU}, 0, true).empty());  // functional: NRC 0x7F suppressed
}

// ---------------------------------------------------------------- 0x31 routines ----

TEST_F(UdsProgrammingTest, RoutineControl) {
  EXPECT_EQ(request({0x31U, 0x01U, 0xFFU, 0x00U}), nrc(0x31U, Nrc::RequestOutOfRange));  // not in default
  enterProgramming();
  EXPECT_EQ(request({0x31U, 0x01U, 0xFFU, 0x00U, 0xAAU, 0xBBU}), (Bytes{0x71U, 0x01U, 0xFFU, 0x00U, 0x00U}));
  EXPECT_EQ(last_option, (Bytes{0xAAU, 0xBBU}));
  EXPECT_EQ(request({0x31U, 0x02U, 0xFFU, 0x00U}), nrc(0x31U, Nrc::SubFunctionNotSupported));  // stop: n/a
  EXPECT_EQ(request({0x31U, 0x01U, 0x12U, 0x34U}), nrc(0x31U, Nrc::RequestOutOfRange));  // unknown id
  EXPECT_EQ(request({0x31U, 0x01U, 0xFFU}), nrc(0x31U, Nrc::IncorrectMessageLengthOrInvalidFormat));
  EXPECT_EQ(request({0x31U, 0x01U, 0x02U, 0x01U}), nrc(0x31U, Nrc::ConditionsNotCorrect));  // handler's NRC
  EXPECT_TRUE(request({0x31U, 0x81U, 0xFFU, 0x00U}).empty());  // suppressPosRsp
}

TEST_F(UdsProgrammingTest, RoutineRegistryRejectsDuplicatesAndOverflow) {
  auto noop = [](const std::uint8_t*, std::size_t, std::uint8_t*) { return RoutineResult{}; };
  EXPECT_FALSE(server.addRoutine(0xFF00U, kSessionAll, noop));
  for (std::uint16_t id = 1U; id <= UdsServer::kMaxRoutines - 2U; ++id) {
    EXPECT_TRUE(server.addRoutine(id, kSessionAll, noop));
  }
  EXPECT_FALSE(server.addRoutine(0x7777U, kSessionAll, noop));
}

// -------------------------------------------------------------- 0x34/0x36/0x37 ----

TEST_F(UdsProgrammingTest, CompleteDownloadSequence) {
  enterProgramming();
  // dataFormat 0x00, addressAndLengthFormat 0x44: 4-byte address, 4-byte size
  EXPECT_EQ(request({0x34U, 0x00U, 0x44U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x01U, 0x00U}),
            (Bytes{0x74U, 0x20U, 0x0FU, 0xFFU}));
  EXPECT_EQ(download.size_, 256U);
  EXPECT_TRUE(server.downloadActive());
  EXPECT_EQ(request({0x36U, 0x01U, 0x11U, 0x22U}), (Bytes{0x76U, 0x01U}));
  EXPECT_EQ(request({0x36U, 0x02U, 0x33U}), (Bytes{0x76U, 0x02U}));
  EXPECT_EQ(download.received, (Bytes{0x11U, 0x22U, 0x33U}));
  EXPECT_EQ(request({0x37U}), (Bytes{0x77U}));
  EXPECT_FALSE(server.downloadActive());
  EXPECT_EQ(request({0x36U, 0x03U, 0x44U}), nrc(0x36U, Nrc::RequestSequenceError));  // after exit
  EXPECT_EQ(request({0x37U}), nrc(0x37U, Nrc::RequestSequenceError));
}

TEST_F(UdsProgrammingTest, RequestDownloadFormatChecks) {
  enterProgramming();
  EXPECT_EQ(request({0x34U, 0x00U, 0x44U, 0x00U}), nrc(0x34U, Nrc::IncorrectMessageLengthOrInvalidFormat));
  EXPECT_EQ(request({0x34U, 0x00U, 0x50U}), nrc(0x34U, Nrc::RequestOutOfRange));  // 0 address bytes
  EXPECT_EQ(request({0x34U, 0x11U, 0x11U, 0x00U, 0x10U}), nrc(0x34U, Nrc::RequestOutOfRange));  // compressed
  EXPECT_EQ(request({0x34U, 0x00U, 0x11U, 0x00U, 0x10U}), (Bytes{0x74U, 0x20U, 0x0FU, 0xFFU}));  // 1+1 bytes
  EXPECT_EQ(download.size_, 0x10U);
  EXPECT_EQ(request({0x34U, 0x00U, 0x11U, 0x00U, 0x10U}), nrc(0x34U, Nrc::ConditionsNotCorrect));  // active
}

TEST_F(UdsProgrammingTest, HandlerNrcsArePassedThrough) {
  enterProgramming();
  download.request_nrc = Nrc::UploadDownloadNotAccepted;
  EXPECT_EQ(request({0x34U, 0x00U, 0x11U, 0x00U, 0x10U}), nrc(0x34U, Nrc::UploadDownloadNotAccepted));
  EXPECT_FALSE(server.downloadActive());
  download.request_nrc.reset();
  ASSERT_EQ(request({0x34U, 0x00U, 0x11U, 0x00U, 0x10U}).at(0), 0x74U);
  download.transfer_nrc = Nrc::WrongBlockSequenceCounter;
  EXPECT_EQ(request({0x36U, 0x05U, 0x00U}), nrc(0x36U, Nrc::WrongBlockSequenceCounter));
  download.transfer_nrc.reset();
  download.exit_nrc = Nrc::RequestSequenceError;
  EXPECT_EQ(request({0x37U}), nrc(0x37U, Nrc::RequestSequenceError));
  EXPECT_TRUE(server.downloadActive());  // a failed exit keeps the download open
}

TEST_F(UdsProgrammingTest, TransferDataLengthLimit) {
  enterProgramming();
  ASSERT_EQ(request({0x34U, 0x00U, 0x22U, 0x00U, 0x00U, 0x20U, 0x00U}).at(0), 0x74U);
  Bytes block(UdsServer::kMaxBlockLength + 1U, 0x00U);
  block[0] = 0x36U;
  block[1] = 0x01U;
  std::array<std::uint8_t, 8U> out{};
  EXPECT_EQ(server.handleRequest(block.data(), block.size(), false, at(0), out.data(), out.size()), 3U);
  EXPECT_EQ(out[2], static_cast<std::uint8_t>(Nrc::IncorrectMessageLengthOrInvalidFormat));
  block.resize(UdsServer::kMaxBlockLength);  // exactly the announced maximum is accepted
  EXPECT_EQ(server.handleRequest(block.data(), block.size(), false, at(0), out.data(), out.size()), 2U);
  EXPECT_EQ(download.received.size(), UdsServer::kMaxBlockLength - 2U);
}

TEST_F(UdsProgrammingTest, LeavingProgrammingAbortsTheDownload) {
  enterProgramming();
  ASSERT_EQ(request({0x34U, 0x00U, 0x11U, 0x00U, 0x10U}).at(0), 0x74U);
  ASSERT_EQ(request({0x10U, 0x01U}).at(0), 0x50U);  // explicit session change
  EXPECT_EQ(download.aborts, 1);
  EXPECT_FALSE(server.downloadActive());

  ASSERT_EQ(request({0x10U, 0x03U}, 100).at(0), 0x50U);
  ASSERT_EQ(request({0x10U, 0x02U}, 100).at(0), 0x50U);
  ASSERT_EQ(request({0x34U, 0x00U, 0x11U, 0x00U, 0x10U}, 100).at(0), 0x74U);
  server.poll(at(5200));  // S3 timeout
  EXPECT_EQ(server.session(), Session::Default);
  EXPECT_EQ(download.aborts, 2);

  ASSERT_EQ(request({0x10U, 0x03U}, 6000).at(0), 0x50U);
  ASSERT_EQ(request({0x10U, 0x02U}, 6000).at(0), 0x50U);
  ASSERT_EQ(request({0x34U, 0x00U, 0x11U, 0x00U, 0x10U}, 6000).at(0), 0x74U);
  ASSERT_EQ(request({0x11U, 0x01U}, 6000), (Bytes{0x51U, 0x01U}));  // ECU reset
  EXPECT_EQ(download.aborts, 3);
  EXPECT_EQ(server.takePendingReset(), ResetType::HardReset);
}

TEST_F(UdsProgrammingTest, NoHandlerMeansDownloadNotAccepted) {
  server.setDownloadHandler(nullptr);
  enterProgramming();
  EXPECT_EQ(request({0x34U, 0x00U, 0x11U, 0x00U, 0x10U}), nrc(0x34U, Nrc::UploadDownloadNotAccepted));
}

}  // namespace
}  // namespace vecu::diag
