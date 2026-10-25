// UDS server tests: every implemented service, positive and negative responses.

#include "diagnostics/uds_server.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace vecu::diag {
namespace {

using std::chrono::milliseconds;
using Bytes = std::vector<std::uint8_t>;

constexpr faults::DtcCode kDtcSpeed = 0xC12187U;
constexpr faults::DtcCode kDtcPlausibility = 0x403164U;

core::TimePoint at(int ms) { return core::TimePoint{} + milliseconds{ms}; }

class UdsServerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ASSERT_TRUE(dtcs.registerDtc(kDtcSpeed));
    ASSERT_TRUE(dtcs.registerDtc(kDtcPlausibility));
    ASSERT_TRUE(server.addDid(0xF190U, 17U, kSessionAll, [](std::uint8_t* out) {
      const char* vin = "VECUSIM0000000001";
      std::copy_n(vin, 17, out);
    }));
    ASSERT_TRUE(server.addDid(0x0100U, 2U, kSessionAll, [](std::uint8_t* out) {
      out[0] = 0x1F;  // 80.00 km/h = 8000 = 0x1F40
      out[1] = 0x40;
    }));
    ASSERT_TRUE(server.addDid(0xF18CU, 4U, kSessionExtended, [](std::uint8_t* out) {
      out[0] = 'S';
      out[1] = 'N';
      out[2] = '0';
      out[3] = '1';
    }));
  }

  Bytes request(const Bytes& req, int now_ms = 0, bool functional = false) {
    std::array<std::uint8_t, 256U> buffer{};
    const std::size_t n = server.handleRequest(req.data(), req.size(), functional, at(now_ms), buffer.data(),
                                               buffer.size());
    return Bytes(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(n));
  }

  static Bytes nrc(std::uint8_t service, Nrc code) { return {0x7FU, service, static_cast<std::uint8_t>(code)}; }

  faults::DtcStore dtcs;
  UdsServer server{UdsServerConfig{}, dtcs};
};

// --------------------------------------------------------------------- general rules ----

TEST_F(UdsServerTest, UnknownServiceAndEmptyRequest) {
  EXPECT_EQ(request({0x2EU, 0xF1U, 0x90U}), nrc(0x2EU, Nrc::ServiceNotSupported));
  EXPECT_TRUE(request({}).empty());
}

TEST_F(UdsServerTest, FunctionalRequestsSuppressSelectedNrcs) {
  EXPECT_TRUE(request({0x2EU, 0x00U}, 0, true).empty());              // 0x11 suppressed
  EXPECT_TRUE(request({0x10U, 0x05U}, 0, true).empty());              // 0x12 suppressed
  EXPECT_EQ(request({0x10U}, 0, true), nrc(0x10U, Nrc::IncorrectMessageLengthOrInvalidFormat));  // 0x13 sent
  EXPECT_EQ(request({0x3EU, 0x00U}, 0, true), (Bytes{0x7EU, 0x00U}));  // positive response sent
}

// ------------------------------------------------------------------ 0x10 sessions ----

TEST_F(UdsServerTest, SessionControlResponseCarriesTimingParameters) {
  EXPECT_EQ(request({0x10U, 0x03U}), (Bytes{0x50U, 0x03U, 0x00U, 0x32U, 0x01U, 0xF4U}));  // P2 50 ms, P2* 5000 ms
  EXPECT_EQ(server.session(), Session::Extended);
  EXPECT_EQ(request({0x10U, 0x01U}), (Bytes{0x50U, 0x01U, 0x00U, 0x32U, 0x01U, 0xF4U}));
  EXPECT_EQ(server.session(), Session::Default);
}

TEST_F(UdsServerTest, SessionControlErrors) {
  EXPECT_EQ(request({0x10U, 0x02U}), nrc(0x10U, Nrc::SubFunctionNotSupported));  // programming: Phase 6
  EXPECT_EQ(request({0x10U, 0x7FU}), nrc(0x10U, Nrc::SubFunctionNotSupported));
  EXPECT_EQ(request({0x10U, 0x03U, 0x00U}), nrc(0x10U, Nrc::IncorrectMessageLengthOrInvalidFormat));
  EXPECT_EQ(server.session(), Session::Default);
}

TEST_F(UdsServerTest, SuppressPositiveResponseBit) {
  EXPECT_TRUE(request({0x10U, 0x83U}).empty());
  EXPECT_EQ(server.session(), Session::Extended);  // the request was still executed
  EXPECT_TRUE(request({0x3EU, 0x80U}).empty());
  EXPECT_EQ(request({0x3EU, 0x81U}), nrc(0x3EU, Nrc::SubFunctionNotSupported));  // NRCs are not suppressed
  EXPECT_EQ(server.statistics().suppressed_responses, 2U);
}

TEST_F(UdsServerTest, S3TimeoutReturnsToDefaultSession) {
  (void)request({0x10U, 0x03U}, 0);
  server.poll(at(4000));
  (void)request({0x3EU, 0x80U}, 4000);  // TesterPresent keeps the session alive
  server.poll(at(8999));
  EXPECT_EQ(server.session(), Session::Extended);
  server.poll(at(9001));  // > 5000 ms since the last request
  EXPECT_EQ(server.session(), Session::Default);
  EXPECT_EQ(server.statistics().session_timeouts, 1U);
}

// ------------------------------------------------------------------- 0x11 ECU reset ----

TEST_F(UdsServerTest, HardResetIsAcceptedThenExecutedLater) {
  (void)request({0x10U, 0x03U});
  EXPECT_EQ(request({0x11U, 0x01U}), (Bytes{0x51U, 0x01U}));
  EXPECT_EQ(server.session(), Session::Default);
  const auto reset = server.takePendingReset();
  ASSERT_TRUE(reset.has_value());
  EXPECT_EQ(*reset, ResetType::HardReset);
  EXPECT_FALSE(server.takePendingReset().has_value());
}

TEST_F(UdsServerTest, SoftResetOnlyInExtendedSession) {
  EXPECT_EQ(request({0x11U, 0x03U}), nrc(0x11U, Nrc::SubFunctionNotSupportedInActiveSession));
  EXPECT_FALSE(server.takePendingReset().has_value());
  (void)request({0x10U, 0x03U});
  EXPECT_EQ(request({0x11U, 0x03U}), (Bytes{0x51U, 0x03U}));
  EXPECT_EQ(server.takePendingReset(), ResetType::SoftReset);
}

TEST_F(UdsServerTest, ResetErrors) {
  EXPECT_EQ(request({0x11U, 0x02U}), nrc(0x11U, Nrc::SubFunctionNotSupported));  // keyOffOnReset
  EXPECT_EQ(request({0x11U}), nrc(0x11U, Nrc::IncorrectMessageLengthOrInvalidFormat));
}

// ---------------------------------------------------------------- 0x22 read DID ----

TEST_F(UdsServerTest, ReadSingleAndMultipleDids) {
  const Bytes vin = request({0x22U, 0xF1U, 0x90U});
  ASSERT_EQ(vin.size(), 3U + 17U);
  EXPECT_EQ(std::string(vin.begin() + 3, vin.end()), "VECUSIM0000000001");
  EXPECT_EQ(request({0x22U, 0x01U, 0x00U, 0x01U, 0x00U}),
            (Bytes{0x62U, 0x01U, 0x00U, 0x1FU, 0x40U, 0x01U, 0x00U, 0x1FU, 0x40U}));
}

TEST_F(UdsServerTest, UnsupportedDidsAreOmittedOrRejected) {
  EXPECT_EQ(request({0x22U, 0x01U, 0x00U, 0xAAU, 0xAAU}), (Bytes{0x62U, 0x01U, 0x00U, 0x1FU, 0x40U}));
  EXPECT_EQ(request({0x22U, 0xAAU, 0xAAU}), nrc(0x22U, Nrc::RequestOutOfRange));
}

TEST_F(UdsServerTest, SessionRestrictedDid) {
  EXPECT_EQ(request({0x22U, 0xF1U, 0x8CU}), nrc(0x22U, Nrc::RequestOutOfRange));
  (void)request({0x10U, 0x03U});
  EXPECT_EQ(request({0x22U, 0xF1U, 0x8CU}), (Bytes{0x62U, 0xF1U, 0x8CU, 'S', 'N', '0', '1'}));
}

TEST_F(UdsServerTest, ReadDidLengthChecks) {
  EXPECT_EQ(request({0x22U, 0xF1U}), nrc(0x22U, Nrc::IncorrectMessageLengthOrInvalidFormat));
  EXPECT_EQ(request({0x22U}), nrc(0x22U, Nrc::IncorrectMessageLengthOrInvalidFormat));
  Bytes too_many{0x22U};
  for (int i = 0; i < 9; ++i) {
    too_many.push_back(0x01U);
    too_many.push_back(0x00U);
  }
  EXPECT_EQ(request(too_many), nrc(0x22U, Nrc::IncorrectMessageLengthOrInvalidFormat));
}

TEST_F(UdsServerTest, ResponseTooLongForBuffer) {
  std::array<std::uint8_t, 10U> small{};
  const Bytes req{0x22U, 0xF1U, 0x90U};
  ASSERT_EQ(server.handleRequest(req.data(), req.size(), false, at(0), small.data(), small.size()), 3U);
  EXPECT_EQ(small[2], static_cast<std::uint8_t>(Nrc::ResponseTooLong));
}

TEST_F(UdsServerTest, DidRegistrationRules) {
  EXPECT_FALSE(server.addDid(0xF190U, 1U, kSessionAll, [](std::uint8_t*) {}));  // duplicate
  EXPECT_FALSE(server.addDid(0x1234U, 0U, kSessionAll, [](std::uint8_t*) {}));
  EXPECT_FALSE(server.addDid(0x1234U, 65U, kSessionAll, [](std::uint8_t*) {}));
  EXPECT_FALSE(server.addDid(0x1234U, 1U, kSessionAll, UdsServer::DidReader{}));
}

// ----------------------------------------------------------- 0x19 / 0x14 DTCs ----

TEST_F(UdsServerTest, ReadDtcCountByStatusMask) {
  (void)dtcs.reportTestResult(kDtcSpeed, true);
  EXPECT_EQ(request({0x19U, 0x01U, 0x08U}), (Bytes{0x59U, 0x01U, 0x7FU, 0x01U, 0x00U, 0x01U}));
  EXPECT_EQ(request({0x19U, 0x01U, 0x80U}), (Bytes{0x59U, 0x01U, 0x7FU, 0x01U, 0x00U, 0x00U}));  // unsupported bit
}

TEST_F(UdsServerTest, ReadDtcByStatusMask) {
  (void)dtcs.reportTestResult(kDtcSpeed, true);
  EXPECT_EQ(request({0x19U, 0x02U, 0x09U}), (Bytes{0x59U, 0x02U, 0x7FU, 0xC1U, 0x21U, 0x87U, 0x2FU}));
  (void)dtcs.reportTestResult(kDtcSpeed, false);
  EXPECT_EQ(request({0x19U, 0x02U, 0x01U}), (Bytes{0x59U, 0x02U, 0x7FU}));  // no longer testFailed
}

TEST_F(UdsServerTest, ReadSupportedDtcs) {
  EXPECT_EQ(request({0x19U, 0x0AU}),
            (Bytes{0x59U, 0x0AU, 0x7FU, 0xC1U, 0x21U, 0x87U, 0x50U, 0x40U, 0x31U, 0x64U, 0x50U}));
}

TEST_F(UdsServerTest, ReadDtcErrors) {
  EXPECT_EQ(request({0x19U, 0x04U, 0x00U}), nrc(0x19U, Nrc::SubFunctionNotSupported));
  EXPECT_EQ(request({0x19U, 0x02U}), nrc(0x19U, Nrc::IncorrectMessageLengthOrInvalidFormat));
  EXPECT_EQ(request({0x19U, 0x0AU, 0x00U}), nrc(0x19U, Nrc::IncorrectMessageLengthOrInvalidFormat));
}

TEST_F(UdsServerTest, ClearAllAndSingleDtc) {
  (void)dtcs.reportTestResult(kDtcSpeed, true);
  (void)dtcs.reportTestResult(kDtcPlausibility, true);
  EXPECT_EQ(request({0x14U, 0xC1U, 0x21U, 0x87U}), (Bytes{0x54U}));
  std::uint8_t status = 0U;
  ASSERT_TRUE(dtcs.status(kDtcSpeed, status));
  EXPECT_EQ(status, faults::dtc_status::kInitial);
  ASSERT_TRUE(dtcs.status(kDtcPlausibility, status));
  EXPECT_NE(status, faults::dtc_status::kInitial);
  EXPECT_EQ(request({0x14U, 0xFFU, 0xFFU, 0xFFU}), (Bytes{0x54U}));
  EXPECT_EQ(request({0x19U, 0x01U, 0x08U}), (Bytes{0x59U, 0x01U, 0x7FU, 0x01U, 0x00U, 0x00U}));
}

TEST_F(UdsServerTest, ClearHandlerReceivesClearedGroup) {
  std::vector<faults::DtcCode> cleared;
  server.setClearHandler([&cleared](faults::DtcCode code) { cleared.push_back(code); });
  (void)request({0x14U, 0xFFU, 0xFFU, 0xFFU});
  (void)request({0x14U, 0x40U, 0x31U, 0x64U});
  (void)request({0x14U, 0x12U, 0x34U, 0x56U});  // unknown: NRC, handler not called
  EXPECT_EQ(cleared, (std::vector<faults::DtcCode>{faults::kDtcGroupAll, kDtcPlausibility}));
}

TEST_F(UdsServerTest, ClearErrors) {
  EXPECT_EQ(request({0x14U, 0x12U, 0x34U, 0x56U}), nrc(0x14U, Nrc::RequestOutOfRange));
  EXPECT_EQ(request({0x14U, 0xFFU, 0xFFU}), nrc(0x14U, Nrc::IncorrectMessageLengthOrInvalidFormat));
}

// ------------------------------------------------------------- 0x3E tester present ----

TEST_F(UdsServerTest, TesterPresent) {
  EXPECT_EQ(request({0x3EU, 0x00U}), (Bytes{0x7EU, 0x00U}));
  EXPECT_EQ(request({0x3EU}), nrc(0x3EU, Nrc::IncorrectMessageLengthOrInvalidFormat));
  const UdsStatistics& s = server.statistics();
  EXPECT_EQ(s.requests, 2U);
  EXPECT_EQ(s.positive_responses, 1U);
  EXPECT_EQ(s.negative_responses, 1U);
}

}  // namespace
}  // namespace vecu::diag
