// ISO-TP protocol tests with deterministic time and exact frame-level expectations.

#include "diagnostics/isotp.hpp"

#include <gtest/gtest.h>

#include <deque>
#include <numeric>
#include <vector>

namespace vecu::diag {
namespace {

using std::chrono::milliseconds;

constexpr std::uint32_t kTesterTx = 0x7E0U;
constexpr std::uint32_t kEcuTx = 0x7E8U;

core::TimePoint at(int ms) { return core::TimePoint{} + milliseconds{ms}; }

can::CanFrame frame(std::uint32_t id, std::initializer_list<std::uint8_t> bytes) {
  can::CanFrame f;
  f.id = id;
  f.dlc = 8U;
  f.data.fill(0xCCU);
  std::size_t i = 0U;
  for (const std::uint8_t b : bytes) {
    f.data[i++] = b;
  }
  return f;
}

std::vector<std::uint8_t> pattern(std::size_t n) {
  std::vector<std::uint8_t> v(n);
  std::iota(v.begin(), v.end(), static_cast<std::uint8_t>(0));
  return v;
}

/// Two endpoints (tester <-> ECU) connected by frame queues.
struct Link {
  explicit Link(IsoTpConfig ecu_cfg = {}, IsoTpConfig tester_cfg = {}) {
    tester_cfg.tx_id = kTesterTx;
    tester_cfg.rx_id = kEcuTx;
    ecu_cfg.tx_id = kEcuTx;
    ecu_cfg.rx_id = kTesterTx;
    tester = std::make_unique<IsoTpEndpoint>(tester_cfg, [this](const can::CanFrame& f) {
      to_ecu.push_back(f);
      return true;
    });
    ecu = std::make_unique<IsoTpEndpoint>(ecu_cfg, [this](const can::CanFrame& f) {
      to_tester.push_back(f);
      return true;
    });
  }

  /// Delivers queued frames in both directions until quiet (no time passes).
  void pump(core::TimePoint now) {
    while (!to_ecu.empty() || !to_tester.empty()) {
      while (!to_ecu.empty()) {
        const can::CanFrame f = to_ecu.front();
        to_ecu.pop_front();
        ++frames;
        ecu->onFrame(f, now);
      }
      while (!to_tester.empty()) {
        const can::CanFrame f = to_tester.front();
        to_tester.pop_front();
        ++frames;
        tester->onFrame(f, now);
      }
      tester->poll(now);
      ecu->poll(now);
    }
  }

  std::unique_ptr<IsoTpEndpoint> tester;
  std::unique_ptr<IsoTpEndpoint> ecu;
  std::deque<can::CanFrame> to_ecu;
  std::deque<can::CanFrame> to_tester;
  int frames{0};
};

TEST(IsoTpStMin, DecodesAllRanges) {
  EXPECT_EQ(decodeStMin(0x00U), milliseconds{0});
  EXPECT_EQ(decodeStMin(0x7FU), milliseconds{127});
  EXPECT_EQ(decodeStMin(0xF1U), std::chrono::microseconds{100});
  EXPECT_EQ(decodeStMin(0xF9U), std::chrono::microseconds{900});
  EXPECT_EQ(decodeStMin(0x80U), milliseconds{127});  // reserved
  EXPECT_EQ(decodeStMin(0xFAU), milliseconds{127});  // reserved
}

TEST(IsoTp, SingleFrameLayoutWithPadding) {
  Link link;
  const std::vector<std::uint8_t> req{0x22U, 0xF1U, 0x90U};
  ASSERT_EQ(link.tester->send(req.data(), req.size(), at(0)), IsoTpSendResult::Ok);
  ASSERT_EQ(link.to_ecu.size(), 1U);
  EXPECT_EQ(link.to_ecu.front(), frame(kTesterTx, {0x03U, 0x22U, 0xF1U, 0x90U, 0xCCU, 0xCCU, 0xCCU, 0xCCU}));
  link.pump(at(0));
  ASSERT_TRUE(link.ecu->hasMessage());
  EXPECT_EQ(std::vector<std::uint8_t>(link.ecu->message(), link.ecu->message() + link.ecu->messageLength()), req);
  link.ecu->consumeMessage();
  EXPECT_FALSE(link.ecu->hasMessage());
}

TEST(IsoTp, MultiFrameSequenceMatchesStandard) {
  Link link;
  const auto payload = pattern(20);  // FF(6) + CF(7) + CF(7)
  ASSERT_EQ(link.ecu->send(payload.data(), payload.size(), at(0)), IsoTpSendResult::Ok);
  ASSERT_EQ(link.to_tester.size(), 1U);
  EXPECT_EQ(link.to_tester.front(), frame(kEcuTx, {0x10U, 0x14U, 0, 1, 2, 3, 4, 5}));
  EXPECT_TRUE(link.ecu->txBusy());

  // Tester answers with FC CTS (BS 0, STmin 0) and the ECU sends both CFs.
  link.tester->onFrame(link.to_tester.front(), at(1));
  link.to_tester.pop_front();
  ASSERT_EQ(link.to_ecu.size(), 1U);
  EXPECT_EQ(link.to_ecu.front(), frame(kTesterTx, {0x30U, 0x00U, 0x00U}));
  link.ecu->onFrame(link.to_ecu.front(), at(2));
  link.to_ecu.pop_front();
  ASSERT_EQ(link.to_tester.size(), 2U);
  EXPECT_EQ(link.to_tester[0], frame(kEcuTx, {0x21U, 6, 7, 8, 9, 10, 11, 12}));
  EXPECT_EQ(link.to_tester[1], frame(kEcuTx, {0x22U, 13, 14, 15, 16, 17, 18, 19}));
  link.pump(at(3));
  ASSERT_TRUE(link.tester->hasMessage());
  EXPECT_EQ(std::vector<std::uint8_t>(link.tester->message(), link.tester->message() + 20), payload);
  EXPECT_FALSE(link.ecu->txBusy());
  EXPECT_EQ(link.ecu->messagesSent(), 1U);
}

TEST(IsoTp, MaximumLengthWithSequenceWrapAndBlocks) {
  IsoTpConfig tester_cfg;
  tester_cfg.block_size = 8U;  // receiver requests a new FC every 8 CFs
  Link link({}, tester_cfg);
  const auto payload = pattern(kIsoTpMaxPayload);
  ASSERT_EQ(link.ecu->send(payload.data(), payload.size(), at(0)), IsoTpSendResult::Ok);
  link.pump(at(0));
  ASSERT_TRUE(link.tester->hasMessage());
  ASSERT_EQ(link.tester->messageLength(), kIsoTpMaxPayload);
  EXPECT_TRUE(std::equal(payload.begin(), payload.end(), link.tester->message()));
  const int cfs = static_cast<int>((kIsoTpMaxPayload - 6U + 6U) / 7U);  // 585 CFs
  const int fcs = 1 + (cfs - 1) / 8;                                     // first FC + one per full block
  EXPECT_EQ(link.frames, 1 + cfs + fcs);
  EXPECT_EQ(link.tester->errorCount(), 0U);
}

TEST(IsoTp, StMinPacesConsecutiveFrames) {
  IsoTpConfig tester_cfg;
  tester_cfg.st_min = 5U;  // 5 ms between CFs
  Link link({}, tester_cfg);
  const auto payload = pattern(30);  // FF + 4 CFs
  ASSERT_EQ(link.ecu->send(payload.data(), payload.size(), at(0)), IsoTpSendResult::Ok);
  link.tester->onFrame(link.to_tester.front(), at(0));
  link.to_tester.clear();
  link.ecu->onFrame(link.to_ecu.front(), at(0));  // FC: first CF immediately
  link.to_ecu.clear();
  EXPECT_EQ(link.to_tester.size(), 1U);
  link.ecu->poll(at(4));
  EXPECT_EQ(link.to_tester.size(), 1U);  // STmin not yet elapsed
  link.ecu->poll(at(5));
  EXPECT_EQ(link.to_tester.size(), 2U);
  link.ecu->poll(at(10));
  link.ecu->poll(at(15));
  EXPECT_EQ(link.to_tester.size(), 4U);
  EXPECT_FALSE(link.ecu->txBusy());
}

TEST(IsoTp, FlowControlWaitThenContinue) {
  Link link;
  const auto payload = pattern(10);
  ASSERT_EQ(link.ecu->send(payload.data(), payload.size(), at(0)), IsoTpSendResult::Ok);
  link.to_tester.clear();
  link.ecu->onFrame(frame(kTesterTx, {0x31U, 0, 0}), at(500));  // WAIT restarts N_Bs
  link.ecu->poll(at(1200));
  EXPECT_TRUE(link.ecu->txBusy());  // 700 ms since the WAIT frame: still within N_Bs
  link.ecu->onFrame(frame(kTesterTx, {0x30U, 0, 0}), at(1300));
  EXPECT_FALSE(link.ecu->txBusy());
  EXPECT_EQ(link.to_tester.size(), 1U);
}

TEST(IsoTp, TooManyWaitFramesAbort) {
  IsoTpConfig ecu_cfg;
  ecu_cfg.max_wait_frames = 2U;
  Link link(ecu_cfg);
  const auto payload = pattern(10);
  ASSERT_EQ(link.ecu->send(payload.data(), payload.size(), at(0)), IsoTpSendResult::Ok);
  for (int i = 0; i < 3; ++i) {
    link.ecu->onFrame(frame(kTesterTx, {0x31U, 0, 0}), at(i));
  }
  EXPECT_FALSE(link.ecu->txBusy());
  EXPECT_EQ(link.ecu->takeError(), IsoTpError::WaitLimitExceeded);
}

TEST(IsoTp, OverflowFlowControlAbortsTransmission) {
  Link link;
  const auto payload = pattern(100);
  ASSERT_EQ(link.ecu->send(payload.data(), payload.size(), at(0)), IsoTpSendResult::Ok);
  link.ecu->onFrame(frame(kTesterTx, {0x32U, 0, 0}), at(1));
  EXPECT_FALSE(link.ecu->txBusy());
  EXPECT_EQ(link.ecu->takeError(), IsoTpError::ReceiverOverflow);
  EXPECT_EQ(link.ecu->takeError(), IsoTpError::None);  // taken once
}

TEST(IsoTp, SenderTimesOutWithoutFlowControl) {
  Link link;
  const auto payload = pattern(10);
  ASSERT_EQ(link.ecu->send(payload.data(), payload.size(), at(0)), IsoTpSendResult::Ok);
  EXPECT_EQ(link.ecu->send(payload.data(), payload.size(), at(1)), IsoTpSendResult::Busy);
  link.ecu->poll(at(999));
  EXPECT_TRUE(link.ecu->txBusy());
  link.ecu->poll(at(1000));
  EXPECT_FALSE(link.ecu->txBusy());
  EXPECT_EQ(link.ecu->takeError(), IsoTpError::TimeoutBs);
}

TEST(IsoTp, ReceiverTimesOutWithoutConsecutiveFrame) {
  Link link;
  link.ecu->onFrame(frame(kTesterTx, {0x10U, 0x0AU, 1, 2, 3, 4, 5, 6}), at(0));
  EXPECT_TRUE(link.ecu->rxBusy());
  link.ecu->poll(at(1000));
  EXPECT_FALSE(link.ecu->rxBusy());
  EXPECT_EQ(link.ecu->takeError(), IsoTpError::TimeoutCr);
  link.ecu->onFrame(frame(kTesterTx, {0x21U, 7, 8, 9, 10}), at(1001));  // late CF: ignored
  EXPECT_FALSE(link.ecu->hasMessage());
}

TEST(IsoTp, WrongSequenceNumberAbortsReception) {
  Link link;
  link.ecu->onFrame(frame(kTesterTx, {0x10U, 0x10U, 1, 2, 3, 4, 5, 6}), at(0));
  link.ecu->onFrame(frame(kTesterTx, {0x22U, 7, 8, 9, 10, 11, 12, 13}), at(1));  // expected SN 1
  EXPECT_FALSE(link.ecu->rxBusy());
  EXPECT_FALSE(link.ecu->hasMessage());
  EXPECT_EQ(link.ecu->takeError(), IsoTpError::WrongSequenceNumber);
}

TEST(IsoTp, NewSingleFrameInterruptsReception) {
  Link link;
  link.ecu->onFrame(frame(kTesterTx, {0x10U, 0x10U, 1, 2, 3, 4, 5, 6}), at(0));
  link.ecu->onFrame(frame(kTesterTx, {0x02U, 0x3EU, 0x00U}), at(1));
  ASSERT_TRUE(link.ecu->hasMessage());
  EXPECT_EQ(link.ecu->messageLength(), 2U);
  EXPECT_EQ(link.ecu->takeError(), IsoTpError::UnexpectedPdu);
}

TEST(IsoTp, InvalidFramesAreIgnored) {
  Link link;
  link.ecu->onFrame(frame(kTesterTx, {0x00U}), at(0));                       // SF_DL 0
  link.ecu->onFrame(frame(kTesterTx, {0x08U, 1, 2, 3, 4, 5, 6, 7}), at(0));  // SF_DL 8
  link.ecu->onFrame(frame(kTesterTx, {0x10U, 0x05U, 1, 2, 3, 4, 5, 6}), at(0));  // FF_DL <= 7
  link.ecu->onFrame(frame(kTesterTx, {0x21U, 1, 2}), at(0));                 // CF without FF
  link.ecu->onFrame(frame(kTesterTx, {0x30U, 0, 0}), at(0));                 // FC while not sending
  link.ecu->onFrame(frame(0x123U, {0x02U, 0x3EU, 0x00U}), at(0));            // other identifier
  EXPECT_FALSE(link.ecu->hasMessage());
  EXPECT_FALSE(link.ecu->rxBusy());
  EXPECT_TRUE(link.to_tester.empty());
}

TEST(IsoTp, RejectsInvalidSendLengthAndReportsTxFailure) {
  Link link;
  const auto big = pattern(kIsoTpMaxPayload + 1U);
  EXPECT_EQ(link.ecu->send(big.data(), big.size(), at(0)), IsoTpSendResult::InvalidLength);
  EXPECT_EQ(link.ecu->send(big.data(), 0U, at(0)), IsoTpSendResult::InvalidLength);

  IsoTpConfig cfg;
  IsoTpEndpoint broken(cfg, [](const can::CanFrame&) { return false; });
  const std::uint8_t byte = 0x3EU;
  EXPECT_EQ(broken.send(&byte, 1U, at(0)), IsoTpSendResult::TxFailed);
  EXPECT_EQ(broken.takeError(), IsoTpError::TxFailed);
}

}  // namespace
}  // namespace vecu::diag
