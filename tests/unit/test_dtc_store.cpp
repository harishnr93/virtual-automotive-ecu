#include "faults/dtc_store.hpp"

#include <gtest/gtest.h>

#include <array>

namespace vecu::faults {
namespace {

using namespace dtc_status;  // NOLINT(google-build-using-namespace)

constexpr DtcCode kA = 0xC12187U;
constexpr DtcCode kB = 0x403164U;

class DtcStoreTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ASSERT_TRUE(store.registerDtc(kA));
    ASSERT_TRUE(store.registerDtc(kB));
  }
  std::uint8_t statusOf(DtcCode code) {
    std::uint8_t s = 0U;
    EXPECT_TRUE(store.status(code, s));
    return s;
  }
  DtcStore store;
};

TEST_F(DtcStoreTest, RegistrationRules) {
  EXPECT_FALSE(store.registerDtc(kA));          // duplicate
  EXPECT_FALSE(store.registerDtc(0x1000000U));  // > 24 bit
  EXPECT_FALSE(store.registerDtc(kDtcGroupAll));
  EXPECT_EQ(store.size(), 2U);
  EXPECT_EQ(statusOf(kA), kInitial);  // 0x50: not completed since clear / this cycle
}

TEST_F(DtcStoreTest, FailedTestSetsStatusBits) {
  ASSERT_TRUE(store.reportTestResult(kA, true));
  EXPECT_EQ(statusOf(kA), kTestFailed | kTestFailedThisOperationCycle | kPendingDtc | kConfirmedDtc |
                              kTestFailedSinceLastClear);  // 0x2F
}

TEST_F(DtcStoreTest, PassedTestClearsOnlyTestFailed) {
  (void)store.reportTestResult(kA, true);
  (void)store.reportTestResult(kA, false);
  EXPECT_EQ(statusOf(kA), kTestFailedThisOperationCycle | kPendingDtc | kConfirmedDtc | kTestFailedSinceLastClear);
  (void)store.reportTestResult(kB, false);
  EXPECT_EQ(statusOf(kB), 0x00U);  // tested and passed, never failed
}

TEST_F(DtcStoreTest, OccurrenceCountsFailureTransitions) {
  std::array<DtcRecord, DtcStore::kCapacity> records{};
  for (int i = 0; i < 3; ++i) {
    (void)store.reportTestResult(kA, true);
    (void)store.reportTestResult(kA, true);  // still failed: same occurrence
    (void)store.reportTestResult(kA, false);
  }
  ASSERT_EQ(store.snapshot(records.data(), records.size(), kConfirmedDtc), 1U);
  EXPECT_EQ(records[0].occurrence_count, 3U);
}

TEST_F(DtcStoreTest, ClearSingleAndAll) {
  (void)store.reportTestResult(kA, true);
  (void)store.reportTestResult(kB, true);
  ASSERT_TRUE(store.clear(kA));
  EXPECT_EQ(statusOf(kA), kInitial);
  EXPECT_NE(statusOf(kB), kInitial);
  ASSERT_TRUE(store.clear(kDtcGroupAll));
  EXPECT_EQ(statusOf(kB), kInitial);
  EXPECT_FALSE(store.clear(0x123456U));
}

TEST_F(DtcStoreTest, OperationCycleAgesPendingState) {
  (void)store.reportTestResult(kA, true);
  (void)store.reportTestResult(kA, false);
  store.startOperationCycle();  // failed in the previous cycle: still pending
  EXPECT_EQ(statusOf(kA) & (kPendingDtc | kTestFailedThisOperationCycle | kTestNotCompletedThisOperationCycle),
            kPendingDtc | kTestNotCompletedThisOperationCycle);
  (void)store.reportTestResult(kA, false);
  store.startOperationCycle();  // a whole cycle without failure: no longer pending
  EXPECT_EQ(statusOf(kA) & kPendingDtc, 0U);
  EXPECT_NE(statusOf(kA) & kConfirmedDtc, 0U);  // confirmed stays until cleared
}

TEST_F(DtcStoreTest, SnapshotFiltersByStatusMask) {
  (void)store.reportTestResult(kB, true);
  std::array<DtcRecord, DtcStore::kCapacity> records{};
  ASSERT_EQ(store.snapshot(records.data(), records.size(), kConfirmedDtc), 1U);
  EXPECT_EQ(records[0].code, kB);
  EXPECT_EQ(store.countMatching(kTestFailed), 1U);
  EXPECT_EQ(store.countMatching(0x00U), 2U);  // mask 0: all supported
  EXPECT_EQ(store.snapshot(records.data(), 1U, 0x00U), 1U);  // respects capacity
  EXPECT_FALSE(store.reportTestResult(0x123456U, true));
}

}  // namespace
}  // namespace vecu::faults
