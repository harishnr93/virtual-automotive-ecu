// CRC32, DTC aging/import and the NVM file (round trip, corruption detection, atomic write).

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "core/crc32.hpp"
#include "faults/dtc_store.hpp"
#include "faults/nvm_storage.hpp"

namespace vecu::faults {
namespace {

using namespace dtc_status;  // NOLINT(google-build-using-namespace)

TEST(Crc32, StandardCheckValues) {
  const std::string check = "123456789";
  EXPECT_EQ(core::crc32(reinterpret_cast<const std::uint8_t*>(check.data()), check.size()), 0xCBF43926U);
  EXPECT_EQ(core::crc32(nullptr, 0U), 0x00000000U);
  const std::uint8_t zero = 0U;
  EXPECT_EQ(core::crc32(&zero, 1U), 0xD202EF8DU);
}

TEST(Crc32, IncrementalEqualsOneShot) {
  const std::string text = "Virtual Automotive ECU";
  const auto* data = reinterpret_cast<const std::uint8_t*>(text.data());
  std::uint32_t crc = core::crc32Init();
  crc = core::crc32Update(crc, data, 7U);
  crc = core::crc32Update(crc, data + 7, text.size() - 7U);
  EXPECT_EQ(core::crc32Final(crc), core::crc32(data, text.size()));
}

TEST(DtcAging, ConfirmedDtcAgesOutAfterThresholdCleanCycles) {
  DtcStore store(3U);
  ASSERT_TRUE(store.registerDtc(0xC12187U));
  (void)store.reportTestResult(0xC12187U, true);
  (void)store.reportTestResult(0xC12187U, false);
  std::uint8_t status = 0U;
  for (int cycle = 1; cycle <= 3; ++cycle) {
    store.startOperationCycle();  // ends a cycle; the first one contained the failure
    (void)store.reportTestResult(0xC12187U, false);
    ASSERT_TRUE(store.status(0xC12187U, status));
    EXPECT_NE(status & kConfirmedDtc, 0U) << "cycle " << cycle;
  }
  store.startOperationCycle();  // third clean cycle completed
  ASSERT_TRUE(store.status(0xC12187U, status));
  EXPECT_EQ(status & kConfirmedDtc, 0U);
  EXPECT_NE(status & kTestFailedSinceLastClear, 0U);  // history since clear is kept
}

TEST(DtcAging, FailureResetsAgingCounter) {
  DtcStore store(2U);
  ASSERT_TRUE(store.registerDtc(0x403164U));
  (void)store.reportTestResult(0x403164U, true);
  store.startOperationCycle();
  store.startOperationCycle();  // 1 clean cycle
  (void)store.reportTestResult(0x403164U, true);
  store.startOperationCycle();  // failed in this cycle: counter back to 0
  store.startOperationCycle();  // 1 clean cycle again
  std::uint8_t status = 0U;
  ASSERT_TRUE(store.status(0x403164U, status));
  EXPECT_NE(status & kConfirmedDtc, 0U);
}

TEST(DtcStoreImport, RestoresKnownCodesAndIgnoresUnknown) {
  DtcStore store;
  ASSERT_TRUE(store.registerDtc(0xC12187U));
  const std::vector<DtcRecord> saved{{0xC12187U, 0x2EU, 4U, 1U}, {0x123456U, 0x09U, 1U, 0U}};
  EXPECT_EQ(store.importRecords(saved.data(), saved.size()), 1U);
  std::array<DtcRecord, DtcStore::kCapacity> out{};
  ASSERT_EQ(store.snapshot(out.data(), out.size(), 0U), 1U);
  EXPECT_EQ(out[0].status, 0x2EU);
  EXPECT_EQ(out[0].occurrence_count, 4U);
  EXPECT_EQ(out[0].aging_counter, 1U);
}

class NvmTest : public ::testing::Test {
 protected:
  void TearDown() override {
    (void)std::remove(path.c_str());
    (void)std::remove((path + ".tmp").c_str());
  }
  std::string path = ::testing::TempDir() + "vecu_nvm_" + std::to_string(::getpid()) + ".bin";

  static NvmData sample() {
    NvmData data;
    data.operation_cycles = 7U;
    data.reset_count = 3U;
    data.last_reset_reason = ResetReason::Watchdog;
    data.records[0] = DtcRecord{0xC12187U, 0x2EU, 2U, 1U};
    data.records[1] = DtcRecord{0xF00047U, 0x2FU, 1U, 0U};
    data.record_count = 2U;
    return data;
  }
  std::vector<char> readFile() const {
    std::ifstream file(path, std::ios::binary);
    return std::vector<char>((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  }
  void writeFile(const std::vector<char>& bytes) const {
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    file.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  }
};

TEST_F(NvmTest, RoundTripAndAtomicWrite) {
  ASSERT_EQ(saveNvm(path, sample()), NvmStatus::Ok);
  EXPECT_EQ(readFile().size(), 20U + 2U * 8U + 4U);
  EXPECT_FALSE(std::ifstream(path + ".tmp").good());  // temp file renamed away
  NvmData loaded;
  ASSERT_EQ(loadNvm(path, loaded), NvmStatus::Ok);
  EXPECT_EQ(loaded.operation_cycles, 7U);
  EXPECT_EQ(loaded.reset_count, 3U);
  EXPECT_EQ(loaded.last_reset_reason, ResetReason::Watchdog);
  ASSERT_EQ(loaded.record_count, 2U);
  EXPECT_EQ(loaded.records[1].code, 0xF00047U);
  EXPECT_EQ(loaded.records[1].status, 0x2FU);
  EXPECT_EQ(loaded.records[0].occurrence_count, 2U);
}

TEST_F(NvmTest, MissingFileIsNotFound) {
  NvmData data;
  EXPECT_EQ(loadNvm(path, data), NvmStatus::NotFound);
}

TEST_F(NvmTest, EverySingleBitFlipIsDetected) {
  ASSERT_EQ(saveNvm(path, sample()), NvmStatus::Ok);
  const std::vector<char> good = readFile();
  for (std::size_t byte = 0U; byte < good.size(); ++byte) {
    for (int bit = 0; bit < 8; ++bit) {
      std::vector<char> bad = good;
      bad[byte] = static_cast<char>(bad[byte] ^ (1 << bit));
      writeFile(bad);
      NvmData data;
      ASSERT_EQ(loadNvm(path, data), NvmStatus::Corrupted) << "byte " << byte << " bit " << bit;
    }
  }
}

TEST_F(NvmTest, TruncationAndForeignFilesAreDetected) {
  ASSERT_EQ(saveNvm(path, sample()), NvmStatus::Ok);
  std::vector<char> bytes = readFile();
  bytes.resize(bytes.size() - 5U);  // torn write
  writeFile(bytes);
  NvmData data;
  EXPECT_EQ(loadNvm(path, data), NvmStatus::Corrupted);
  writeFile({'h', 'e', 'l', 'l', 'o'});
  EXPECT_EQ(loadNvm(path, data), NvmStatus::Corrupted);
  EXPECT_EQ(data.record_count, 0U);  // output untouched on failure
}

TEST_F(NvmTest, UnwritableLocationReportsIoError) {
  EXPECT_EQ(saveNvm("/nonexistent-dir/nvm.bin", sample()), NvmStatus::IoError);
}

}  // namespace
}  // namespace vecu::faults
