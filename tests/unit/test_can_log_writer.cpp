#include "can/can_log_writer.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdio>
#include <fstream>
#include <string>

namespace vecu::can {
namespace {

CanFrame sampleFrame() {
  CanFrame frame;
  frame.id = 0x100U;
  frame.dlc = 2U;
  frame.data[0] = 0x12U;
  frame.data[1] = 0xABU;
  return frame;
}

TEST(CanLogWriter, FormatsCandumpLogLine) {
  EXPECT_EQ(formatCandumpLogLine(sampleFrame(), 1697812345123456LL, "vcan0"),
            "(1697812345.123456) vcan0 100#12AB");
  EXPECT_EQ(formatCandumpLogLine(sampleFrame(), 5000001LL, "vbus0"), "(0000000005.000001) vbus0 100#12AB");
}

TEST(CanLogWriter, WritesFile) {
  const std::string path = ::testing::TempDir() + "vecu_log_" + std::to_string(::getpid()) + ".log";
  {
    CanLogWriter writer;
    ASSERT_EQ(writer.open(path, "vbus0"), CanStatus::Ok);
    EXPECT_TRUE(writer.isOpen());
    writer.write(sampleFrame(), 1000000LL);
    writer.write(sampleFrame(), 2000000LL);
  }  // destructor flushes and closes the file (RAII)
  std::ifstream file(path);
  std::string line1;
  std::string line2;
  ASSERT_TRUE(std::getline(file, line1));
  ASSERT_TRUE(std::getline(file, line2));
  EXPECT_EQ(line1, "(0000000001.000000) vbus0 100#12AB");
  EXPECT_EQ(line2, "(0000000002.000000) vbus0 100#12AB");
  (void)std::remove(path.c_str());
}

TEST(CanLogWriter, ReportsUnwritablePath) {
  CanLogWriter writer;
  EXPECT_EQ(writer.open("/nonexistent-dir/x.log", "vbus0"), CanStatus::IoError);
  EXPECT_FALSE(writer.isOpen());
  writer.write(sampleFrame(), 0);  // must be a harmless no-op
}

}  // namespace
}  // namespace vecu::can
