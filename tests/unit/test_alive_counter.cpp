#include "can/alive_counter.hpp"

#include <gtest/gtest.h>

namespace vecu::can {
namespace {

TEST(AliveCounter, GeneratorWrapsAtSixteen) {
  AliveCounterGenerator generator;
  for (std::uint8_t expected = 0U; expected < 16U; ++expected) {
    EXPECT_EQ(generator.next(), expected);
  }
  EXPECT_EQ(generator.next(), 0U);
}

TEST(AliveCounter, CheckerAcceptsGeneratorSequenceIncludingWrap) {
  AliveCounterGenerator generator;
  AliveCounterChecker checker;
  EXPECT_EQ(checker.check(generator.next()), AliveCheckResult::First);
  for (int i = 0; i < 40; ++i) {
    EXPECT_EQ(checker.check(generator.next()), AliveCheckResult::Ok);
  }
  EXPECT_EQ(checker.lostFrames(), 0U);
}

TEST(AliveCounter, DetectsRepeatedValue) {
  AliveCounterChecker checker;
  (void)checker.check(5U);
  EXPECT_EQ(checker.check(5U), AliveCheckResult::Repeated);
}

TEST(AliveCounter, DetectsSkippedValuesAndCountsLostFrames) {
  AliveCounterChecker checker;
  (void)checker.check(14U);
  EXPECT_EQ(checker.check(2U), AliveCheckResult::Skipped);  // 15, 0, 1 lost
  EXPECT_EQ(checker.lostFrames(), 3U);
  EXPECT_EQ(checker.check(3U), AliveCheckResult::Ok);
}

TEST(AliveCounter, RejectsInvalidValue) {
  AliveCounterChecker checker;
  EXPECT_EQ(checker.check(16U), AliveCheckResult::Invalid);
}

TEST(AliveCounter, ResetResynchronises) {
  AliveCounterChecker checker;
  (void)checker.check(1U);
  checker.reset();
  EXPECT_EQ(checker.check(9U), AliveCheckResult::First);
  EXPECT_EQ(checker.check(10U), AliveCheckResult::Ok);
}

}  // namespace
}  // namespace vecu::can
