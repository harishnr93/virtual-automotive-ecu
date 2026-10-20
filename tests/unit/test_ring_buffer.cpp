#include "core/ring_buffer.hpp"

#include <gtest/gtest.h>

namespace vecu::core {
namespace {

TEST(RingBuffer, StartsEmpty) {
  RingBuffer<int, 3U> buffer;
  EXPECT_TRUE(buffer.empty());
  EXPECT_FALSE(buffer.full());
  EXPECT_EQ(buffer.size(), 0U);
  EXPECT_EQ(buffer.capacity(), 3U);
  int value = 0;
  EXPECT_FALSE(buffer.pop(value));
}

TEST(RingBuffer, PreservesFifoOrderAcrossWrapAround) {
  RingBuffer<int, 3U> buffer;
  int value = 0;
  for (int round = 0; round < 5; ++round) {  // forces head/tail to wrap several times
    ASSERT_TRUE(buffer.push(round * 10 + 1));
    ASSERT_TRUE(buffer.push(round * 10 + 2));
    ASSERT_TRUE(buffer.pop(value));
    EXPECT_EQ(value, round * 10 + 1);
    ASSERT_TRUE(buffer.pop(value));
    EXPECT_EQ(value, round * 10 + 2);
  }
  EXPECT_TRUE(buffer.empty());
}

TEST(RingBuffer, RejectsPushWhenFullWithoutOverwriting) {
  RingBuffer<int, 2U> buffer;
  ASSERT_TRUE(buffer.push(1));
  ASSERT_TRUE(buffer.push(2));
  EXPECT_TRUE(buffer.full());
  EXPECT_FALSE(buffer.push(3));
  int value = 0;
  ASSERT_TRUE(buffer.pop(value));
  EXPECT_EQ(value, 1);  // oldest element kept, newest dropped
}

TEST(RingBuffer, ClearEmptiesBuffer) {
  RingBuffer<int, 2U> buffer;
  ASSERT_TRUE(buffer.push(1));
  buffer.clear();
  EXPECT_TRUE(buffer.empty());
  ASSERT_TRUE(buffer.push(7));
  int value = 0;
  ASSERT_TRUE(buffer.pop(value));
  EXPECT_EQ(value, 7);
}

}  // namespace
}  // namespace vecu::core
