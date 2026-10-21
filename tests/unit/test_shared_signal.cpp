#include "core/shared_signal.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>

#include "core/os_port.hpp"

namespace vecu::core {
namespace {

struct Pair {
  std::int64_t value;
  std::int64_t negated;  // invariant: negated == -value (detects torn reads)
};

TEST(SharedSignal, StartsUnwrittenWithInitialValue) {
  const SharedSignal<int> signal{42};
  const auto sample = signal.read();
  EXPECT_EQ(sample.value, 42);
  EXPECT_EQ(sample.sequence, 0U);
}

TEST(SharedSignal, WriteUpdatesValueTimestampAndSequence) {
  SharedSignal<int> signal;
  const TimePoint t1 = TimePoint{} + std::chrono::milliseconds{10};
  signal.write(7, t1);
  auto sample = signal.read();
  EXPECT_EQ(sample.value, 7);
  EXPECT_EQ(sample.timestamp, t1);
  EXPECT_EQ(sample.sequence, 1U);
  signal.write(8, t1 + std::chrono::milliseconds{10});
  sample = signal.read();
  EXPECT_EQ(sample.value, 8);
  EXPECT_EQ(sample.sequence, 2U);
}

TEST(SharedSignal, ConcurrentReadersNeverSeeTornData) {
  SharedSignal<Pair> signal{Pair{0, 0}};
  std::atomic<bool> stop{false};
  std::atomic<std::uint64_t> torn{0U};
  std::atomic<std::uint64_t> reads{0U};
  std::atomic<std::uint64_t> sequence_went_back{0U};

  os::Thread writer;
  os::Thread reader;
  ASSERT_TRUE(writer.start(os::ThreadConfig{}, [&] {
    for (std::int64_t i = 1; i <= 200000; ++i) {
      signal.write(Pair{i, -i}, now());
    }
    stop = true;
  }));
  ASSERT_TRUE(reader.start(os::ThreadConfig{}, [&] {
    std::uint64_t last_sequence = 0U;
    while (!stop) {
      const auto sample = signal.read();
      if (sample.value.negated != -sample.value.value) {
        torn.fetch_add(1U);
      }
      if (sample.sequence < last_sequence) {
        sequence_went_back.fetch_add(1U);
      }
      last_sequence = sample.sequence;
      reads.fetch_add(1U);
    }
  }));
  writer.join();
  reader.join();
  EXPECT_GT(reads.load(), 0U);
  EXPECT_EQ(torn.load(), 0U);
  EXPECT_EQ(sequence_went_back.load(), 0U);
  EXPECT_EQ(signal.read().sequence, 200000U);
}

}  // namespace
}  // namespace vecu::core
