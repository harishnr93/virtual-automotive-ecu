#include "core/os_port.hpp"

#include <gtest/gtest.h>
#include <pthread.h>
#include <time.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace vecu::os {
namespace {

using std::chrono::milliseconds;

TEST(OsPort, SteadyClockIsMonotonic) {
  // sleepUntil() relies on steady_clock and CLOCK_MONOTONIC sharing the same epoch.
  timespec ts{};
  ASSERT_EQ(clock_gettime(CLOCK_MONOTONIC, &ts), 0);
  const auto monotonic_ns = static_cast<long long>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
  const auto steady_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(core::now().time_since_epoch()).count();
  EXPECT_LT(std::llabs(steady_ns - monotonic_ns), 5000000LL);  // same clock, read ~simultaneously
}

TEST(OsPort, SleepUntilWakesAtOrAfterAbsoluteTime) {
  const core::TimePoint wake = core::now() + milliseconds{20};
  sleepUntil(wake);
  EXPECT_GE(core::now(), wake);
}

TEST(OsPort, SleepUntilPastTimeReturnsImmediately) {
  const core::TimePoint before = core::now();
  sleepUntil(before - milliseconds{100});
  EXPECT_LT(core::now() - before, milliseconds{50});
}

TEST(OsThread, RunsEntryAndJoins) {
  std::atomic<bool> ran{false};
  std::string name;
  {
    Thread thread;
    ThreadConfig config;
    config.name = "VeryLongThreadNameForTest";  // truncated to 15 characters
    ASSERT_TRUE(thread.start(config, [&ran, &name] {
      std::array<char, 16U> buffer{};
      (void)pthread_getname_np(pthread_self(), buffer.data(), buffer.size());
      name = buffer.data();
      ran = true;
    }));
    EXPECT_TRUE(thread.joinable());
    EXPECT_FALSE(thread.start(config, [] {}));  // cannot start twice
  }  // destructor joins
  EXPECT_TRUE(ran);
  EXPECT_EQ(name, "VeryLongThreadN");
}

TEST(OsThread, RejectsEmptyEntry) {
  Thread thread;
  EXPECT_FALSE(thread.start(ThreadConfig{}, Thread::Entry{}));
}

TEST(OsThread, RealtimeRequestAlwaysYieldsARunningThread) {
  // With rtprio permission the thread gets SCHED_FIFO; without it, it must still run
  // (fallback to SCHED_OTHER) and report why.
  std::atomic<int> policy{-1};
  Thread thread;
  ThreadConfig config;
  config.name = "RtProbe";
  config.priority = 10;
  config.request_realtime = true;
  ASSERT_TRUE(thread.start(config, [&policy] {
    int current = 0;
    sched_param parameters{};
    (void)pthread_getschedparam(pthread_self(), &current, &parameters);
    policy = current;
  }));
  thread.join();
  if (thread.schedulingClass() == SchedulingClass::RealTimeFifo) {
    EXPECT_EQ(policy.load(), SCHED_FIFO);
    EXPECT_EQ(thread.realtimeError(), 0);
  } else {
    EXPECT_EQ(policy.load(), SCHED_OTHER);
    EXPECT_NE(thread.realtimeError(), 0);
  }
}

TEST(PiMutex, UsesPriorityInheritanceAndExcludesMutually) {
  PiMutex mutex;
  EXPECT_TRUE(mutex.priorityInheritance());

  long counter = 0;  // deliberately non-atomic: only the mutex protects it
  constexpr int kThreads = 4;
  constexpr int kIterations = 20000;
  std::vector<std::unique_ptr<Thread>> threads;
  for (int i = 0; i < kThreads; ++i) {
    threads.push_back(std::make_unique<Thread>());
    ASSERT_TRUE(threads.back()->start(ThreadConfig{}, [&mutex, &counter] {
      for (int k = 0; k < kIterations; ++k) {
        const std::lock_guard<PiMutex> lock(mutex);
        ++counter;
      }
    }));
  }
  threads.clear();  // join
  EXPECT_EQ(counter, static_cast<long>(kThreads) * kIterations);
}

TEST(PiMutex, TryLockFailsWhileHeldByAnotherThread) {
  PiMutex mutex;
  mutex.lock();
  std::atomic<bool> acquired{true};
  {
    Thread thread;
    ASSERT_TRUE(thread.start(ThreadConfig{}, [&mutex, &acquired] { acquired = mutex.try_lock(); }));
  }
  EXPECT_FALSE(acquired);
  mutex.unlock();
  EXPECT_TRUE(mutex.try_lock());
  mutex.unlock();
}

}  // namespace
}  // namespace vecu::os
