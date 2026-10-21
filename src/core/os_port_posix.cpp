#include "core/os_port.hpp"

#include <sched.h>
#include <sys/mman.h>

#include <cerrno>
#include <chrono>
#include <ctime>
#include <memory>
#include <utility>

namespace vecu::os {

namespace {

constexpr std::size_t kMaxThreadNameLength = 15U;  // Linux limit (16 bytes incl. terminator)

/// Everything the new thread needs; ownership passes to the thread.
struct StartContext {
  Thread::Entry entry;
  std::string name;
};

extern "C" void* threadTrampoline(void* argument) {
  const std::unique_ptr<StartContext> context(static_cast<StartContext*>(argument));
  // The thread names itself *before* running the entry function. Naming it from the
  // creator after pthread_create() would race with the entry function.
  (void)pthread_setname_np(pthread_self(), context->name.c_str());
  context->entry();
  return nullptr;
}

/// RAII wrapper so pthread_attr_destroy runs on every path.
class ThreadAttributes {
 public:
  ThreadAttributes() noexcept : valid_(pthread_attr_init(&attributes_) == 0) {}
  ~ThreadAttributes() {
    if (valid_) {
      (void)pthread_attr_destroy(&attributes_);
    }
  }
  ThreadAttributes(const ThreadAttributes&) = delete;
  ThreadAttributes& operator=(const ThreadAttributes&) = delete;
  ThreadAttributes(ThreadAttributes&&) = delete;
  ThreadAttributes& operator=(ThreadAttributes&&) = delete;

  [[nodiscard]] bool valid() const noexcept { return valid_; }
  [[nodiscard]] pthread_attr_t* get() noexcept { return &attributes_; }

 private:
  pthread_attr_t attributes_{};
  bool valid_{false};
};

int clampPriority(int priority) noexcept {
  if (priority < kMinPriority) {
    return kMinPriority;
  }
  return (priority > kMaxPriority) ? kMaxPriority : priority;
}

}  // namespace

const char* toString(SchedulingClass scheduling_class) noexcept {
  switch (scheduling_class) {
    case SchedulingClass::RealTimeFifo:
      return "SCHED_FIFO";
    case SchedulingClass::Normal:
      return "SCHED_OTHER";
  }
  return "Unknown";
}

// ------------------------------------------------------------------------------- Thread ----

Thread::~Thread() { join(); }

bool Thread::start(const ThreadConfig& config, Entry entry) {
  if (started_ || !entry) {
    return false;
  }
  auto context = std::make_unique<StartContext>();
  context->entry = std::move(entry);
  context->name = config.name.substr(0U, kMaxThreadNameLength);
  bool created = false;

  if (config.request_realtime) {
    ThreadAttributes attributes;
    sched_param parameters{};
    parameters.sched_priority = clampPriority(config.priority);
    if (attributes.valid() &&
        (pthread_attr_setinheritsched(attributes.get(), PTHREAD_EXPLICIT_SCHED) == 0) &&
        (pthread_attr_setschedpolicy(attributes.get(), SCHED_FIFO) == 0) &&
        (pthread_attr_setschedparam(attributes.get(), &parameters) == 0)) {
      const int rc = pthread_create(&handle_, attributes.get(), threadTrampoline, context.get());
      if (rc == 0) {
        created = true;
        scheduling_class_ = SchedulingClass::RealTimeFifo;
      } else {
        realtime_error_ = rc;  // typically EPERM: no real-time privilege
      }
    } else {
      realtime_error_ = EINVAL;
    }
  }

  if (!created) {
    if (pthread_create(&handle_, nullptr, threadTrampoline, context.get()) != 0) {
      return false;
    }
    scheduling_class_ = SchedulingClass::Normal;
  }
  (void)context.release();  // ownership passed to threadTrampoline
  started_ = true;
  return true;
}

void Thread::join() noexcept {
  if (started_) {
    (void)pthread_join(handle_, nullptr);
    started_ = false;
  }
}

// ---------------------------------------------------------------------------- sleepUntil ----

void sleepUntil(core::TimePoint wake_time) noexcept {
  // libstdc++'s steady_clock is CLOCK_MONOTONIC (verified by the OsPort.SteadyClockIsMonotonic
  // test), so its epoch-relative value can be passed to clock_nanosleep directly.
  const auto since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(wake_time.time_since_epoch());
  if (since_epoch.count() <= 0) {
    return;
  }
  timespec ts{};
  ts.tv_sec = static_cast<std::time_t>(since_epoch.count() / 1000000000LL);
  ts.tv_nsec = static_cast<long>(since_epoch.count() % 1000000000LL);
  // Restart after signal interruptions; absolute time makes the restart exact.
  while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {
  }
}

// ------------------------------------------------------------------------------ PiMutex ----

PiMutex::PiMutex() noexcept {
  pthread_mutexattr_t attributes{};
  if (pthread_mutexattr_init(&attributes) == 0) {
    priority_inheritance_ = (pthread_mutexattr_setprotocol(&attributes, PTHREAD_PRIO_INHERIT) == 0) &&
                            (pthread_mutex_init(&mutex_, &attributes) == 0);
    (void)pthread_mutexattr_destroy(&attributes);
  }
  if (!priority_inheritance_) {
    (void)pthread_mutex_init(&mutex_, nullptr);  // fall back to a plain mutex
  }
}

PiMutex::~PiMutex() { (void)pthread_mutex_destroy(&mutex_); }

void PiMutex::lock() noexcept { (void)pthread_mutex_lock(&mutex_); }

void PiMutex::unlock() noexcept { (void)pthread_mutex_unlock(&mutex_); }

bool PiMutex::try_lock() noexcept { return pthread_mutex_trylock(&mutex_) == 0; }

// --------------------------------------------------------------------- lockProcessMemory ----

bool lockProcessMemory() noexcept { return mlockall(MCL_CURRENT | MCL_FUTURE) == 0; }

}  // namespace vecu::os
