#include "faults/fault_manager.hpp"

#include <algorithm>
#include <mutex>

namespace vecu::faults {

const char* toString(FaultState state) noexcept {
  switch (state) {
    case FaultState::NotTested:
      return "NotTested";
    case FaultState::PreFailed:
      return "PreFailed";
    case FaultState::Failed:
      return "Failed";
    case FaultState::PrePassed:
      return "PrePassed";
    case FaultState::Passed:
      return "Passed";
  }
  return "Unknown";
}

bool FaultManager::addFault(FaultId id, const FaultConfig& config) {
  const DebounceConfig& d = config.debounce;
  const bool debounce_ok = (d.type == DebounceType::Time)
                               ? ((d.fail_time.count() >= 0) && (d.pass_time.count() >= 0))
                               : ((d.increment > 0) && (d.decrement > 0) && (d.fail_threshold > d.pass_threshold));
  const std::lock_guard<os::PiMutex> lock(mutex_);
  if ((id >= kMaxFaults) || faults_[id].used || !debounce_ok || config.name.empty()) {
    return false;
  }
  if ((config.dtc != 0U) && !dtcs_.registerDtc(config.dtc) && !dtcs_.isSupported(config.dtc)) {
    return false;
  }
  faults_[id] = Entry{true, config, FaultState::NotTested, 0, core::TimePoint{}};
  return true;
}

void FaultManager::transition(FaultId id, Entry& entry, FaultState to, core::TimePoint now) noexcept {
  if (entry.state == to) {
    return;
  }
  FaultEvent& event = events_[event_sequence_ % kEventMemory];
  ++event_sequence_;
  event = FaultEvent{event_sequence_, id, entry.state, to, now};
  entry.state = to;
}

void FaultManager::updateTime(FaultId id, Entry& entry, bool failed, core::TimePoint now) noexcept {
  const DebounceConfig& d = entry.config.debounce;
  switch (entry.state) {
    case FaultState::NotTested:
    case FaultState::Passed:
      if (failed) {
        entry.since = now;
        transition(id, entry, (d.fail_time.count() == 0) ? FaultState::Failed : FaultState::PreFailed, now);
      } else {
        transition(id, entry, FaultState::Passed, now);
      }
      break;
    case FaultState::PreFailed:
      if (!failed) {
        transition(id, entry, FaultState::Passed, now);  // never qualified: debounced away
      } else if ((now - entry.since) >= d.fail_time) {
        transition(id, entry, FaultState::Failed, now);
      } else {
        // still debouncing
      }
      break;
    case FaultState::Failed:
      if (!failed && !entry.config.latched) {
        entry.since = now;
        transition(id, entry, (d.pass_time.count() == 0) ? FaultState::Passed : FaultState::PrePassed, now);
      }
      break;
    case FaultState::PrePassed:
      if (failed) {
        transition(id, entry, FaultState::Failed, now);  // recovery interrupted
      } else if ((now - entry.since) >= d.pass_time) {
        transition(id, entry, FaultState::Passed, now);  // recovery condition met
      } else {
        // still healing
      }
      break;
  }
}

void FaultManager::updateCounter(FaultId id, Entry& entry, bool failed, core::TimePoint now) noexcept {
  const DebounceConfig& d = entry.config.debounce;
  if (entry.config.latched && (entry.state == FaultState::Failed)) {
    return;
  }
  entry.counter = failed ? std::min<std::int32_t>(entry.counter + d.increment, d.fail_threshold)
                         : std::max<std::int32_t>(entry.counter - d.decrement, d.pass_threshold);
  const bool was_active = active(entry.state);
  if (entry.counter >= d.fail_threshold) {
    transition(id, entry, FaultState::Failed, now);
  } else if (entry.counter <= d.pass_threshold) {
    transition(id, entry, FaultState::Passed, now);
  } else {
    transition(id, entry, was_active ? FaultState::PrePassed : FaultState::PreFailed, now);
  }
}

bool FaultManager::report(FaultId id, bool failed, core::TimePoint now) noexcept {
  const std::lock_guard<os::PiMutex> lock(mutex_);
  if ((id >= kMaxFaults) || !faults_[id].used) {
    return false;
  }
  Entry& entry = faults_[id];
  if (entry.config.debounce.type == DebounceType::Time) {
    updateTime(id, entry, failed, now);
  } else {
    updateCounter(id, entry, failed, now);
  }
  // Only qualified states reach the DTC memory. Re-reporting on every call means that an
  // active fault immediately re-fails its DTC after a tester cleared it.
  if ((entry.config.dtc != 0U) && ((entry.state == FaultState::Failed) || (entry.state == FaultState::Passed))) {
    (void)dtcs_.reportTestResult(entry.config.dtc, entry.state == FaultState::Failed);
  }
  return true;
}

void FaultManager::startOperationCycle() noexcept {
  {
    const std::lock_guard<os::PiMutex> lock(mutex_);
    for (Entry& entry : faults_) {
      entry.state = FaultState::NotTested;  // re-test everything in the new cycle
      entry.counter = 0;
    }
  }
  dtcs_.startOperationCycle();
}

void FaultManager::onDtcCleared(DtcCode code_or_group) noexcept {
  const std::lock_guard<os::PiMutex> lock(mutex_);
  for (Entry& entry : faults_) {
    if (entry.used && ((code_or_group == kDtcGroupAll) || (entry.config.dtc == code_or_group))) {
      entry.state = FaultState::NotTested;
      entry.counter = 0;
    }
  }
}

FaultState FaultManager::state(FaultId id) const noexcept {
  const std::lock_guard<os::PiMutex> lock(mutex_);
  return ((id < kMaxFaults) && faults_[id].used) ? faults_[id].state : FaultState::NotTested;
}

bool FaultManager::isActive(FaultId id) const noexcept { return active(state(id)); }

bool FaultManager::functionInhibited() const noexcept {
  const std::lock_guard<os::PiMutex> lock(mutex_);
  return std::any_of(faults_.begin(), faults_.end(), [](const Entry& entry) {
    return entry.used && entry.config.inhibits_function && active(entry.state);
  });
}

std::uint16_t FaultManager::activeMask() const noexcept {
  const std::lock_guard<os::PiMutex> lock(mutex_);
  unsigned int mask = 0U;
  for (std::size_t i = 0U; i < kMaxFaults; ++i) {
    if (faults_[i].used && active(faults_[i].state)) {
      mask |= (1U << i);
    }
  }
  return static_cast<std::uint16_t>(mask);
}

std::string_view FaultManager::name(FaultId id) const noexcept {
  const std::lock_guard<os::PiMutex> lock(mutex_);
  return ((id < kMaxFaults) && faults_[id].used) ? faults_[id].config.name : std::string_view{};
}

std::size_t FaultManager::eventsSince(std::uint32_t after, FaultEvent* out, std::size_t capacity) const noexcept {
  const std::lock_guard<os::PiMutex> lock(mutex_);
  if ((out == nullptr) || (event_sequence_ <= after)) {
    return 0U;
  }
  // Older events may have been overwritten: start at the oldest one still in memory.
  const std::uint32_t oldest = (event_sequence_ > kEventMemory) ? (event_sequence_ - kEventMemory + 1U) : 1U;
  std::size_t copied = 0U;
  for (std::uint32_t seq = std::max(after + 1U, oldest); (seq <= event_sequence_) && (copied < capacity); ++seq) {
    out[copied] = events_[(seq - 1U) % kEventMemory];
    ++copied;
  }
  return copied;
}

std::uint32_t FaultManager::eventCount() const noexcept {
  const std::lock_guard<os::PiMutex> lock(mutex_);
  return event_sequence_;
}

}  // namespace vecu::faults
