#include "faults/dtc_store.hpp"

#include <mutex>

namespace vecu::faults {

using namespace dtc_status;  // NOLINT(google-build-using-namespace): local constants only

bool DtcStore::registerDtc(DtcCode code) {
  const std::lock_guard<os::PiMutex> lock(mutex_);
  if ((count_ >= kCapacity) || (code > 0xFFFFFFU) || (code == kDtcGroupAll) || (find(code) != nullptr)) {
    return false;
  }
  records_[count_] = DtcRecord{code, kInitial, 0U};
  ++count_;
  return true;
}

bool DtcStore::reportTestResult(DtcCode code, bool failed) noexcept {
  const std::lock_guard<os::PiMutex> lock(mutex_);
  DtcRecord* record = find(code);
  if (record == nullptr) {
    return false;
  }
  auto status = static_cast<unsigned int>(record->status);
  status &= ~static_cast<unsigned int>(kTestNotCompletedSinceLastClear | kTestNotCompletedThisOperationCycle);
  if (failed) {
    if ((status & kTestFailed) == 0U) {
      ++record->occurrence_count;
    }
    status |= kTestFailed | kTestFailedThisOperationCycle | kPendingDtc | kConfirmedDtc | kTestFailedSinceLastClear;
  } else {
    status &= ~static_cast<unsigned int>(kTestFailed);
  }
  record->status = static_cast<std::uint8_t>(status);
  return true;
}

bool DtcStore::clear(DtcCode code_or_group) noexcept {
  const std::lock_guard<os::PiMutex> lock(mutex_);
  if (code_or_group == kDtcGroupAll) {
    for (std::size_t i = 0U; i < count_; ++i) {
      records_[i].status = kInitial;
      records_[i].occurrence_count = 0U;
    }
    return true;
  }
  DtcRecord* record = find(code_or_group);
  if (record == nullptr) {
    return false;
  }
  record->status = kInitial;
  record->occurrence_count = 0U;
  return true;
}

void DtcStore::startOperationCycle() noexcept {
  const std::lock_guard<os::PiMutex> lock(mutex_);
  for (std::size_t i = 0U; i < count_; ++i) {
    auto status = static_cast<unsigned int>(records_[i].status);
    // A cycle that completed without failure ends the pending state.
    if ((status & kTestFailedThisOperationCycle) == 0U) {
      status &= ~static_cast<unsigned int>(kPendingDtc);
    }
    status &= ~static_cast<unsigned int>(kTestFailedThisOperationCycle);
    status |= kTestNotCompletedThisOperationCycle;
    records_[i].status = static_cast<std::uint8_t>(status);
  }
}

bool DtcStore::status(DtcCode code, std::uint8_t& status) const noexcept {
  const std::lock_guard<os::PiMutex> lock(mutex_);
  const DtcRecord* record = find(code);
  if (record == nullptr) {
    return false;
  }
  status = record->status;
  return true;
}

bool DtcStore::isSupported(DtcCode code) const noexcept {
  const std::lock_guard<os::PiMutex> lock(mutex_);
  return find(code) != nullptr;
}

bool DtcStore::matches(const DtcRecord& record, std::uint8_t mask) noexcept {
  return (mask == 0U) || ((record.status & mask & kAvailabilityMask) != 0U);
}

std::size_t DtcStore::snapshot(DtcRecord* out, std::size_t capacity, std::uint8_t mask) const noexcept {
  const std::lock_guard<os::PiMutex> lock(mutex_);
  std::size_t written = 0U;
  for (std::size_t i = 0U; (i < count_) && (written < capacity) && (out != nullptr); ++i) {
    if (matches(records_[i], mask)) {
      out[written] = records_[i];
      ++written;
    }
  }
  return written;
}

std::size_t DtcStore::countMatching(std::uint8_t mask) const noexcept {
  const std::lock_guard<os::PiMutex> lock(mutex_);
  std::size_t matching = 0U;
  for (std::size_t i = 0U; i < count_; ++i) {
    if (matches(records_[i], mask)) {
      ++matching;
    }
  }
  return matching;
}

std::size_t DtcStore::size() const noexcept {
  const std::lock_guard<os::PiMutex> lock(mutex_);
  return count_;
}

DtcRecord* DtcStore::find(DtcCode code) noexcept {
  for (std::size_t i = 0U; i < count_; ++i) {
    if (records_[i].code == code) {
      return &records_[i];
    }
  }
  return nullptr;
}

const DtcRecord* DtcStore::find(DtcCode code) const noexcept {
  for (std::size_t i = 0U; i < count_; ++i) {
    if (records_[i].code == code) {
      return &records_[i];
    }
  }
  return nullptr;
}

}  // namespace vecu::faults
