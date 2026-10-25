#include "faults/dtc_store.hpp"

#include <mutex>

namespace vecu::faults {

using namespace dtc_status;  // NOLINT(google-build-using-namespace): local constants only

bool DtcStore::registerDtc(DtcCode code) {
  const std::lock_guard<os::PiMutex> lock(mutex_);
  if ((count_ >= kCapacity) || (code > 0xFFFFFFU) || (code == kDtcGroupAll) || (find(code) != nullptr)) {
    return false;
  }
  records_[count_] = DtcRecord{code, kInitial, 0U, 0U};
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
    record->aging_counter = 0U;
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
      records_[i].aging_counter = 0U;
    }
    return true;
  }
  DtcRecord* record = find(code_or_group);
  if (record == nullptr) {
    return false;
  }
  record->status = kInitial;
  record->occurrence_count = 0U;
  record->aging_counter = 0U;
  return true;
}

void DtcStore::startOperationCycle() noexcept {
  const std::lock_guard<os::PiMutex> lock(mutex_);
  for (std::size_t i = 0U; i < count_; ++i) {
    auto status = static_cast<unsigned int>(records_[i].status);
    const bool failed_in_cycle = (status & kTestFailedThisOperationCycle) != 0U;
    // A cycle that completed without failure ends the pending state ...
    if (!failed_in_cycle) {
      status &= ~static_cast<unsigned int>(kPendingDtc);
    }
    // ... and counts towards aging of a confirmed DTC.
    if ((status & kConfirmedDtc) != 0U) {
      if (failed_in_cycle) {
        records_[i].aging_counter = 0U;
      } else if (records_[i].aging_counter < 0xFFU) {
        ++records_[i].aging_counter;
      }
      if ((aging_threshold_ > 0U) && (records_[i].aging_counter >= aging_threshold_)) {
        status &= ~static_cast<unsigned int>(kConfirmedDtc);  // aged out
        records_[i].aging_counter = 0U;
      }
    }
    status &= ~static_cast<unsigned int>(kTestFailedThisOperationCycle);
    status |= kTestNotCompletedThisOperationCycle;
    records_[i].status = static_cast<std::uint8_t>(status);
  }
}

std::size_t DtcStore::importRecords(const DtcRecord* records, std::size_t count) noexcept {
  const std::lock_guard<os::PiMutex> lock(mutex_);
  std::size_t restored = 0U;
  for (std::size_t i = 0U; (records != nullptr) && (i < count); ++i) {
    DtcRecord* record = find(records[i].code);
    if (record != nullptr) {
      record->status = static_cast<std::uint8_t>(records[i].status & kAvailabilityMask);
      record->occurrence_count = records[i].occurrence_count;
      record->aging_counter = records[i].aging_counter;
      ++restored;
    }
  }
  return restored;
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
