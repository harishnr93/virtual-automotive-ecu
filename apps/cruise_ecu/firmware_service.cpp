#include "cruise_ecu/firmware_service.hpp"

#include <algorithm>

#include "core/clock.hpp"

namespace vecu::cruise {

namespace {

namespace fw = vecu::firmware;
using diag::Nrc;

constexpr std::uint8_t kNoSlot = 0xFFU;
constexpr std::uint8_t kRoutineOk = 0x00U;
constexpr std::uint8_t kRoutineFailed = 0x01U;

void putVersion(std::uint8_t* out, fw::Version v) {
  out[0] = v.major;
  out[1] = v.minor;
  out[2] = v.patch;
}

void putU32(std::uint8_t* out, std::uint32_t value) {
  for (unsigned int i = 0U; i < 4U; ++i) {
    out[i] = static_cast<std::uint8_t>((value >> (24U - 8U * i)) & 0xFFU);
  }
}

double elapsedMs(core::TimePoint start) { return core::toMillis(core::now() - start); }

}  // namespace

FirmwareService::FirmwareService(const core::Logger& log) : log_(log) {
  // The image delivered "from the factory": provisioned into slot A of a blank flash.
  fw::ImageHeader header;
  header.hardware_id = kHardwareId;
  header.version = kFactoryVersion;
  header.security_version = kFactorySecurityVersion;
  factory_image_ = fw::buildImage(header, fw::makePayload(kFactoryPayloadSize, kFactoryVersion.value()));
}

bool FirmwareService::open(const std::string& path) {
  const std::lock_guard<std::mutex> lock(mutex_);
  if (path.empty()) {
    ram_flash_.emplace(fw::FlashLayout::kTotalSize, fw::FlashLayout::kSectorSize);
    flash_ = &*ram_flash_;
    log_.info("FLASH volatile (RAM): updates are lost at exit (use --flash FILE)");
  } else {
    const fw::FlashStatus status =
        fw::FileFlash::open(path, fw::FlashLayout::kTotalSize, fw::FlashLayout::kSectorSize, file_flash_);
    if (status != fw::FlashStatus::Ok) {
      log_.error(core::strCat("FLASH cannot open ", path, ": ", fw::toString(status)));
      return false;
    }
    flash_ = &file_flash_;
    log_.info(core::strCat("FLASH file=", path, " size=", fw::FlashLayout::kTotalSize));
  }
  control_.emplace(*flash_);
  bootloader_.emplace(*flash_, *control_, verifier_);
  updater_.emplace(*flash_, *control_, verifier_);
  return true;
}

fw::BootResult FirmwareService::boot() {
  const std::lock_guard<std::mutex> lock(mutex_);
  const core::TimePoint start = core::now();
  running_ = bootloader_->boot(&factory_image_);
  boot_time_ = core::now();
  timings_.last_boot_ms = elapsedMs(start);
  trial_ = running_.application_valid && (running_.record.state[running_.slot] == fw::SlotState::Trial);
  if (running_.application_valid) {
    updater_->setPolicy(fw::UpdatePolicy{kHardwareId, running_.header.version, running_.header.security_version,
                                         false});
    updater_->setRunningSlot(running_.slot);
  } else {
    updater_->setPolicy(fw::UpdatePolicy{kHardwareId, fw::Version{}, 0U, true});  // anything valid
    updater_->setRunningSlot(std::nullopt);
  }
  updater_->abort();  // a reset ends any download
  const char* slot_name = (running_.slot == 0U) ? "A" : "B";
  log_.info(core::strCat(
      "BOOT event=", fw::toString(running_.event), " app_valid=", running_.application_valid, " slot=", slot_name,
      " version=", running_.application_valid ? running_.header.version.toString() : std::string{"-"},
      " security_version=", running_.header.security_version,
      " state_a=", fw::toString(running_.record.state[0]), " state_b=", fw::toString(running_.record.state[1]),
      " attempts=", static_cast<unsigned int>(running_.record.boot_attempts), "/",
      static_cast<unsigned int>(running_.record.max_boot_attempts),
      (running_.rejected_image_error != fw::ImageError::None)
          ? core::strCat(" rejected=", fw::toString(running_.rejected_image_error))
          : std::string{},
      " boot_ms=", timings_.last_boot_ms));
  if (running_.record_status != fw::FlashStatus::Ok) {
    log_.error(core::strCat("BOOT record write failed: ", fw::toString(running_.record_status)));
  }
  return running_;
}

bool FirmwareService::confirmTrialIfHealthy(core::TimePoint healthy_since, core::TimePoint now,
                                            std::chrono::duration<double> confirm_after) {
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!trial_ || ((now - std::max(boot_time_, healthy_since)) < confirm_after)) {
    return false;
  }
  const fw::FlashStatus status = bootloader_->confirmActive();
  if (status != fw::FlashStatus::Ok) {
    log_.error(core::strCat("FW_CONFIRM failed: ", fw::toString(status)));
    return false;
  }
  trial_ = false;
  log_.info(core::strCat("FW_CONFIRMED version=", running_.header.version.toString(),
                         " slot=", (running_.slot == 0U) ? "A" : "B"));
  return true;
}

fw::Version FirmwareService::runningVersion() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return running_.application_valid ? running_.header.version : fw::Version{};
}

bool FirmwareService::trialActive() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return trial_;
}

bool FirmwareService::defectFlag() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return running_.application_valid && ((running_.header.flags & fw::image_flags::kSimulatedDefectHang) != 0U);
}

FirmwareTimings FirmwareService::timings() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return timings_;
}

// ---------------------------------------------------------------------------- UDS ----

bool FirmwareService::registerUds(diag::UdsServer& uds) {
  uds.setDownloadHandler(this);
  bool ok = uds.addDid(0x0400U, 7U, diag::kSessionAll, [this](std::uint8_t* out) { readBootStatus(out); });
  ok = uds.addDid(0x0401U, 6U, diag::kSessionAll, [this](std::uint8_t* out) { readUpdateStatus(out); }) && ok;
  ok = uds.addDid(0x0402U, 10U, diag::kSessionAll, [this](std::uint8_t* out) { readVersions(out); }) && ok;
  ok = uds.addRoutine(0xFF00U, diag::kSessionProgramming,
                      [this](const std::uint8_t*, std::size_t, std::uint8_t* status) { return eraseRoutine(status); }) &&
       ok;
  ok = uds.addRoutine(0xFF01U, diag::kSessionProgramming,
                      [this](const std::uint8_t*, std::size_t, std::uint8_t* status) { return checkRoutine(status); }) &&
       ok;
  ok = uds.addRoutine(0x0201U, diag::kSessionExtended | diag::kSessionProgramming,
                      [this](const std::uint8_t*, std::size_t, std::uint8_t* status) {
                        return rollbackRoutine(status);
                      }) &&
       ok;
  return ok;
}

void FirmwareService::readBootStatus(std::uint8_t* out) const {
  const std::lock_guard<std::mutex> lock(mutex_);
  fw::BootRecord record;
  if (!control_->load(record)) {
    record = running_.record;
  }
  out[0] = running_.application_valid ? running_.slot : kNoSlot;
  out[1] = static_cast<std::uint8_t>(record.state[0]);
  out[2] = static_cast<std::uint8_t>(record.state[1]);
  out[3] = record.boot_attempts;
  out[4] = record.max_boot_attempts;
  out[5] = static_cast<std::uint8_t>(running_.event);
  out[6] = static_cast<std::uint8_t>(running_.rejected_image_error);
}

void FirmwareService::readUpdateStatus(std::uint8_t* out) const {
  const std::lock_guard<std::mutex> lock(mutex_);
  out[0] = static_cast<std::uint8_t>(updater_->state());
  out[1] = static_cast<std::uint8_t>(updater_->lastImageError());
  putU32(out + 2, updater_->bytesReceived());
}

void FirmwareService::readVersions(std::uint8_t* out) const {
  const std::lock_guard<std::mutex> lock(mutex_);
  fw::BootRecord record;
  if (!control_->load(record)) {
    record = running_.record;
  }
  putVersion(out, record.version[0]);
  putVersion(out + 3, record.version[1]);
  putU32(out + 6, running_.application_valid ? running_.header.security_version : 0U);
}

diag::RoutineResult FirmwareService::eraseRoutine(std::uint8_t* status) {
  const std::lock_guard<std::mutex> lock(mutex_);
  const core::TimePoint start = core::now();
  const fw::UpdateStatus result = updater_->eraseInactiveSlot();
  const double ms = elapsedMs(start);
  timings_.max_erase_ms = std::max(timings_.max_erase_ms, ms);
  log_.info(core::strCat("FW_ERASE slot=", (updater_->targetSlot() == 0U) ? "A" : "B",
                         " result=", fw::toString(result), " ms=", ms));
  switch (result) {
    case fw::UpdateStatus::Ok:
      status[0] = kRoutineOk;
      return diag::RoutineResult{std::nullopt, 1U};
    case fw::UpdateStatus::NotPossible:  // running image not yet confirmed
    case fw::UpdateStatus::SequenceError:
      return diag::RoutineResult{Nrc::ConditionsNotCorrect, 0U};
    default:
      return diag::RoutineResult{Nrc::GeneralProgrammingFailure, 0U};
  }
}

diag::RoutineResult FirmwareService::checkRoutine(std::uint8_t* status) {
  const std::lock_guard<std::mutex> lock(mutex_);
  const core::TimePoint start = core::now();
  const fw::UpdateStatus result = updater_->checkImage();
  const double ms = elapsedMs(start);
  timings_.max_check_ms = std::max(timings_.max_check_ms, ms);
  log_.info(core::strCat("FW_CHECK result=", fw::toString(result), " image_error=",
                         fw::toString(updater_->lastImageError()), " ms=", ms,
                         (result == fw::UpdateStatus::Ok)
                             ? core::strCat(" version=", updater_->verifiedHeader().version.toString(),
                                            " (activated for the next reset as trial image)")
                             : std::string{}));
  switch (result) {
    case fw::UpdateStatus::Ok:
    case fw::UpdateStatus::VerificationFailed:  // routine ran: the result is in the status record
      status[0] = (result == fw::UpdateStatus::Ok) ? kRoutineOk : kRoutineFailed;
      status[1] = static_cast<std::uint8_t>(updater_->lastImageError());
      return diag::RoutineResult{std::nullopt, 2U};
    case fw::UpdateStatus::SequenceError:
      return diag::RoutineResult{Nrc::RequestSequenceError, 0U};
    default:
      return diag::RoutineResult{Nrc::GeneralProgrammingFailure, 0U};
  }
}

diag::RoutineResult FirmwareService::rollbackRoutine(std::uint8_t* status) {
  const std::lock_guard<std::mutex> lock(mutex_);
  const fw::UpdateStatus result = updater_->rollbackToPrevious();
  log_.info(core::strCat("FW_ROLLBACK_REQUEST result=", fw::toString(result), " (effective at the next reset)"));
  switch (result) {
    case fw::UpdateStatus::Ok:
      status[0] = kRoutineOk;
      return diag::RoutineResult{std::nullopt, 1U};
    case fw::UpdateStatus::NotPossible:
    case fw::UpdateStatus::SequenceError:
      return diag::RoutineResult{Nrc::ConditionsNotCorrect, 0U};
    default:
      return diag::RoutineResult{Nrc::GeneralProgrammingFailure, 0U};
  }
}

std::optional<Nrc> FirmwareService::requestDownload(std::uint32_t address, std::uint32_t size) {
  const std::lock_guard<std::mutex> lock(mutex_);
  const fw::UpdateStatus result = updater_->requestDownload(address, size);
  log_.info(core::strCat("FW_DOWNLOAD_START address=0x", std::hex, address, std::dec, " size=", size,
                         " result=", fw::toString(result)));
  switch (result) {
    case fw::UpdateStatus::Ok:
      download_start_ = core::now();
      return std::nullopt;
    case fw::UpdateStatus::SequenceError:
      return Nrc::UploadDownloadNotAccepted;  // memory not erased
    default:
      return Nrc::RequestOutOfRange;  // address / size not acceptable
  }
}

std::optional<Nrc> FirmwareService::transferData(std::uint8_t counter, const std::uint8_t* data, std::size_t length) {
  const std::lock_guard<std::mutex> lock(mutex_);
  const core::TimePoint start = core::now();
  const fw::UpdateStatus result = updater_->transferData(counter, data, length);
  timings_.max_block_ms = std::max(timings_.max_block_ms, elapsedMs(start));
  switch (result) {
    case fw::UpdateStatus::Ok:
      return std::nullopt;
    case fw::UpdateStatus::WrongBlockCounter:
      return Nrc::WrongBlockSequenceCounter;
    case fw::UpdateStatus::TooMuchData:
      return Nrc::TransferDataSuspended;
    case fw::UpdateStatus::SequenceError:
      return Nrc::RequestSequenceError;
    default:
      log_.error(core::strCat("FW_DOWNLOAD programming failure at ", updater_->bytesReceived(), " bytes"));
      return Nrc::GeneralProgrammingFailure;
  }
}

std::optional<Nrc> FirmwareService::requestTransferExit() {
  const std::lock_guard<std::mutex> lock(mutex_);
  const fw::UpdateStatus result = updater_->transferExit();
  if (result == fw::UpdateStatus::Ok) {
    timings_.last_download_s = elapsedMs(download_start_) / 1000.0;
    timings_.last_download_bytes = updater_->bytesReceived();
    log_.info(core::strCat("FW_DOWNLOAD_DONE bytes=", timings_.last_download_bytes,
                           " s=", timings_.last_download_s, " kib_per_s=",
                           (timings_.last_download_s > 0.0)
                               ? (timings_.last_download_bytes / 1024.0) / timings_.last_download_s
                               : 0.0,
                           " max_block_ms=", timings_.max_block_ms));
    return std::nullopt;
  }
  return (result == fw::UpdateStatus::SequenceError) ? Nrc::RequestSequenceError : Nrc::GeneralProgrammingFailure;
}

void FirmwareService::abortDownload() {
  const std::lock_guard<std::mutex> lock(mutex_);
  updater_->abort();
  log_.warning(core::strCat("FW_DOWNLOAD_ABORTED bytes=", updater_->bytesReceived(),
                            " (programming session left; target slot stays unbootable)"));
}

}  // namespace vecu::cruise
