#include "diagnostics/uds_server.hpp"

#include <utility>

namespace vecu::diag {

namespace {

constexpr std::uint8_t kSuppressBit = 0x80U;
constexpr std::uint8_t kDtcFormatIso14229 = 0x01U;  // DTCFormatIdentifier: ISO 14229-1 DTC format

std::uint8_t subFunction(const std::uint8_t* req) noexcept { return static_cast<std::uint8_t>(req[1] & 0x7FU); }
bool suppressBit(const std::uint8_t* req) noexcept { return (req[1] & kSuppressBit) != 0U; }

/// NRCs that are not sent for functionally addressed requests (ISO 14229-1).
bool suppressedForFunctional(Nrc nrc) noexcept {
  return (nrc == Nrc::ServiceNotSupported) || (nrc == Nrc::SubFunctionNotSupported) ||
         (nrc == Nrc::RequestOutOfRange) || (nrc == Nrc::SubFunctionNotSupportedInActiveSession) ||
         (nrc == Nrc::ServiceNotSupportedInActiveSession);
}

}  // namespace

const char* toString(Nrc nrc) noexcept {
  switch (nrc) {
    case Nrc::GeneralReject:
      return "generalReject";
    case Nrc::ServiceNotSupported:
      return "serviceNotSupported";
    case Nrc::SubFunctionNotSupported:
      return "subFunctionNotSupported";
    case Nrc::IncorrectMessageLengthOrInvalidFormat:
      return "incorrectMessageLengthOrInvalidFormat";
    case Nrc::ResponseTooLong:
      return "responseTooLong";
    case Nrc::ConditionsNotCorrect:
      return "conditionsNotCorrect";
    case Nrc::RequestOutOfRange:
      return "requestOutOfRange";
    case Nrc::SubFunctionNotSupportedInActiveSession:
      return "subFunctionNotSupportedInActiveSession";
    case Nrc::ServiceNotSupportedInActiveSession:
      return "serviceNotSupportedInActiveSession";
  }
  return "unknown";
}

const char* toString(Session session) noexcept {
  switch (session) {
    case Session::Default:
      return "default";
    case Session::Programming:
      return "programming";
    case Session::Extended:
      return "extended";
  }
  return "unknown";
}

// ------------------------------------------------------------------------- Response ----

bool UdsServer::Response::put(std::uint8_t value) {
  std::uint8_t* slot = reserve(1U);
  if (slot != nullptr) {
    *slot = value;
  }
  return slot != nullptr;
}

bool UdsServer::Response::put16(std::uint16_t value) {
  return put(static_cast<std::uint8_t>(value >> 8U)) && put(static_cast<std::uint8_t>(value & 0xFFU));
}

bool UdsServer::Response::put24(std::uint32_t value) {
  return put(static_cast<std::uint8_t>((value >> 16U) & 0xFFU)) && put(static_cast<std::uint8_t>((value >> 8U) & 0xFFU)) &&
         put(static_cast<std::uint8_t>(value & 0xFFU));
}

std::uint8_t* UdsServer::Response::reserve(std::size_t count) {
  if (overflow_ || (buffer_ == nullptr) || (size_ + count > capacity_)) {
    overflow_ = true;
    return nullptr;
  }
  std::uint8_t* start = buffer_ + size_;
  size_ += count;
  return start;
}

// ------------------------------------------------------------------------ UdsServer ----

UdsServer::UdsServer(const UdsServerConfig& config, faults::DtcStore& dtcs) : config_(config), dtcs_(dtcs) {}

bool UdsServer::addDid(std::uint16_t id, std::uint16_t length, SessionMask sessions, DidReader reader) {
  if ((did_count_ >= kMaxDids) || (length == 0U) || (length > kMaxDidLength) || !reader ||
      (findDid(id) != nullptr)) {
    return false;
  }
  dids_[did_count_] = DidEntry{id, length, sessions, std::move(reader)};
  ++did_count_;
  return true;
}

const UdsServer::DidEntry* UdsServer::findDid(std::uint16_t id) const noexcept {
  for (std::size_t i = 0U; i < did_count_; ++i) {
    if (dids_[i].id == id) {
      return &dids_[i];
    }
  }
  return nullptr;
}

SessionMask UdsServer::currentSessionMask() const noexcept {
  return (session_ == Session::Extended) ? kSessionExtended : kSessionDefault;
}

std::optional<ResetType> UdsServer::takePendingReset() noexcept {
  std::optional<ResetType> reset = pending_reset_;
  pending_reset_.reset();
  return reset;
}

void UdsServer::poll(core::TimePoint now) {
  if ((session_ != Session::Default) && ((now - last_request_) > config_.s3_server)) {
    session_ = Session::Default;  // S3 timeout: tester gone
    ++stats_.session_timeouts;
  }
}

std::size_t UdsServer::handleRequest(const std::uint8_t* request, std::size_t length, bool functional,
                                     core::TimePoint now, std::uint8_t* response, std::size_t capacity) {
  if ((request == nullptr) || (length == 0U) || (response == nullptr) || (capacity < 3U)) {
    return 0U;
  }
  ++stats_.requests;
  last_request_ = now;  // any request keeps a non-default session alive

  const std::uint8_t service = request[0];
  Response rsp(response, capacity);
  (void)rsp.put(static_cast<std::uint8_t>(service + sid::kPositiveResponseOffset));

  Outcome outcome;
  switch (service) {
    case sid::kDiagnosticSessionControl:
      outcome = sessionControl(request, length, rsp);
      break;
    case sid::kEcuReset:
      outcome = ecuReset(request, length, rsp);
      break;
    case sid::kClearDiagnosticInformation:
      outcome = clearDiagnosticInformation(request, length, rsp);
      break;
    case sid::kReadDtcInformation:
      outcome = readDtcInformation(request, length, rsp);
      break;
    case sid::kReadDataByIdentifier:
      outcome = readDataByIdentifier(request, length, rsp);
      break;
    case sid::kTesterPresent:
      outcome = testerPresent(request, length, rsp);
      break;
    default:
      outcome.nrc = Nrc::ServiceNotSupported;
      break;
  }
  if (!outcome.nrc && rsp.overflow()) {
    outcome.nrc = Nrc::ResponseTooLong;
  }

  if (outcome.nrc) {
    if (functional && suppressedForFunctional(*outcome.nrc)) {
      ++stats_.suppressed_responses;
      return 0U;
    }
    response[0] = sid::kNegativeResponse;
    response[1] = service;
    response[2] = static_cast<std::uint8_t>(*outcome.nrc);
    ++stats_.negative_responses;
    return 3U;
  }
  if (outcome.suppress_positive) {
    ++stats_.suppressed_responses;
    return 0U;
  }
  ++stats_.positive_responses;
  return rsp.size();
}

// ------------------------------------------------------------------- 0x10 Session ----

UdsServer::Outcome UdsServer::sessionControl(const std::uint8_t* req, std::size_t len, Response& rsp) {
  Outcome outcome;
  if (len != 2U) {
    outcome.nrc = Nrc::IncorrectMessageLengthOrInvalidFormat;
    return outcome;
  }
  const std::uint8_t requested = subFunction(req);
  if ((requested != static_cast<std::uint8_t>(Session::Default)) &&
      (requested != static_cast<std::uint8_t>(Session::Extended))) {
    outcome.nrc = Nrc::SubFunctionNotSupported;  // programming session: Phase 6
    return outcome;
  }
  session_ = static_cast<Session>(requested);
  outcome.suppress_positive = suppressBit(req);
  (void)rsp.put(requested);
  (void)rsp.put16(static_cast<std::uint16_t>(config_.p2_server.count()));
  (void)rsp.put16(static_cast<std::uint16_t>(config_.p2_star_server.count() / 10));  // 10 ms resolution
  return outcome;
}

// --------------------------------------------------------------------- 0x11 Reset ----

UdsServer::Outcome UdsServer::ecuReset(const std::uint8_t* req, std::size_t len, Response& rsp) {
  Outcome outcome;
  if (len != 2U) {
    outcome.nrc = Nrc::IncorrectMessageLengthOrInvalidFormat;
    return outcome;
  }
  const std::uint8_t type = subFunction(req);
  if (type == static_cast<std::uint8_t>(ResetType::HardReset)) {
    pending_reset_ = ResetType::HardReset;
  } else if (type == static_cast<std::uint8_t>(ResetType::SoftReset)) {
    if (session_ != Session::Extended) {
      outcome.nrc = Nrc::SubFunctionNotSupportedInActiveSession;
      return outcome;
    }
    pending_reset_ = ResetType::SoftReset;
  } else {
    outcome.nrc = Nrc::SubFunctionNotSupported;
    return outcome;
  }
  session_ = Session::Default;  // every reset returns to the default session
  outcome.suppress_positive = suppressBit(req);
  (void)rsp.put(type);
  return outcome;
}

// -------------------------------------------------------------- 0x14 Clear DTCs ----

UdsServer::Outcome UdsServer::clearDiagnosticInformation(const std::uint8_t* req, std::size_t len,
                                                         Response& /*rsp*/) {
  Outcome outcome;
  if (len != 4U) {
    outcome.nrc = Nrc::IncorrectMessageLengthOrInvalidFormat;
    return outcome;
  }
  const std::uint32_t group = (static_cast<std::uint32_t>(req[1]) << 16U) |
                              (static_cast<std::uint32_t>(req[2]) << 8U) | req[3];
  if (!dtcs_.clear(group)) {
    outcome.nrc = Nrc::RequestOutOfRange;
  } else if (clear_handler_) {
    clear_handler_(group);
  }
  return outcome;  // positive response: 0x54 only
}

// ------------------------------------------------------------ 0x19 Read DTC info ----

UdsServer::Outcome UdsServer::readDtcInformation(const std::uint8_t* req, std::size_t len, Response& rsp) {
  constexpr std::uint8_t kCountByMask = 0x01U;
  constexpr std::uint8_t kDtcByMask = 0x02U;
  constexpr std::uint8_t kSupportedDtc = 0x0AU;
  Outcome outcome;
  if (len < 2U) {
    outcome.nrc = Nrc::IncorrectMessageLengthOrInvalidFormat;
    return outcome;
  }
  const std::uint8_t report = req[1];  // 0x19 has no suppress bit in this implementation
  const bool with_mask = (report == kCountByMask) || (report == kDtcByMask);
  if (!with_mask && (report != kSupportedDtc)) {
    outcome.nrc = Nrc::SubFunctionNotSupported;
    return outcome;
  }
  if (len != (with_mask ? 3U : 2U)) {
    outcome.nrc = Nrc::IncorrectMessageLengthOrInvalidFormat;
    return outcome;
  }
  const std::uint8_t mask = with_mask ? req[2] : 0x00U;  // 0x00 = all supported DTCs
  (void)rsp.put(report);
  (void)rsp.put(faults::dtc_status::kAvailabilityMask);

  if (report == kCountByMask) {
    (void)rsp.put(kDtcFormatIso14229);
    // A mask with no supported bit matches nothing (count 0), unlike the "all" query.
    const std::size_t count = ((mask & faults::dtc_status::kAvailabilityMask) == 0U) ? 0U : dtcs_.countMatching(mask);
    (void)rsp.put16(static_cast<std::uint16_t>(count));
    return outcome;
  }
  if (with_mask && ((mask & faults::dtc_status::kAvailabilityMask) == 0U)) {
    return outcome;  // no DTC can match: empty list
  }
  std::array<faults::DtcRecord, faults::DtcStore::kCapacity> records{};
  const std::size_t count = dtcs_.snapshot(records.data(), records.size(), mask);
  for (std::size_t i = 0U; i < count; ++i) {
    (void)rsp.put24(records[i].code);
    (void)rsp.put(static_cast<std::uint8_t>(records[i].status & faults::dtc_status::kAvailabilityMask));
  }
  return outcome;
}

// ---------------------------------------------------------- 0x22 Read data by ID ----

UdsServer::Outcome UdsServer::readDataByIdentifier(const std::uint8_t* req, std::size_t len, Response& rsp) {
  Outcome outcome;
  if ((len < 3U) || (((len - 1U) % 2U) != 0U) || (((len - 1U) / 2U) > kMaxDidsPerRequest)) {
    outcome.nrc = Nrc::IncorrectMessageLengthOrInvalidFormat;
    return outcome;
  }
  std::size_t supported = 0U;
  for (std::size_t i = 1U; i + 1U < len; i += 2U) {
    const auto id = static_cast<std::uint16_t>((static_cast<unsigned int>(req[i]) << 8U) | req[i + 1U]);
    const DidEntry* did = findDid(id);
    if ((did == nullptr) || ((did->sessions & currentSessionMask()) == 0U)) {
      continue;  // unsupported (in this session): omitted from the response
    }
    ++supported;
    (void)rsp.put16(id);
    std::uint8_t* data = rsp.reserve(did->length);
    if (data != nullptr) {
      did->reader(data);
    }
  }
  if (supported == 0U) {
    outcome.nrc = Nrc::RequestOutOfRange;
  }
  return outcome;
}

// ----------------------------------------------------------- 0x3E Tester present ----

UdsServer::Outcome UdsServer::testerPresent(const std::uint8_t* req, std::size_t len, Response& rsp) {
  Outcome outcome;
  if (len != 2U) {
    outcome.nrc = Nrc::IncorrectMessageLengthOrInvalidFormat;
    return outcome;
  }
  if (subFunction(req) != 0x00U) {
    outcome.nrc = Nrc::SubFunctionNotSupported;
    return outcome;
  }
  outcome.suppress_positive = suppressBit(req);
  (void)rsp.put(0x00U);
  return outcome;
}

}  // namespace vecu::diag
