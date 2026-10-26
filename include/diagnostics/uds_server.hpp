#pragma once

// Educational UDS diagnostic server inspired by ISO 14229-1.
//
// Implemented services (see docs/phase4_diagnostics.md and docs/phase6_firmware_update.md):
//   0x10 DiagnosticSessionControl   0x01 default, 0x02 programming (from extended only), 0x03 extended
//   0x11 ECUReset                   0x01 hardReset, 0x03 softReset (extended session only) — simulated
//   0x14 ClearDiagnosticInformation group 0xFFFFFF or a single DTC        (not in programming)
//   0x19 ReadDTCInformation         0x01 / 0x02 / 0x0A                    (not in programming)
//   0x22 ReadDataByIdentifier       up to 8 DIDs per request, per-DID session restrictions
//   0x31 RoutineControl             0x01 startRoutine, per-routine session restrictions
//   0x34 RequestDownload            dataFormatIdentifier 0x00 only         (programming only)
//   0x36 TransferData                                                     (programming only)
//   0x37 RequestTransferExit                                              (programming only)
//   0x3E TesterPresent              0x00
//
// Request handling follows the order of the general server response behaviour of ISO 14229-1:
// service supported -> service in active session (0x7F) -> length -> sub-function ->
// sub-function in session -> service-specific checks. Supports the
// suppressPosRspMsgIndicationBit (0x80) and the suppression of NRC 0x11/0x12/0x31/0x7E/0x7F
// for functionally addressed requests.
//
// The server knows nothing about flash or images: the download services are forwarded to an
// IDownloadHandler and routines to registered callbacks, so the protocol layer can be tested
// without a firmware stack. Not implemented: SecurityAccess (0x27) — a real ECU would require it
// before programming; see the Phase 6 documentation.
// Pure logic: time is passed in, transport is outside (ISO-TP).

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <utility>

#include "core/clock.hpp"
#include "faults/dtc_store.hpp"

namespace vecu::diag {

namespace sid {
inline constexpr std::uint8_t kDiagnosticSessionControl = 0x10U;
inline constexpr std::uint8_t kEcuReset = 0x11U;
inline constexpr std::uint8_t kClearDiagnosticInformation = 0x14U;
inline constexpr std::uint8_t kReadDtcInformation = 0x19U;
inline constexpr std::uint8_t kReadDataByIdentifier = 0x22U;
inline constexpr std::uint8_t kRoutineControl = 0x31U;
inline constexpr std::uint8_t kRequestDownload = 0x34U;
inline constexpr std::uint8_t kTransferData = 0x36U;
inline constexpr std::uint8_t kRequestTransferExit = 0x37U;
inline constexpr std::uint8_t kTesterPresent = 0x3EU;
inline constexpr std::uint8_t kNegativeResponse = 0x7FU;
inline constexpr std::uint8_t kPositiveResponseOffset = 0x40U;
}  // namespace sid

/// Negative response codes used by this server.
enum class Nrc : std::uint8_t {
  GeneralReject = 0x10U,
  ServiceNotSupported = 0x11U,
  SubFunctionNotSupported = 0x12U,
  IncorrectMessageLengthOrInvalidFormat = 0x13U,
  ResponseTooLong = 0x14U,
  ConditionsNotCorrect = 0x22U,
  RequestSequenceError = 0x24U,
  RequestOutOfRange = 0x31U,
  UploadDownloadNotAccepted = 0x70U,
  TransferDataSuspended = 0x71U,
  GeneralProgrammingFailure = 0x72U,
  WrongBlockSequenceCounter = 0x73U,
  SubFunctionNotSupportedInActiveSession = 0x7EU,
  ServiceNotSupportedInActiveSession = 0x7FU
};

enum class Session : std::uint8_t { Default = 0x01U, Programming = 0x02U, Extended = 0x03U };
enum class ResetType : std::uint8_t { HardReset = 0x01U, KeyOffOnReset = 0x02U, SoftReset = 0x03U };

[[nodiscard]] const char* toString(Nrc nrc) noexcept;
[[nodiscard]] const char* toString(Session session) noexcept;

/// Bit mask of sessions in which something is available.
using SessionMask = std::uint8_t;
inline constexpr SessionMask kSessionDefault = 0x01U;
inline constexpr SessionMask kSessionProgramming = 0x02U;
inline constexpr SessionMask kSessionExtended = 0x04U;
inline constexpr SessionMask kSessionAll = kSessionDefault | kSessionProgramming | kSessionExtended;

/// Receives the data of a download (0x34/0x36/0x37). Each call returns an NRC on rejection.
/// The server checks formats, lengths and the call order (0x24 if no download is active);
/// the handler owns block counters, sizes and the memory itself.
class IDownloadHandler {
 public:
  virtual ~IDownloadHandler() = default;
  virtual std::optional<Nrc> requestDownload(std::uint32_t address, std::uint32_t size) = 0;
  virtual std::optional<Nrc> transferData(std::uint8_t counter, const std::uint8_t* data, std::size_t length) = 0;
  virtual std::optional<Nrc> requestTransferExit() = 0;
  /// The download was abandoned (session left, S3 timeout, reset) — never called after Exit.
  virtual void abortDownload() = 0;

 protected:
  IDownloadHandler() = default;
  IDownloadHandler(const IDownloadHandler&) = default;
  IDownloadHandler& operator=(const IDownloadHandler&) = default;
  IDownloadHandler(IDownloadHandler&&) = default;
  IDownloadHandler& operator=(IDownloadHandler&&) = default;
};

/// Result of a routine: an NRC, or the length of the status record written to the output.
struct RoutineResult {
  std::optional<Nrc> nrc;
  std::size_t status_length{0U};
};

struct UdsServerConfig {
  std::chrono::milliseconds p2_server{50};         ///< reported in the session response
  std::chrono::milliseconds p2_star_server{5000};  ///< reported in the session response
  std::chrono::milliseconds s3_server{5000};       ///< non-default session timeout
};

/// Counters for monitoring and tests.
struct UdsStatistics {
  std::uint32_t requests{0U};
  std::uint32_t positive_responses{0U};
  std::uint32_t negative_responses{0U};
  std::uint32_t suppressed_responses{0U};
  std::uint32_t session_timeouts{0U};
};

class UdsServer {
 public:
  static constexpr std::size_t kMaxDids = 32U;
  static constexpr std::size_t kMaxDidsPerRequest = 8U;
  static constexpr std::size_t kMaxDidLength = 64U;
  static constexpr std::size_t kMaxRoutines = 8U;
  static constexpr std::size_t kMaxRoutineStatus = 16U;
  /// maxNumberOfBlockLength of 0x74: a whole 0x36 request (SID + counter + data) must fit.
  static constexpr std::uint16_t kMaxBlockLength = 0x0FFFU;

  using DidReader = std::function<void(std::uint8_t* out)>;  ///< must write exactly `length` bytes
  /// Starts a routine with its option record; writes at most kMaxRoutineStatus status bytes.
  using RoutineHandler =
      std::function<RoutineResult(const std::uint8_t* option, std::size_t option_length, std::uint8_t* status)>;
  /// Decides whether a session change may happen now (e.g. programming only at standstill).
  using SessionGuard = std::function<std::optional<Nrc>(Session requested)>;

  UdsServer(const UdsServerConfig& config, faults::DtcStore& dtcs);

  /// Registers a data identifier (initialisation phase only).
  [[nodiscard]] bool addDid(std::uint16_t id, std::uint16_t length, SessionMask sessions, DidReader reader);

  /// Processes one complete request. Writes the response into `response` and returns its
  /// length; 0 means "no response" (suppressed or ignored).
  std::size_t handleRequest(const std::uint8_t* request, std::size_t length, bool functional,
                            core::TimePoint now, std::uint8_t* response, std::size_t capacity);

  /// Registers a routine for 0x31 startRoutine (initialisation phase only).
  [[nodiscard]] bool addRoutine(std::uint16_t id, SessionMask sessions, RoutineHandler handler);

  void setDownloadHandler(IDownloadHandler* handler) noexcept { download_handler_ = handler; }
  void setSessionGuard(SessionGuard guard) { session_guard_ = std::move(guard); }
  /// Notified on every session change (request, S3 timeout, reset): from, to.
  void setSessionChangeHandler(std::function<void(Session, Session)> handler) {
    session_handler_ = std::move(handler);
  }

  /// Called after a successful 0x14 ClearDiagnosticInformation with the cleared group/DTC,
  /// so the fault manager can restart qualification (ISO 14229-1: clearing resets test status).
  void setClearHandler(std::function<void(faults::DtcCode)> handler) { clear_handler_ = std::move(handler); }

  /// Session supervision: falls back to the default session after S3 without requests.
  void poll(core::TimePoint now);

  /// A reset accepted by 0x11 is executed by the ECU *after* the response was sent.
  [[nodiscard]] std::optional<ResetType> takePendingReset() noexcept;

  [[nodiscard]] Session session() const noexcept { return session_; }
  [[nodiscard]] bool downloadActive() const noexcept { return download_active_; }
  [[nodiscard]] const UdsStatistics& statistics() const noexcept { return stats_; }

 private:
  struct DidEntry {
    std::uint16_t id{0U};
    std::uint16_t length{0U};
    SessionMask sessions{0U};
    DidReader reader;
  };

  struct RoutineEntry {
    std::uint16_t id{0U};
    SessionMask sessions{0U};
    RoutineHandler handler;
  };

  /// Bounds-checked response builder.
  class Response {
   public:
    Response(std::uint8_t* buffer, std::size_t capacity) : buffer_(buffer), capacity_(capacity) {}
    bool put(std::uint8_t value);
    bool put16(std::uint16_t value);
    bool put24(std::uint32_t value);
    std::uint8_t* reserve(std::size_t count);
    [[nodiscard]] std::size_t size() const noexcept { return size_; }
    [[nodiscard]] bool overflow() const noexcept { return overflow_; }

   private:
    std::uint8_t* buffer_;
    std::size_t capacity_;
    std::size_t size_{0U};
    bool overflow_{false};
  };

  /// Result of a service handler: positive (response built) or an NRC.
  struct Outcome {
    std::optional<Nrc> nrc;
    bool suppress_positive{false};
  };

  Outcome sessionControl(const std::uint8_t* req, std::size_t len, Response& rsp);
  Outcome ecuReset(const std::uint8_t* req, std::size_t len, Response& rsp);
  Outcome clearDiagnosticInformation(const std::uint8_t* req, std::size_t len, Response& rsp);
  Outcome readDtcInformation(const std::uint8_t* req, std::size_t len, Response& rsp);
  Outcome readDataByIdentifier(const std::uint8_t* req, std::size_t len, Response& rsp);
  Outcome routineControl(const std::uint8_t* req, std::size_t len, Response& rsp);
  Outcome requestDownload(const std::uint8_t* req, std::size_t len, Response& rsp);
  Outcome transferData(const std::uint8_t* req, std::size_t len, Response& rsp);
  Outcome requestTransferExit(std::size_t len);
  static Outcome testerPresent(const std::uint8_t* req, std::size_t len, Response& rsp);

  [[nodiscard]] bool serviceAllowedInSession(std::uint8_t service) const noexcept;
  void changeSession(Session next);

  [[nodiscard]] const DidEntry* findDid(std::uint16_t id) const noexcept;
  [[nodiscard]] SessionMask currentSessionMask() const noexcept;

  UdsServerConfig config_;
  faults::DtcStore& dtcs_;
  std::array<DidEntry, kMaxDids> dids_{};
  std::size_t did_count_{0U};
  std::array<RoutineEntry, kMaxRoutines> routines_{};
  std::size_t routine_count_{0U};
  IDownloadHandler* download_handler_{nullptr};
  bool download_active_{false};
  SessionGuard session_guard_{};
  std::function<void(Session, Session)> session_handler_{};
  Session session_{Session::Default};
  core::TimePoint last_request_{};
  std::optional<ResetType> pending_reset_{};
  std::function<void(faults::DtcCode)> clear_handler_{};
  UdsStatistics stats_{};
};

}  // namespace vecu::diag
