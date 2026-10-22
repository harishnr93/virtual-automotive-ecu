#pragma once

// Diagnostic CAN identifiers (11-bit, normal addressing) — the common OBD-style assignment.

#include <cstdint>

namespace vecu::diag {

inline constexpr std::uint32_t kPhysicalRequestId = 0x7E0U;    ///< tester -> this ECU
inline constexpr std::uint32_t kFunctionalRequestId = 0x7DFU;  ///< tester -> all ECUs (single frames only)
inline constexpr std::uint32_t kPhysicalResponseId = 0x7E8U;   ///< this ECU -> tester

}  // namespace vecu::diag
