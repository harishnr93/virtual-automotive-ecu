#pragma once

namespace vecu::app {

/// Installs SIGINT/SIGTERM handlers that request an orderly shutdown (no SA_RESTART, so
/// blocking waits return early and the main loop notices promptly).
void installShutdownHandlers() noexcept;

[[nodiscard]] bool shutdownRequested() noexcept;

}  // namespace vecu::app
