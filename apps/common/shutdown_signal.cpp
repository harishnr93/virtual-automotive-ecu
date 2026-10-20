#include "common/shutdown_signal.hpp"

#include <csignal>

namespace vecu::app {

namespace {

// A signal handler can only communicate through a variable of static storage duration;
// volatile sig_atomic_t is the type the C/C++ standards guarantee to be safe here.
volatile std::sig_atomic_t g_shutdown_requested = 0;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

extern "C" void onShutdownSignal(int /*signal_number*/) { g_shutdown_requested = 1; }

}  // namespace

void installShutdownHandlers() noexcept {
  struct sigaction action {};
  action.sa_handler = onShutdownSignal;
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;
  (void)sigaction(SIGINT, &action, nullptr);
  (void)sigaction(SIGTERM, &action, nullptr);
}

bool shutdownRequested() noexcept { return g_shutdown_requested != 0; }

}  // namespace vecu::app
