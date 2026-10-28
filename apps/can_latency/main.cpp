// CAN message latency benchmark.
//
// Two channels on the same bus inside one process: a sender thread transmits numbered frames
// at a fixed period, a receiver thread timestamps their arrival. Both use the same monotonic
// clock, so the difference is the one-way latency through the bus implementation:
//   send() system call -> kernel (UDP multicast loopback or vcan) -> receive() wake-up.
// A real CAN bus adds the frame's time on the wire (about 0.22-0.27 ms for 8 data bytes at
// 500 kbit/s), which a software bus does not have.
//
//   vecu_can_latency --count 5000 --period-us 1000
//   vecu_can_latency --count 5000 --background-fps 2000     # with additional bus traffic
//   vecu_can_latency --bus socketcan --json latency.json

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "can/can_channel_factory.hpp"
#include "core/clock.hpp"
#include "core/os_port.hpp"

namespace {

namespace can = vecu::can;
namespace core = vecu::core;
namespace os = vecu::os;
using std::chrono::microseconds;
using std::chrono::nanoseconds;

constexpr std::uint32_t kProbeId = 0x7A0U;       // outside the vehicle CAN matrix
constexpr std::uint32_t kBackgroundId = 0x7A1U;
constexpr std::int64_t kNotReceived = -1;

struct Options {
  can::CanChannelConfig can{};
  long count{2000};
  long period_us{1000};
  long background_fps{0};
  std::string json_path{};
  std::string samples_path{};
};

bool parseLong(const char* text, long& value) {
  char* end = nullptr;
  errno = 0;
  const long parsed = std::strtol(text, &end, 0);
  if ((errno != 0) || (end == text) || (*end != '\0') || (parsed < 0)) {
    return false;
  }
  value = parsed;
  return true;
}

void printUsage(const char* program) {
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
  std::printf(
      "Usage: %s [options]\n\n"
      "  --bus auto|socketcan|udp  CAN backend (default: auto)\n"
      "  --iface NAME              SocketCAN interface (default: vcan0)\n"
      "  --udp-port PORT           UDP bus port (default: 47000)\n"
      "  --count N                 probe frames (default: 2000)\n"
      "  --period-us US            probe period in microseconds (default: 1000)\n"
      "  --background-fps N        additional bus traffic from a third channel (default: 0)\n"
      "  --json FILE               write the results as JSON\n"
      "  --samples FILE            write every measured latency (us), one per line\n",
      program);
}

/// Returns 0 = run, 1 = help shown, 2 = error.
int parseOptions(int argc, char** argv, Options& options) {
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg{argv[i]};
    if ((arg == "-h") || (arg == "--help")) {
      printUsage(argv[0]);
      return 1;
    }
    if ((i + 1) >= argc) {
      (void)std::fprintf(stderr, "missing value for %s\n", argv[i]);  // NOLINT(cppcoreguidelines-pro-type-vararg)
      return 2;
    }
    const char* value = argv[++i];
    long number = 0;
    bool ok = true;
    if (arg == "--bus") {
      ok = can::parseBackend(value, options.can.backend);
    } else if (arg == "--iface") {
      options.can.socketcan_interface = value;
    } else if (arg == "--udp-port") {
      ok = parseLong(value, number) && (number > 0) && (number <= 65535);
      options.can.udp.port = static_cast<std::uint16_t>(number);
    } else if (arg == "--count") {
      ok = parseLong(value, options.count) && (options.count > 0);
    } else if (arg == "--period-us") {
      ok = parseLong(value, options.period_us) && (options.period_us > 0);
    } else if (arg == "--background-fps") {
      ok = parseLong(value, options.background_fps);
    } else if (arg == "--json") {
      options.json_path = value;
    } else if (arg == "--samples") {
      options.samples_path = value;
    } else {
      ok = false;
    }
    if (!ok) {
      (void)std::fprintf(stderr, "invalid option %s %s\n", argv[i - 1], value);  // NOLINT(cppcoreguidelines-pro-type-vararg)
      return 2;
    }
  }
  return 0;
}

std::unique_ptr<can::ICanChannel> openChannel(const can::CanChannelConfig& config) {
  can::CanStatus status = can::CanStatus::Ok;
  auto channel = can::createCanChannel(config, status);
  if (!channel) {
    (void)std::fprintf(stderr, "cannot open CAN channel: %s\n", can::toString(status));  // NOLINT(cppcoreguidelines-pro-type-vararg)
  }
  return channel;
}

struct Result {
  std::size_t sent{0U};
  std::size_t received{0U};
  std::size_t out_of_order{0U};
  double min_us{0.0};
  double mean_us{0.0};
  double p50_us{0.0};
  double p90_us{0.0};
  double p99_us{0.0};
  double max_us{0.0};
};

double percentile(const std::vector<double>& sorted, double p) {
  const auto index = static_cast<std::size_t>(std::lround(p * static_cast<double>(sorted.size() - 1U)));
  return sorted[index];
}

Result evaluate(const std::vector<std::int64_t>& tx_ns, const std::vector<std::int64_t>& rx_ns, std::size_t ooo) {
  Result r;
  r.sent = tx_ns.size();
  r.out_of_order = ooo;
  std::vector<double> latencies;
  latencies.reserve(tx_ns.size());
  for (std::size_t i = 0U; i < tx_ns.size(); ++i) {
    if (rx_ns[i] != kNotReceived) {
      latencies.push_back(static_cast<double>(rx_ns[i] - tx_ns[i]) / 1000.0);
    }
  }
  r.received = latencies.size();
  if (latencies.empty()) {
    return r;
  }
  std::sort(latencies.begin(), latencies.end());
  const double sum = std::accumulate(latencies.begin(), latencies.end(), 0.0);
  r.min_us = latencies.front();
  r.max_us = latencies.back();
  r.mean_us = sum / static_cast<double>(latencies.size());
  r.p50_us = percentile(latencies, 0.50);
  r.p90_us = percentile(latencies, 0.90);
  r.p99_us = percentile(latencies, 0.99);
  return r;
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  const int parsed = parseOptions(argc, argv, options);
  if (parsed != 0) {
    return parsed == 1 ? 0 : 2;
  }
  const auto tx = openChannel(options.can);
  const auto rx = openChannel(options.can);
  // cppcheck-suppress knownConditionTrueFalse ; false positive: createCanChannel() returns nullptr on failure
  if ((tx == nullptr) || (rx == nullptr)) {
    return 1;
  }
  std::unique_ptr<can::ICanChannel> background;
  if (options.background_fps > 0) {
    background = openChannel(options.can);
    // cppcheck-suppress knownConditionTrueFalse ; see above
    if (background == nullptr) {
      return 1;
    }
  }

  const auto count = static_cast<std::size_t>(options.count);
  std::vector<std::int64_t> tx_ns(count, 0);
  std::vector<std::int64_t> rx_ns(count, kNotReceived);  // preallocated: no allocation while measuring
  std::atomic<bool> stop_rx{false};
  std::atomic<bool> stop_background{false};
  std::size_t out_of_order = 0U;

  std::thread receiver([&] {
    std::uint32_t last = std::numeric_limits<std::uint32_t>::max();
    can::CanFrame frame;
    while (!stop_rx.load()) {
      if (rx->receive(frame, microseconds{20000}) != can::CanStatus::Ok) {
        continue;
      }
      const std::int64_t now_ns = core::now().time_since_epoch().count();
      if ((frame.id != kProbeId) || (frame.dlc != 8U)) {
        continue;  // background traffic
      }
      std::uint32_t seq = 0U;
      for (unsigned int b = 0U; b < 4U; ++b) {
        seq |= static_cast<std::uint32_t>(frame.data[b]) << (8U * b);
      }
      if (seq < count) {
        rx_ns[seq] = now_ns;
        if ((last != std::numeric_limits<std::uint32_t>::max()) && (seq < last)) {
          ++out_of_order;
        }
        last = seq;
      }
    }
  });

  std::thread load;
  if (background) {
    load = std::thread([&] {
      const auto period = nanoseconds{1000000000LL / options.background_fps};
      can::CanFrame frame{kBackgroundId, false, 8U, {}};
      core::TimePoint next = core::now();
      while (!stop_background.load()) {
        (void)background->send(frame);
        next += period;
        os::sleepUntil(next);
      }
    });
  }

  std::this_thread::sleep_for(std::chrono::milliseconds{100});  // receiver ready
  const auto period = microseconds{options.period_us};
  core::TimePoint next = core::now();
  std::size_t tx_errors = 0U;
  for (std::size_t i = 0U; i < count; ++i) {
    can::CanFrame frame{kProbeId, false, 8U, {}};
    for (unsigned int b = 0U; b < 4U; ++b) {
      frame.data[b] = static_cast<std::uint8_t>((i >> (8U * b)) & 0xFFU);
    }
    tx_ns[i] = core::now().time_since_epoch().count();
    if (tx->send(frame) != can::CanStatus::Ok) {
      ++tx_errors;
    }
    next += period;
    os::sleepUntil(next);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds{200});  // late frames
  stop_background = true;
  if (load.joinable()) {
    load.join();
  }
  stop_rx = true;
  receiver.join();

  const Result r = evaluate(tx_ns, rx_ns, out_of_order);
  const can::CanChannelStats rx_stats = rx->stats();
  // NOLINTBEGIN(cppcoreguidelines-pro-type-vararg)
  std::printf("CAN latency over %.*s (one-way, same host, monotonic clock)\n",
              static_cast<int>(rx->name().size()), rx->name().data());
  std::printf("  probes sent %zu, received %zu, lost %zu, out of order %zu, tx errors %zu, rx drops %llu\n",
              r.sent, r.received, r.sent - r.received, r.out_of_order, tx_errors,
              static_cast<unsigned long long>(rx_stats.rx_overruns));
  std::printf("  latency us: min %.1f  mean %.1f  p50 %.1f  p90 %.1f  p99 %.1f  max %.1f\n", r.min_us, r.mean_us,
              r.p50_us, r.p90_us, r.p99_us, r.max_us);
  std::printf("CAN_LATENCY_SUMMARY bus=%.*s period_us=%ld background_fps=%ld sent=%zu received=%zu lost=%zu "
              "min_us=%.1f mean_us=%.1f p50_us=%.1f p90_us=%.1f p99_us=%.1f max_us=%.1f\n",
              static_cast<int>(rx->name().size()), rx->name().data(), options.period_us, options.background_fps,
              r.sent, r.received, r.sent - r.received, r.min_us, r.mean_us, r.p50_us, r.p90_us, r.p99_us, r.max_us);
  // NOLINTEND(cppcoreguidelines-pro-type-vararg)
  if (!options.json_path.empty()) {
    std::array<char, 512U> json{};
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    (void)std::snprintf(json.data(), json.size(),
                        R"({"bus": "%.*s", "period_us": %ld, "background_fps": %ld, "sent": %zu, "received": %zu, )"
                        R"("out_of_order": %zu, "tx_errors": %zu, "min_us": %.3f, "mean_us": %.3f, "p50_us": %.3f, )"
                        R"("p90_us": %.3f, "p99_us": %.3f, "max_us": %.3f})",
                        static_cast<int>(rx->name().size()), rx->name().data(), options.period_us,
                        options.background_fps, r.sent, r.received, r.out_of_order, tx_errors, r.min_us, r.mean_us,
                        r.p50_us, r.p90_us, r.p99_us, r.max_us);
    std::ofstream file(options.json_path);
    file << json.data() << '\n';
    if (!file) {
      (void)std::fprintf(stderr, "cannot write %s\n", options.json_path.c_str());  // NOLINT(cppcoreguidelines-pro-type-vararg)
      return 1;
    }
  }
  if (!options.samples_path.empty()) {
    std::ofstream samples(options.samples_path);
    for (std::size_t i = 0U; i < count; ++i) {
      if (rx_ns[i] != kNotReceived) {
        samples << (static_cast<double>(rx_ns[i] - tx_ns[i]) / 1000.0) << '\n';
      }
    }
    if (!samples) {
      (void)std::fprintf(stderr, "cannot write %s\n", options.samples_path.c_str());  // NOLINT(cppcoreguidelines-pro-type-vararg)
      return 1;
    }
  }
  return (r.received == r.sent) ? 0 : 3;
}
