#include "can/can_channel_factory.hpp"

#include <system_error>

#include "can/socketcan_channel.hpp"
#include "core/logger.hpp"

namespace vecu::can {

namespace {

std::string errnoText(int error) { return std::error_code(error, std::generic_category()).message(); }

}  // namespace

const char* toString(CanBackend backend) noexcept {
  switch (backend) {
    case CanBackend::Auto:
      return "auto";
    case CanBackend::SocketCan:
      return "socketcan";
    case CanBackend::Udp:
      return "udp";
  }
  return "unknown";
}

bool parseBackend(std::string_view text, CanBackend& backend) noexcept {
  if (text == "auto") {
    backend = CanBackend::Auto;
  } else if (text == "socketcan") {
    backend = CanBackend::SocketCan;
  } else if (text == "udp") {
    backend = CanBackend::Udp;
  } else {
    return false;
  }
  return true;
}

std::unique_ptr<ICanChannel> createCanChannel(const CanChannelConfig& config, CanStatus& status) {
  const core::Logger log("can");

  if ((config.backend == CanBackend::SocketCan) || (config.backend == CanBackend::Auto)) {
    auto socketcan = std::make_unique<SocketCanChannel>(config.socketcan_interface);
    status = socketcan->open();
    if (status == CanStatus::Ok) {
      log.info(core::strCat("backend=socketcan interface=", config.socketcan_interface));
      return socketcan;
    }
    const std::string reason = core::strCat("SocketCAN interface '", config.socketcan_interface,
                                            "' unavailable: ", toString(status), " (",
                                            errnoText(socketcan->lastErrno()), ")");
    if (config.backend == CanBackend::SocketCan) {
      log.error(reason);
      return nullptr;
    }
    log.warning(core::strCat(reason, "; falling back to UDP software CAN bus"));
  }

  auto udp = std::make_unique<UdpCanChannel>(config.udp);
  status = udp->open();
  if (status != CanStatus::Ok) {
    log.error(core::strCat("UDP software CAN bus ", config.udp.group_address, ":", config.udp.port,
                           " failed: ", toString(status), " (", errnoText(udp->lastErrno()), ")"));
    return nullptr;
  }
  log.info(core::strCat("backend=udp bus=", config.udp.group_address, ":", config.udp.port,
                        " label=", udp->name()));
  return udp;
}

}  // namespace vecu::can
