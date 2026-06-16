/// @file UdpTransport.hpp
/// @brief Real-network UDP transport for the DIS leg (issue #47) — POSIX sockets, std-only, opt-in.
///
/// This is the only part of interop that touches the network, and it is **never** used by the
/// deterministic CI test (which uses `LoopbackTransport` so no port is bound and nothing can
/// flake). A driver constructs it explicitly, behind a runtime flag, to emit/receive DIS PDUs on
/// the standard exercise port (3000). Non-blocking receive. Built only on POSIX hosts (guarded by
/// `GNCSIM_INTEROP_UDP`); on platforms without BSD sockets the symbol is simply absent.
#pragma once

#if defined(GNCSIM_INTEROP_UDP)

#include <cstdint>
#include <string>
#include <vector>

#include "gncsim/interop/Transport.hpp"

namespace gncsim::interop {

/// @brief UDP broadcast/unicast datagram transport (one PDU per datagram).
class UdpTransport : public ITransport {
 public:
  /// @param remote_addr  dotted-quad destination (e.g. "127.0.0.1" or a broadcast address)
  /// @param port         DIS exercise port (3000 by convention)
  /// @param bind_local   also bind locally to receive (set false for a send-only emitter)
  UdpTransport(const std::string& remote_addr, std::uint16_t port, bool bind_local);
  ~UdpTransport() override;

  UdpTransport(const UdpTransport&) = delete;
  UdpTransport& operator=(const UdpTransport&) = delete;
  UdpTransport(UdpTransport&&) = delete;
  UdpTransport& operator=(UdpTransport&&) = delete;

  void send(const std::vector<std::uint8_t>& datagram) override;
  bool receive(std::vector<std::uint8_t>& out) override;

 private:
  int fd_ = -1;
  std::uint32_t remote_be_ = 0;  // remote address, network byte order
  std::uint16_t port_ = 0;
};

}  // namespace gncsim::interop

#endif  // GNCSIM_INTEROP_UDP
