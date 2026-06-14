/// @file Transport.hpp
/// @brief Datagram transport seam for the DIS leg (issue #47).
///
/// The DIS adapter encodes PDUs and hands them to a transport; a receiving federate pulls them back
/// out. Two implementations:
///
///  - `LoopbackTransport` — an in-process queue. Hermetic and deterministic: no socket, no port, no
///    network. This is what the CI test uses (two endpoints exchanging entity state with zero flake
///    risk).
///  - `UdpTransport` — real IEEE-1278 broadcast/unicast over UDP, behind a runtime flag, for an
///    actual federation. Never exercised in the deterministic test (it could bind a port and
///    flake).
///
/// Keeping the transport behind this interface is what lets the same DIS adapter run hermetically
/// in CI and over a real wire in the field.
#pragma once

#include <cstdint>
#include <vector>

namespace gncsim::interop {

/// @brief Send/receive raw datagrams (each `send` is one PDU; each `receive` returns one PDU).
class ITransport {
 public:
  ITransport() = default;
  ITransport(const ITransport&) = default;
  ITransport(ITransport&&) = default;
  ITransport& operator=(const ITransport&) = default;
  ITransport& operator=(ITransport&&) = default;
  virtual ~ITransport() = default;

  /// @brief Send one datagram (one encoded PDU).
  virtual void send(const std::vector<std::uint8_t>& datagram) = 0;

  /// @brief Receive one datagram if available. Returns false (and leaves `out` untouched) when none
  ///        is pending. Non-blocking.
  virtual bool receive(std::vector<std::uint8_t>& out) = 0;
};

/// @brief In-process FIFO transport — no sockets. Sent datagrams queue up and are drained in order
///        by `receive()`. Two `LoopbackTransport`s can be cross-wired (each sending into the
///        other's inbox) to model two federates exchanging state with no real network.
class LoopbackTransport : public ITransport {
 public:
  void send(const std::vector<std::uint8_t>& datagram) override {
    if (peer_ != nullptr) {
      peer_->inbox_.push_back(datagram);
    } else {
      inbox_.push_back(datagram);  // loop back to self when no peer is wired
    }
  }

  bool receive(std::vector<std::uint8_t>& out) override {
    if (read_pos_ >= inbox_.size()) return false;
    out = inbox_[read_pos_++];
    return true;
  }

  /// @brief Cross-wire so this endpoint's sends land in `peer`'s inbox.
  void connectTo(LoopbackTransport& peer) { peer_ = &peer; }

  std::size_t pending() const { return inbox_.size() - read_pos_; }

 private:
  std::vector<std::vector<std::uint8_t>> inbox_;
  std::size_t read_pos_ = 0;
  LoopbackTransport* peer_ = nullptr;
};

}  // namespace gncsim::interop
