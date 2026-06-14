/// @file DisAdapter.hpp
/// @brief DIS federation adapter (issue #47): the `IFederateAdapter` that puts the sim on an
///        IEEE-1278 wire.
///
/// On each `publish()` it encodes every entity to an Entity State PDU and every track to a
/// Track/Detection PDU and sends them over the injected `ITransport`. A peer federate can drain its
/// transport and `decode*` the datagrams back into `EntityStateMsg` / `TrackMsg`. With a
/// `LoopbackTransport` this is fully hermetic (the CI test); with a `UdpTransport` it is a real DIS
/// emitter. This is the one *working* federation leg — HLA/DDS are interface + docs only.
#pragma once

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "gncsim/interop/Dis.hpp"
#include "gncsim/interop/Federation.hpp"
#include "gncsim/interop/Transport.hpp"

namespace gncsim::interop {

/// @brief Publishes bus snapshots as DIS PDUs over a transport.
class DisAdapter : public IFederateAdapter {
 public:
  /// @param transport  the datagram sink (loopback for tests, UDP for a real federation)
  /// @param site_id    DIS simulation-address site id
  /// @param app_id     DIS simulation-address application id
  explicit DisAdapter(std::shared_ptr<ITransport> transport, std::uint16_t site_id = 1,
                      std::uint16_t app_id = 1)
      : transport_(std::move(transport)), site_id_(site_id), app_id_(app_id) {}

  void onStart(std::uint16_t exercise_id, std::uint16_t site_id, std::uint16_t app_id) override {
    exercise_id_ = static_cast<std::uint8_t>(exercise_id & 0xFF);
    site_id_ = site_id;
    app_id_ = app_id;
  }

  void publish(const StepSnapshot& snap) override {
    for (const auto& e : snap.entities) {
      transport_->send(dis::encodeEntityState(e, exercise_id_, site_id_, app_id_));
      ++pdus_sent_;
    }
    for (const auto& tr : snap.tracks) {
      transport_->send(dis::encodeTrack(tr, exercise_id_, site_id_, app_id_));
      ++pdus_sent_;
    }
  }

  void onStop() override {}

  std::uint64_t pdusSent() const { return pdus_sent_; }

 private:
  std::shared_ptr<ITransport> transport_;
  std::uint8_t exercise_id_ = 1;
  std::uint16_t site_id_ = 1;
  std::uint16_t app_id_ = 1;
  std::uint64_t pdus_sent_ = 0;
};

/// @brief Drain a transport and decode every pending DIS PDU into typed messages.
///
/// Helper for a receiving federate / the test: pulls datagrams until the transport is empty,
/// dispatches on the PDU type, and appends to the right output vector.
void drainDis(ITransport& transport, std::vector<EntityStateMsg>& entities_out,
              std::vector<TrackMsg>& tracks_out);

}  // namespace gncsim::interop
