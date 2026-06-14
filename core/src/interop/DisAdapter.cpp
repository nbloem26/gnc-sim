// gnc-sim — interop: receive-side helper for the DIS adapter (issue #47).
#include "gncsim/interop/DisAdapter.hpp"

#include "gncsim/interop/Dis.hpp"

namespace gncsim::interop {

void drainDis(ITransport& transport, std::vector<EntityStateMsg>& entities_out,
              std::vector<TrackMsg>& tracks_out) {
  std::vector<std::uint8_t> datagram;
  while (transport.receive(datagram)) {
    const dis::PduHeader h = dis::peekHeader(datagram);
    switch (static_cast<dis::PduType>(h.pdu_type)) {
      case dis::PduType::kEntityState:
        entities_out.push_back(dis::decodeEntityState(datagram));
        break;
      case dis::PduType::kTrackDetection:
        tracks_out.push_back(dis::decodeTrack(datagram));
        break;
      default:
        break;  // unknown PDU type: ignore (a real federate would log + skip)
    }
  }
}

}  // namespace gncsim::interop
