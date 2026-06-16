// gnc-sim — interop: drain a SimResult into per-step bus snapshots (issue #47).
//
// Pure, deterministic translation of telemetry frames into the federation bus representation. No
// RNG, no I/O. Entity ids are fixed (1 = interceptor, 2 = threat); a track message is emitted only
// when the fused-track estimate is populated (issue #5 path), so default runs carry entities only.
#include "gncsim/interop/Federation.hpp"

#include <cmath>

namespace gncsim::interop {

namespace {

constexpr std::uint32_t kInterceptorId = 1;
constexpr std::uint32_t kThreatId = 2;
constexpr std::uint32_t kTrackId = 100;

bool isPopulated(const Vector3& v) {
  return std::fabs(v.x) > 0.0 || std::fabs(v.y) > 0.0 || std::fabs(v.z) > 0.0;
}

}  // namespace

std::vector<StepSnapshot> snapshotsFromResult(const SimResult& result) {
  std::vector<StepSnapshot> out;
  out.reserve(result.frames.size());

  std::uint64_t step = 0;
  for (const Frame& f : result.frames) {
    StepSnapshot snap;
    snap.step = step++;
    snap.t_s = f.t;

    EntityStateMsg interceptor;
    interceptor.entity_id = kInterceptorId;
    interceptor.force_id = 1;     // friendly
    interceptor.entity_kind = 1;  // platform / interceptor
    interceptor.t_s = f.t;
    interceptor.pos_m = f.veh_pos;
    interceptor.vel_mps = f.veh_vel;
    interceptor.att = f.veh_att;
    snap.entities.push_back(interceptor);

    EntityStateMsg threat;
    threat.entity_id = kThreatId;
    threat.force_id = 2;     // opposing
    threat.entity_kind = 2;  // threat
    threat.t_s = f.t;
    threat.pos_m = f.tgt_pos;
    threat.vel_mps = f.tgt_vel;
    snap.entities.push_back(threat);

    // Emit a fused-track message only when the multi-tracker path populated an estimate.
    if (isPopulated(f.track_pos_est)) {
      TrackMsg tr;
      tr.track_id = kTrackId;
      tr.t_s = f.t;
      tr.pos_est_m = f.track_pos_est;
      tr.vel_est_mps = f.nav_vel_est;
      // Quality as the inverse of (1 + NIS): high when the last update was consistent.
      tr.quality = 1.0 / (1.0 + std::fabs(f.track_nis));
      snap.tracks.push_back(tr);
    }

    out.push_back(std::move(snap));
  }
  return out;
}

}  // namespace gncsim::interop
