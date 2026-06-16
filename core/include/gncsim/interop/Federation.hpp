/// @file Federation.hpp
/// @brief Distributed-simulation federation seam (issue #47) — the publish/subscribe interfaces a
///        federation leg (DIS / HLA / DDS) implements, plus the per-step state structs published on
///        the bus.
///
/// **This module is opt-in interop tooling and is NOT part of the pure GNC hot path.** `core/`'s
/// `runSimulation()` never references anything here, so the default native/WASM trajectory and RNG
/// draw order stay byte-identical (AGENTS.md golden rule #1/#2). A driver (the CLI, a test) owns a
/// `MessageBus`, drains a `SimResult` (or a live step) into it, and the bus fans the snapshots out
/// to whichever adapters are attached — a `FileRecorder` for deterministic record/replay, a
/// `DisAdapter` for the IEEE-1278 wire leg, or a documented HLA/DDS adapter.
///
/// The wire/recording representation deliberately works in `double`-precision SI units in the sim's
/// local ENU world frame (see Types.hpp / docs/DATA_CONTRACT.md). A real DIS/HLA/DDS gateway maps
/// those to the federation's geocentric frame at the adapter boundary; that mapping is the
/// adapter's job, not the bus's.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "gncsim/core/Types.hpp"
#include "gncsim/math/Quaternion.hpp"
#include "gncsim/math/Vector3.hpp"

namespace gncsim::interop {

/// @brief A simulation entity (vehicle or threat) as published on the federation bus.
///
/// This is the bus-level analogue of a DIS Entity State PDU: a stable numeric id, a kind, and the
/// truth kinematic state in the sim's ENU world frame. Units are SI (metres, m/s, rad/s); the
/// `_*` suffixes follow the project naming convention (AGENTS.md).
struct EntityStateMsg {
  std::uint32_t entity_id = 0;   ///< stable per-run entity id (1 = interceptor, 2 = threat, …)
  std::uint8_t force_id = 0;     ///< 0 other, 1 friendly, 2 opposing (DIS ForceID semantics)
  std::uint8_t entity_kind = 0;  ///< coarse kind: 0 unknown, 1 platform/interceptor, 2 threat
  double t_s = 0.0;              ///< simulation time of this snapshot [s]
  Vector3 pos_m;                 ///< ENU position [m]
  Vector3 vel_mps;               ///< ENU velocity [m/s]
  Quaternion att;                ///< body→ENU attitude (identity on the 3DOF path)
  Vector3 ang_vel_radps;         ///< body angular rate [rad/s] (0 on the 3DOF path)
};

/// @brief A target track / detection as published on the bus (the optional second PDU type).
///
/// Mirrors the fused-track telemetry (`Frame::track_pos_est`, issue #5): an estimate of a tracked
/// object's state with a scalar quality figure. A DIS leg can carry this in a vendor/experimental
/// PDU; HLA as an interaction; DDS as a separate topic.
struct TrackMsg {
  std::uint32_t track_id = 0;  ///< track id (need not equal any entity_id)
  double t_s = 0.0;            ///< simulation time of this estimate [s]
  Vector3 pos_est_m;           ///< estimated ENU position [m]
  Vector3 vel_est_mps;         ///< estimated ENU velocity [m/s]
  double quality = 0.0;        ///< track quality / confidence figure (e.g. inverse NIS); ≥0
};

/// @brief One bus snapshot: every entity + track published for a single simulation step.
struct StepSnapshot {
  std::uint64_t step = 0;  ///< monotonically increasing step index
  double t_s = 0.0;        ///< simulation time of the step [s]
  std::vector<EntityStateMsg> entities;
  std::vector<TrackMsg> tracks;
};

/// @brief The federation adapter seam — the publish/subscribe interface a federation leg
/// implements.
///
/// The `MessageBus` drives each attached adapter: `onStart` once, `publish` per step, `onStop`
/// once. The DIS adapter (the concrete, tested leg) implements this by encoding each snapshot to
/// IEEE-1278 PDUs and pushing them over a transport. The HLA and DDS adapters implement the *same*
/// three methods — see docs/DISTRIBUTED.md for exactly where an RTI/DDS vendor plugs in. Keeping
/// the seam this small is what lets the heavy RTI live entirely outside this repo.
class IFederateAdapter {
 public:
  IFederateAdapter() = default;
  IFederateAdapter(const IFederateAdapter&) = default;
  IFederateAdapter(IFederateAdapter&&) = default;
  IFederateAdapter& operator=(const IFederateAdapter&) = default;
  IFederateAdapter& operator=(IFederateAdapter&&) = default;
  virtual ~IFederateAdapter() = default;

  /// @brief Federation/run metadata announced once before any snapshot is published.
  virtual void onStart(std::uint16_t exercise_id, std::uint16_t site_id, std::uint16_t app_id) = 0;

  /// @brief Publish one step's snapshot. Called once per simulation step, in step order.
  virtual void publish(const StepSnapshot& snap) = 0;

  /// @brief End of run. After this the adapter must have flushed everything it owns.
  virtual void onStop() = 0;
};

/// @brief Drain a complete `SimResult` into a sequence of per-step bus snapshots.
///
/// Pure, deterministic, allocation-only: it walks `result.frames` in order and emits one
/// `StepSnapshot` per frame with the interceptor (id 1) and threat (id 2) entities, plus a track
/// message when the fused-track estimate is populated. Used by the CLI/tests to feed a federated
/// run from an already-computed result; the bus then replays the snapshots through adapters.
std::vector<StepSnapshot> snapshotsFromResult(const SimResult& result);

}  // namespace gncsim::interop
