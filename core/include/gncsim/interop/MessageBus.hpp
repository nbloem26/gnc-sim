/// @file MessageBus.hpp
/// @brief Internal publish/record bus (issue #47): fan one per-step snapshot out to N federation
///        adapters in a fixed, deterministic order.
///
/// The bus owns nothing about the wire — it is a thin, ordered multiplexer. A driver attaches
/// adapters (DIS, a FileRecorder, a future HLA/DDS leg), calls `start()`, then `publish()` once per
/// step, then `stop()`. Adapters are visited in attach order on every call, so a fixed set of
/// adapters produces byte-identical side effects for the same snapshot stream — the property the
/// record→replay determinism test relies on.
///
/// Not in the pure hot path: `runSimulation()` never constructs a bus.
#pragma once

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "gncsim/interop/Federation.hpp"

namespace gncsim::interop {

/// @brief Ordered fan-out of step snapshots to attached federation adapters.
class MessageBus {
 public:
  /// @brief Attach an adapter. Adapters are driven in attach order; ownership is transferred.
  void attach(std::shared_ptr<IFederateAdapter> adapter) {
    adapters_.push_back(std::move(adapter));
  }

  /// @brief Announce the run to every adapter (DIS exercise/site/application ids).
  void start(std::uint16_t exercise_id, std::uint16_t site_id, std::uint16_t app_id) {
    exercise_id_ = exercise_id;
    site_id_ = site_id;
    app_id_ = app_id;
    for (const auto& a : adapters_) a->onStart(exercise_id, site_id, app_id);
    started_ = true;
  }

  /// @brief Fan one step's snapshot out to every adapter in attach order.
  void publish(const StepSnapshot& snap) {
    for (const auto& a : adapters_) a->publish(snap);
  }

  /// @brief End the run; flush every adapter.
  void stop() {
    for (const auto& a : adapters_) a->onStop();
    started_ = false;
  }

  std::size_t adapterCount() const { return adapters_.size(); }
  bool started() const { return started_; }

 private:
  std::vector<std::shared_ptr<IFederateAdapter>> adapters_;
  std::uint16_t exercise_id_ = 0;
  std::uint16_t site_id_ = 0;
  std::uint16_t app_id_ = 0;
  bool started_ = false;
};

}  // namespace gncsim::interop
