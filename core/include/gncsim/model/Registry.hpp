// gnc-sim — model registry (issue #31). Resolves config strings to concrete model implementations
// behind the interfaces in Interfaces.hpp. This is the single place that knows which string selects
// which model, so the Runner is thin orchestration and trade studies are config sweeps.
//
// Keys (the strings already used in the data contract / configs):
//   guidance.law      : "pronav" | "apn" | "none"        -> IGuidance
//   nav.filter        : "alpha_beta" | "ekf" | "imm"     -> INavigator
//   trackers.sensors[].type : "radar" | "ir"            -> ISensor
//   model             : "3dof" | "6dof"                  -> IDynamics
//   env.frame         : "flat" (round handled separately)-> IEnvironment
//   target.maneuver   : "constant" | "weave"             -> IThreat
//
// An unknown key throws std::invalid_argument with a clear message (replacing silent fall-through).
// The default config strings resolve to exactly today's models, so existing configs are unchanged.
//
// Pure: no globals, no I/O. The registry is constructed on demand from the SimConfig; resolution is
// a small switch on the config string. Factories return std::unique_ptr to the interface.
#pragma once

#include <memory>
#include <string>

#include "gncsim/core/Config.hpp"
#include "gncsim/model/Interfaces.hpp"

namespace gncsim {

// Stateless resolver from SimConfig sub-blocks + config strings to concrete models. Each factory
// builds a fresh model adapter wrapping the existing numerics. Construct one per run (cheap).
class ModelRegistry {
 public:
  ModelRegistry() = default;

  // guidance.law -> IGuidance. Throws std::invalid_argument on an unknown law.
  std::unique_ptr<IGuidance> makeGuidance(const std::string& law, const GuidanceConfig& cfg) const;

  // nav.filter -> INavigator. `dt` and the EKF tuning come from the relevant config blocks.
  // Throws std::invalid_argument on an unknown filter.
  std::unique_ptr<INavigator> makeNavigator(const std::string& filter, const NavConfig& nav,
                                            const SeekerNoise& seeker, double dt) const;

  // A fixed external track sensor description -> ISensor. Throws on an unknown type.
  std::unique_ptr<ISensor> makeSensor(const TrackerSensorConfig& sc) const;

  // model -> IDynamics ("3dof" point mass | "6dof" rigid body). Throws on an unknown model.
  std::unique_ptr<IDynamics> makeDynamics(const std::string& model, const VehicleConfig& vehicle,
                                          Integrator integ) const;

  // env.frame -> IEnvironment (flat-Earth gravity + USSA76 atmosphere). Throws on an unknown frame.
  std::unique_ptr<IEnvironment> makeEnvironment(const EnvConfig& env) const;

  // target.maneuver -> IThreat. Throws on an unknown maneuver.
  std::unique_ptr<IThreat> makeThreat(const TargetConfig& target) const;
};

}  // namespace gncsim
