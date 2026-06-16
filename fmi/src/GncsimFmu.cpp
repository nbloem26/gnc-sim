/// @file GncsimFmu.cpp
/// @brief FMI 2.0 for Co-Simulation slave wrapping the pure gnc-sim core.
///
/// This is the ONLY new surface that exports the engagement as a co-simulation FMU. It is a
/// thin, pure wrapper around `gncsim::runSimulation()` — the core (`core/`) is untouched and
/// stays WASM-safe. The wrapper lives here in `fmi/`, built only behind `-DGNCSIM_BUILD_FMI=ON`.
///
/// ## Determinism contract (why outputs bit-match runSimulation)
/// `runSimulation()` advances a fixed-step (RK4, seeded mt19937_64) engagement to completion and
/// returns one telemetry `Frame` per `dt` step. A co-simulation master cannot, in general,
/// re-derive that loop step-by-step without re-implementing the GNC pipeline (and risking FP
/// reordering / nondeterminism — see AGENTS.md golden rule #2). So instead, at
/// `fmi2ExitInitializationMode` this slave runs the WHOLE `runSimulation()` ONCE from the
/// parameter-derived `SimConfig`, caches the resulting `SimResult`, and each `fmi2DoStep`
/// advances a cursor through the precomputed frames, exposing that frame's state as the FMU
/// outputs. The streamed outputs are therefore, by construction, BIT-IDENTICAL to a direct
/// `runSimulation(cfg)` of the same config — which is exactly what the in-repo master test
/// asserts. The communication step size must equal the core `dt` (or an integer multiple);
/// see docs/FMI.md for the time-stepping + determinism discussion.
///
/// ## Co-simulation input (external controller import)
/// The slave exposes an `accel_cmd_override` input vector + enable flag for the closed-loop
/// "import a Simulink controller" use case. Closing that loop requires the core to accept a
/// per-step external guidance command, which `runSimulation()` does not currently expose; the
/// interface is reserved here and the design is documented in docs/FMI.md as a follow-up. With
/// the override DISABLED (default) the FMU is the deterministic open-loop export above.
#include <cmath>
#include <cstdint>
#include <cstring>
#include <new>
#include <string>
#include <vector>

#include "GncsimFmuVars.hpp"
#include "gncsim/core/Config.hpp"
#include "gncsim/core/Types.hpp"
#include "gncsim/scenario/Runner.hpp"

extern "C" {
#include "fmi2/fmi2Functions.h"
}

namespace {

using gncsim::fmi::Vr;

constexpr const char* kModelGuid = "{b0f6e1d2-7c3a-4e5f-9a8b-1c2d3e4f5061}";

// Internal slave state. One per fmi2Instantiate. Holds the parameter block, the cached full-run
// SimResult, and the per-step cursor.
struct GncsimSlave {
  std::string instance_name;
  fmi2CallbackLogger* logger = nullptr;
  fmi2ComponentEnvironment env = nullptr;
  bool logging_on = false;

  // Parameter block (value-reference indexed; only the parameter VRs are meaningful here).
  // Defaults mirror SimConfig's `homing_3dof`-style defaults so an unconfigured instance still
  // produces a valid engagement.
  double seed = 1.0;
  double dt_step_s = 0.005;
  double t_end_s = 60.0;
  double launch_speed_mps = 600.0;
  double launch_elevation_deg = 45.0;
  double launch_azimuth_deg = 0.0;
  double target_pos_m[3] = {8000.0, 0.0, 3000.0};
  double target_vel_mps[3] = {-250.0, 0.0, 0.0};

  // Co-sim input (reserved; see file header / docs/FMI.md).
  double accel_cmd_override_mps2[3] = {0.0, 0.0, 0.0};
  double accel_cmd_override_enable = 0.0;

  // Cached engagement (populated at ExitInitializationMode).
  gncsim::SimResult result;
  bool initialized = false;

  // Communication-point cursor: index of the frame currently exposed at the outputs, and the
  // current communication time. Advanced by fmi2DoStep.
  std::size_t frame_index = 0;
  double current_time_s = 0.0;
  bool done = false;
};

GncsimSlave* asSlave(fmi2Component c) { return static_cast<GncsimSlave*>(c); }

void logMessage(GncsimSlave* s, fmi2Status status, const char* category, const char* msg) {
  if (s != nullptr && s->logging_on && s->logger != nullptr) {
    s->logger(s->env, s->instance_name.c_str(), status, category, "%s", msg);
  }
}

// Build a SimConfig from the slave's parameter block. Mirrors the homing scenario the core
// already supports; only the parameters surfaced as FMU variables are overridden, the rest take
// SimConfig's defaults (so the result is a well-formed engagement).
gncsim::SimConfig buildConfig(const GncsimSlave& s) {
  gncsim::SimConfig cfg;  // struct defaults: scenario "homing", model "3dof", RK4.
  cfg.seed = static_cast<std::uint64_t>(std::llround(s.seed));
  cfg.dt = s.dt_step_s;
  cfg.t_end = s.t_end_s;
  cfg.vehicle.launch_speed = s.launch_speed_mps;
  cfg.vehicle.launch_elevation_deg = s.launch_elevation_deg;
  cfg.vehicle.launch_azimuth_deg = s.launch_azimuth_deg;
  cfg.target.pos0 = {s.target_pos_m[0], s.target_pos_m[1], s.target_pos_m[2]};
  cfg.target.vel0 = {s.target_vel_mps[0], s.target_vel_mps[1], s.target_vel_mps[2]};
  return cfg;
}

// The frame currently under the cursor (clamped to the last frame once the engagement is done so
// outputs are well-defined after the final step).
const gncsim::Frame& currentFrame(const GncsimSlave& s) {
  if (s.result.frames.empty()) {
    static const gncsim::Frame kEmpty{};
    return kEmpty;
  }
  std::size_t idx = s.frame_index;
  if (idx >= s.result.frames.size()) idx = s.result.frames.size() - 1;
  return s.result.frames[idx];
}

// Running CPA miss / intercept flag exposed at the outputs.
//
// While the engagement is still being stepped, this streams the running minimum sampled range —
// a monotonically-refined IN-PROGRESS estimate of closest approach. ONCE the engagement is fully
// stepped (`done`), it reports the core's authoritative figures: SimResult.miss_distance (the
// analytic, sub-dt-accurate CPA — see AGENTS.md golden rule #3) and SimResult.intercept. The
// final values therefore equal runSimulation()'s exactly (the in-repo master asserts this), while
// the streamed value is never below the true CPA.
struct RunningMiss {
  double miss_m;
  bool intercept;
};

constexpr double kLethalRadius_m = 3.0;  // matches Runner.cpp intercept threshold

RunningMiss runningMiss(const GncsimSlave& s) {
  RunningMiss rm{0.0, false};
  if (s.result.frames.empty()) return rm;
  // Done: report the core's analytic CPA + intercept verdict verbatim.
  if (s.done) {
    rm.miss_m = s.result.miss_distance;
    rm.intercept = s.result.intercept;
    return rm;
  }
  // In progress: running minimum sampled range up to the current cursor.
  std::size_t upto = s.frame_index;
  if (upto >= s.result.frames.size()) upto = s.result.frames.size() - 1;
  double best = s.result.frames[0].range;
  for (std::size_t i = 0; i <= upto; ++i) {
    if (s.result.frames[i].range < best) best = s.result.frames[i].range;
  }
  rm.miss_m = best;
  rm.intercept = best < kLethalRadius_m;
  return rm;
}

// Read one scalar Real by value reference.
double readReal(const GncsimSlave& s, fmi2ValueReference vr) {
  const gncsim::Frame& f = currentFrame(s);
  switch (vr) {
    // Parameters
    case Vr::kVrSeed:
      return s.seed;
    case Vr::kVrDtStep_s:
      return s.dt_step_s;
    case Vr::kVrTEnd_s:
      return s.t_end_s;
    case Vr::kVrLaunchSpeed_mps:
      return s.launch_speed_mps;
    case Vr::kVrLaunchElevation_deg:
      return s.launch_elevation_deg;
    case Vr::kVrLaunchAzimuth_deg:
      return s.launch_azimuth_deg;
    case Vr::kVrTargetPosX_m:
      return s.target_pos_m[0];
    case Vr::kVrTargetPosY_m:
      return s.target_pos_m[1];
    case Vr::kVrTargetPosZ_m:
      return s.target_pos_m[2];
    case Vr::kVrTargetVelX_mps:
      return s.target_vel_mps[0];
    case Vr::kVrTargetVelY_mps:
      return s.target_vel_mps[1];
    case Vr::kVrTargetVelZ_mps:
      return s.target_vel_mps[2];
    // Inputs
    case Vr::kVrAccelCmdOverrideX_mps2:
      return s.accel_cmd_override_mps2[0];
    case Vr::kVrAccelCmdOverrideY_mps2:
      return s.accel_cmd_override_mps2[1];
    case Vr::kVrAccelCmdOverrideZ_mps2:
      return s.accel_cmd_override_mps2[2];
    case Vr::kVrAccelCmdOverrideEnable:
      return s.accel_cmd_override_enable;
    // Outputs
    case Vr::kVrTime_s:
      return f.t;
    case Vr::kVrVehPosX_m:
      return f.veh_pos.x;
    case Vr::kVrVehPosY_m:
      return f.veh_pos.y;
    case Vr::kVrVehPosZ_m:
      return f.veh_pos.z;
    case Vr::kVrVehVelX_mps:
      return f.veh_vel.x;
    case Vr::kVrVehVelY_mps:
      return f.veh_vel.y;
    case Vr::kVrVehVelZ_mps:
      return f.veh_vel.z;
    case Vr::kVrTgtPosX_m:
      return f.tgt_pos.x;
    case Vr::kVrTgtPosY_m:
      return f.tgt_pos.y;
    case Vr::kVrTgtPosZ_m:
      return f.tgt_pos.z;
    case Vr::kVrAccelCmdX_mps2:
      return f.accel_cmd.x;
    case Vr::kVrAccelCmdY_mps2:
      return f.accel_cmd.y;
    case Vr::kVrAccelCmdZ_mps2:
      return f.accel_cmd.z;
    case Vr::kVrRange_m:
      return f.range;
    case Vr::kVrClosingSpeed_mps:
      return f.v_closing;
    case Vr::kVrMiss_m:
      return runningMiss(s).miss_m;
    case Vr::kVrIntercept:
      return runningMiss(s).intercept ? 1.0 : 0.0;
    case Vr::kVrDone:
      return s.done ? 1.0 : 0.0;
    default:
      return 0.0;
  }
}

// Write one scalar Real by value reference (only parameters/inputs are writable).
bool writeReal(GncsimSlave& s, fmi2ValueReference vr, double value) {
  switch (vr) {
    case Vr::kVrSeed:
      s.seed = value;
      return true;
    case Vr::kVrDtStep_s:
      s.dt_step_s = value;
      return true;
    case Vr::kVrTEnd_s:
      s.t_end_s = value;
      return true;
    case Vr::kVrLaunchSpeed_mps:
      s.launch_speed_mps = value;
      return true;
    case Vr::kVrLaunchElevation_deg:
      s.launch_elevation_deg = value;
      return true;
    case Vr::kVrLaunchAzimuth_deg:
      s.launch_azimuth_deg = value;
      return true;
    case Vr::kVrTargetPosX_m:
      s.target_pos_m[0] = value;
      return true;
    case Vr::kVrTargetPosY_m:
      s.target_pos_m[1] = value;
      return true;
    case Vr::kVrTargetPosZ_m:
      s.target_pos_m[2] = value;
      return true;
    case Vr::kVrTargetVelX_mps:
      s.target_vel_mps[0] = value;
      return true;
    case Vr::kVrTargetVelY_mps:
      s.target_vel_mps[1] = value;
      return true;
    case Vr::kVrTargetVelZ_mps:
      s.target_vel_mps[2] = value;
      return true;
    case Vr::kVrAccelCmdOverrideX_mps2:
      s.accel_cmd_override_mps2[0] = value;
      return true;
    case Vr::kVrAccelCmdOverrideY_mps2:
      s.accel_cmd_override_mps2[1] = value;
      return true;
    case Vr::kVrAccelCmdOverrideZ_mps2:
      s.accel_cmd_override_mps2[2] = value;
      return true;
    case Vr::kVrAccelCmdOverrideEnable:
      s.accel_cmd_override_enable = value;
      return true;
    default:
      return false;  // outputs are read-only
  }
}

}  // namespace

extern "C" {

// --- Inquire version numbers ---------------------------------------------------------------

const char* fmi2GetTypesPlatform(void) { return fmi2TypesPlatform; }
const char* fmi2GetVersion(void) { return "2.0"; }

fmi2Status fmi2SetDebugLogging(fmi2Component c, fmi2Boolean loggingOn, size_t /*nCategories*/,
                               const fmi2String[] /*categories*/) {
  if (c == nullptr) return fmi2Error;
  asSlave(c)->logging_on = (loggingOn == fmi2True);
  return fmi2OK;
}

// --- Creation / destruction ----------------------------------------------------------------

fmi2Component fmi2Instantiate(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID,
                              fmi2String /*fmuResourceLocation*/,
                              const fmi2CallbackFunctions* functions, fmi2Boolean /*visible*/,
                              fmi2Boolean loggingOn) {
  if (fmuType != fmi2CoSimulation) return nullptr;  // co-simulation slave only
  if (fmuGUID == nullptr || std::strcmp(fmuGUID, kModelGuid) != 0) return nullptr;

  auto* s = new (std::nothrow) GncsimSlave();
  if (s == nullptr) return nullptr;
  s->instance_name = (instanceName != nullptr) ? instanceName : "gncsim";
  s->logging_on = (loggingOn == fmi2True);
  if (functions != nullptr) {
    s->logger = functions->logger;
    s->env = functions->componentEnvironment;
  }
  return static_cast<fmi2Component>(s);
}

void fmi2FreeInstance(fmi2Component c) {
  if (c == nullptr) return;
  delete asSlave(c);
}

// --- Initialization / termination ----------------------------------------------------------

fmi2Status fmi2SetupExperiment(fmi2Component c, fmi2Boolean /*toleranceDefined*/,
                               fmi2Real /*tolerance*/, fmi2Real startTime,
                               fmi2Boolean stopTimeDefined, fmi2Real stopTime) {
  if (c == nullptr) return fmi2Error;
  auto* s = asSlave(c);
  s->current_time_s = startTime;
  if (stopTimeDefined == fmi2True && stopTime > 0.0) s->t_end_s = stopTime;
  return fmi2OK;
}

fmi2Status fmi2EnterInitializationMode(fmi2Component c) {
  if (c == nullptr) return fmi2Error;
  return fmi2OK;
}

fmi2Status fmi2ExitInitializationMode(fmi2Component c) {
  if (c == nullptr) return fmi2Error;
  auto* s = asSlave(c);
  // Run the WHOLE engagement once, now that all parameters are set. This is the determinism
  // anchor: the cached frames ARE runSimulation()'s output (see file header).
  const gncsim::SimConfig cfg = buildConfig(*s);
  s->result = gncsim::runSimulation(cfg);
  s->frame_index = 0;
  s->done = s->result.frames.empty();
  s->initialized = true;
  logMessage(
      s, fmi2OK, "init",
      ("engagement precomputed: " + std::to_string(s->result.frames.size()) + " frames").c_str());
  return fmi2OK;
}

fmi2Status fmi2Terminate(fmi2Component c) {
  if (c == nullptr) return fmi2Error;
  return fmi2OK;
}

fmi2Status fmi2Reset(fmi2Component c) {
  if (c == nullptr) return fmi2Error;
  auto* s = asSlave(c);
  s->result = gncsim::SimResult{};
  s->frame_index = 0;
  s->current_time_s = 0.0;
  s->done = false;
  s->initialized = false;
  return fmi2OK;
}

// --- Get / set -----------------------------------------------------------------------------

fmi2Status fmi2GetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr,
                       fmi2Real value[]) {
  if (c == nullptr) return fmi2Error;
  if (nvr > 0 && (vr == nullptr || value == nullptr)) return fmi2Error;
  const auto* s = asSlave(c);
  for (size_t i = 0; i < nvr; ++i) {
    if (vr[i] >= Vr::kVrCount) return fmi2Error;
    value[i] = readReal(*s, vr[i]);
  }
  return fmi2OK;
}

fmi2Status fmi2SetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr,
                       const fmi2Real value[]) {
  if (c == nullptr) return fmi2Error;
  if (nvr > 0 && (vr == nullptr || value == nullptr)) return fmi2Error;
  auto* s = asSlave(c);
  for (size_t i = 0; i < nvr; ++i) {
    if (vr[i] >= Vr::kVrCount) return fmi2Error;
    if (!writeReal(*s, vr[i], value[i])) return fmi2Error;  // attempt to set a read-only output
  }
  return fmi2OK;
}

// No Integer/Boolean/String variables in this FMU (everything is modelled as Real for a uniform,
// numeric co-sim interface). The setters/getters are still provided per the FMI 2.0 API contract.
fmi2Status fmi2GetInteger(fmi2Component c, const fmi2ValueReference[], size_t nvr, fmi2Integer[]) {
  if (c == nullptr) return fmi2Error;
  return nvr == 0 ? fmi2OK : fmi2Error;
}
fmi2Status fmi2GetBoolean(fmi2Component c, const fmi2ValueReference[], size_t nvr, fmi2Boolean[]) {
  if (c == nullptr) return fmi2Error;
  return nvr == 0 ? fmi2OK : fmi2Error;
}
fmi2Status fmi2GetString(fmi2Component c, const fmi2ValueReference[], size_t nvr, fmi2String[]) {
  if (c == nullptr) return fmi2Error;
  return nvr == 0 ? fmi2OK : fmi2Error;
}
fmi2Status fmi2SetInteger(fmi2Component c, const fmi2ValueReference[], size_t nvr,
                          const fmi2Integer[]) {
  if (c == nullptr) return fmi2Error;
  return nvr == 0 ? fmi2OK : fmi2Error;
}
fmi2Status fmi2SetBoolean(fmi2Component c, const fmi2ValueReference[], size_t nvr,
                          const fmi2Boolean[]) {
  if (c == nullptr) return fmi2Error;
  return nvr == 0 ? fmi2OK : fmi2Error;
}
fmi2Status fmi2SetString(fmi2Component c, const fmi2ValueReference[], size_t nvr,
                         const fmi2String[]) {
  if (c == nullptr) return fmi2Error;
  return nvr == 0 ? fmi2OK : fmi2Error;
}

// --- FMU state (not supported: canGetAndSetFMUstate=false in modelDescription.xml) ---------

fmi2Status fmi2GetFMUstate(fmi2Component, fmi2FMUstate*) { return fmi2Error; }
fmi2Status fmi2SetFMUstate(fmi2Component, fmi2FMUstate) { return fmi2Error; }
fmi2Status fmi2FreeFMUstate(fmi2Component, fmi2FMUstate*) { return fmi2Error; }
fmi2Status fmi2SerializedFMUstateSize(fmi2Component, fmi2FMUstate, size_t*) { return fmi2Error; }
fmi2Status fmi2SerializeFMUstate(fmi2Component, fmi2FMUstate, fmi2Byte[], size_t) {
  return fmi2Error;
}
fmi2Status fmi2DeSerializeFMUstate(fmi2Component, const fmi2Byte[], size_t, fmi2FMUstate*) {
  return fmi2Error;
}

fmi2Status fmi2GetDirectionalDerivative(fmi2Component, const fmi2ValueReference[], size_t,
                                        const fmi2ValueReference[], size_t, const fmi2Real[],
                                        fmi2Real[]) {
  return fmi2Error;  // providesDirectionalDerivative=false
}

// --- Co-simulation stepping ----------------------------------------------------------------

fmi2Status fmi2SetRealInputDerivatives(fmi2Component, const fmi2ValueReference[], size_t,
                                       const fmi2Integer[], const fmi2Real[]) {
  return fmi2Error;  // canInterpolateInputs=false
}
fmi2Status fmi2GetRealOutputDerivatives(fmi2Component, const fmi2ValueReference[], size_t,
                                        const fmi2Integer[], fmi2Real[]) {
  return fmi2Error;  // maxOutputDerivativeOrder=0
}

fmi2Status fmi2DoStep(fmi2Component c, fmi2Real currentCommunicationPoint,
                      fmi2Real communicationStepSize, fmi2Boolean /*noSetFMUStatePriorToCurrent*/) {
  if (c == nullptr) return fmi2Error;
  auto* s = asSlave(c);
  if (!s->initialized) return fmi2Error;
  if (communicationStepSize < 0.0) return fmi2Error;

  // A zero-length step is a no-op (some masters probe with it).
  if (communicationStepSize == 0.0) {
    s->current_time_s = currentCommunicationPoint;
    return fmi2OK;
  }

  // Advance the frame cursor by the number of core dt steps spanned by this communication step.
  // The communication step is expected to be the core dt (or an integer multiple); we round to
  // the nearest whole number of dt steps so floating-point accumulation of the communication
  // point never drops or duplicates a frame. See docs/FMI.md.
  const double steps_real = communicationStepSize / s->dt_step_s;
  const long steps = std::lround(steps_real);
  if (steps <= 0) {
    // Sub-dt communication step: not supported (the core only knows about dt-granular frames).
    logMessage(s, fmi2Warning, "dostep", "communication step smaller than core dt; ignored");
    s->current_time_s = currentCommunicationPoint + communicationStepSize;
    return fmi2Warning;
  }

  const std::size_t last = s->result.frames.empty() ? 0 : s->result.frames.size() - 1;
  std::size_t next = s->frame_index + static_cast<std::size_t>(steps);
  if (next >= last) {
    next = last;
    s->done = true;
  }
  s->frame_index = next;
  s->current_time_s = currentCommunicationPoint + communicationStepSize;
  return fmi2OK;
}

fmi2Status fmi2CancelStep(fmi2Component) { return fmi2Error; }  // canRunAsynchronuously=false

fmi2Status fmi2GetStatus(fmi2Component, const fmi2StatusKind, fmi2Status*) { return fmi2Discard; }

fmi2Status fmi2GetRealStatus(fmi2Component c, const fmi2StatusKind kind, fmi2Real* value) {
  if (c == nullptr || value == nullptr) return fmi2Error;
  if (kind == fmi2LastSuccessfulTime) {
    *value = asSlave(c)->current_time_s;
    return fmi2OK;
  }
  return fmi2Discard;
}

fmi2Status fmi2GetIntegerStatus(fmi2Component, const fmi2StatusKind, fmi2Integer*) {
  return fmi2Discard;
}

fmi2Status fmi2GetBooleanStatus(fmi2Component c, const fmi2StatusKind kind, fmi2Boolean* value) {
  if (c == nullptr || value == nullptr) return fmi2Error;
  if (kind == fmi2Terminated) {
    *value = asSlave(c)->done ? fmi2True : fmi2False;
    return fmi2OK;
  }
  return fmi2Discard;
}

fmi2Status fmi2GetStringStatus(fmi2Component, const fmi2StatusKind, fmi2String*) {
  return fmi2Discard;
}

}  // extern "C"
