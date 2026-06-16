/// @file GncsimFmuVars.hpp
/// @brief Single source of truth for the FMU's variable layout (value references).
///
/// Both the FMI 2.0 Co-Simulation slave (GncsimFmu.cpp) and the modelDescription.xml
/// generator (GenerateModelDescription.cpp) include this header so the value references
/// in the binary and in the XML can NEVER drift apart. Each enumerator is the fmi2
/// `valueReference` of one scalar Real variable.
///
/// The FMU wraps the pure core's runSimulation() (see fmi/GncsimFmu.cpp for the
/// determinism contract). Inputs are parameters/inputs consumed at initialization;
/// outputs stream the per-frame engagement telemetry, one communication step per
/// core fixed `dt` step.
#pragma once

#include <cstdint>

namespace gncsim::fmi {

// Value references for the FMU's scalar Real variables. Stable, contiguous from 0 so the
// generated modelDescription.xml indexing is trivial. Units carried in names per AGENTS.md.
enum Vr : std::uint32_t {
  // --- Parameters (set before fmi2ExitInitializationMode; fix the engagement) ---
  kVrSeed = 0,                 // RNG seed (integer-valued real parameter)
  kVrDtStep_s = 1,             // core fixed integration step [s]
  kVrTEnd_s = 2,               // max sim time [s]
  kVrLaunchSpeed_mps = 3,      // interceptor launch speed [m/s]
  kVrLaunchElevation_deg = 4,  // interceptor launch elevation [deg]
  kVrLaunchAzimuth_deg = 5,    // interceptor launch azimuth [deg]
  kVrTargetPosX_m = 6,         // initial target ENU position [m]
  kVrTargetPosY_m = 7,
  kVrTargetPosZ_m = 8,
  kVrTargetVelX_mps = 9,  // initial target ENU velocity [m/s]
  kVrTargetVelY_mps = 10,
  kVrTargetVelZ_mps = 11,

  // --- Input (co-sim input; reserved for external-controller import, see FMI.md) ---
  kVrAccelCmdOverrideX_mps2 = 12,  // external commanded-accel override X [m/s^2]
  kVrAccelCmdOverrideY_mps2 = 13,
  kVrAccelCmdOverrideZ_mps2 = 14,
  kVrAccelCmdOverrideEnable = 15,  // 1 => use the override input instead of internal guidance

  // --- Outputs (streamed per communication step) ---
  kVrTime_s = 16,     // current engagement time [s]
  kVrVehPosX_m = 17,  // interceptor ENU position [m]
  kVrVehPosY_m = 18,
  kVrVehPosZ_m = 19,
  kVrVehVelX_mps = 20,  // interceptor ENU velocity [m/s]
  kVrVehVelY_mps = 21,
  kVrVehVelZ_mps = 22,
  kVrTgtPosX_m = 23,  // target ENU position [m]
  kVrTgtPosY_m = 24,
  kVrTgtPosZ_m = 25,
  kVrAccelCmdX_mps2 = 26,  // realized guidance command [m/s^2]
  kVrAccelCmdY_mps2 = 27,
  kVrAccelCmdZ_mps2 = 28,
  kVrRange_m = 29,           // current vehicle-to-target range [m]
  kVrClosingSpeed_mps = 30,  // closing speed [m/s]
  kVrMiss_m = 31,            // running closest-approach (CPA) miss distance [m]
  kVrIntercept = 32,         // 1.0 once intercept (CPA < lethal radius), else 0.0
  kVrDone = 33,              // 1.0 once the engagement has been fully stepped, else 0.0

  kVrCount = 34  // total number of scalar Real variables
};

}  // namespace gncsim::fmi
