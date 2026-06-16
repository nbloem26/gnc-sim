/// @file FmiMaster.cpp
/// @brief Minimal in-repo FMI 2.0 Co-Simulation master + correctness/determinism test.
///
/// A small, dependency-free co-simulation master (NO external simulator) that:
///   1. dlopen()s the exported co-sim FMU shared library (the SAME .so packaged into gncsim.fmu),
///   2. resolves the fmi2* C entry points,
///   3. drives a full engagement through Instantiate -> SetupExperiment -> EnterInitialization ->
///      (SetReal parameters) -> ExitInitialization -> repeated DoStep/GetReal -> Terminate ->
///      FreeInstance, recording the per-step interceptor state, range and miss, and
///   4. asserts those streamed outputs are BIT-IDENTICAL to a direct gncsim::runSimulation() of
///      the same config (the determinism contract — see fmi/src/GncsimFmu.cpp).
///
/// Exit code 0 on success; non-zero (with a diagnostic) on the first mismatch. Wired into CTest
/// by fmi/CMakeLists.txt as `fmi_master_test`.
///
/// Usage: fmi_master <path-to-fmu-shared-library>
#include <dlfcn.h>

#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
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

// Resolved fmi2* entry points from the loaded shared library.
struct FmiApi {
  fmi2InstantiateTYPE* instantiate = nullptr;
  fmi2FreeInstanceTYPE* freeInstance = nullptr;
  fmi2SetupExperimentTYPE* setupExperiment = nullptr;
  fmi2EnterInitializationModeTYPE* enterInit = nullptr;
  fmi2ExitInitializationModeTYPE* exitInit = nullptr;
  fmi2SetRealTYPE* setReal = nullptr;
  fmi2GetRealTYPE* getReal = nullptr;
  fmi2DoStepTYPE* doStep = nullptr;
  fmi2TerminateTYPE* terminate = nullptr;
  fmi2GetVersionTYPE* getVersion = nullptr;
};

template <typename T>
bool resolve(void* handle, const char* name, T*& out) {
  // POSIX dlsym returns void*; the function-pointer cast is the documented idiom.
  void* sym = dlsym(handle, name);
  if (sym == nullptr) {
    std::fprintf(stderr, "fmi_master: missing symbol '%s'\n", name);
    return false;
  }
  out = reinterpret_cast<T*>(sym);
  return true;
}

// Logger callback (the FMU only logs when loggingOn). `message` is a printf-style format string
// followed by the matching varargs, per the FMI 2.0 fmi2CallbackLogger contract.
void loggerCb(fmi2ComponentEnvironment, fmi2String instanceName, fmi2Status, fmi2String category,
              fmi2String message, ...) {
  std::printf("[fmu:%s:%s] ", instanceName != nullptr ? instanceName : "?",
              category != nullptr ? category : "?");
  va_list args;
  va_start(args, message);
  // va_list is correctly va_start'd above and va_end'd below; the analyzer mis-flags this
  // textbook varargs forwarding. NOLINTNEXTLINE(clang-analyzer-valist.Uninitialized)
  std::vprintf(message != nullptr ? message : "", args);
  va_end(args);
  std::printf("\n");
}

double getOne(const FmiApi& api, fmi2Component comp, Vr vr) {
  const fmi2ValueReference ref = vr;
  fmi2Real value = 0.0;
  api.getReal(comp, &ref, 1, &value);
  return value;
}

void setOne(const FmiApi& api, fmi2Component comp, Vr vr, double value) {
  const fmi2ValueReference ref = vr;
  const fmi2Real v = value;
  api.setReal(comp, &ref, 1, &v);
}

// Bit-exact comparison: the streamed FMU output and the direct-run telemetry come from the SAME
// computation, so they must be EXACTLY equal (no tolerance). Comparing the object representation
// (rather than `==`) is deliberate — it catches a single ULP of difference AND treats two
// identical NaN bit patterns as equal, which is exactly the determinism property under test.
// NOLINTNEXTLINE(bugprone-suspicious-memory-comparison): object-representation compare is intended.
bool bitEqual(double a, double b) { return std::memcmp(&a, &b, sizeof(double)) == 0; }

int fail(const char* what, std::size_t i, double fmu, double direct) {
  std::fprintf(stderr, "fmi_master: MISMATCH at frame %zu, field '%s': fmu=%.17g direct=%.17g\n", i,
               what, fmu, direct);
  return 1;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: fmi_master <path-to-fmu-shared-library>\n");
    return 2;
  }
  const char* lib_path = argv[1];

  void* handle = dlopen(lib_path, RTLD_NOW | RTLD_LOCAL);
  if (handle == nullptr) {
    std::fprintf(stderr, "fmi_master: dlopen('%s') failed: %s\n", lib_path, dlerror());
    return 1;
  }

  FmiApi api;
  bool ok = resolve(handle, "fmi2Instantiate", api.instantiate) &&
            resolve(handle, "fmi2FreeInstance", api.freeInstance) &&
            resolve(handle, "fmi2SetupExperiment", api.setupExperiment) &&
            resolve(handle, "fmi2EnterInitializationMode", api.enterInit) &&
            resolve(handle, "fmi2ExitInitializationMode", api.exitInit) &&
            resolve(handle, "fmi2SetReal", api.setReal) &&
            resolve(handle, "fmi2GetReal", api.getReal) &&
            resolve(handle, "fmi2DoStep", api.doStep) &&
            resolve(handle, "fmi2Terminate", api.terminate) &&
            resolve(handle, "fmi2GetVersion", api.getVersion);
  if (!ok) {
    dlclose(handle);
    return 1;
  }

  std::printf("fmi_master: loaded '%s', FMI version %s\n", lib_path, api.getVersion());

  // The reference config: a homing engagement with non-default parameters, so the test exercises
  // the FMU's SetReal parameter path (not just the struct defaults). Keep this in sync with what
  // we push into the FMU below.
  gncsim::SimConfig cfg;  // scenario "homing", model "3dof", RK4
  cfg.seed = 7;
  cfg.dt = 0.005;
  cfg.t_end = 20.0;
  cfg.vehicle.launch_speed = 900.0;
  cfg.vehicle.launch_elevation_deg = 35.0;
  cfg.vehicle.launch_azimuth_deg = 5.0;
  cfg.target.pos0 = {9000.0, 500.0, 3500.0};
  cfg.target.vel0 = {-280.0, 10.0, -20.0};

  const gncsim::SimResult direct = gncsim::runSimulation(cfg);
  std::printf("fmi_master: direct runSimulation -> %zu frames, miss=%.6f m, intercept=%d\n",
              direct.frames.size(), direct.miss_distance, direct.intercept ? 1 : 0);

  // --- Drive the FMU over the engagement -----------------------------------------------------
  fmi2CallbackFunctions callbacks{};
  callbacks.logger = loggerCb;
  callbacks.allocateMemory = std::calloc;
  callbacks.freeMemory = std::free;
  callbacks.stepFinished = nullptr;
  callbacks.componentEnvironment = nullptr;

  const char* guid = "{b0f6e1d2-7c3a-4e5f-9a8b-1c2d3e4f5061}";
  fmi2Component comp =
      api.instantiate("gncsim_master", fmi2CoSimulation, guid, "", &callbacks, fmi2False, fmi2True);
  if (comp == nullptr) {
    std::fprintf(stderr, "fmi_master: fmi2Instantiate returned NULL\n");
    dlclose(handle);
    return 1;
  }

  api.setupExperiment(comp, fmi2False, 0.0, 0.0, fmi2True, cfg.t_end);
  api.enterInit(comp);
  // Push the parameters before exiting initialization (the FMU runs the engagement at Exit).
  setOne(api, comp, Vr::kVrSeed, static_cast<double>(cfg.seed));
  setOne(api, comp, Vr::kVrDtStep_s, cfg.dt);
  setOne(api, comp, Vr::kVrTEnd_s, cfg.t_end);
  setOne(api, comp, Vr::kVrLaunchSpeed_mps, cfg.vehicle.launch_speed);
  setOne(api, comp, Vr::kVrLaunchElevation_deg, cfg.vehicle.launch_elevation_deg);
  setOne(api, comp, Vr::kVrLaunchAzimuth_deg, cfg.vehicle.launch_azimuth_deg);
  setOne(api, comp, Vr::kVrTargetPosX_m, cfg.target.pos0.x);
  setOne(api, comp, Vr::kVrTargetPosY_m, cfg.target.pos0.y);
  setOne(api, comp, Vr::kVrTargetPosZ_m, cfg.target.pos0.z);
  setOne(api, comp, Vr::kVrTargetVelX_mps, cfg.target.vel0.x);
  setOne(api, comp, Vr::kVrTargetVelY_mps, cfg.target.vel0.y);
  setOne(api, comp, Vr::kVrTargetVelZ_mps, cfg.target.vel0.z);
  api.exitInit(comp);

  // Step the FMU one core dt at a time, reading the outputs at each communication point and
  // comparing them bit-for-bit to the matching direct-run frame. Frame 0 is the initial
  // (pre-step) communication point; thereafter doStep advances one frame.
  const std::size_t n = direct.frames.size();
  int rc = 0;
  for (std::size_t i = 0; i < n; ++i) {
    const gncsim::Frame& fr = direct.frames[i];

    const double t = getOne(api, comp, Vr::kVrTime_s);
    const double px = getOne(api, comp, Vr::kVrVehPosX_m);
    const double py = getOne(api, comp, Vr::kVrVehPosY_m);
    const double pz = getOne(api, comp, Vr::kVrVehPosZ_m);
    const double vx = getOne(api, comp, Vr::kVrVehVelX_mps);
    const double tgx = getOne(api, comp, Vr::kVrTgtPosX_m);
    const double rng = getOne(api, comp, Vr::kVrRange_m);
    const double ax = getOne(api, comp, Vr::kVrAccelCmdX_mps2);

    if (!bitEqual(t, fr.t)) {
      rc = fail("time_s", i, t, fr.t);
      break;
    }
    if (!bitEqual(px, fr.veh_pos.x)) {
      rc = fail("veh_pos_x_m", i, px, fr.veh_pos.x);
      break;
    }
    if (!bitEqual(py, fr.veh_pos.y)) {
      rc = fail("veh_pos_y_m", i, py, fr.veh_pos.y);
      break;
    }
    if (!bitEqual(pz, fr.veh_pos.z)) {
      rc = fail("veh_pos_z_m", i, pz, fr.veh_pos.z);
      break;
    }
    if (!bitEqual(vx, fr.veh_vel.x)) {
      rc = fail("veh_vel_x_mps", i, vx, fr.veh_vel.x);
      break;
    }
    if (!bitEqual(tgx, fr.tgt_pos.x)) {
      rc = fail("tgt_pos_x_m", i, tgx, fr.tgt_pos.x);
      break;
    }
    if (!bitEqual(rng, fr.range)) {
      rc = fail("range_m", i, rng, fr.range);
      break;
    }
    if (!bitEqual(ax, fr.accel_cmd.x)) {
      rc = fail("accel_cmd_x_mps2", i, ax, fr.accel_cmd.x);
      break;
    }

    // Advance one core dt (except after the final frame).
    if (i + 1 < n) {
      const double tc = static_cast<double>(i) * cfg.dt;
      const fmi2Status st = api.doStep(comp, tc, cfg.dt, fmi2True);
      if (st != fmi2OK) {
        std::fprintf(stderr, "fmi_master: doStep returned status %d at frame %zu\n", st, i);
        rc = 1;
        break;
      }
    }
  }

  // After the full run, the FMU's final miss must equal the direct CPA miss (and the intercept
  // flag must agree). This is the headline summary-output check.
  if (rc == 0) {
    const double fmu_miss = getOne(api, comp, Vr::kVrMiss_m);
    const double fmu_intercept = getOne(api, comp, Vr::kVrIntercept);
    if (!bitEqual(fmu_miss, direct.miss_distance)) {
      rc = fail("miss_m (final)", n, fmu_miss, direct.miss_distance);
    } else if ((fmu_intercept != 0.0) != direct.intercept) {
      std::fprintf(stderr, "fmi_master: intercept flag mismatch fmu=%g direct=%d\n", fmu_intercept,
                   direct.intercept ? 1 : 0);
      rc = 1;
    } else {
      std::printf(
          "fmi_master: PASS — %zu frames bit-identical to runSimulation; final miss=%.6f m "
          "(intercept=%d)\n",
          n, fmu_miss, direct.intercept ? 1 : 0);
    }
  }

  api.terminate(comp);
  api.freeInstance(comp);
  dlclose(handle);
  return rc;
}
