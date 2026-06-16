/// @file GenerateModelDescription.cpp
/// @brief Emits the FMU's modelDescription.xml from the shared variable layout.
///
/// A tiny build-time tool (NOT shipped in the .fmu) that writes a valid FMI 2.0
/// modelDescription.xml whose <ScalarVariable> value references exactly match
/// GncsimFmuVars.hpp — generating the XML from the same header makes drift between the
/// binary and the descriptor impossible. The packaging script (fmi/scripts/package_fmu.sh)
/// runs this, then zips the XML + shared library into gncsim.fmu.
///
/// Usage: generate_model_description <out.xml>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

#include "GncsimFmuVars.hpp"

namespace {

using gncsim::fmi::Vr;

// One scalar variable row in the descriptor. This is a build-only static table (the tool is not
// shipped in the .fmu); the field order below favors readability (vr first) over byte packing —
// the trailing NOLINT silences the padding optimizer (not a hot path).
struct VarSpec {  // NOLINT(clang-analyzer-optin.performance.Padding)
  Vr vr;
  const char* name;
  const char* causality;    // "parameter" | "input" | "output" | "local"
  const char* variability;  // "fixed" | "tunable" | "discrete" | "continuous"
  const char* initial;      // "exact" | "calculated" | "" (omit)
  double start;             // start value (used for parameters/inputs)
  bool has_start;
  const char* description;
};

std::string xmlEscape(const std::string& in) {
  std::string out;
  out.reserve(in.size());
  for (char c : in) {
    switch (c) {
      case '&':
        out += "&amp;";
        break;
      case '<':
        out += "&lt;";
        break;
      case '>':
        out += "&gt;";
        break;
      case '"':
        out += "&quot;";
        break;
      default:
        out += c;
        break;
    }
  }
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: generate_model_description <out.xml>\n";
    return 2;
  }

  // The full variable table. Order == display order; valueReference == Vr enumerator.
  const VarSpec vars[] = {
      // Parameters (fixed; set before exiting initialization).
      {Vr::kVrSeed, "seed", "parameter", "fixed", "exact", 1.0, true, "RNG seed"},
      {Vr::kVrDtStep_s, "dt_step_s", "parameter", "fixed", "exact", 0.005, true,
       "core fixed integration step [s]"},
      {Vr::kVrTEnd_s, "t_end_s", "parameter", "fixed", "exact", 60.0, true, "max sim time [s]"},
      {Vr::kVrLaunchSpeed_mps, "launch_speed_mps", "parameter", "fixed", "exact", 600.0, true,
       "interceptor launch speed [m/s]"},
      {Vr::kVrLaunchElevation_deg, "launch_elevation_deg", "parameter", "fixed", "exact", 45.0,
       true, "interceptor launch elevation [deg]"},
      {Vr::kVrLaunchAzimuth_deg, "launch_azimuth_deg", "parameter", "fixed", "exact", 0.0, true,
       "interceptor launch azimuth [deg]"},
      {Vr::kVrTargetPosX_m, "target_pos_x_m", "parameter", "fixed", "exact", 8000.0, true,
       "initial target ENU position X [m]"},
      {Vr::kVrTargetPosY_m, "target_pos_y_m", "parameter", "fixed", "exact", 0.0, true,
       "initial target ENU position Y [m]"},
      {Vr::kVrTargetPosZ_m, "target_pos_z_m", "parameter", "fixed", "exact", 3000.0, true,
       "initial target ENU position Z [m]"},
      {Vr::kVrTargetVelX_mps, "target_vel_x_mps", "parameter", "fixed", "exact", -250.0, true,
       "initial target ENU velocity X [m/s]"},
      {Vr::kVrTargetVelY_mps, "target_vel_y_mps", "parameter", "fixed", "exact", 0.0, true,
       "initial target ENU velocity Y [m/s]"},
      {Vr::kVrTargetVelZ_mps, "target_vel_z_mps", "parameter", "fixed", "exact", 0.0, true,
       "initial target ENU velocity Z [m/s]"},
      // Inputs (continuous co-sim inputs; reserved for external-controller import).
      {Vr::kVrAccelCmdOverrideX_mps2, "accel_cmd_override_x_mps2", "input", "continuous", "", 0.0,
       true, "external commanded-accel override X [m/s^2] (reserved)"},
      {Vr::kVrAccelCmdOverrideY_mps2, "accel_cmd_override_y_mps2", "input", "continuous", "", 0.0,
       true, "external commanded-accel override Y [m/s^2] (reserved)"},
      {Vr::kVrAccelCmdOverrideZ_mps2, "accel_cmd_override_z_mps2", "input", "continuous", "", 0.0,
       true, "external commanded-accel override Z [m/s^2] (reserved)"},
      {Vr::kVrAccelCmdOverrideEnable, "accel_cmd_override_enable", "input", "discrete", "", 0.0,
       true, "1 => use external override instead of internal guidance (reserved)"},
      // Outputs (streamed per communication step). initial="calculated".
      {Vr::kVrTime_s, "time_s", "output", "continuous", "calculated", 0.0, false,
       "current engagement time [s]"},
      {Vr::kVrVehPosX_m, "veh_pos_x_m", "output", "continuous", "calculated", 0.0, false,
       "interceptor ENU position X [m]"},
      {Vr::kVrVehPosY_m, "veh_pos_y_m", "output", "continuous", "calculated", 0.0, false,
       "interceptor ENU position Y [m]"},
      {Vr::kVrVehPosZ_m, "veh_pos_z_m", "output", "continuous", "calculated", 0.0, false,
       "interceptor ENU position Z [m]"},
      {Vr::kVrVehVelX_mps, "veh_vel_x_mps", "output", "continuous", "calculated", 0.0, false,
       "interceptor ENU velocity X [m/s]"},
      {Vr::kVrVehVelY_mps, "veh_vel_y_mps", "output", "continuous", "calculated", 0.0, false,
       "interceptor ENU velocity Y [m/s]"},
      {Vr::kVrVehVelZ_mps, "veh_vel_z_mps", "output", "continuous", "calculated", 0.0, false,
       "interceptor ENU velocity Z [m/s]"},
      {Vr::kVrTgtPosX_m, "tgt_pos_x_m", "output", "continuous", "calculated", 0.0, false,
       "target ENU position X [m]"},
      {Vr::kVrTgtPosY_m, "tgt_pos_y_m", "output", "continuous", "calculated", 0.0, false,
       "target ENU position Y [m]"},
      {Vr::kVrTgtPosZ_m, "tgt_pos_z_m", "output", "continuous", "calculated", 0.0, false,
       "target ENU position Z [m]"},
      {Vr::kVrAccelCmdX_mps2, "accel_cmd_x_mps2", "output", "continuous", "calculated", 0.0, false,
       "realized guidance command X [m/s^2]"},
      {Vr::kVrAccelCmdY_mps2, "accel_cmd_y_mps2", "output", "continuous", "calculated", 0.0, false,
       "realized guidance command Y [m/s^2]"},
      {Vr::kVrAccelCmdZ_mps2, "accel_cmd_z_mps2", "output", "continuous", "calculated", 0.0, false,
       "realized guidance command Z [m/s^2]"},
      {Vr::kVrRange_m, "range_m", "output", "continuous", "calculated", 0.0, false,
       "current vehicle-to-target range [m]"},
      {Vr::kVrClosingSpeed_mps, "closing_speed_mps", "output", "continuous", "calculated", 0.0,
       false, "closing speed [m/s]"},
      {Vr::kVrMiss_m, "miss_m", "output", "continuous", "calculated", 0.0, false,
       "running closest-approach (CPA) miss distance [m]"},
      {Vr::kVrIntercept, "intercept", "output", "discrete", "calculated", 0.0, false,
       "1.0 once intercept (CPA < lethal radius)"},
      {Vr::kVrDone, "done", "output", "discrete", "calculated", 0.0, false,
       "1.0 once the engagement has been fully stepped"},
  };
  constexpr int kNumVars = sizeof(vars) / sizeof(vars[0]);
  static_assert(true, "");

  std::ofstream out(argv[1]);
  if (!out) {
    std::cerr << "cannot write: " << argv[1] << "\n";
    return 1;
  }

  out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  out << "<fmiModelDescription\n"
      << "  fmiVersion=\"2.0\"\n"
      << "  modelName=\"gncsim\"\n"
      << "  guid=\"{b0f6e1d2-7c3a-4e5f-9a8b-1c2d3e4f5061}\"\n"
      << "  description=\"gnc-sim guided-interceptor engagement (FMI 2.0 Co-Simulation export)\"\n"
      << "  generationTool=\"gnc-sim fmi/ (issue #44)\"\n"
      << "  variableNamingConvention=\"flat\"\n"
      << "  numberOfEventIndicators=\"0\">\n";

  // Co-simulation capabilities. canHandleVariableCommunicationStepSize=true: a master may use any
  // integer multiple of the core dt as the communication step.
  out << "  <CoSimulation\n"
      << "    modelIdentifier=\"gncsim\"\n"
      << "    canHandleVariableCommunicationStepSize=\"true\"\n"
      << "    canInterpolateInputs=\"false\"\n"
      << "    maxOutputDerivativeOrder=\"0\"\n"
      << "    canRunAsynchronuously=\"false\"\n"
      << "    canBeInstantiatedOnlyOncePerProcess=\"false\"\n"
      << "    canGetAndSetFMUstate=\"false\"\n"
      << "    canSerializeFMUstate=\"false\"\n"
      << "    providesDirectionalDerivative=\"false\"/>\n";

  out << "  <DefaultExperiment startTime=\"0.0\" stopTime=\"60.0\" stepSize=\"0.005\"/>\n";

  out << "  <ModelVariables>\n";
  for (const VarSpec& v : vars) {
    out << "    <ScalarVariable name=\"" << xmlEscape(v.name) << "\" valueReference=\"" << v.vr
        << "\" causality=\"" << v.causality << "\" variability=\"" << v.variability << "\"";
    if (v.initial[0] != '\0') out << " initial=\"" << v.initial << "\"";
    out << " description=\"" << xmlEscape(v.description) << "\">\n";
    out << "      <Real";
    if (v.has_start) out << " start=\"" << v.start << "\"";
    out << "/>\n";
    out << "    </ScalarVariable>\n";
  }
  out << "  </ModelVariables>\n";

  // ModelStructure: list the output indices (1-based, into ModelVariables order). FMI requires
  // <Outputs> to enumerate every causality="output" variable.
  out << "  <ModelStructure>\n";
  out << "    <Outputs>\n";
  for (int i = 0; i < kNumVars; ++i) {
    if (std::string(vars[i].causality) == "output") {
      out << "      <Unknown index=\"" << (i + 1) << "\"/>\n";
    }
  }
  out << "    </Outputs>\n";
  out << "  </ModelStructure>\n";

  out << "</fmiModelDescription>\n";
  out.close();

  std::cout << "wrote " << argv[1] << " (" << kNumVars << " scalar variables)\n";
  return 0;
}
