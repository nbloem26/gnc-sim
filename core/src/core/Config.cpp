// gnc-sim — JSON parsing for SimConfig. Tolerant: any missing key keeps the struct default.
#include "gncsim/core/Config.hpp"

#include <cmath>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace gncsim {

namespace {
using nlohmann::json;

template <typename T>
T get_or(const json& j, const char* key, const T& fallback) {
  if (j.is_object() && j.contains(key) && !j.at(key).is_null()) {
    return j.at(key).get<T>();
  }
  return fallback;
}

Vector3 get_vec(const json& j, const char* key, const Vector3& fallback) {
  if (j.is_object() && j.contains(key) && j.at(key).is_array() && j.at(key).size() == 3) {
    const auto& a = j.at(key);
    return {a[0].get<double>(), a[1].get<double>(), a[2].get<double>()};
  }
  return fallback;
}

Integrator parseIntegrator(const std::string& s) {
  if (s == "euler") return Integrator::Euler;
  if (s == "rk2") return Integrator::RK2;
  return Integrator::RK4;
}
}  // namespace

Vector3 launchVelocity(const VehicleConfig& v) {
  const double el_rad = v.launch_elevation_deg * M_PI / 180.0;
  const double az_rad = v.launch_azimuth_deg * M_PI / 180.0;
  // ENU: x=East, y=North, z=Up. Azimuth measured from East toward North.
  const double horiz_mps = v.launch_speed * std::cos(el_rad);
  return {horiz_mps * std::cos(az_rad), horiz_mps * std::sin(az_rad),
          v.launch_speed * std::sin(el_rad)};
}

SimConfig loadConfigFromString(const std::string& json_text) {
  json j;
  try {
    j = json::parse(json_text);
  } catch (const std::exception& e) {
    throw std::runtime_error(std::string("config JSON parse error: ") + e.what());
  }

  SimConfig c;
  c.scenario = get_or<std::string>(j, "scenario", c.scenario);
  c.model = get_or<std::string>(j, "model", c.model);
  c.seed = get_or<std::uint64_t>(j, "seed", c.seed);
  c.dt = get_or<double>(j, "dt", c.dt);
  c.t_end = get_or<double>(j, "t_end", c.t_end);
  c.integrator = parseIntegrator(get_or<std::string>(j, "integrator", "rk4"));

  if (j.contains("origin")) {
    const auto& o = j["origin"];
    c.origin.lat0_deg = get_or<double>(o, "lat0_deg", c.origin.lat0_deg);
    c.origin.lon0_deg = get_or<double>(o, "lon0_deg", c.origin.lon0_deg);
    c.origin.alt0_m = get_or<double>(o, "alt0_m", c.origin.alt0_m);
  }

  if (j.contains("env")) {
    const auto& e = j["env"];
    c.env.g0 = get_or<double>(e, "g0", c.env.g0);
    c.env.altitude_dependent_g =
        get_or<bool>(e, "altitude_dependent_g", c.env.altitude_dependent_g);
    c.env.atmosphere = get_or<bool>(e, "atmosphere", c.env.atmosphere);
    // Frame selector: tolerant — anything other than "round" falls back to the flat default so the
    // flat-Earth regression baseline is never disturbed by a typo.
    const std::string frame = get_or<std::string>(e, "frame", c.env.frame);
    c.env.frame = (frame == "round") ? "round" : "flat";
    c.env.j2 = get_or<bool>(e, "j2", c.env.j2);

    // High-fidelity environment (issue #41). Tolerant: unknown strings fall back to the defaults so
    // the flat/round baselines are never disturbed by a typo.
    const std::string gm = get_or<std::string>(e, "gravity_model", c.env.gravity_model);
    c.env.gravity_model = (gm == "egm") ? "egm" : "central";
    const std::string am = get_or<std::string>(e, "atmosphere_model", c.env.atmosphere_model);
    c.env.atmosphere_model = (am == "extended") ? "extended" : "ussa76";
    c.env.rotating_ecef = get_or<bool>(e, "rotating_ecef", c.env.rotating_ecef);
    if (e.contains("gravity")) {
      const auto& g = e["gravity"];
      c.env.gravity.include_j2 = get_or<bool>(g, "j2", c.env.gravity.include_j2);
      c.env.gravity.include_j3 = get_or<bool>(g, "j3", c.env.gravity.include_j3);
      c.env.gravity.include_j4 = get_or<bool>(g, "j4", c.env.gravity.include_j4);
    }
    if (e.contains("wind")) {
      const auto& w = e["wind"];
      c.env.wind.enabled = get_or<bool>(w, "enabled", c.env.wind.enabled);
      c.env.wind.surface_mps = get_or<double>(w, "surface_mps", c.env.wind.surface_mps);
      c.env.wind.jet_mps = get_or<double>(w, "jet_mps", c.env.wind.jet_mps);
      c.env.wind.jet_alt_m = get_or<double>(w, "jet_alt_m", c.env.wind.jet_alt_m);
      c.env.wind.decay_scale_m = get_or<double>(w, "decay_scale_m", c.env.wind.decay_scale_m);
      c.env.wind.dir_deg = get_or<double>(w, "dir_deg", c.env.wind.dir_deg);
    }
  }

  if (j.contains("aero")) {
    const auto& a = j["aero"];
    c.aero.ref_area = get_or<double>(a, "ref_area", c.aero.ref_area);
    c.aero.cd0 = get_or<double>(a, "cd0", c.aero.cd0);
    c.aero.cn_alpha = get_or<double>(a, "cn_alpha", c.aero.cn_alpha);
    if (a.contains("cd_mach") && a["cd_mach"].is_array()) {
      for (const auto& row : a["cd_mach"]) {
        if (row.is_array() && row.size() == 2) {
          c.aero.cd_mach.push_back({row[0].get<double>(), row[1].get<double>()});
        }
      }
    }
    // High-fidelity 6DOF aero (issue #35): reference length, Cn/Cm tables, damping derivatives.
    c.aero.ref_length = get_or<double>(a, "ref_length", c.aero.ref_length);
    c.aero.cm_alpha = get_or<double>(a, "cm_alpha", c.aero.cm_alpha);
    c.aero.cm_q = get_or<double>(a, "cm_q", c.aero.cm_q);
    c.aero.cl_p = get_or<double>(a, "cl_p", c.aero.cl_p);
    const auto read_xyz_table = [&](const char* key, std::vector<std::array<double, 3>>& out) {
      if (a.contains(key) && a[key].is_array()) {
        for (const auto& row : a[key]) {
          if (row.is_array() && row.size() == 3) {
            out.push_back({row[0].get<double>(), row[1].get<double>(), row[2].get<double>()});
          }
        }
      }
    };
    read_xyz_table("cn_table", c.aero.cn_table);
    read_xyz_table("cm_table", c.aero.cm_table);
  }

  if (j.contains("vehicle")) {
    const auto& v = j["vehicle"];
    c.vehicle.pos0 = get_vec(v, "pos0", c.vehicle.pos0);
    c.vehicle.launch_speed = get_or<double>(v, "launch_speed", c.vehicle.launch_speed);
    c.vehicle.launch_elevation_deg =
        get_or<double>(v, "launch_elevation_deg", c.vehicle.launch_elevation_deg);
    c.vehicle.launch_azimuth_deg =
        get_or<double>(v, "launch_azimuth_deg", c.vehicle.launch_azimuth_deg);
    c.vehicle.mass0 = get_or<double>(v, "mass0", c.vehicle.mass0);
    c.vehicle.inertia = get_or<double>(v, "inertia", c.vehicle.inertia);
    // Full inertia tensor (issue #35); any unset principal moment stays <=0 -> use scalar
    // `inertia`.
    c.vehicle.ixx = get_or<double>(v, "ixx", c.vehicle.ixx);
    c.vehicle.iyy = get_or<double>(v, "iyy", c.vehicle.iyy);
    c.vehicle.izz = get_or<double>(v, "izz", c.vehicle.izz);
    c.vehicle.ixy = get_or<double>(v, "ixy", c.vehicle.ixy);
    c.vehicle.ixz = get_or<double>(v, "ixz", c.vehicle.ixz);
    c.vehicle.iyz = get_or<double>(v, "iyz", c.vehicle.iyz);
  }

  if (j.contains("propulsion")) {
    const auto& p = j["propulsion"];
    c.propulsion.thrust = get_or<double>(p, "thrust", c.propulsion.thrust);
    c.propulsion.burn_time = get_or<double>(p, "burn_time", c.propulsion.burn_time);
    c.propulsion.propellant_mass =
        get_or<double>(p, "propellant_mass", c.propulsion.propellant_mass);
    c.propulsion.boost_ref_area = get_or<double>(p, "boost_ref_area", c.propulsion.boost_ref_area);
    c.propulsion.stage_time = get_or<double>(p, "stage_time", c.propulsion.stage_time);
    c.propulsion.stage_mass_drop =
        get_or<double>(p, "stage_mass_drop", c.propulsion.stage_mass_drop);
  }

  if (j.contains("guidance")) {
    const auto& g = j["guidance"];
    c.guidance.law = get_or<std::string>(g, "law", c.guidance.law);
    c.guidance.nav_constant = get_or<double>(g, "nav_constant", c.guidance.nav_constant);
    c.guidance.max_accel = get_or<double>(g, "max_accel", c.guidance.max_accel);
    c.guidance.time_constant = get_or<double>(g, "time_constant", c.guidance.time_constant);
    c.guidance.apn_filter_tau = get_or<double>(g, "apn_filter_tau", c.guidance.apn_filter_tau);
    // Optimal ZEM/ZEV parameters (issue #40); only consulted when law == "zemzev".
    if (g.contains("zemzev")) {
      const auto& z = g["zemzev"];
      c.guidance.zemzev.n_zem = get_or<double>(z, "n_zem", c.guidance.zemzev.n_zem);
      c.guidance.zemzev.n_zev = get_or<double>(z, "n_zev", c.guidance.zemzev.n_zev);
      c.guidance.zemzev.desired_closing_mps =
          get_or<double>(z, "desired_closing_mps", c.guidance.zemzev.desired_closing_mps);
      c.guidance.zemzev.tgo_floor_s =
          get_or<double>(z, "tgo_floor_s", c.guidance.zemzev.tgo_floor_s);
      c.guidance.zemzev.handover_range_m =
          get_or<double>(z, "handover_range_m", c.guidance.zemzev.handover_range_m);
      c.guidance.zemzev.handover_blend_m =
          get_or<double>(z, "handover_blend_m", c.guidance.zemzev.handover_blend_m);
    }
    // Reaction-control divert actuation (issue #40); inert unless enabled.
    if (g.contains("divert")) {
      const auto& d = g["divert"];
      c.guidance.divert.enabled = get_or<bool>(d, "enabled", c.guidance.divert.enabled);
      c.guidance.divert.divert_limit_mps2 =
          get_or<double>(d, "divert_limit_mps2", c.guidance.divert.divert_limit_mps2);
    }
  }

  if (j.contains("control")) {
    const auto& ct = j["control"];
    c.control.kp = get_or<double>(ct, "kp", c.control.kp);
    c.control.kd = get_or<double>(ct, "kd", c.control.kd);
    c.control.max_fin_deflection =
        get_or<double>(ct, "max_fin_deflection", c.control.max_fin_deflection);
  }

  if (j.contains("actuator")) {
    const auto& ac = j["actuator"];
    c.actuator.tau = get_or<double>(ac, "tau", c.actuator.tau);
    c.actuator.rate_limit = get_or<double>(ac, "rate_limit", c.actuator.rate_limit);
    c.actuator.deflection_limit =
        get_or<double>(ac, "deflection_limit", c.actuator.deflection_limit);
    c.actuator.effectiveness = get_or<double>(ac, "effectiveness", c.actuator.effectiveness);
  }

  if (j.contains("sensors")) {
    const auto& s = j["sensors"];
    c.sensors.enable = get_or<bool>(s, "enable", c.sensors.enable);
    if (s.contains("imu")) {
      const auto& m = s["imu"];
      c.sensors.imu.accel_white = get_or<double>(m, "accel_white", c.sensors.imu.accel_white);
      c.sensors.imu.accel_bias_instability =
          get_or<double>(m, "accel_bias_instability", c.sensors.imu.accel_bias_instability);
      c.sensors.imu.accel_bias_tau =
          get_or<double>(m, "accel_bias_tau", c.sensors.imu.accel_bias_tau);
      c.sensors.imu.accel_rrw = get_or<double>(m, "accel_rrw", c.sensors.imu.accel_rrw);
      c.sensors.imu.accel_scale_factor =
          get_or<double>(m, "accel_scale_factor", c.sensors.imu.accel_scale_factor);
      c.sensors.imu.gyro_white = get_or<double>(m, "gyro_white", c.sensors.imu.gyro_white);
      c.sensors.imu.gyro_bias_instability =
          get_or<double>(m, "gyro_bias_instability", c.sensors.imu.gyro_bias_instability);
      c.sensors.imu.gyro_bias_tau = get_or<double>(m, "gyro_bias_tau", c.sensors.imu.gyro_bias_tau);
      c.sensors.imu.gyro_rrw = get_or<double>(m, "gyro_rrw", c.sensors.imu.gyro_rrw);
      c.sensors.imu.gyro_scale_factor =
          get_or<double>(m, "gyro_scale_factor", c.sensors.imu.gyro_scale_factor);
    }
    if (s.contains("seeker")) {
      const auto& sk = s["seeker"];
      c.sensors.seeker.los_white = get_or<double>(sk, "los_white", c.sensors.seeker.los_white);
      c.sensors.seeker.los_bias = get_or<double>(sk, "los_bias", c.sensors.seeker.los_bias);
      c.sensors.seeker.glint = get_or<double>(sk, "glint", c.sensors.seeker.glint);
    }
  }

  if (j.contains("nav")) {
    const auto& n = j["nav"];
    c.nav.filter = get_or<std::string>(n, "filter", c.nav.filter);
    c.nav.process_accel_psd = get_or<double>(n, "process_accel_psd", c.nav.process_accel_psd);
    c.nav.range_white = get_or<double>(n, "range_white", c.nav.range_white);
    c.nav.imm_q_cv = get_or<double>(n, "imm_q_cv", c.nav.imm_q_cv);
    c.nav.imm_q_man = get_or<double>(n, "imm_q_man", c.nav.imm_q_man);
    c.nav.imm_p_stay = get_or<double>(n, "imm_p_stay", c.nav.imm_p_stay);
  }

  if (j.contains("target")) {
    const auto& t = j["target"];
    c.target.pos0 = get_vec(t, "pos0", c.target.pos0);
    c.target.vel0 = get_vec(t, "vel0", c.target.vel0);
    c.target.maneuver = get_or<std::string>(t, "maneuver", c.target.maneuver);
    c.target.maneuver_g = get_or<double>(t, "maneuver_g", c.target.maneuver_g);
    c.target.maneuver_freq = get_or<double>(t, "maneuver_freq", c.target.maneuver_freq);
    c.target.maneuver_phase_deg =
        get_or<double>(t, "maneuver_phase_deg", c.target.maneuver_phase_deg);

    // --- Threat suite (issue #42): multi-stage ICBM / HGV / RV+penaids sub-blocks. ---
    if (t.contains("icbm")) {
      const auto& ic = t["icbm"];
      c.target.icbm.payload_mass_kg =
          get_or<double>(ic, "payload_mass_kg", c.target.icbm.payload_mass_kg);
      if (ic.contains("stages") && ic["stages"].is_array()) {
        c.target.icbm.stages.clear();
        for (const auto& s : ic["stages"]) {
          IcbmStage stage;
          stage.thrust_n = get_or<double>(s, "thrust_n", stage.thrust_n);
          stage.burn_time_s = get_or<double>(s, "burn_time_s", stage.burn_time_s);
          stage.propellant_mass_kg =
              get_or<double>(s, "propellant_mass_kg", stage.propellant_mass_kg);
          stage.dry_mass_kg = get_or<double>(s, "dry_mass_kg", stage.dry_mass_kg);
          c.target.icbm.stages.push_back(stage);
        }
      }
    }
    if (t.contains("hgv")) {
      const auto& h = t["hgv"];
      c.target.hgv.ld_ratio = get_or<double>(h, "ld_ratio", c.target.hgv.ld_ratio);
      c.target.hgv.ballistic_coeff =
          get_or<double>(h, "ballistic_coeff", c.target.hgv.ballistic_coeff);
      c.target.hgv.rho0_kgpm3 = get_or<double>(h, "rho0_kgpm3", c.target.hgv.rho0_kgpm3);
      c.target.hgv.scale_height_m =
          get_or<double>(h, "scale_height_m", c.target.hgv.scale_height_m);
      c.target.hgv.pull_up_alt_m = get_or<double>(h, "pull_up_alt_m", c.target.hgv.pull_up_alt_m);
    }
    if (t.contains("rv")) {
      const auto& rv = t["rv"];
      c.target.rv.penaid_count = get_or<int>(rv, "penaid_count", c.target.rv.penaid_count);
      c.target.rv.deploy_time_s = get_or<double>(rv, "deploy_time_s", c.target.rv.deploy_time_s);
      c.target.rv.deploy_dv_mps = get_or<double>(rv, "deploy_dv_mps", c.target.rv.deploy_dv_mps);
      c.target.rv.penaid_decel_mps2 =
          get_or<double>(rv, "penaid_decel_mps2", c.target.rv.penaid_decel_mps2);
    }
  }

  if (j.contains("trackers")) {
    const auto& tr = j["trackers"];
    c.trackers.enabled = get_or<bool>(tr, "enabled", c.trackers.enabled);
    c.trackers.process_psd = get_or<double>(tr, "process_psd", c.trackers.process_psd);
    if (tr.contains("sensors") && tr["sensors"].is_array()) {
      for (const auto& s : tr["sensors"]) {
        if (!s.is_object()) continue;
        TrackerSensorConfig sc;
        sc.type = get_or<std::string>(s, "type", sc.type);
        // Tolerant: keep the recognized types; anything else falls back to a plain radar.
        if (sc.type != "ir" && sc.type != "radar_pheno" && sc.type != "ir_pheno") {
          sc.type = "radar";
        }
        sc.pos = get_vec(s, "pos", sc.pos);
        sc.sigma_az = get_or<double>(s, "sigma_az", sc.sigma_az);
        sc.sigma_el = get_or<double>(s, "sigma_el", sc.sigma_el);
        sc.sigma_range = get_or<double>(s, "sigma_range", sc.sigma_range);
        sc.sigma_range_rate = get_or<double>(s, "sigma_range_rate", sc.sigma_range_rate);
        // --- Phenomenology sub-blocks (issue #39); only consulted by the *_pheno types ---
        if (s.contains("cfar") && s["cfar"].is_object()) {
          const auto& cf = s["cfar"];
          sc.cfar.pfa = get_or<double>(cf, "pfa", sc.cfar.pfa);
          sc.cfar.num_ref_cells = get_or<int>(cf, "num_ref_cells", sc.cfar.num_ref_cells);
        }
        if (s.contains("radar") && s["radar"].is_object()) {
          const auto& rd = s["radar"];
          sc.radar.rcs_mean_m2 = get_or<double>(rd, "rcs_mean_m2", sc.radar.rcs_mean_m2);
          sc.radar.swerling = get_or<int>(rd, "swerling", sc.radar.swerling);
          sc.radar.range_ref_m = get_or<double>(rd, "range_ref_m", sc.radar.range_ref_m);
          sc.radar.rcs_ref_m2 = get_or<double>(rd, "rcs_ref_m2", sc.radar.rcs_ref_m2);
          sc.radar.snr_ref_db = get_or<double>(rd, "snr_ref_db", sc.radar.snr_ref_db);
          sc.radar.clutter_cnr_db = get_or<double>(rd, "clutter_cnr_db", sc.radar.clutter_cnr_db);
          sc.radar.jammer_jnr_db = get_or<double>(rd, "jammer_jnr_db", sc.radar.jammer_jnr_db);
        }
        if (s.contains("ir") && s["ir"].is_object()) {
          const auto& ir = s["ir"];
          sc.ir_pheno.netd_k = get_or<double>(ir, "netd_k", sc.ir_pheno.netd_k);
          sc.ir_pheno.target_contrast_k =
              get_or<double>(ir, "target_contrast_k", sc.ir_pheno.target_contrast_k);
          sc.ir_pheno.range_ref_m = get_or<double>(ir, "range_ref_m", sc.ir_pheno.range_ref_m);
          sc.ir_pheno.atm_extinction_per_m =
              get_or<double>(ir, "atm_extinction_per_m", sc.ir_pheno.atm_extinction_per_m);
          sc.ir_pheno.theta_resolution_rad =
              get_or<double>(ir, "theta_resolution_rad", sc.ir_pheno.theta_resolution_rad);
          sc.ir_pheno.centroid_gain =
              get_or<double>(ir, "centroid_gain", sc.ir_pheno.centroid_gain);
        }
        c.trackers.sensors.push_back(sc);
      }
    }
    // --- Multi-target data association (issue #38); only consulted when mode == "jpda" ---
    if (tr.contains("association") && tr["association"].is_object()) {
      const auto& as = tr["association"];
      auto& a = c.trackers.association;
      a.mode = get_or<std::string>(as, "mode", a.mode);
      if (a.mode != "jpda") a.mode = "none";  // tolerant: unknown mode -> legacy single-target path
      a.prob_detect = get_or<double>(as, "prob_detect", a.prob_detect);
      a.gate_chi2 = get_or<double>(as, "gate_chi2", a.gate_chi2);
      a.clutter_density = get_or<double>(as, "clutter_density", a.clutter_density);
      a.clutter_rate = get_or<double>(as, "clutter_rate", a.clutter_rate);
      a.clutter_az_spread_rad =
          get_or<double>(as, "clutter_az_spread_rad", a.clutter_az_spread_rad);
      a.clutter_range_spread_m =
          get_or<double>(as, "clutter_range_spread_m", a.clutter_range_spread_m);
      a.confirm_m = get_or<int>(as, "confirm_m", a.confirm_m);
      a.confirm_n = get_or<int>(as, "confirm_n", a.confirm_n);
      a.delete_misses = get_or<int>(as, "delete_misses", a.delete_misses);
      a.num_cso = get_or<int>(as, "num_cso", a.num_cso);
      a.cso_separation_m = get_or<double>(as, "cso_separation_m", a.cso_separation_m);
      a.init_pos_sigma_m = get_or<double>(as, "init_pos_sigma_m", a.init_pos_sigma_m);
      a.init_vel_sigma_mps = get_or<double>(as, "init_vel_sigma_mps", a.init_vel_sigma_mps);
    }

    // --- Fire-control C2 datalink latency / dropout (issue #46); only consulted when enabled ---
    if (tr.contains("datalink") && tr["datalink"].is_object()) {
      const auto& dl = tr["datalink"];
      auto& d = c.trackers.datalink;
      d.enabled = get_or<bool>(dl, "enabled", d.enabled);
      d.latency_s = get_or<double>(dl, "latency_s", d.latency_s);
      d.dropout_prob = get_or<double>(dl, "dropout_prob", d.dropout_prob);
    }
  }

  if (j.contains("decoys")) {
    const auto& d = j["decoys"];
    c.decoys.enabled = get_or<bool>(d, "enabled", c.decoys.enabled);
    c.decoys.count = get_or<int>(d, "count", c.decoys.count);
    c.decoys.separation = get_or<double>(d, "separation", c.decoys.separation);
    c.decoys.separability = get_or<double>(d, "separability", c.decoys.separability);
    c.decoys.target_intensity = get_or<double>(d, "target_intensity", c.decoys.target_intensity);
    c.decoys.target_size = get_or<double>(d, "target_size", c.decoys.target_size);
    c.decoys.target_decel = get_or<double>(d, "target_decel", c.decoys.target_decel);
    c.decoys.decoy_intensity = get_or<double>(d, "decoy_intensity", c.decoys.decoy_intensity);
    c.decoys.decoy_size = get_or<double>(d, "decoy_size", c.decoys.decoy_size);
    c.decoys.decoy_decel = get_or<double>(d, "decoy_decel", c.decoys.decoy_decel);
    c.decoys.feature_spread = get_or<double>(d, "feature_spread", c.decoys.feature_spread);
    c.decoys.measurement_noise = get_or<double>(d, "measurement_noise", c.decoys.measurement_noise);
    c.decoys.score_filter_tau = get_or<double>(d, "score_filter_tau", c.decoys.score_filter_tau);
  }

  if (j.contains("cueing")) {
    const auto& cu = j["cueing"];
    c.cueing.enabled = get_or<bool>(cu, "enabled", c.cueing.enabled);
    c.cueing.launch_criterion =
        get_or<std::string>(cu, "launch_criterion", c.cueing.launch_criterion);
    // Tolerant: anything but the fixed-delay keyword falls back to the track-covariance criterion.
    if (c.cueing.launch_criterion != "fixed_delay") c.cueing.launch_criterion = "track_cov";
    c.cueing.cov_trace_threshold =
        get_or<double>(cu, "cov_trace_threshold", c.cueing.cov_trace_threshold);
    c.cueing.max_cue_time = get_or<double>(cu, "max_cue_time", c.cueing.max_cue_time);
    c.cueing.loft_deg = get_or<double>(cu, "loft_deg", c.cueing.loft_deg);
  }

  if (j.contains("monte_carlo")) {
    const auto& m = j["monte_carlo"];
    c.monte_carlo.num_cases = get_or<int>(m, "num_cases", c.monte_carlo.num_cases);
    c.monte_carlo.launch_speed_sigma =
        get_or<double>(m, "launch_speed_sigma", c.monte_carlo.launch_speed_sigma);
    c.monte_carlo.launch_elevation_sigma_deg =
        get_or<double>(m, "launch_elevation_sigma_deg", c.monte_carlo.launch_elevation_sigma_deg);
    c.monte_carlo.target_pos_sigma =
        get_or<double>(m, "target_pos_sigma", c.monte_carlo.target_pos_sigma);
  }

  // --- Many-on-many engagement campaign (issue #45). Opt-in; absent block => disabled. ---
  if (j.contains("many_on_many")) {
    const auto& mm = j["many_on_many"];
    auto& cfg = c.many_on_many;
    cfg.enabled = get_or<bool>(mm, "enabled", cfg.enabled);
    cfg.doctrine = get_or<std::string>(mm, "doctrine", cfg.doctrine);
    // Tolerant: unknown doctrine strings fall back to the salvo default.
    if (cfg.doctrine != "shoot_look_shoot" && cfg.doctrine != "raid") cfg.doctrine = "salvo";
    cfg.wta_method = get_or<std::string>(mm, "wta_method", cfg.wta_method);
    if (cfg.wta_method != "auction") cfg.wta_method = "greedy";
    cfg.shots_per_threat = get_or<int>(mm, "shots_per_threat", cfg.shots_per_threat);
    cfg.max_waves = get_or<int>(mm, "max_waves", cfg.max_waves);
    cfg.pk_sigma_m = get_or<double>(mm, "pk_sigma_m", cfg.pk_sigma_m);
    cfg.pk_max = get_or<double>(mm, "pk_max", cfg.pk_max);
    cfg.num_trials = get_or<int>(mm, "num_trials", cfg.num_trials);

    if (mm.contains("interceptors") && mm["interceptors"].is_array()) {
      for (const auto& iv : mm["interceptors"]) {
        ManyInterceptorSpec s;
        s.pos0_m = get_vec(iv, "pos0_m", s.pos0_m);
        s.launch_speed_mps = get_or<double>(iv, "launch_speed_mps", s.launch_speed_mps);
        s.launch_elevation_deg = get_or<double>(iv, "launch_elevation_deg", s.launch_elevation_deg);
        s.launch_azimuth_deg = get_or<double>(iv, "launch_azimuth_deg", s.launch_azimuth_deg);
        cfg.interceptors.push_back(s);
      }
    }
    if (mm.contains("threats") && mm["threats"].is_array()) {
      for (const auto& tv : mm["threats"]) {
        ManyThreatSpec s;
        s.pos0_m = get_vec(tv, "pos0_m", s.pos0_m);
        s.vel0_mps = get_vec(tv, "vel0_mps", s.vel0_mps);
        s.maneuver = get_or<std::string>(tv, "maneuver", s.maneuver);
        s.maneuver_g = get_or<double>(tv, "maneuver_g", s.maneuver_g);
        s.maneuver_freq_hz = get_or<double>(tv, "maneuver_freq_hz", s.maneuver_freq_hz);
        s.release_time_s = get_or<double>(tv, "release_time_s", s.release_time_s);
        s.value = get_or<double>(tv, "value", s.value);
        cfg.threats.push_back(s);
      }
    }
  }

  return c;
}

}  // namespace gncsim
