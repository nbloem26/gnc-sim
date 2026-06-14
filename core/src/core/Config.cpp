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
  const double el = v.launch_elevation_deg * M_PI / 180.0;
  const double az = v.launch_azimuth_deg * M_PI / 180.0;
  // ENU: x=East, y=North, z=Up. Azimuth measured from East toward North.
  const double horiz = v.launch_speed * std::cos(el);
  return {horiz * std::cos(az), horiz * std::sin(az), v.launch_speed * std::sin(el)};
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
  }

  if (j.contains("control")) {
    const auto& ct = j["control"];
    c.control.kp = get_or<double>(ct, "kp", c.control.kp);
    c.control.kd = get_or<double>(ct, "kd", c.control.kd);
    c.control.max_fin_deflection =
        get_or<double>(ct, "max_fin_deflection", c.control.max_fin_deflection);
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

  return c;
}

}  // namespace gncsim
