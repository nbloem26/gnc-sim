// gnc-sim — SimResult serialization (see Serialize.hpp). Columnar JSON + CSV strings + manifest.
#include "gncsim/core/Serialize.hpp"

#include <nlohmann/json.hpp>
#include <sstream>
#include <vector>

namespace gncsim {

namespace {
using nlohmann::json;

// Build a columnar dictionary {column -> [values...]} from the frames.
json seriesObject(const SimResult& r) {
  const std::size_t n = r.frames.size();
  auto col = [n]() {
    std::vector<double> v;
    v.reserve(n);
    return v;
  };

  std::vector<double> t = col();
  std::vector<double> vx = col(), vy = col(), vz = col();
  std::vector<double> vvx = col(), vvy = col(), vvz = col();
  std::vector<double> roll = col(), pitch = col(), yaw = col();
  std::vector<double> mass = col(), mach = col(), thrust = col();
  std::vector<double> tx = col(), ty = col(), tz = col();
  std::vector<double> tvx = col(), tvy = col(), tvz = col();
  std::vector<double> acx = col(), acy = col(), acz = col();
  std::vector<double> los = col(), losr = col(), vc = col(), rng = col();
  std::vector<double> nx = col(), ny = col(), nz = col();
  std::vector<double> nis = col();
  std::vector<double> trk_x = col(), trk_y = col(), trk_z = col();
  std::vector<double> trk_nis = col();
  std::vector<double> sel_obj = col(), dsc_ok = col(), dsc_margin = col();
  std::vector<double> imu_at_x = col(), imu_am_x = col();
  std::vector<double> imu_gt_x = col(), imu_gm_x = col();
  std::vector<double> sk_t = col(), sk_m = col();

  for (const auto& f : r.frames) {
    t.push_back(f.t);
    vx.push_back(f.veh_pos.x);
    vy.push_back(f.veh_pos.y);
    vz.push_back(f.veh_pos.z);
    vvx.push_back(f.veh_vel.x);
    vvy.push_back(f.veh_vel.y);
    vvz.push_back(f.veh_vel.z);
    const Vector3 e = f.veh_att.toEuler();
    roll.push_back(e.x);
    pitch.push_back(e.y);
    yaw.push_back(e.z);
    mass.push_back(f.mass);
    mach.push_back(f.mach);
    thrust.push_back(f.thrust);
    tx.push_back(f.tgt_pos.x);
    ty.push_back(f.tgt_pos.y);
    tz.push_back(f.tgt_pos.z);
    tvx.push_back(f.tgt_vel.x);
    tvy.push_back(f.tgt_vel.y);
    tvz.push_back(f.tgt_vel.z);
    acx.push_back(f.accel_cmd.x);
    acy.push_back(f.accel_cmd.y);
    acz.push_back(f.accel_cmd.z);
    los.push_back(f.los_angle);
    losr.push_back(f.los_rate);
    vc.push_back(f.v_closing);
    rng.push_back(f.range);
    nx.push_back(f.nav_pos_est.x);
    ny.push_back(f.nav_pos_est.y);
    nz.push_back(f.nav_pos_est.z);
    nis.push_back(f.nav_nis);
    trk_x.push_back(f.track_pos_est.x);
    trk_y.push_back(f.track_pos_est.y);
    trk_z.push_back(f.track_pos_est.z);
    trk_nis.push_back(f.track_nis);
    sel_obj.push_back(f.selected_obj);
    dsc_ok.push_back(f.discrim_correct);
    dsc_margin.push_back(f.discrim_margin);
    imu_at_x.push_back(f.imu_accel_true.x);
    imu_am_x.push_back(f.imu_accel_meas.x);
    imu_gt_x.push_back(f.imu_gyro_true.x);
    imu_gm_x.push_back(f.imu_gyro_meas.x);
    sk_t.push_back(f.seeker_los_true);
    sk_m.push_back(f.seeker_los_meas);
  }

  json s;
  s["t"] = t;
  s["veh_x"] = vx;
  s["veh_y"] = vy;
  s["veh_z"] = vz;
  s["veh_vx"] = vvx;
  s["veh_vy"] = vvy;
  s["veh_vz"] = vvz;
  s["roll"] = roll;
  s["pitch"] = pitch;
  s["yaw"] = yaw;
  s["mass"] = mass;
  s["mach"] = mach;
  s["thrust"] = thrust;
  s["tgt_x"] = tx;
  s["tgt_y"] = ty;
  s["tgt_z"] = tz;
  s["tgt_vx"] = tvx;
  s["tgt_vy"] = tvy;
  s["tgt_vz"] = tvz;
  s["accel_cmd_x"] = acx;
  s["accel_cmd_y"] = acy;
  s["accel_cmd_z"] = acz;
  s["los_angle"] = los;
  s["los_rate"] = losr;
  s["v_closing"] = vc;
  s["range"] = rng;
  s["nav_x"] = nx;
  s["nav_y"] = ny;
  s["nav_z"] = nz;
  s["nav_nis"] = nis;
  s["track_x"] = trk_x;
  s["track_y"] = trk_y;
  s["track_z"] = trk_z;
  s["track_nis"] = trk_nis;
  s["selected_obj"] = sel_obj;
  s["discrim_correct"] = dsc_ok;
  s["discrim_margin"] = dsc_margin;
  s["imu_accel_true_x"] = imu_at_x;
  s["imu_accel_meas_x"] = imu_am_x;
  s["imu_gyro_true_x"] = imu_gt_x;
  s["imu_gyro_meas_x"] = imu_gm_x;
  s["seeker_los_true"] = sk_t;
  s["seeker_los_meas"] = sk_m;
  return s;
}

json metaObject(const SimResult& r) {
  json m;
  m["scenario"] = r.scenario;
  m["model"] = r.model;
  m["seed"] = r.seed;
  m["dt"] = r.dt;
  m["t_end"] = r.t_end;
  m["intercept"] = r.intercept;
  m["miss_distance"] = r.miss_distance;
  m["intercept_time"] = r.intercept_time;
  // Interceptor launch / cue time (issue #8). 0 on the default launch-at-t=0 path; > 0 on the cued
  // launch-on-track path.
  m["launch_time"] = r.launch_time;
  m["git_sha"] = r.git_sha;
  m["origin"] = {{"lat0_deg", r.origin.lat0_deg},
                 {"lon0_deg", r.origin.lon0_deg},
                 {"alt0_m", r.origin.alt0_m}};
  return m;
}
}  // namespace

std::string toJsonString(const SimResult& r, int indent) {
  json out = metaObject(r);
  out["series"] = seriesObject(r);
  return out.dump(indent);
}

std::string toManifestJson(const SimResult& r, const std::string& config_echo) {
  json out = metaObject(r);
  out["num_frames"] = r.frames.size();
  if (!config_echo.empty()) {
    try {
      out["config"] = json::parse(config_echo);
    } catch (...) {
      out["config_raw"] = config_echo;
    }
  }
  return out.dump(2);
}

std::map<std::string, std::string> toCsvFiles(const SimResult& r) {
  auto fmt = [](std::ostringstream& os) { os.precision(9); };

  std::ostringstream veh, tgt, gnc, sens, trk, disc;
  fmt(veh);
  fmt(tgt);
  fmt(gnc);
  fmt(sens);
  fmt(trk);
  fmt(disc);

  veh << "t,x,y,z,vx,vy,vz,roll,pitch,yaw,mass,mach,thrust\n";
  tgt << "t,x,y,z,vx,vy,vz\n";
  gnc << "t,accel_cmd_x,accel_cmd_y,accel_cmd_z,fin_x,fin_y,fin_z,los_angle,los_rate,v_closing,"
         "range,nav_x,nav_y,nav_z,nav_nis\n";
  sens << "t,imu_accel_true_x,imu_accel_meas_x,imu_gyro_true_x,imu_gyro_meas_x,seeker_los_true,"
          "seeker_los_meas\n";
  // Multi-sensor target track (issue #5): fused absolute target-position estimate vs the true
  // target position, plus the last sensor-update NIS. All zero on non-tracker paths.
  trk << "t,track_x,track_y,track_z,tgt_x,tgt_y,tgt_z,track_nis\n";
  // Seeker target discrimination against decoys (issue #6): the homed-on object index, whether it
  // was the true target, and the integrated-score margin to the runner-up. All zero/default (object
  // 0, correct=1, margin=0) on every non-decoy path, so the file is always present but inert.
  disc << "t,selected_obj,discrim_correct,discrim_margin\n";

  for (const auto& f : r.frames) {
    const Vector3 e = f.veh_att.toEuler();
    veh << f.t << ',' << f.veh_pos.x << ',' << f.veh_pos.y << ',' << f.veh_pos.z << ','
        << f.veh_vel.x << ',' << f.veh_vel.y << ',' << f.veh_vel.z << ',' << e.x << ',' << e.y
        << ',' << e.z << ',' << f.mass << ',' << f.mach << ',' << f.thrust << '\n';
    tgt << f.t << ',' << f.tgt_pos.x << ',' << f.tgt_pos.y << ',' << f.tgt_pos.z << ','
        << f.tgt_vel.x << ',' << f.tgt_vel.y << ',' << f.tgt_vel.z << '\n';
    gnc << f.t << ',' << f.accel_cmd.x << ',' << f.accel_cmd.y << ',' << f.accel_cmd.z << ','
        << f.fin_deflection.x << ',' << f.fin_deflection.y << ',' << f.fin_deflection.z << ','
        << f.los_angle << ',' << f.los_rate << ',' << f.v_closing << ',' << f.range << ','
        << f.nav_pos_est.x << ',' << f.nav_pos_est.y << ',' << f.nav_pos_est.z << ',' << f.nav_nis
        << '\n';
    sens << f.t << ',' << f.imu_accel_true.x << ',' << f.imu_accel_meas.x << ','
         << f.imu_gyro_true.x << ',' << f.imu_gyro_meas.x << ',' << f.seeker_los_true << ','
         << f.seeker_los_meas << '\n';
    trk << f.t << ',' << f.track_pos_est.x << ',' << f.track_pos_est.y << ',' << f.track_pos_est.z
        << ',' << f.tgt_pos.x << ',' << f.tgt_pos.y << ',' << f.tgt_pos.z << ',' << f.track_nis
        << '\n';
    disc << f.t << ',' << f.selected_obj << ',' << f.discrim_correct << ',' << f.discrim_margin
         << '\n';
  }

  return {{"vehicle.csv", veh.str()},  {"target.csv", tgt.str()}, {"gnc.csv", gnc.str()},
          {"sensors.csv", sens.str()}, {"track.csv", trk.str()},  {"discrim.csv", disc.str()}};
}

}  // namespace gncsim
