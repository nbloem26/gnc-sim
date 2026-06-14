// gnc-sim — multi-sensor target-track EKF implementation (see TargetTrackEkf.hpp). ENU, SI units.
//
// All matrices are fixed-capacity (max measurement dim = 4, state dim = 6) and row-major in
// std::array; the active dimension `m` (2 for IR angles-only, 4 for radar) is passed at runtime.
// The predict step exploits the sparse nearly-constant-velocity structure [[I, dt*I],[0, I]]
// exactly as the relative-state Ekf does. The sequential update uses dense m x 6 / m x m helpers
// and a Gauss-Jordan inverse (m up to 4) with the Joseph form for the covariance to preserve
// symmetry/PSD.
#include "gncsim/gnc/TargetTrackEkf.hpp"

#include <cmath>

namespace gncsim {

namespace {

constexpr double kPi = 3.14159265358979323846;

// Wrap an angle to [-pi, pi] so an az/el innovation never picks up a spurious ~2*pi jump at the
// wrap-around boundary.
double wrapPi(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

// In-place Gauss-Jordan inverse of an m x m matrix (m <= 4), row-major in a fixed 16-element array.
// Partial pivoting for numerical robustness. Returns false if (near-)singular.
bool invertMxM(std::array<double, 16>& a, int m) {
  std::array<double, 16> inv{};
  for (int i = 0; i < m; ++i) inv[i * m + i] = 1.0;

  for (int col = 0; col < m; ++col) {
    // Partial pivot: pick the largest-magnitude entry in this column at/below the diagonal.
    int pivot = col;
    double best = std::fabs(a[col * m + col]);
    for (int r = col + 1; r < m; ++r) {
      const double v = std::fabs(a[r * m + col]);
      if (v > best) {
        best = v;
        pivot = r;
      }
    }
    if (best < 1e-300) return false;
    if (pivot != col) {
      for (int k = 0; k < m; ++k) {
        std::swap(a[col * m + k], a[pivot * m + k]);
        std::swap(inv[col * m + k], inv[pivot * m + k]);
      }
    }
    const double diag = a[col * m + col];
    const double inv_diag = 1.0 / diag;
    for (int k = 0; k < m; ++k) {
      a[col * m + k] *= inv_diag;
      inv[col * m + k] *= inv_diag;
    }
    for (int r = 0; r < m; ++r) {
      if (r == col) continue;
      const double factor = a[r * m + col];
      if (factor == 0.0) continue;
      for (int k = 0; k < m; ++k) {
        a[r * m + k] -= factor * a[col * m + k];
        inv[r * m + k] -= factor * inv[col * m + k];
      }
    }
  }
  a = inv;
  return true;
}

}  // namespace

TargetTrackEkf::TargetTrackEkf(double dt, double process_psd) : dt_(dt), q_(process_psd) {}

void TargetTrackEkf::bootstrap(const Vector3& pos, const Vector3& vel) {
  x_[0] = pos.x;
  x_[1] = pos.y;
  x_[2] = pos.z;
  x_[3] = vel.x;
  x_[4] = vel.y;
  x_[5] = vel.z;

  for (int k = 0; k < 36; ++k) p_[k] = 0.0;
  auto P0 = [&](int idx) -> double& { return p_[idx * 6 + idx]; };
  const double pos_var = 1.0e6;  // ~1 km std
  const double vel_var = 1.0e6;  // ~1 km/s std
  P0(0) = P0(1) = P0(2) = pos_var;
  P0(3) = P0(4) = P0(5) = vel_var;

  initialized_ = true;
  nis_ = 0.0;
}

double TargetTrackEkf::covTrace() const {
  double s = 0.0;
  for (int i = 0; i < 6; ++i) s += p_[i * 6 + i];
  return s;
}

// ── Time update (nearly-constant-velocity) ──────────────────────────────────────────────────────
void TargetTrackEkf::predict() {
  if (!initialized_) return;

  const double dt = dt_;

  // x' = F x, F = [[I, dt I],[0, I]]:  pos' = pos + vel*dt ; vel' = vel.
  x_[0] += x_[3] * dt;
  x_[1] += x_[4] * dt;
  x_[2] += x_[5] * dt;

  // P' = F P F^T + Q. Same sparse structure exploited as in Ekf.cpp.
  auto P = [&](int row, int col) -> double& { return p_[row * 6 + col]; };

  // FP = F * P (row i of F: rows 0..2 = row i + dt*row(i+3); rows 3..5 = identity).
  std::array<double, 36> fp{};
  for (int j = 0; j < 6; ++j) {
    for (int i = 0; i < 3; ++i) fp[i * 6 + j] = P(i, j) + dt * P(i + 3, j);
    for (int i = 3; i < 6; ++i) fp[i * 6 + j] = P(i, j);
  }
  // FP * F^T (col j: cols 0..2 = col j + dt*col(j+3); cols 3..5 unchanged).
  std::array<double, 36> fpft{};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 3; ++j) fpft[i * 6 + j] = fp[i * 6 + j] + dt * fp[i * 6 + (j + 3)];
    for (int j = 3; j < 6; ++j) fpft[i * 6 + j] = fp[i * 6 + j];
  }

  // Process noise Q (continuous white-noise acceleration), per axis.
  const double q_pp = q_ * dt * dt * dt / 3.0;
  const double q_pv = q_ * dt * dt / 2.0;
  const double q_vv = q_ * dt;
  for (int k = 0; k < 36; ++k) p_[k] = fpft[k];
  for (int a = 0; a < 3; ++a) {
    P(a, a) += q_pp;
    P(a + 3, a + 3) += q_vv;
    P(a, a + 3) += q_pv;
    P(a + 3, a) += q_pv;
  }
}

// ── Measurement model h(x) + Jacobian H for a sensor at a fixed position ─────────────────────────
void TargetTrackEkf::measurementModel(const TrackSensor& sensor, std::array<double, 4>& h,
                                      std::array<double, 24>& H, int& dim) const {
  dim = sensor.dim();
  for (auto& v : H) v = 0.0;

  // Relative geometry from the sensor to the target.
  const double rx = x_[0] - sensor.pos.x;
  const double ry = x_[1] - sensor.pos.y;
  const double rz = x_[2] - sensor.pos.z;
  const double vx = x_[3], vy = x_[4], vz = x_[5];

  const double rho2 = rx * rx + ry * ry;
  const double rho = std::sqrt(rho2);
  const double r2 = rho2 + rz * rz;
  const double r = std::sqrt(r2);

  // az = atan2(ry, rx)
  h[0] = std::atan2(ry, rx);
  // el = atan2(rz, rho)
  h[1] = std::atan2(rz, rho);

  if (rho > 1e-9 && r > 1e-9) {
    // d(az)/d(pos): [-ry/rho2, rx/rho2, 0]
    H[0 * 6 + 0] = -ry / rho2;
    H[0 * 6 + 1] = rx / rho2;
    // d(el)/d(pos): [-rx*rz/(r2*rho), -ry*rz/(r2*rho), rho/r2]
    H[1 * 6 + 0] = -rx * rz / (r2 * rho);
    H[1 * 6 + 1] = -ry * rz / (r2 * rho);
    H[1 * 6 + 2] = rho / r2;
  }

  if (sensor.type == TrackSensorType::Radar) {
    // range = |rel|
    h[2] = r;
    h[3] = 0.0;
    if (r > 1e-9) {
      H[2 * 6 + 0] = rx / r;
      H[2 * 6 + 1] = ry / r;
      H[2 * 6 + 2] = rz / r;

      // range_rate = (rel · vel) / |rel|   (sensor static, so rel_vel = target vel)
      const double rdotv = rx * vx + ry * vy + rz * vz;
      h[3] = rdotv / r;
      // d(rr)/d(pos_i) = vel_i/r - (rel·vel) * rel_i / r^3
      const double inv_r = 1.0 / r;
      const double inv_r3 = inv_r * inv_r * inv_r;
      H[3 * 6 + 0] = vx * inv_r - rdotv * rx * inv_r3;
      H[3 * 6 + 1] = vy * inv_r - rdotv * ry * inv_r3;
      H[3 * 6 + 2] = vz * inv_r - rdotv * rz * inv_r3;
      // d(rr)/d(vel_i) = rel_i / r
      H[3 * 6 + 3] = rx * inv_r;
      H[3 * 6 + 4] = ry * inv_r;
      H[3 * 6 + 5] = rz * inv_r;
    }
  }
}

// ── Sequential measurement update from one sensor ───────────────────────────────────────────────
double TargetTrackEkf::update(const TrackSensor& sensor, const std::vector<double>& z) {
  const int m = sensor.dim();

  if (!initialized_) {
    // Bootstrap: only a radar carries range, so only a radar can seed absolute position.
    // Angles-only (IR) before any radar fix is uninformative for bootstrap — skip and wait.
    if (sensor.type != TrackSensorType::Radar || static_cast<int>(z.size()) < 4) {
      nis_ = 0.0;
      return 0.0;
    }
    const double az = z[0], el = z[1], range = z[2];
    const double cos_el = std::cos(el);
    bootstrap(
        Vector3{sensor.pos.x + range * cos_el * std::cos(az),
                sensor.pos.y + range * cos_el * std::sin(az), sensor.pos.z + range * std::sin(el)},
        Vector3{});
    return 0.0;
  }

  if (static_cast<int>(z.size()) < m) {
    nis_ = 0.0;
    return 0.0;
  }

  std::array<double, 4> h{};
  std::array<double, 24> H{};  // m x 6, row-major (capacity 4x6)
  int dim = 0;
  measurementModel(sensor, h, H, dim);

  // R diagonal for this sensor.
  std::array<double, 4> r_diag{};
  r_diag[0] = sensor.sigma_az * sensor.sigma_az;
  r_diag[1] = sensor.sigma_el * sensor.sigma_el;
  if (m == 4) {
    r_diag[2] = sensor.sigma_range * sensor.sigma_range;
    r_diag[3] = sensor.sigma_range_rate * sensor.sigma_range_rate;
  }

  auto P = [&](int row, int col) -> double { return p_[row * 6 + col]; };

  // PHt = P * H^T  -> 6 x m
  std::array<double, 24> PHt{};  // 6 x m (capacity 6x4)
  for (int i = 0; i < 6; ++i) {
    for (int k = 0; k < m; ++k) {
      double s = 0.0;
      for (int j = 0; j < 6; ++j) s += P(i, j) * H[k * 6 + j];
      PHt[i * m + k] = s;
    }
  }

  // S = H * PHt + R  -> m x m
  std::array<double, 16> S{};
  for (int a = 0; a < m; ++a) {
    for (int b = 0; b < m; ++b) {
      double s = 0.0;
      for (int j = 0; j < 6; ++j) s += H[a * 6 + j] * PHt[j * m + b];
      S[a * m + b] = s;
    }
    S[a * m + a] += r_diag[a];
  }

  std::array<double, 16> Sinv = S;
  if (!invertMxM(Sinv, m)) {
    nis_ = 0.0;
    return 0.0;  // singular innovation covariance: skip rather than blow up
  }

  // Kalman gain K = PHt * Sinv  -> 6 x m
  std::array<double, 24> K{};
  for (int i = 0; i < 6; ++i) {
    for (int b = 0; b < m; ++b) {
      double s = 0.0;
      for (int a = 0; a < m; ++a) s += PHt[i * m + a] * Sinv[a * m + b];
      K[i * m + b] = s;
    }
  }

  // Innovation y = z - h(x). Wrap az/el; range/range_rate are plain differences.
  std::array<double, 4> y{};
  y[0] = wrapPi(z[0] - h[0]);
  y[1] = wrapPi(z[1] - h[1]);
  if (m == 4) {
    y[2] = z[2] - h[2];
    y[3] = z[3] - h[3];
  }

  // NIS = y^T Sinv y (chi-square, dof = m).
  double nis = 0.0;
  for (int a = 0; a < m; ++a) {
    double s = 0.0;
    for (int b = 0; b < m; ++b) s += Sinv[a * m + b] * y[b];
    nis += y[a] * s;
  }
  nis_ = nis;

  // State update: x += K y.
  for (int i = 0; i < 6; ++i) {
    double s = 0.0;
    for (int a = 0; a < m; ++a) s += K[i * m + a] * y[a];
    x_[i] += s;
  }

  // Joseph-form covariance: P = (I - K H) P (I - K H)^T + K R K^T.
  std::array<double, 36> A{};  // I - K H
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      double kh = 0.0;
      for (int a = 0; a < m; ++a) kh += K[i * m + a] * H[a * 6 + j];
      A[i * 6 + j] = (i == j ? 1.0 : 0.0) - kh;
    }
  }
  std::array<double, 36> AP{};  // A * P
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      double s = 0.0;
      for (int k = 0; k < 6; ++k) s += A[i * 6 + k] * P(k, j);
      AP[i * 6 + j] = s;
    }
  }
  std::array<double, 36> APAt{};  // AP * A^T
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      double s = 0.0;
      for (int k = 0; k < 6; ++k) s += AP[i * 6 + k] * A[j * 6 + k];
      APAt[i * 6 + j] = s;
    }
  }
  std::array<double, 36> KRKt{};  // K R K^T (R diagonal)
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      double s = 0.0;
      for (int a = 0; a < m; ++a) s += K[i * m + a] * r_diag[a] * K[j * m + a];
      KRKt[i * 6 + j] = s;
    }
  }
  for (int k = 0; k < 36; ++k) p_[k] = APAt[k] + KRKt[k];

  return nis_;
}

}  // namespace gncsim
