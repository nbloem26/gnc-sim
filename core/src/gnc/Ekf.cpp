// gnc-sim — Extended Kalman Filter implementation (see Ekf.hpp). World frame ENU, SI units.
//
// All matrices are fixed-size and row-major in std::array. The state-transition F has the sparse
// nearly-constant-velocity block structure [[I, dt*I],[0, I]] and the process noise Q has the
// classic continuous-white-noise-acceleration form; we exploit that structure rather than running a
// general 6x6 multiply for the predict step. The update step uses small dense helpers (6x3, 3x3,
// 3x3 inverse via cofactors) and the Joseph form for the covariance to preserve symmetry / PSD.
#include "gncsim/gnc/Ekf.hpp"

#include <cmath>

namespace gncsim {

namespace {

constexpr double kPi = 3.14159265358979323846;

// Wrap an angle to [-pi, pi]. Used on the azimuth innovation so a wrap-around measurement
// (e.g. truth near +pi, estimate near -pi) doesn't produce a ~2*pi spurious innovation.
double wrapPi(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

// 3x3 matrix inverse via cofactors / adjugate. Returns false if (near-)singular.
// in and out are row-major 9-element arrays.
bool invert3x3(const std::array<double, 9>& m, std::array<double, 9>& out) {
  const double a = m[0], b = m[1], c = m[2];
  const double d = m[3], e = m[4], f = m[5];
  const double g = m[6], h = m[7], i = m[8];

  const double A = e * i - f * h;   // cofactor 00
  const double B = -(d * i - f * g);  // cofactor 01
  const double C = d * h - e * g;   // cofactor 02
  const double det = a * A + b * B + c * C;
  if (std::fabs(det) < 1e-300) return false;
  const double inv_det = 1.0 / det;

  // Adjugate (transpose of the cofactor matrix), scaled by 1/det.
  out[0] = A * inv_det;
  out[1] = (c * h - b * i) * inv_det;
  out[2] = (b * f - c * e) * inv_det;
  out[3] = B * inv_det;
  out[4] = (a * i - c * g) * inv_det;
  out[5] = (c * d - a * f) * inv_det;
  out[6] = C * inv_det;
  out[7] = (b * g - a * h) * inv_det;
  out[8] = (a * e - b * d) * inv_det;
  return true;
}

}  // namespace

Ekf::Ekf(double dt, double process_accel_psd, double sigma_az, double sigma_el, double sigma_range)
    : dt_(dt),
      q_(process_accel_psd),
      r_diag_{sigma_az * sigma_az, sigma_el * sigma_el, sigma_range * sigma_range} {}

// ── Time update ───────────────────────────────────────────────────────────────────────────────
void Ekf::predict(const Vector3& a_vehicle) {
  if (!initialized_) return;

  // Control input: in the relative frame the known forcing term is minus the vehicle's own accel
  // (gravity cancels as a common-mode term between target and vehicle).
  const double ux = -a_vehicle.x, uy = -a_vehicle.y, uz = -a_vehicle.z;
  const double dt = dt_;
  const double half_dt2 = 0.5 * dt * dt;

  // x' = F x + B u, with F = [[I, dt I],[0, I]]:
  //   pos' = pos + vel*dt + 0.5*u*dt^2
  //   vel' = vel + u*dt
  x_[0] += x_[3] * dt + half_dt2 * ux;
  x_[1] += x_[4] * dt + half_dt2 * uy;
  x_[2] += x_[5] * dt + half_dt2 * uz;
  x_[3] += ux * dt;
  x_[4] += uy * dt;
  x_[5] += uz * dt;

  // P' = F P F^T + Q. F mixes the velocity block into the position block per axis; with the sparse
  // structure this is, treating P as 2x2 blocks of diagonal 3x3 sub-blocks per axis [Ppp Ppv; Pvp Pvv]:
  //   Ppp' = Ppp + dt*(Ppv + Pvp) + dt^2*Pvv
  //   Ppv' = Ppv + dt*Pvv
  //   Pvp' = Pvp + dt*Pvv
  //   Pvv' = Pvv
  // We do this on the full 6x6 (general, robust to any cross terms the update may introduce).
  auto P = [&](int row, int col) -> double& { return p_[row * 6 + col]; };

  // Compute FP = F * P first (row-major), where F row i:
  //   rows 0..2: row i  + dt * row (i+3)
  //   rows 3..5: row i  (identity)
  std::array<double, 36> fp{};
  for (int j = 0; j < 6; ++j) {
    for (int i = 0; i < 3; ++i) {
      fp[i * 6 + j] = P(i, j) + dt * P(i + 3, j);
    }
    for (int i = 3; i < 6; ++i) {
      fp[i * 6 + j] = P(i, j);
    }
  }
  // Now FP * F^T. F^T column j: same structure transposed -> (FP F^T)[i][j]:
  //   for j in 0..2: FP[i][j] + dt * FP[i][j+3]
  //   for j in 3..5: FP[i][j]
  std::array<double, 36> fpft{};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 3; ++j) {
      fpft[i * 6 + j] = fp[i * 6 + j] + dt * fp[i * 6 + (j + 3)];
    }
    for (int j = 3; j < 6; ++j) {
      fpft[i * 6 + j] = fp[i * 6 + j];
    }
  }

  // Process noise Q (continuous white-noise acceleration), per axis:
  //   Q = q * [[dt^3/3 I, dt^2/2 I],[dt^2/2 I, dt I]]
  const double q_pp = q_ * dt * dt * dt / 3.0;
  const double q_pv = q_ * dt * dt / 2.0;
  const double q_vv = q_ * dt;
  for (int k = 0; k < 36; ++k) p_[k] = fpft[k];
  for (int a = 0; a < 3; ++a) {
    P(a, a) += q_pp;          // position-position
    P(a + 3, a + 3) += q_vv;  // velocity-velocity
    P(a, a + 3) += q_pv;      // position-velocity
    P(a + 3, a) += q_pv;      // velocity-position
  }
}

// ── Measurement update ──────────────────────────────────────────────────────────────────────────
void Ekf::update(double az, double el, double range) {
  if (!initialized_) {
    // Bootstrap: reconstruct the relative position from the spherical measurement, velocity unknown.
    const double cos_el = std::cos(el);
    x_[0] = range * cos_el * std::cos(az);
    x_[1] = range * cos_el * std::sin(az);
    x_[2] = range * std::sin(el);
    x_[3] = x_[4] = x_[5] = 0.0;

    // Large, diagonal initial covariance: position roughly to the measurement scale, velocity wide.
    for (int k = 0; k < 36; ++k) p_[k] = 0.0;
    auto P0 = [&](int idx) -> double& { return p_[idx * 6 + idx]; };
    const double pos_var = 1.0e6;   // ~1 km std
    const double vel_var = 1.0e6;   // ~1 km/s std
    P0(0) = P0(1) = P0(2) = pos_var;
    P0(3) = P0(4) = P0(5) = vel_var;

    initialized_ = true;
    nis_ = 0.0;
    return;
  }

  const double px = x_[0], py = x_[1], pz = x_[2];
  const double rho2 = px * px + py * py;
  double rho = std::sqrt(rho2);
  const double r2 = rho2 + pz * pz;
  double r = std::sqrt(r2);
  // Guard a degenerate geometry (vehicle and target coincident): skip the update, keep prior.
  if (rho < 1e-9 || r < 1e-9) {
    nis_ = 0.0;
    return;
  }

  // Predicted measurement h(x).
  const double az_pred = std::atan2(py, px);
  const double el_pred = std::atan2(pz, rho);
  const double r_pred = r;

  // Measurement Jacobian H (3x6); velocity columns are zero.
  // Row layout: [d/dpx, d/dpy, d/dpz, 0, 0, 0]
  std::array<double, 18> H{};  // 3x6 row-major
  // az = atan2(py, px)
  H[0] = -py / rho2;
  H[1] = px / rho2;
  H[2] = 0.0;
  // el = atan2(pz, rho)
  H[6] = -px * pz / (r2 * rho);
  H[7] = -py * pz / (r2 * rho);
  H[8] = rho / r2;
  // range = r
  H[12] = px / r;
  H[13] = py / r;
  H[14] = pz / r;

  auto P = [&](int row, int col) -> double { return p_[row * 6 + col]; };

  // PHt = P * H^T  -> 6x3
  std::array<double, 18> PHt{};  // 6x3 row-major
  for (int i = 0; i < 6; ++i) {
    for (int k = 0; k < 3; ++k) {
      double s = 0.0;
      for (int j = 0; j < 6; ++j) {
        s += P(i, j) * H[k * 6 + j];  // H[k][j]
      }
      PHt[i * 3 + k] = s;
    }
  }

  // S = H * PHt + R  -> 3x3
  std::array<double, 9> S{};
  for (int a = 0; a < 3; ++a) {
    for (int b = 0; b < 3; ++b) {
      double s = 0.0;
      for (int j = 0; j < 6; ++j) {
        s += H[a * 6 + j] * PHt[j * 3 + b];
      }
      S[a * 3 + b] = s;
    }
    S[a * 3 + a] += r_diag_[a];
  }

  std::array<double, 9> Sinv{};
  if (!invert3x3(S, Sinv)) {
    nis_ = 0.0;
    return;  // singular innovation covariance: skip rather than blow up
  }

  // Kalman gain K = PHt * Sinv  -> 6x3
  std::array<double, 18> K{};  // 6x3 row-major
  for (int i = 0; i < 6; ++i) {
    for (int b = 0; b < 3; ++b) {
      double s = 0.0;
      for (int a = 0; a < 3; ++a) {
        s += PHt[i * 3 + a] * Sinv[a * 3 + b];
      }
      K[i * 3 + b] = s;
    }
  }

  // Innovation y = z - h(x); wrap the azimuth (and elevation, bounded to [-pi/2,pi/2], stays small).
  std::array<double, 3> y{wrapPi(az - az_pred), wrapPi(el - el_pred), range - r_pred};

  // NIS = y^T Sinv y (chi-square, dof = 3).
  double nis = 0.0;
  for (int a = 0; a < 3; ++a) {
    double s = 0.0;
    for (int b = 0; b < 3; ++b) s += Sinv[a * 3 + b] * y[b];
    nis += y[a] * s;
  }
  nis_ = nis;

  // State update: x += K y.
  for (int i = 0; i < 6; ++i) {
    x_[i] += K[i * 3 + 0] * y[0] + K[i * 3 + 1] * y[1] + K[i * 3 + 2] * y[2];
  }

  // Joseph-form covariance update: P = (I - K H) P (I - K H)^T + K R K^T.
  // First A = (I - K H)  -> 6x6.
  std::array<double, 36> A{};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      double kh = 0.0;
      for (int a = 0; a < 3; ++a) kh += K[i * 3 + a] * H[a * 6 + j];
      A[i * 6 + j] = (i == j ? 1.0 : 0.0) - kh;
    }
  }
  // AP = A * P  -> 6x6.
  std::array<double, 36> AP{};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      double s = 0.0;
      for (int k = 0; k < 6; ++k) s += A[i * 6 + k] * P(k, j);
      AP[i * 6 + j] = s;
    }
  }
  // APAt = AP * A^T  -> 6x6.
  std::array<double, 36> APAt{};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      double s = 0.0;
      for (int k = 0; k < 6; ++k) s += AP[i * 6 + k] * A[j * 6 + k];  // A^T[k][j] = A[j][k]
      APAt[i * 6 + j] = s;
    }
  }
  // KRKt = K R K^T  (R diagonal)  -> 6x6.
  std::array<double, 36> KRKt{};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      double s = 0.0;
      for (int a = 0; a < 3; ++a) s += K[i * 3 + a] * r_diag_[a] * K[j * 3 + a];
      KRKt[i * 6 + j] = s;
    }
  }
  for (int k = 0; k < 36; ++k) p_[k] = APAt[k] + KRKt[k];
}

}  // namespace gncsim
