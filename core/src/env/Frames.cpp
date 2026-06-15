// gnc-sim — round-Earth frame conversions (WGS-84). Hand-rolled geodesy, no external deps.
// Only reached when SimConfig.env.frame == "round". See gncsim/env/Frames.hpp for the frame
// definitions and the data contract.
#include "gncsim/env/Frames.hpp"

#include <cmath>

namespace gncsim {

namespace {

// Prime vertical radius of curvature N at geodetic latitude.
double primeVerticalRadius(double sin_lat) {
  return wgs84::kA / std::sqrt(1.0 - wgs84::kE2 * sin_lat * sin_lat);
}

constexpr double kDeg2Rad = M_PI / 180.0;

}  // namespace

// =============================================================================================
// geodetic <-> ECEF
// =============================================================================================
Vector3 geodeticToEcef(double lat_rad, double lon_rad, double alt_m) {
  const double sin_lat = std::sin(lat_rad);
  const double cos_lat = std::cos(lat_rad);
  const double sin_lon = std::sin(lon_rad);
  const double cos_lon = std::cos(lon_rad);
  const double N_m = primeVerticalRadius(sin_lat);
  const double x_m = (N_m + alt_m) * cos_lat * cos_lon;
  const double y_m = (N_m + alt_m) * cos_lat * sin_lon;
  const double z_m = (N_m * (1.0 - wgs84::kE2) + alt_m) * sin_lat;
  return {x_m, y_m, z_m};
}

// Bowring's iterative method: converges to well below mm in a few iterations everywhere.
void ecefToGeodetic(const Vector3& ecef, double& lat_rad, double& lon_rad, double& alt_m) {
  const double x_m = ecef.x;
  const double y_m = ecef.y;
  const double z_m = ecef.z;
  lon_rad = std::atan2(y_m, x_m);

  const double p_m = std::sqrt(x_m * x_m + y_m * y_m);
  // Handle the polar singularity (p_m == 0) explicitly.
  if (p_m < 1e-9) {
    lat_rad = (z_m >= 0.0) ? (M_PI / 2.0) : (-M_PI / 2.0);
    alt_m = std::fabs(z_m) - wgs84::kB;
    return;
  }

  // Initial latitude guess (spherical), then iterate.
  double lat = std::atan2(z_m, p_m * (1.0 - wgs84::kE2));
  for (int i = 0; i < 8; ++i) {
    const double sin_lat = std::sin(lat);
    const double N_m = primeVerticalRadius(sin_lat);
    const double alt_iter_m = p_m / std::cos(lat) - N_m;
    const double next = std::atan2(z_m, p_m * (1.0 - wgs84::kE2 * N_m / (N_m + alt_iter_m)));
    if (std::fabs(next - lat) < 1e-14) {
      lat = next;
      break;
    }
    lat = next;
  }
  lat_rad = lat;
  const double sin_lat = std::sin(lat);
  const double N_m = primeVerticalRadius(sin_lat);
  alt_m = p_m / std::cos(lat) - N_m;
}

// =============================================================================================
// ENU <-> ECEF about a geodetic origin
// =============================================================================================
//
// The ENU->ECEF rotation matrix R (columns = East, North, Up unit vectors in ECEF) at origin
// (lat0, lon0):
//   East  = [-sinλ,           cosλ,          0     ]
//   North = [-sinφ cosλ,     -sinφ sinλ,     cosφ  ]
//   Up    = [ cosφ cosλ,      cosφ sinλ,     sinφ  ]
// ecef_vec = R * enu_vec; enu_vec = R^T * ecef_vec.
namespace {

struct EnuBasis {
  double slat, clat, slon, clon;
};

EnuBasis originBasis(const GeodeticOrigin& o) {
  const double lat_rad = o.lat0_deg * kDeg2Rad;
  const double lon_rad = o.lon0_deg * kDeg2Rad;
  return {std::sin(lat_rad), std::cos(lat_rad), std::sin(lon_rad), std::cos(lon_rad)};
}

Vector3 enuVecToEcefImpl(const Vector3& e, const EnuBasis& b) {
  // R * [E, N, U].
  const double x = -b.slon * e.x - b.slat * b.clon * e.y + b.clat * b.clon * e.z;
  const double y = b.clon * e.x - b.slat * b.slon * e.y + b.clat * b.slon * e.z;
  const double z = b.clat * e.y + b.slat * e.z;
  return {x, y, z};
}

Vector3 ecefVecToEnuImpl(const Vector3& v, const EnuBasis& b) {
  // R^T * [vx, vy, vz].
  const double e = -b.slon * v.x + b.clon * v.y;
  const double n = -b.slat * b.clon * v.x - b.slat * b.slon * v.y + b.clat * v.z;
  const double u = b.clat * b.clon * v.x + b.clat * b.slon * v.y + b.slat * v.z;
  return {e, n, u};
}

}  // namespace

Vector3 enuVecToEcef(const Vector3& v_enu, const GeodeticOrigin& origin) {
  return enuVecToEcefImpl(v_enu, originBasis(origin));
}

Vector3 ecefVecToEnu(const Vector3& v_ecef, const GeodeticOrigin& origin) {
  return ecefVecToEnuImpl(v_ecef, originBasis(origin));
}

Vector3 enuToEcef(const Vector3& enu, const GeodeticOrigin& origin) {
  const Vector3 origin_ecef =
      geodeticToEcef(origin.lat0_deg * kDeg2Rad, origin.lon0_deg * kDeg2Rad, origin.alt0_m);
  return origin_ecef + enuVecToEcefImpl(enu, originBasis(origin));
}

Vector3 ecefToEnu(const Vector3& ecef, const GeodeticOrigin& origin) {
  const Vector3 origin_ecef =
      geodeticToEcef(origin.lat0_deg * kDeg2Rad, origin.lon0_deg * kDeg2Rad, origin.alt0_m);
  return ecefVecToEnuImpl(ecef - origin_ecef, originBasis(origin));
}

// =============================================================================================
// ECI <-> ECEF  (rotation about +Z by the Earth angle theta = omega * t)
// =============================================================================================
namespace {
Vector3 rotateZ(const Vector3& v, double theta_rad) {
  const double c = std::cos(theta_rad);
  const double s = std::sin(theta_rad);
  return {c * v.x - s * v.y, s * v.x + c * v.y, v.z};
}
}  // namespace

Vector3 eciToEcef(const Vector3& eci, double t) {
  // ECEF = Rz(-omega*t) * ECI.
  return rotateZ(eci, -wgs84::kOmega * t);
}

Vector3 ecefToEci(const Vector3& ecef, double t) { return rotateZ(ecef, wgs84::kOmega * t); }

Vector3 eciVelToEcef(const Vector3& r_eci, const Vector3& v_eci, double t) {
  // v_ecef = Rz(-wt) * (v_eci - omega x r_eci), omega = [0,0,kOmega].
  const Vector3 omega{0.0, 0.0, wgs84::kOmega};
  const Vector3 v_rot = v_eci - omega.cross(r_eci);
  return rotateZ(v_rot, -wgs84::kOmega * t);
}

Vector3 ecefVelToEci(const Vector3& r_ecef, const Vector3& v_ecef, double t) {
  // v_eci = Rz(wt) * v_ecef + omega x r_eci.
  const Vector3 omega{0.0, 0.0, wgs84::kOmega};
  const Vector3 r_eci = ecefToEci(r_ecef, t);
  return rotateZ(v_ecef, wgs84::kOmega * t) + omega.cross(r_eci);
}

// =============================================================================================
// Gravity (central point-mass + optional J2)
// =============================================================================================
Vector3 centralGravity(const Vector3& r, bool with_j2) {
  const double rmag_m = r.norm();
  if (rmag_m < 1.0) return Vector3{};  // guard the singularity at the centre
  const double inv_r2 = 1.0 / (rmag_m * rmag_m);
  const double inv_r = 1.0 / rmag_m;
  // Point-mass term.
  Vector3 g = r * (-wgs84::kGM * inv_r2 * inv_r);  // -GM/r^3 * r

  if (with_j2) {
    // J2 perturbation in an Earth-fixed/inertial frame sharing the spin axis (+Z).
    //   factor = 1.5 * J2 * GM * a^2 / r^4
    //   a_x = factor * (5 z^2/r^2 - 1) x/r
    //   a_y = factor * (5 z^2/r^2 - 1) y/r
    //   a_z = factor * (5 z^2/r^2 - 3) z/r
    const double factor = 1.5 * wgs84::kJ2 * wgs84::kGM * wgs84::kA * wgs84::kA * inv_r2 * inv_r2;
    const double zr2 = 5.0 * (r.z * r.z) * inv_r2;
    g.x += factor * (zr2 - 1.0) * r.x * inv_r;
    g.y += factor * (zr2 - 1.0) * r.y * inv_r;
    g.z += factor * (zr2 - 3.0) * r.z * inv_r;
  }
  return g;
}

}  // namespace gncsim
