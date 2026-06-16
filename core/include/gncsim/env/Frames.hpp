// gnc-sim — round-Earth frame abstraction (WGS-84). Pure, header-declared geodesy used only when
// SimConfig.env.frame == "round". The flat-Earth path never touches this module, so the default
// (flat) trajectory stays byte-identical.
//
// Frames used here:
//   geodetic  : (latitude, longitude, altitude) on the WGS-84 ellipsoid.
//   ECEF      : Earth-Centred Earth-Fixed, rotates with the planet. +Z = spin axis, +X = (lat 0,
//               lon 0), right-handed. WGS-84 reference frame.
//   ENU       : local East-North-Up tangent plane about a geodetic origin (the telemetry frame
//               and the data contract — see docs/DATA_CONTRACT.md).
//   ECI       : Earth-Centred Inertial. Coincident with ECEF at t = 0 (GMST = 0 assumed) and the
//               translational integration frame in round mode (inertial => no Coriolis term).
//
// See core/src/env/Frames.cpp for the implementations and docs for the propagation-frame choice.
#pragma once

#include "gncsim/core/Types.hpp"
#include "gncsim/math/Vector3.hpp"

namespace gncsim {

// WGS-84 reference ellipsoid + Earth model constants (SI units).
namespace wgs84 {
constexpr double kA = 6378137.0;            // semi-major axis [m]
constexpr double kF = 1.0 / 298.257223563;  // flattening [-]
constexpr double kB = kA * (1.0 - kF);      // semi-minor axis [m]
constexpr double kE2 = kF * (2.0 - kF);     // first eccentricity squared e^2 [-]
constexpr double kOmega = 7.292115e-5;      // Earth rotation rate [rad/s]
constexpr double kGM = 3.986004418e14;      // gravitational parameter GM [m^3/s^2]
constexpr double kJ2 = 1.082626683e-3;      // second zonal harmonic J2 [-]
}  // namespace wgs84

// --- geodetic <-> ECEF ----------------------------------------------------------------------
// Closed-form geodetic -> ECEF.
Vector3 geodeticToEcef(double lat_rad, double lon_rad, double alt_m);

// Iterative (Bowring) ECEF -> geodetic. Outputs are by reference; converges to mm precision.
void ecefToGeodetic(const Vector3& ecef, double& lat_rad, double& lon_rad, double& alt_m);

// --- ENU <-> ECEF about a geodetic origin ---------------------------------------------------
// Rotate/translate a point. The ENU axes are defined at `origin`: East/North tangent to the
// ellipsoid, Up along the local geodetic normal.
Vector3 ecefToEnu(const Vector3& ecef, const GeodeticOrigin& origin);
Vector3 enuToEcef(const Vector3& enu, const GeodeticOrigin& origin);

// Rotate a free vector (velocity, acceleration) between ENU and ECEF — rotation only, no origin
// translation.
Vector3 ecefVecToEnu(const Vector3& v_ecef, const GeodeticOrigin& origin);
Vector3 enuVecToEcef(const Vector3& v_enu, const GeodeticOrigin& origin);

// --- ECI <-> ECEF (simple z-axis rotation by the Earth angle omega*t) -----------------------
// ECI and ECEF are coincident at t = 0; ECEF lags ECI by the rotated Earth angle thereafter.
Vector3 eciToEcef(const Vector3& eci, double t);
Vector3 ecefToEci(const Vector3& ecef, double t);

// Rotate a velocity from ECI to ECEF accounting for the rotating-frame transport term
// (v_ecef = Rz(-wt) * (v_eci - omega x r_eci)). Used to initialise / report velocities.
Vector3 eciVelToEcef(const Vector3& r_eci, const Vector3& v_eci, double t);
Vector3 ecefVelToEci(const Vector3& r_ecef, const Vector3& v_ecef, double t);

// --- gravity in ECI/ECEF --------------------------------------------------------------------
// Central point-mass gravity g = -GM/r^3 * r. If with_j2 is true, adds the J2 oblateness term.
// Valid in either ECI or ECEF (both share Earth's spin axis as +Z); pass the matching position.
Vector3 centralGravity(const Vector3& r, bool with_j2);

}  // namespace gncsim
