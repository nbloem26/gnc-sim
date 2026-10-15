/// @file Linearize.hpp
/// @brief Numeric linearization of the 6DOF airframe+actuator pitch channel about a flight
///        condition (issue #122).
///
/// Opt-in, pure, additive analysis entry — it does NOT touch `runSimulation`/`run_sim`. Given a
/// SimConfig (the real airframe: AeroConfig Cn/Cm tables + slopes, the inertia tensor, the fin
/// actuator) and a trim flight condition (Mach, altitude, angle of attack), it forms the
/// short-period pitch-plane state derivative
///
///     state x = [alpha_rad, pitchrate_radps],   control u = fin_deflection_rad
///       alpha_dot = q  -  (q_bar S Cn(alpha,M)) / (m V)
///       q_dot     = ( q_bar S d Cm(alpha,M)  +  (q_bar S d^2 / (2V)) Cmq q
///                     +  effectiveness * delta ) / Iyy
///
/// and numerically Jacobians it by CENTRAL DIFFERENCE (fixed perturbation, fixed evaluation order)
/// to recover the linear small-signal model  x_dot = A x + B u. From A/B it derives the open-loop
/// short-period transfer characteristics ωn, ζ and the steady-state control effectiveness
/// (accel-per-deflection DC gain), which the Controls Studio shapes.
///
/// Everything here is project types + pure arithmetic: deterministic, no RNG, no new deps, and it
/// compiles to WASM exactly like the rest of the core.
#pragma once

#include <array>

#include "gncsim/core/Config.hpp"

namespace gncsim {

/// @brief Flight condition the airframe is linearized about.
struct TrimCondition {
  double mach = 2.0;           ///< freestream Mach number (sets q_bar via the atmosphere)
  double altitude_m = 5000.0;  ///< geometric altitude [m] (USSA76 density + speed of sound)
  double alpha_rad = 0.05;     ///< trim angle of attack [rad] the Jacobian is taken about
};

/// @brief Result of linearizing the airframe+actuator pitch channel about a TrimCondition.
struct LinearizeResult {
  // Short-period state-space (state [alpha_rad, pitchrate_radps], control fin_deflection_rad):
  //   x_dot = A x + B u
  std::array<double, 4> a_matrix{};  ///< 2x2 A, row-major [a00,a01,a10,a11]
  std::array<double, 2> b_matrix{};  ///< 2x1 B [b0,b1]

  // Open-loop short-period mode characteristics derived from A:
  //   char. poly s^2 - tr(A) s + det(A);  ωn = sqrt(det),  2 ζ ωn = -tr(A).
  double omega_n_radps = 0.0;  ///< undamped natural frequency [rad/s] (0 if non-oscillatory)
  double zeta = 0.0;           ///< damping ratio (dimensionless)
  bool stable = false;         ///< both eigenvalues in the open left-half plane

  // Steady-state control effectiveness: the DC accel-per-deflection gain
  //   K = a_z / delta  at equilibrium  [ (m/s^2) / rad ].
  double control_effectiveness_mps2_per_rad = 0.0;

  // The flight condition / airframe numbers used (for display + provenance).
  double q_bar_pa = 0.0;          ///< dynamic pressure q_bar = 0.5 rho V^2 [Pa]
  double speed_mps = 0.0;         ///< freestream speed V = M * a [m/s]
  double iyy_kgm2 = 0.0;          ///< pitch moment of inertia used [kg*m^2]
  double cm_alpha_per_rad = 0.0;  ///< local static-stability slope dCm/dalpha [1/rad]
  double cn_alpha_per_rad = 0.0;  ///< local normal-force slope dCn/dalpha [1/rad]
};

/// @brief Linearize the airframe+actuator pitch channel of `cfg` about `trim`.
///
/// Pure, deterministic. Uses the SimConfig's AeroConfig (Cn/Cm tables or linear slopes), the
/// vehicle inertia tensor (Iyy), the actuator effectiveness, and USSA76 at `trim.altitude_m`.
LinearizeResult linearizeAirframe(const SimConfig& cfg, const TrimCondition& trim);

}  // namespace gncsim
