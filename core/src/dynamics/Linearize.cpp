// gnc-sim — numeric linearization of the 6DOF airframe+actuator pitch channel (issue #122).
//
// Forms the short-period pitch-plane state derivative directly from the SAME models the 6DOF core
// flies — AeroModel's Cn(alpha,M)/Cm(alpha,M) (table or linear slope), the full inertia tensor
// (Iyy), USSA76 dynamic pressure, and the fin-actuator effectiveness — then central-differences it
// to recover the linear small-signal A/B. From A/B it derives the open-loop short-period ωn, ζ and
// the steady-state control effectiveness. Pure arithmetic, fixed evaluation order, no RNG, no deps.
#include "gncsim/dynamics/Linearize.hpp"

#include <array>
#include <cmath>

#include "gncsim/aero/Aero.hpp"
#include "gncsim/dynamics/Dynamics6dofHiFi.hpp"
#include "gncsim/env/Environment.hpp"

namespace gncsim {

namespace {

// Central-difference perturbation sizes. Fixed (not state-scaled) so the Jacobian is bit-stable and
// the evaluation order is identical on native and WASM. Small enough that the local linearization
// is accurate, large enough to stay well above double round-off on the coefficient tables.
constexpr double kDAlphaRad = 1.0e-6;   // angle-of-attack perturbation [rad]
constexpr double kDRateRadps = 1.0e-6;  // pitch-rate perturbation [rad/s]
constexpr double kDDeltaRad = 1.0e-6;   // fin-deflection perturbation [rad]

// The reduced pitch-plane state derivative the airframe+actuator flies, evaluated about the trim
// flight condition. State x = [alpha_rad, pitchrate_radps], control delta = fin_deflection_rad.
//
//   alpha_dot = q  -  a_z / V          (a_z = q_bar S Cn(alpha,M) / m : the normal accel rotates
//   the
//                                        velocity vector, which subtracts from the body pitch rate)
//   q_dot     = ( q_bar S d Cm(alpha,M)  +  (q_bar S d^2 / (2V)) Cmq q
//                 +  effectiveness * delta ) / Iyy
//
// This is exactly the static restoring moment + rate damping of AeroModel::momentAero plus the
// FinActuator control moment, reduced to the symmetric pitch plane.
struct PitchPlant {
  const AeroModel* aero;
  double q_bar_pa;
  double speed_mps;
  double mach;
  double ref_area_m2;
  double ref_length_m;
  double mass_kg;
  double iyy_kgm2;
  double cm_q;
  double effectiveness_nm_per_rad;

  // Returns [alpha_dot, q_dot].
  std::array<double, 2> deriv(double alpha_rad, double rate_radps, double delta_rad) const {
    const double cn = aero->normalForceCoeff(alpha_rad, mach);
    const double cm = aero->pitchMomentCoeff(alpha_rad, mach);

    const double a_z_mps2 = q_bar_pa * ref_area_m2 * cn / mass_kg;
    const double alpha_dot = rate_radps - a_z_mps2 / speed_mps;

    const double m_static_nm = q_bar_pa * ref_area_m2 * ref_length_m * cm;
    const double damp_scale =
        q_bar_pa * ref_area_m2 * ref_length_m * ref_length_m / (2.0 * speed_mps);
    const double m_damp_nm = damp_scale * cm_q * rate_radps;
    const double m_ctrl_nm = effectiveness_nm_per_rad * delta_rad;
    const double q_dot = (m_static_nm + m_damp_nm + m_ctrl_nm) / iyy_kgm2;

    return {alpha_dot, q_dot};
  }
};

}  // namespace

LinearizeResult linearizeAirframe(const SimConfig& cfg, const TrimCondition& trim) {
  LinearizeResult out;

  const AeroModel aero(cfg.aero);
  const AtmSample atm = atmosphereUSSA76(trim.altitude_m);

  // Speed of sound -> freestream speed -> dynamic pressure. Guard a degenerate atmosphere.
  const double a_sound = atm.speed_of_sound > 0.0 ? atm.speed_of_sound : 340.0;
  const double speed_mps = trim.mach * a_sound > 1.0 ? trim.mach * a_sound : 1.0;
  const double q_bar_pa = 0.5 * atm.density * speed_mps * speed_mps;

  const InertiaTensor inertia = inertiaFromVehicle(cfg.vehicle);
  // Pitch moment of inertia is the (1,1) entry of the symmetric tensor (row-major [4]).
  const double iyy = inertia.I[4] > 0.0 ? inertia.I[4] : 1.0;

  PitchPlant plant;
  plant.aero = &aero;
  plant.q_bar_pa = q_bar_pa;
  plant.speed_mps = speed_mps;
  plant.mach = trim.mach;
  plant.ref_area_m2 = cfg.aero.ref_area;
  plant.ref_length_m = cfg.aero.ref_length;
  plant.mass_kg = cfg.vehicle.mass0 > 0.0 ? cfg.vehicle.mass0 : 1.0;
  plant.iyy_kgm2 = iyy;
  plant.cm_q = cfg.aero.cm_q;
  plant.effectiveness_nm_per_rad = cfg.actuator.effectiveness;

  const double a0 = trim.alpha_rad;

  // --- Central-difference Jacobians about the trim point. ---
  // A = d(xdot)/dx : column 0 = d/d alpha, column 1 = d/d q.
  const auto da = plant.deriv(a0 + kDAlphaRad, 0.0, 0.0);
  const auto db = plant.deriv(a0 - kDAlphaRad, 0.0, 0.0);
  const auto dc = plant.deriv(a0, kDRateRadps, 0.0);
  const auto dd = plant.deriv(a0, -kDRateRadps, 0.0);

  const double a00 = (da[0] - db[0]) / (2.0 * kDAlphaRad);   // d alpha_dot / d alpha
  const double a10 = (da[1] - db[1]) / (2.0 * kDAlphaRad);   // d q_dot     / d alpha
  const double a01 = (dc[0] - dd[0]) / (2.0 * kDRateRadps);  // d alpha_dot / d q
  const double a11 = (dc[1] - dd[1]) / (2.0 * kDRateRadps);  // d q_dot     / d q

  // B = d(xdot)/d delta.
  const auto de = plant.deriv(a0, 0.0, kDDeltaRad);
  const auto df = plant.deriv(a0, 0.0, -kDDeltaRad);
  const double b0 = (de[0] - df[0]) / (2.0 * kDDeltaRad);
  const double b1 = (de[1] - df[1]) / (2.0 * kDDeltaRad);

  out.a_matrix = {a00, a01, a10, a11};
  out.b_matrix = {b0, b1};

  // --- Short-period mode from A: eigenvalues of the 2x2. ---
  // char. poly  lambda^2 - tr lambda + det,  tr = a00 + a11,  det = a00 a11 - a01 a10.
  const double tr = a00 + a11;
  const double det = a00 * a11 - a01 * a10;
  // ωn^2 = det (for the complex-pair short period); 2 ζ ωn = -tr.
  if (det > 0.0) {
    const double wn = std::sqrt(det);
    out.omega_n_radps = wn;
    out.zeta = (wn > 0.0) ? (-tr / (2.0 * wn)) : 0.0;
  } else {
    // Statically unstable (a divergent real pole pair): no real natural frequency. Report 0 ωn and
    // leave ζ at 0; `stable` below still reflects the (un)stable eigenvalues.
    out.omega_n_radps = 0.0;
    out.zeta = 0.0;
  }
  // Stable iff both eigenvalues have negative real part: tr < 0 AND det > 0 (Hurwitz, 2x2).
  out.stable = (tr < 0.0) && (det > 0.0);

  // --- Steady-state control effectiveness: the DC accel-per-deflection gain. ---
  // At equilibrium x_dot = 0 => x_ss = -A^-1 B delta; the steady normal accel is
  //   a_z = q_bar S Cn_alpha alpha_ss / m  (linearized), per unit delta.
  // Numerically: perturb delta, solve the 2x2 steady state, read the alpha response, map to accel.
  const double cn_alpha = (aero.normalForceCoeff(a0 + kDAlphaRad, trim.mach) -
                           aero.normalForceCoeff(a0 - kDAlphaRad, trim.mach)) /
                          (2.0 * kDAlphaRad);
  const double cm_alpha = (aero.pitchMomentCoeff(a0 + kDAlphaRad, trim.mach) -
                           aero.pitchMomentCoeff(a0 - kDAlphaRad, trim.mach)) /
                          (2.0 * kDAlphaRad);

  // Steady-state alpha per unit delta from A x_ss = -B (det != 0):
  //   alpha_ss = (-B0 a11 + B1 a01) / det   [Cramer on the 2x2 A x = -B].
  double alpha_ss_per_rad = 0.0;
  if (std::fabs(det) > 1e-300) {
    alpha_ss_per_rad = (-b0 * a11 + b1 * a01) / det;
  }
  const double accel_per_alpha = q_bar_pa * cfg.aero.ref_area * cn_alpha / plant.mass_kg;
  out.control_effectiveness_mps2_per_rad = accel_per_alpha * alpha_ss_per_rad;

  out.q_bar_pa = q_bar_pa;
  out.speed_mps = speed_mps;
  out.iyy_kgm2 = iyy;
  out.cm_alpha_per_rad = cm_alpha;
  out.cn_alpha_per_rad = cn_alpha;

  return out;
}

}  // namespace gncsim
