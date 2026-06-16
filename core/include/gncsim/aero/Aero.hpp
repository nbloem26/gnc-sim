/// @file Aero.hpp
/// @brief Aerodynamics — Mach-dependent drag, plus 6DOF normal force/moment from angle of attack.
///
/// Phase 1 (aero) owns the implementations in core/src/aero/. See docs/THEORY.md §4 for the
/// drag force @f$-\tfrac12\rho V^2 C_d(M) A\,\hat v@f$ and the normal-force term.
#pragma once

#include "gncsim/core/Config.hpp"
#include "gncsim/env/Environment.hpp"
#include "gncsim/math/Quaternion.hpp"
#include "gncsim/math/Vector3.hpp"

namespace gncsim {

/// @brief Aerodynamic force/moment model driven by the AeroConfig drag polar and Cn/Cm tables.
class AeroModel {
 public:
  explicit AeroModel(const AeroConfig& cfg) : cfg_(cfg) {}

  // Linear interpolation over the (mach, Cd) table; falls back to cd0 when the table is empty.
  double dragCoefficient(double mach) const;

  // 3DOF drag-only aerodynamic force in the world frame [N].
  Vector3 dragForce(const Vector3& vel_world, const AtmSample& atm) const;

  // 6DOF aerodynamic force in the world frame [N]: drag (anti-velocity) + normal force from
  // the angle of attack between the body x-axis and the velocity vector.
  Vector3 force6dof(const Vector3& vel_world, const Quaternion& att, const AtmSample& atm) const;

  // --- High-fidelity 6DOF aero (issue #35) ---

  // Normal-force coefficient Cn(alpha, mach). Uses the cn_table when present (bilinear
  // interpolation over angle-of-attack [rad] and Mach, edge-clamped), else the linear slope
  // cfg.cn_alpha * alpha.
  double normalForceCoeff(double alpha, double mach) const;

  // Static pitch/yaw moment coefficient Cm(alpha, mach). Uses the cm_table when present, else the
  // linear slope cfg.cm_alpha * alpha. By convention Cm < 0 for alpha > 0 is statically stable.
  double pitchMomentCoeff(double alpha, double mach) const;

  // Hi-fi 6DOF aerodynamic force [N] in the world frame: table-driven normal force (Cn) plus drag.
  Vector3 force6dofHiFi(const Vector3& vel_world, const Quaternion& att,
                        const AtmSample& atm) const;

  // Hi-fi 6DOF aerodynamic moment [N*m] in the BODY frame: static restoring moment from Cm(alpha)
  // about the pitch/yaw axes, plus pitch/yaw (cm_q) and roll (cl_p) rate damping. `ang_vel_world`
  // is the body angular rate expressed in the world frame (EntityState::angVel is body-frame
  // already, so the Runner passes it through att for the damping projection — see implementation).
  Vector3 momentAero(const Vector3& vel_world, const Quaternion& att, const Vector3& ang_vel_body,
                     const AtmSample& atm) const;

 private:
  AeroConfig cfg_;
};

}  // namespace gncsim
