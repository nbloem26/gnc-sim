// gnc-sim — aerodynamics: Mach-dependent drag, plus 6DOF normal force from angle of attack.
// Phase 1 (aero) owns the implementations in core/src/aero/.
#pragma once

#include "gncsim/core/Config.hpp"
#include "gncsim/env/Environment.hpp"
#include "gncsim/math/Quaternion.hpp"
#include "gncsim/math/Vector3.hpp"

namespace gncsim {

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

 private:
  AeroConfig cfg_;
};

}  // namespace gncsim
