// gnc-sim — high-fidelity 6DOF rigid-body EOM (issue #35). Full 3x3 inertia tensor with the
// gyroscopic coupling -omega x (I omega) in the rotational equations, integrated together with
// translation and the attitude quaternion under the chosen fixed-step integrator.
//
// This is the rotational counterpart to step6dof, generalized from a scalar inertia proxy to the
// full symmetric inertia tensor:  I*omega_dot = M_body - omega x (I*omega),  q_dot = 0.5*q ⊗ omega.
// The 3x3 inverse is hand-rolled (cofactor/adjugate, in the style of core/src/gnc/Ekf.cpp) so no
// new linear-algebra dependency is introduced and native<->WASM parity is preserved.
//
// Pure: no I/O, no globals, deterministic fixed-FP-order arithmetic.
#pragma once

#include <array>

#include "gncsim/core/Config.hpp"
#include "gncsim/core/Types.hpp"
#include "gncsim/math/Vector3.hpp"

namespace gncsim {

// Symmetric 3x3 inertia tensor with its precomputed inverse. Row-major 9-element storage.
struct InertiaTensor {
  std::array<double, 9> I{};     // tensor (symmetric)
  std::array<double, 9> Iinv{};  // its inverse

  // Build from principal moments + products of inertia. Falls back to a near-singular guard: if the
  // tensor is non-invertible the inverse is set to a diagonal proxy so the EOM stays finite.
  static InertiaTensor fromComponents(double ixx, double iyy, double izz, double ixy, double ixz,
                                      double iyz);

  Vector3 apply(const Vector3& w) const;     // I * w
  Vector3 applyInv(const Vector3& w) const;  // I^-1 * w
};

// Build the inertia tensor from a VehicleConfig (issue #35). A principal moment <= 0 falls back to
// the scalar `inertia` proxy, so the default hi-fi config reproduces a uniform-inertia body and an
// explicit tensor is fully opt-in. Products of inertia default to zero.
InertiaTensor inertiaFromVehicle(const VehicleConfig& v);

// Advance the full 6DOF state one step with the full inertia tensor and gyroscopic coupling.
// `force_world` excludes gravity; `moment_body` is the total body-frame torque [N*m]; `gravity` is
// the acceleration vector [m/s^2]. The quaternion is renormalized after the step.
EntityState step6dofHiFi(const EntityState& s, const Vector3& force_world,
                         const Vector3& moment_body, const InertiaTensor& inertia,
                         const Vector3& gravity, double dt, Integrator integ);

}  // namespace gncsim
