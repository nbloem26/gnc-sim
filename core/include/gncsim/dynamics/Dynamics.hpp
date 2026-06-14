/// @file Dynamics.hpp
/// @brief Rigid-body equations of motion + integration (`3dof`, `6dof`).
///
/// The Runner computes the total external world-frame force (aero + thrust) and gravity, then
/// calls these to advance one step. Phase 1 (dynamics) owns the implementations in
/// core/src/dynamics/. Governing equations: docs/THEORY.md §2.
#pragma once

#include "gncsim/core/Config.hpp"
#include "gncsim/core/Types.hpp"
#include "gncsim/math/Vector3.hpp"

namespace gncsim {

/// @brief Advance translational point-mass state (pos, vel) one step (`3dof`).
/// @param force_world External world-frame force [N], excluding gravity (aero + thrust).
/// @param gravity Gravitational acceleration [m/s²]. Attitude/angular fields pass through
/// unchanged.
EntityState step3dof(const EntityState& s, const Vector3& force_world, const Vector3& gravity,
                     double dt, Integrator integ);

/// @brief Advance full 6DOF state (pos, vel, attitude quaternion, body rate) one step (`6dof`).
///
/// `moment_body` is the control+aero moment [N·m]; `inertia` is a scalar moment-of-inertia proxy
/// (no gyroscopic coupling — that lives in Dynamics6dofHiFi.hpp). The quaternion is renormalized
/// after the step.
EntityState step6dof(const EntityState& s, const Vector3& force_world, const Vector3& moment_body,
                     double inertia, const Vector3& gravity, double dt, Integrator integ);

}  // namespace gncsim
