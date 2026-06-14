// gnc-sim — rigid-body equations of motion + integration. The Runner computes total external
// world-frame force (aero + thrust) and gravity, then calls these to advance one step.
// Phase 1 (dynamics) owns the implementations in core/src/dynamics/.
#pragma once

#include "gncsim/core/Config.hpp"
#include "gncsim/core/Types.hpp"
#include "gncsim/math/Vector3.hpp"

namespace gncsim {

// Advance translational state (pos, vel) one step. `force_world` excludes gravity; `gravity`
// is the acceleration vector [m/s^2]. Attitude/angular fields are passed through unchanged.
EntityState step3dof(const EntityState& s, const Vector3& force_world, const Vector3& gravity,
                     double dt, Integrator integ);

// Advance full 6DOF state (pos, vel, attitude quaternion, body angular rate). `moment_body` is
// the control+aero moment [N*m]; `inertia` is a scalar moment-of-inertia proxy. Quaternion is
// renormalized after the step.
EntityState step6dof(const EntityState& s, const Vector3& force_world, const Vector3& moment_body,
                     double inertia, const Vector3& gravity, double dt, Integrator integ);

}  // namespace gncsim
