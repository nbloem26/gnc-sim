/// @file Integrators.hpp
/// @brief Generic fixed-step ODE integrators (RK4, RK2/midpoint, Euler).
///
/// `State` must support `operator+(State)` and `operator*(double)`; `Deriv` is callable as
/// `f(t, State) -> State`. Fixed step (not adaptive) is a determinism requirement — see
/// docs/THEORY.md §2.1. RK4 is the default.
#pragma once

namespace gncsim {

/// @brief One forward-Euler step, @f$O(\Delta t)@f$.
template <typename State, typename Deriv>
State eulerStep(const State& y, double t, double dt, Deriv f) {
  return y + f(t, y) * dt;
}

/// @brief One explicit-midpoint (RK2) step, @f$O(\Delta t^2)@f$.
template <typename State, typename Deriv>
State rk2Step(const State& y, double t, double dt, Deriv f) {
  const State k1 = f(t, y);
  const State k2 = f(t + 0.5 * dt, y + k1 * (0.5 * dt));
  return y + k2 * dt;
}

/// @brief One classic 4th-order Runge-Kutta step, global error @f$O(\Delta t^4)@f$ (the default).
template <typename State, typename Deriv>
State rk4Step(const State& y, double t, double dt, Deriv f) {
  const State k1 = f(t, y);
  const State k2 = f(t + 0.5 * dt, y + k1 * (0.5 * dt));
  const State k3 = f(t + 0.5 * dt, y + k2 * (0.5 * dt));
  const State k4 = f(t + dt, y + k3 * dt);
  return y + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);
}

}  // namespace gncsim
