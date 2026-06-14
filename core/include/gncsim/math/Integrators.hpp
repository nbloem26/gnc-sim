// gnc-sim — generic fixed-step integrators (RK4, RK2/midpoint, Euler).
// State must support: operator+(State), operator*(double).  Deriv is callable as f(t,
// State)->State.
#pragma once

namespace gncsim {

template <typename State, typename Deriv>
State eulerStep(const State& y, double t, double dt, Deriv f) {
  return y + f(t, y) * dt;
}

// Explicit midpoint (RK2).
template <typename State, typename Deriv>
State rk2Step(const State& y, double t, double dt, Deriv f) {
  const State k1 = f(t, y);
  const State k2 = f(t + 0.5 * dt, y + k1 * (0.5 * dt));
  return y + k2 * dt;
}

// Classic 4th-order Runge-Kutta.
template <typename State, typename Deriv>
State rk4Step(const State& y, double t, double dt, Deriv f) {
  const State k1 = f(t, y);
  const State k2 = f(t + 0.5 * dt, y + k1 * (0.5 * dt));
  const State k3 = f(t + 0.5 * dt, y + k2 * (0.5 * dt));
  const State k4 = f(t + dt, y + k3 * dt);
  return y + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);
}

}  // namespace gncsim
