// gnc-sim — seeded RNG. Deterministic AND identical across native (libstdc++) and WASM (libc++):
// std::mt19937_64 is standardized (same bits everywhere), but std::normal_distribution /
// std::uniform_real_distribution are implementation-defined and diverge between toolchains. So we
// derive uniforms from the raw engine and use our own Box-Muller for Gaussians — same sequence on
// every platform, which is what makes the native<->WASM parity hold even with sensor noise enabled.
#pragma once

#include <cmath>
#include <cstdint>
#include <random>

#include "gncsim/math/Vector3.hpp"

namespace gncsim {

class Rng {
 public:
  explicit Rng(std::uint64_t seed) : engine_(seed) {}

  // 53-bit uniform in [0,1) from the standardized mt19937_64 output, then scaled to [lo,hi).
  double uniform(double lo = 0.0, double hi = 1.0) {
    const double u = static_cast<double>(engine_() >> 11) * (1.0 / 9007199254740992.0);  // 2^53
    return lo + (hi - lo) * u;
  }

  // Box-Muller (cached). Pure arithmetic on our uniforms — no std distribution dependency.
  double gaussian(double mean = 0.0, double stddev = 1.0) {
    if (have_spare_) {
      have_spare_ = false;
      return mean + stddev * spare_;
    }
    double u1 = uniform(0.0, 1.0);
    if (u1 < 1e-300) u1 = 1e-300;  // avoid log(0)
    const double u2 = uniform(0.0, 1.0);
    const double mag = std::sqrt(-2.0 * std::log(u1));
    constexpr double kTwoPi = 6.283185307179586476925286766559;
    spare_ = mag * std::sin(kTwoPi * u2);
    have_spare_ = true;
    return mean + stddev * (mag * std::cos(kTwoPi * u2));
  }

  Vector3 gaussianVec(double stddev) {
    return {gaussian(0.0, stddev), gaussian(0.0, stddev), gaussian(0.0, stddev)};
  }

  std::mt19937_64& engine() { return engine_; }

 private:
  std::mt19937_64 engine_;
  bool have_spare_ = false;
  double spare_ = 0.0;
};

}  // namespace gncsim
