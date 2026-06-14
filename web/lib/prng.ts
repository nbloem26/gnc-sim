/**
 * Seeded deterministic PRNG for the web Monte Carlo batch.
 *
 * The C++ core uses std::mt19937_64 + a hand-rolled Box-Muller (see
 * core/include/gncsim/core/Rng.hpp) so native<->WASM stay bit-identical. The web
 * panel cannot reproduce mt19937_64 bit-for-bit cheaply, but it does NOT need to:
 * the per-case seed we hand to the WASM run still drives the bit-identical sensor
 * noise stream inside the core. All this module has to guarantee is that the web
 * batch is itself reproducible (same master seed -> same dispersions), so we use a
 * small, fast, well-distributed 32-bit generator (mulberry32) plus our own
 * Box-Muller, mirroring the core's Gaussian construction.
 */

/** mulberry32: a compact, deterministic 32-bit PRNG seeded from a single integer. */
export class Prng {
  private state: number;
  private haveSpare = false;
  private spare = 0;

  constructor(seed: number) {
    // Force to an unsigned 32-bit seed so the same `seed` always replays identically.
    this.state = seed >>> 0;
  }

  /** Uniform in [0, 1). */
  uniform(lo = 0, hi = 1): number {
    this.state = (this.state + 0x6d2b79f5) >>> 0;
    let t = this.state;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const u = ((t ^ (t >>> 14)) >>> 0) / 4294967296; // 2^32
    return lo + (hi - lo) * u;
  }

  /** A fresh unsigned-32-bit integer (used to derive per-case WASM seeds). */
  nextUint32(): number {
    return Math.floor(this.uniform(0, 4294967296)) >>> 0;
  }

  /** Box-Muller Gaussian (cached spare), matching the core's construction. */
  gaussian(mean = 0, stddev = 1): number {
    if (this.haveSpare) {
      this.haveSpare = false;
      return mean + stddev * this.spare;
    }
    let u1 = this.uniform(0, 1);
    if (u1 < 1e-300) u1 = 1e-300; // avoid log(0)
    const u2 = this.uniform(0, 1);
    const mag = Math.sqrt(-2 * Math.log(u1));
    const twoPi = 2 * Math.PI;
    this.spare = mag * Math.sin(twoPi * u2);
    this.haveSpare = true;
    return mean + stddev * (mag * Math.cos(twoPi * u2));
  }
}
