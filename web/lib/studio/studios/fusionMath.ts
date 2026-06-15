/**
 * fusionMath.ts — pure client-side linear algebra + sensor-fusion models for the
 * Fusion / Tracking Studio (issue #106). No engine call, no external deps:
 * hand-rolled small (2x2 / 3x3) matrices.
 *
 * The geometry mirrors the C++ EKF measurement model in
 * `core/src/gnc/TargetTrackEkf.cpp` (az/el/range), specialised to the 2D plane
 * (x, y) so the studio can show position covariance ellipses and a GDOP map:
 *
 *   - bearing_rad = atan2(ry, rx),  ∂bearing/∂pos = [ −ry/ρ²,  rx/ρ² ]
 *   - range_m     = ρ = √(rx²+ry²), ∂range/∂pos   = [  rx/ρ ,  ry/ρ  ]
 *
 * where (rx, ry) = target − sensor.  We build each sensor's measurement Jacobian
 * H (rows = active measurements, cols = [x_m, y_m]) and diagonal noise R, then
 * fuse in **information form**:
 *
 *   Fisher information  I = Σ_s  Hₛᵀ Rₛ⁻¹ Hₛ        (additive across sensors)
 *   fused covariance    P = I⁻¹                      (Cramér–Rao lower bound)
 *
 * Units carried in names: *_m metres, *_rad radians, *_m2 metres² (variance).
 */

// ----------------------------------------------------------------------------
// Types
// ----------------------------------------------------------------------------

export type SensorKind = 'radar' | 'ir';

/** One placed sensor in the 2D plane. */
export interface Sensor {
  id: string;
  kind: SensorKind;
  x_m: number;
  y_m: number;
  /** 1σ angular (bearing) noise, radians. Both radar and IR have this. */
  sigmaAngle_rad: number;
  /**
   * 1σ range noise, metres. Radar measures range directly (small σ); IR is
   * angles-only and gets a deliberately weak range (large σ) so its row is
   * effectively bearing-only in the information sum.
   */
  sigmaRange_m: number;
}

/** Symmetric 2x2 covariance, row-major [a, b; b, c]. */
export interface Cov2 {
  xx_m2: number;
  xy_m2: number;
  yy_m2: number;
}

/** 1σ error-ellipse geometry derived from a 2x2 covariance. */
export interface Ellipse {
  cx_m: number;
  cy_m: number;
  /** Semi-axis lengths (1σ), metres. a ≥ b. */
  a_m: number;
  b_m: number;
  /** Orientation of the major (a) axis, radians, measured from +x. */
  angle_rad: number;
}

// ----------------------------------------------------------------------------
// Small dense linear algebra (2x2)
// ----------------------------------------------------------------------------

/** 2x2 symmetric inverse. Returns null if singular (det ≈ 0). */
export function invSym2(m: Cov2): Cov2 | null {
  const det = m.xx_m2 * m.yy_m2 - m.xy_m2 * m.xy_m2;
  if (!Number.isFinite(det) || Math.abs(det) < 1e-30) return null;
  const inv = 1 / det;
  return {
    xx_m2: m.yy_m2 * inv,
    xy_m2: -m.xy_m2 * inv,
    yy_m2: m.xx_m2 * inv,
  };
}

/** Eigenvalues / eigenvectors of a symmetric 2x2 → 1σ ellipse axes. */
export function covToEllipse(cov: Cov2, cx_m: number, cy_m: number): Ellipse {
  const a = cov.xx_m2;
  const b = cov.xy_m2;
  const c = cov.yy_m2;
  // Eigenvalues of [[a,b],[b,c]].
  const tr = a + c;
  const disc = Math.sqrt(Math.max(0, ((a - c) * (a - c)) / 4 + b * b));
  const lambda1 = tr / 2 + disc; // larger
  const lambda2 = tr / 2 - disc; // smaller
  // Eigenvector for the larger eigenvalue.
  let angle_rad: number;
  if (Math.abs(b) < 1e-30) {
    angle_rad = a >= c ? 0 : Math.PI / 2;
  } else {
    angle_rad = Math.atan2(lambda1 - a, b);
  }
  return {
    cx_m,
    cy_m,
    a_m: Math.sqrt(Math.max(0, lambda1)),
    b_m: Math.sqrt(Math.max(0, lambda2)),
    angle_rad,
  };
}

/** Polyline (closed) tracing a 1σ ellipse, for plotting. */
export function ellipsePolyline(
  e: Ellipse,
  points = 96,
): { x_m: number[]; y_m: number[] } {
  const x_m = new Array<number>(points + 1);
  const y_m = new Array<number>(points + 1);
  const ca = Math.cos(e.angle_rad);
  const sa = Math.sin(e.angle_rad);
  for (let i = 0; i <= points; i++) {
    const t = (i / points) * 2 * Math.PI;
    const px = e.a_m * Math.cos(t);
    const py = e.b_m * Math.sin(t);
    x_m[i] = e.cx_m + px * ca - py * sa;
    y_m[i] = e.cy_m + px * sa + py * ca;
  }
  return { x_m, y_m };
}

// ----------------------------------------------------------------------------
// Per-sensor Fisher information about a target position
// ----------------------------------------------------------------------------

/**
 * Information matrix Hᵀ R⁻¹ H for a single sensor observing a target at
 * (tx_m, ty_m). Each row of H is one scalar measurement; R is diagonal so the
 * contribution is just  Σ_meas (1/σ²) · (∂meas/∂pos)(∂meas/∂pos)ᵀ.
 *
 * Returns an additive 2x2 information contribution (units 1/m²).
 */
export function sensorInformation(
  s: Sensor,
  tx_m: number,
  ty_m: number,
): Cov2 {
  const rx = tx_m - s.x_m;
  const ry = ty_m - s.y_m;
  const rho2 = rx * rx + ry * ry;
  const rho = Math.sqrt(rho2);

  let Ixx = 0;
  let Ixy = 0;
  let Iyy = 0;

  if (rho > 1e-9) {
    // Bearing row: h_b = [−ry/ρ², rx/ρ²], weight 1/σθ².
    const wAng = 1 / (s.sigmaAngle_rad * s.sigmaAngle_rad);
    const hbx = -ry / rho2;
    const hby = rx / rho2;
    Ixx += wAng * hbx * hbx;
    Ixy += wAng * hbx * hby;
    Iyy += wAng * hby * hby;

    // Range row: h_r = [rx/ρ, ry/ρ], weight 1/σr². IR's huge σr makes this ~0.
    const wRng = 1 / (s.sigmaRange_m * s.sigmaRange_m);
    const hrx = rx / rho;
    const hry = ry / rho;
    Ixx += wRng * hrx * hrx;
    Ixy += wRng * hrx * hry;
    Iyy += wRng * hry * hry;
  }

  return { xx_m2: Ixx, xy_m2: Ixy, yy_m2: Iyy };
}

/** Sum of per-sensor information contributions. */
export function fusedInformation(
  sensors: Sensor[],
  tx_m: number,
  ty_m: number,
): Cov2 {
  let xx = 0;
  let xy = 0;
  let yy = 0;
  for (const s of sensors) {
    const I = sensorInformation(s, tx_m, ty_m);
    xx += I.xx_m2;
    xy += I.xy_m2;
    yy += I.yy_m2;
  }
  return { xx_m2: xx, xy_m2: xy, yy_m2: yy };
}

/**
 * Position covariance from an information matrix: P = I⁻¹. If I is singular
 * (e.g. a single bearing-only sensor — unobservable along the line of sight),
 * returns null.
 */
export function covFromInformation(info: Cov2): Cov2 | null {
  return invSym2(info);
}

/** Position-error magnitude √trace(P), metres — the scalar GDOP-style metric. */
export function positionSigma_m(cov: Cov2): number {
  return Math.sqrt(Math.max(0, cov.xx_m2 + cov.yy_m2));
}

/**
 * GDOP-map value at one grid point: √trace(P) where P = (Σ HᵀR⁻¹H)⁻¹. Returns a
 * large sentinel where the geometry is singular/ill-conditioned so the heatmap
 * still renders a finite "bad geometry" region rather than holes.
 */
export function gdopAt(
  sensors: Sensor[],
  tx_m: number,
  ty_m: number,
  cap_m = 1e4,
): number {
  const info = fusedInformation(sensors, tx_m, ty_m);
  const cov = covFromInformation(info);
  if (!cov) return cap_m;
  const sig = positionSigma_m(cov);
  return Number.isFinite(sig) ? Math.min(sig, cap_m) : cap_m;
}

// ----------------------------------------------------------------------------
// Track purity vs clutter / decoy density (PDA-style association model)
// ----------------------------------------------------------------------------

/**
 * Probability that the validated/associated detection is the true target, as a
 * function of clutter (false-alarm + decoy) spatial density. This mirrors the
 * qualitative #38 result: purity degrades as false returns crowd the gate.
 *
 * Model (documented, deliberately simple — a 2D PDA gate):
 *   - The validation gate is an ellipse of area  A_gate = π · γ · |S|^{1/2}
 *     where γ is the gate size (χ² threshold) and |S| ≈ (positionSigma)² is the
 *     innovation-covariance "footprint". We expose A_gate directly as a tunable
 *     gate area so the studio stays self-contained.
 *   - Expected number of false returns in the gate:  μ = ρ_clutter · A_gate.
 *   - With detection probability P_D, the true measurement is present w.p. P_D.
 *     Under the parametric (Poisson clutter) PDA association weights, the
 *     posterior probability the *true* return is picked is approximately
 *
 *        purity = P_D / ( P_D + μ · (1 − P_D·P_G) ... )   →   simplified to
 *        purity ≈ P_D / ( P_D + (1 − P_D) + μ ) ...
 *
 *     We use the clean, monotonic closed form that matches the PDA β₀ weight:
 *
 *        purity = P_D / ( P_D + μ )                                  (P_G≈1)
 *
 *     i.e. the true detection competes against μ Poisson-distributed false ones,
 *     each a priori equally likely inside the gate. purity→1 as μ→0 and
 *     →P_D/(P_D+μ) decay as clutter grows. This is the canonical PDA "all
 *     weights equal inside the gate" limit and reproduces the #38 trend.
 */
export function trackPurity(
  clutterDensity_perM2: number,
  gateArea_m2: number,
  detectionProb: number,
): number {
  const mu = Math.max(0, clutterDensity_perM2) * Math.max(0, gateArea_m2);
  const pd = Math.min(1, Math.max(1e-3, detectionProb));
  return pd / (pd + mu);
}
