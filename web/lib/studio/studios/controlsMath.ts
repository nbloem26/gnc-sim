/**
 * controlsMath.ts — small, hand-rolled linear-systems engine for the
 * Controls / Autopilot Studio (issue #113).
 *
 * This is a *reduced-order* model of the pitch/yaw acceleration autopilot that
 * the C++ core flies in full 6DOF. It exists so the browser can do interactive
 * loop-shaping (Bode / Nyquist / step / actuator saturation) without a WASM
 * round-trip. Everything here is pure math — no engine, no new deps.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * MODEL & ITS PROVENANCE AGAINST THE C++
 * ─────────────────────────────────────────────────────────────────────────────
 *
 * 1. AIRFRAME SHORT-PERIOD  (from core/src/aero/Aero.cpp :: momentAero)
 *
 *    The hi-fi aero applies a static restoring pitch moment  M = q̄·S·d·Cm(α)
 *    with `Cm < 0 for α > 0` => statically stable (see pitchMomentCoeff comment),
 *    plus a rate-damping moment  M_q = (q̄·S·d · d/(2V)) · Cmq · q  that opposes
 *    the body rate (see momentAero's `rate_scale * cfg_.cm_q`). Linearising the
 *    pitch channel about a trim point gives the classic short-period pair:
 *
 *        ωn² = -(q̄·S·d / Iyy) · Cm_alpha          [Cm_alpha < 0 ⇒ ωn² > 0]
 *        2·ζ·ωn = -(q̄·S·d²)/(2·V·Iyy) · Cmq        (the Cmq damping term)
 *
 *    Rather than make the user enter q̄, S, d, Iyy, V (the full 6DOF
 *    linearisation — see FOLLOW-UP), we expose the two *meaningful* physical
 *    knobs a controls engineer actually tunes the airframe with:
 *
 *      • static_margin   — calibers of CG-to-CP separation. Larger SM ⇒ more
 *                          negative Cm_alpha ⇒ a STIFFER airframe ⇒ higher ωn.
 *                          We map  ωn_radps = SHORT_PERIOD_WN_PER_CALIBER · SM.
 *      • the natural damping ζ_airframe is set by Cmq; we hold it at a small
 *        open-loop value (lightly damped airframe) — the autopilot supplies the
 *        rest. The plant pole pair is therefore  ωn(SM), ζ_airframe.
 *
 *    Airframe accel-per-α transfer (the "A_z / δ" path): a stable minimum-phase
 *    2nd-order with DC gain set by control_effectiveness (∝ Cm_delta / Cm_alpha,
 *    the trim deflection-to-accel gain):
 *
 *        G_af(s) = K_af · ωn² / (s² + 2ζ·ωn·s + ωn²)
 *
 * 2. FIN ACTUATOR  (from core/src/gnc/Gnc.cpp :: FinActuator)
 *
 *    The C++ actuator is a first-order lag toward the command (`blend = dt/tau`)
 *    followed by a per-step RATE limit (`max_step_rad = rate_limit*dt`) and a
 *    TRAVEL clamp (`deflection_limit`). Linearly that is just
 *
 *        G_act(s) = 1 / (tau·s + 1),   tau = 1/(2π·bandwidth_hz)
 *
 *    and the rate/travel limits are *nonlinear* — they only bite under a large
 *    step, so they appear in the time-domain actuator-saturation simulation, not
 *    in the linear Bode/Nyquist. The discrete actuator sim below reproduces the
 *    C++ lag→rate-limit→travel-clamp ordering exactly.
 *
 * 3. AUTOPILOT  (from core/src/gnc/Gnc.cpp :: Autopilot::moment — the PD law
 *    `moment = kp·err − kd·angVel`)
 *
 *    The 6DOF autopilot is a PD on attitude error with rate damping. In the
 *    reduced accel loop that is the standard two-loop accel autopilot: an INNER
 *    RATE-FEEDBACK loop (gain Kr — the `kd`/rate-damping analogue, fed back
 *    around the airframe+actuator) wrapped by an OUTER forward ACCEL gain (Ka —
 *    the `kp` analogue). With the plant  P(s) = G_act(s)·G_af(s), closing the
 *    inner rate feedback Kr·s and applying the forward gain Ka gives the
 *    open-loop transfer we shape (loop broken at the accel error):
 *
 *                    Ka · P(s)
 *        L(s) =  ─────────────────────
 *                 1 + Kr·s · P(s)
 *
 *    This is the physically correct structure and it gives BOTH expected
 *    behaviours: Kr (inner rate feedback) adds damping → more phase margin;
 *    cranking Ka (pure forward gain) pushes the gain crossover UP into the
 *    actuator/airframe lag, eroding phase margin → ringing, then instability —
 *    exactly the behaviour the issue asks the margins to exhibit.
 *
 * Units carry SI suffixes per AGENTS.md (e.g. `omega_radps`, `tau_s`, `_deg`).
 */

// ── Complex arithmetic (just enough for frequency response) ──────────────────

export interface Complex {
  re: number;
  im: number;
}

function cAdd(a: Complex, b: Complex): Complex {
  return { re: a.re + b.re, im: a.im + b.im };
}
function cMul(a: Complex, b: Complex): Complex {
  return { re: a.re * b.re - a.im * b.im, im: a.re * b.im + a.im * b.re };
}
function cDiv(a: Complex, b: Complex): Complex {
  const den = b.re * b.re + b.im * b.im;
  return { re: (a.re * b.re + a.im * b.im) / den, im: (a.im * b.re - a.re * b.im) / den };
}
function cAbs(a: Complex): number {
  return Math.hypot(a.re, a.im);
}
/** Phase in radians, continuous via atan2 (caller unwraps). */
function cArg(a: Complex): number {
  return Math.atan2(a.im, a.re);
}

// ── Model knobs → physical constants ─────────────────────────────────────────

/**
 * Maps static margin (calibers) → short-period natural frequency. A 1-caliber
 * static margin gives a ~9 rad/s short-period airframe, a representative value
 * for a tactical interceptor. Linear in SM is the small-perturbation behaviour
 * of ωn² ∝ -Cm_alpha ∝ static margin (CP-CG arm) at fixed q̄.
 */
const SHORT_PERIOD_WN_RADPS_PER_CALIBER = 9.0;
/** Open-loop airframe damping from Cmq alone — a lightly damped airframe. */
const AIRFRAME_ZETA = 0.12;

/** Tunable model parameters (already converted to SI / native units). */
export interface ControlsModel {
  static_margin_caliber: number;
  control_effectiveness: number; // airframe accel DC gain K_af (dimensionless, normalized)
  actuator_tau_s: number; // first-order lag time constant
  actuator_rate_limit_degps: number;
  actuator_travel_limit_deg: number;
  accel_gain_ka: number; // outer accel-loop gain (the kp analogue)
  rate_gain_kr_s: number; // inner rate-loop lead time constant (the kd analogue), seconds
}

/** Airframe short-period natural frequency from the static margin. */
export function airframeOmegaRadps(model: ControlsModel): number {
  return SHORT_PERIOD_WN_RADPS_PER_CALIBER * Math.max(model.static_margin_caliber, 1e-3);
}

// ── Open-loop frequency response  L(jω) ──────────────────────────────────────

/** The bare plant P(jω) = G_act(jω)·G_af(jω) (actuator lag × airframe short-period). */
function plantAt(model: ControlsModel, omega_radps: number): Complex {
  const wn = airframeOmegaRadps(model);
  const s: Complex = { re: 0, im: omega_radps };
  // Actuator: 1/(tau·s + 1)
  const act = cDiv({ re: 1, im: 0 }, { re: 1, im: model.actuator_tau_s * omega_radps });
  // Airframe: K_af·ωn² / (s² + 2ζωn·s + ωn²)
  const num: Complex = { re: model.control_effectiveness * wn * wn, im: 0 };
  const s2 = cMul(s, s);
  const den = cAdd(cAdd(s2, { re: 0, im: 2 * AIRFRAME_ZETA * wn * omega_radps }), { re: wn * wn, im: 0 });
  const airframe = cDiv(num, den);
  return cMul(act, airframe);
}

/**
 * Open-loop transfer (loop broken at the accel error) evaluated at s = jω:
 *
 *   L(s) = Ka·P(s) / (1 + Kr·s·P(s)),   P(s) = G_act(s)·G_af(s)
 *
 * — outer forward accel gain Ka over the inner rate-feedback-closed plant.
 */
export function openLoopAt(model: ControlsModel, omega_radps: number): Complex {
  const P = plantAt(model, omega_radps);
  // Numerator: Ka·P
  const numer = cMul({ re: model.accel_gain_ka, im: 0 }, P);
  // Denominator: 1 + Kr·s·P  (s = jω → Kr·s = j·Kr·ω)
  const KrsP = cMul({ re: 0, im: model.rate_gain_kr_s * omega_radps }, P);
  const denom = cAdd({ re: 1, im: 0 }, KrsP);
  return cDiv(numer, denom);
}

// ── Bode + stability margins ─────────────────────────────────────────────────

export interface BodePoint {
  omega_radps: number;
  gain_db: number;
  phase_deg: number;
}

export interface StabilityMargins {
  bode: BodePoint[];
  /** Gain margin in dB (gain headroom at the −180° phase crossover). +∞ if none. */
  gain_margin_db: number;
  /** Frequency of the phase crossover (where phase = −180°). NaN if none. */
  phase_crossover_radps: number;
  /** Phase margin in deg (phase above −180° at the 0 dB gain crossover). */
  phase_margin_deg: number;
  /** Frequency of the gain crossover (where |L| = 1, i.e. 0 dB). NaN if none. */
  gain_crossover_radps: number;
  /** True if the closed loop is stable (Bode criterion for this minimum-phase L). */
  stable: boolean;
}

/** Logarithmically spaced frequency grid [rad/s]. */
export function logspaceRadps(min_radps: number, max_radps: number, count: number): number[] {
  const out = new Array<number>(count);
  const a = Math.log10(min_radps);
  const b = Math.log10(max_radps);
  for (let i = 0; i < count; i++) out[i] = 10 ** (a + ((b - a) * i) / (count - 1));
  return out;
}

/** Linear interpolation of x where y crosses `target`, between samples i-1 and i. */
function interpCrossing(x0: number, y0: number, x1: number, y1: number, target: number): number {
  const frac = (target - y0) / (y1 - y0);
  // interpolate in log-frequency for accuracy on a log grid
  const lx0 = Math.log10(x0);
  const lx1 = Math.log10(x1);
  return 10 ** (lx0 + frac * (lx1 - lx0));
}

/**
 * Compute the open-loop Bode and extract gain/phase margins.
 *
 * Phase is unwrapped before searching for the −180° crossover. For this
 * minimum-phase L the loop is stable iff the phase margin is positive (phase at
 * the 0 dB crossover lies above −180°); we report that as `stable`.
 */
export function computeMargins(
  model: ControlsModel,
  min_radps = 0.1,
  max_radps = 1000,
  count = 600,
): StabilityMargins {
  const omegas = logspaceRadps(min_radps, max_radps, count);
  const bode: BodePoint[] = [];
  let prevPhase_rad = 0;
  let unwrap_rad = 0;

  for (let i = 0; i < omegas.length; i++) {
    const L = openLoopAt(model, omegas[i]);
    const gain_db = 20 * Math.log10(Math.max(cAbs(L), 1e-300));
    const raw_phase_rad = cArg(L);
    if (i === 0) {
      unwrap_rad = raw_phase_rad; // anchor the unwrap at the first sample's raw phase
    } else {
      let d = raw_phase_rad - prevPhase_rad;
      while (d > Math.PI) d -= 2 * Math.PI;
      while (d < -Math.PI) d += 2 * Math.PI;
      unwrap_rad += d;
    }
    prevPhase_rad = raw_phase_rad;
    bode.push({ omega_radps: omegas[i], gain_db, phase_deg: (unwrap_rad * 180) / Math.PI });
  }

  // Gain crossover: |L| = 0 dB.
  let gain_crossover_radps = NaN;
  let phase_margin_deg = NaN;
  for (let i = 1; i < bode.length; i++) {
    if ((bode[i - 1].gain_db >= 0) !== (bode[i].gain_db >= 0)) {
      gain_crossover_radps = interpCrossing(
        bode[i - 1].omega_radps, bode[i - 1].gain_db,
        bode[i].omega_radps, bode[i].gain_db, 0,
      );
      const phaseFrac = (0 - bode[i - 1].gain_db) / (bode[i].gain_db - bode[i - 1].gain_db);
      const phase_at = bode[i - 1].phase_deg + phaseFrac * (bode[i].phase_deg - bode[i - 1].phase_deg);
      phase_margin_deg = 180 + phase_at; // PM = phase above −180°
      break;
    }
  }

  // Phase crossover: phase = −180°.
  let phase_crossover_radps = NaN;
  let gain_margin_db = Infinity;
  for (let i = 1; i < bode.length; i++) {
    if ((bode[i - 1].phase_deg >= -180) !== (bode[i].phase_deg >= -180)) {
      phase_crossover_radps = interpCrossing(
        bode[i - 1].omega_radps, bode[i - 1].phase_deg,
        bode[i].omega_radps, bode[i].phase_deg, -180,
      );
      const gainFrac = (-180 - bode[i - 1].phase_deg) / (bode[i].phase_deg - bode[i - 1].phase_deg);
      const gain_at = bode[i - 1].gain_db + gainFrac * (bode[i].gain_db - bode[i - 1].gain_db);
      gain_margin_db = -gain_at; // GM = headroom below 0 dB
      break;
    }
  }

  const stable = Number.isNaN(phase_margin_deg) ? true : phase_margin_deg > 0;
  return {
    bode,
    gain_margin_db,
    phase_crossover_radps,
    phase_margin_deg,
    gain_crossover_radps,
    stable,
  };
}

// ── Nyquist (open-loop locus) ────────────────────────────────────────────────

export interface NyquistPoint {
  re: number;
  im: number;
  omega_radps: number;
}

export function computeNyquist(
  model: ControlsModel,
  min_radps = 0.05,
  max_radps = 2000,
  count = 800,
): NyquistPoint[] {
  const omegas = logspaceRadps(min_radps, max_radps, count);
  return omegas.map((w) => {
    const L = openLoopAt(model, w);
    return { re: L.re, im: L.im, omega_radps: w };
  });
}

// ── Closed-loop step response (linear, via bilinear discretization) ──────────

export interface StepResponse {
  t_s: number[];
  accel_response: number[]; // closed-loop accel output to a unit accel command
  rise_time_s: number;
  overshoot_pct: number;
  settling_time_s: number;
  peak: number;
}

/**
 * Closed-loop unit-step response of the linear accel loop, simulated by a
 * state-space realization of  T(s) = L(s)/(1+L(s))  advanced with RK4. We build
 * the loop directly in state space (airframe 2nd-order + actuator 1st-order +
 * the autopilot lead), which avoids forming a high-order polynomial and keeps
 * the same plant the Bode uses.
 *
 *   State x = [a, a_dot, δ]  where a is airframe accel output, δ actuator out.
 *     δ̇      = ( u − δ ) / tau                         (actuator lag)
 *     ä + 2ζωn·ȧ + ωn²·a = K_af·ωn²·δ                  (airframe 2nd order)
 *     u      = Ka·(cmd − a) − Kr·ȧ                     (autopilot: forward accel
 *                                                       gain Ka, inner rate
 *                                                       feedback Kr — the C++
 *                                                       `kp·err − kd·rate` form)
 *   This is the time-domain dual of  L = Ka·P/(1+Kr·s·P): more Ka raises the
 *   loop gain (faster but ringier), more Kr adds damping.
 */
export function computeStepResponse(model: ControlsModel, duration_s = 2.0, dt_s = 0.0005): StepResponse {
  const wn = airframeOmegaRadps(model);
  const tau = Math.max(model.actuator_tau_s, 1e-4);
  const Kaf = model.control_effectiveness;
  const Ka = model.accel_gain_ka;
  const Kr = model.rate_gain_kr_s;
  const cmd = 1.0;

  // x = [a, adot, delta]
  const deriv = (x: number[]): number[] => {
    const a = x[0];
    const adot = x[1];
    const delta = x[2];
    const e = cmd - a;
    const u = Ka * e - Kr * adot; // forward accel gain Ka, inner rate feedback Kr (kp·err − kd·rate)
    const ddelta = (u - delta) / tau;
    const addot = Kaf * wn * wn * delta - 2 * AIRFRAME_ZETA * wn * adot - wn * wn * a;
    return [adot, addot, ddelta];
  };

  const n = Math.max(2, Math.round(duration_s / dt_s));
  const t_s = new Array<number>(n + 1);
  const accel_response = new Array<number>(n + 1);
  let x = [0, 0, 0];
  t_s[0] = 0;
  accel_response[0] = 0;

  for (let i = 1; i <= n; i++) {
    const k1 = deriv(x);
    const k2 = deriv(x.map((xi, j) => xi + 0.5 * dt_s * k1[j]));
    const k3 = deriv(x.map((xi, j) => xi + 0.5 * dt_s * k2[j]));
    const k4 = deriv(x.map((xi, j) => xi + dt_s * k3[j]));
    x = x.map((xi, j) => xi + (dt_s / 6) * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]));
    t_s[i] = i * dt_s;
    accel_response[i] = x[0];
  }

  // Metrics relative to the final (steady-state) value.
  const finalVal = accel_response[accel_response.length - 1];
  const fv = Math.abs(finalVal) < 1e-9 ? 1 : finalVal;
  let peak = -Infinity;
  for (const v of accel_response) if (v > peak) peak = v;
  const overshoot_pct = Math.max(0, ((peak - fv) / fv) * 100);

  // 10%→90% rise time.
  let t10 = NaN;
  let t90 = NaN;
  for (let i = 0; i < accel_response.length; i++) {
    if (Number.isNaN(t10) && accel_response[i] >= 0.1 * fv) t10 = t_s[i];
    if (Number.isNaN(t90) && accel_response[i] >= 0.9 * fv) {
      t90 = t_s[i];
      break;
    }
  }
  const rise_time_s = Number.isNaN(t10) || Number.isNaN(t90) ? NaN : t90 - t10;

  // 2% settling time (last time the response leaves the ±2% band).
  let settling_time_s = NaN;
  for (let i = accel_response.length - 1; i >= 0; i--) {
    if (Math.abs(accel_response[i] - fv) > 0.02 * Math.abs(fv)) {
      settling_time_s = t_s[Math.min(i + 1, t_s.length - 1)];
      break;
    }
  }
  if (Number.isNaN(settling_time_s)) settling_time_s = 0;

  return { t_s, accel_response, rise_time_s, overshoot_pct, settling_time_s, peak };
}

// ── Actuator saturation simulation (the NONLINEAR rate/travel limiting) ───────

export interface ActuatorSim {
  t_s: number[];
  commanded_deg: number[];
  achieved_deg: number[];
  commanded_rate_degps: number[];
  achieved_rate_degps: number[];
  rate_saturated: boolean;
  travel_saturated: boolean;
}

/**
 * Reproduces the C++ FinActuator::step ordering — first-order lag toward the
 * command, then a per-step RATE limit (rate_limit·dt), then a TRAVEL clamp
 * (deflection_limit) — for a commanded step deflection. Shows how a large/fast
 * command gets rate- and travel-limited (the nonlinearity the linear Bode omits).
 *
 * The commanded deflection is a step to `command_deg` (a large fin order, as
 * produced by a hard accel command); we watch the achieved deflection chase it.
 */
export function computeActuatorSim(
  model: ControlsModel,
  command_deg: number,
  duration_s = 0.5,
  dt_s = 0.0005,
): ActuatorSim {
  const tau = Math.max(model.actuator_tau_s, 1e-4);
  const blend = Math.min(dt_s / tau, 1.0); // matches C++ blend = dt/tau
  const max_step_deg = model.actuator_rate_limit_degps * dt_s; // rate_limit*dt
  const travel = model.actuator_travel_limit_deg;
  const cmdClamped = Math.max(-travel, Math.min(travel, command_deg));

  const n = Math.max(2, Math.round(duration_s / dt_s));
  const t_s = new Array<number>(n + 1);
  const commanded_deg = new Array<number>(n + 1);
  const achieved_deg = new Array<number>(n + 1);
  const commanded_rate_degps = new Array<number>(n + 1);
  const achieved_rate_degps = new Array<number>(n + 1);

  let defl = 0;
  let prev = 0;
  let rate_saturated = false;
  let travel_saturated = false;

  for (let i = 0; i <= n; i++) {
    // Step command applied from t=0.
    const cmd = cmdClamped;
    // First-order lag toward the command (C++: target = defl + (cmd-defl)*blend).
    const target = defl + (cmd - defl) * blend;
    // Rate limit the per-step change (C++: clamp d to ±max_step_rad).
    let d = target - defl;
    if (d > max_step_deg) {
      d = max_step_deg;
      rate_saturated = true;
    } else if (d < -max_step_deg) {
      d = -max_step_deg;
      rate_saturated = true;
    }
    let next = defl + d;
    // Travel clamp.
    if (next > travel) {
      next = travel;
      travel_saturated = true;
    } else if (next < -travel) {
      next = -travel;
      travel_saturated = true;
    }

    t_s[i] = i * dt_s;
    commanded_deg[i] = cmd;
    achieved_deg[i] = next;
    // Rates (deg/s): commanded is effectively instantaneous (step) → report the
    // unlimited lag rate; achieved is the realized per-step delta / dt.
    commanded_rate_degps[i] = (target - defl) / dt_s;
    achieved_rate_degps[i] = (next - prev) / dt_s;

    prev = next;
    defl = next;
  }

  return {
    t_s,
    commanded_deg,
    achieved_deg,
    commanded_rate_degps,
    achieved_rate_degps,
    rate_saturated,
    travel_saturated,
  };
}
