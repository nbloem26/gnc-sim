/**
 * campaignMath — a client-side TypeScript mirror of the many-on-many engagement
 * campaign engine in `core/src/scenario/ManyOnMany.cpp` (issue #45).
 *
 * WHY A MIRROR? The many-on-many campaign is a CLI/SDK-only scenario — it is NOT
 * exposed through the browser WASM `run_sim(configJson)` entry, which only runs a
 * single engagement. So the Campaign / BMC2 Studio (issue #111) cannot `await
 * runSim(...)`; instead it re-implements the campaign math here, in pure
 * deterministic TS, faithful to the C++ logic. Every function below is annotated
 * with the ManyOnMany.cpp construct it mirrors.
 *
 * WHAT WE CANNOT MIRROR: ManyOnMany.cpp gets each pairing's miss distance by
 * running the FULL per-engagement physics (`runSimulation`) for every interceptor
 * x threat pairing (`scorePairings`). We have no physics in the browser, so we
 * synthesize a deterministic per-pairing miss distance from a simple engagement
 * geometry model (range + crossing-angle penalty + a seeded jitter). The Pk model,
 * WTA, doctrines and campaign rollups that consume that miss matrix are mirrored
 * verbatim. The BMC2 datalink degradation (issue #46) — latency + dropout — is an
 * additive overlay this studio adds on top of the C++ math.
 *
 * Units follow the repo convention: physical quantities carry an SI unit suffix in
 * their name (`miss_distance_m`, `sigma_m`, `latency_s`).
 *
 * Determinism: any Monte-Carlo here uses the same algorithm shape as the C++ path
 * (a seeded mt19937-style LCG + uniform draws; a threat needs only one successful
 * shot). No `Math.random()` anywhere.
 */

// ----------------------------------------------------------------------------
// Seeded RNG — a small deterministic uniform generator (mulberry32). Mirrors the
// ROLE of `gncsim::Rng` in ManyOnMany.cpp: derive uniforms in [0,1) from a seeded
// integer engine so results are reproducible for a given seed. (We do not need
// bit-parity with the C++ mt19937_64 — this studio is a planning mirror, not the
// parity-checked core — only determinism.)
// ----------------------------------------------------------------------------

export function makeRng(seed: number): () => number {
  let s = seed >>> 0;
  return function next(): number {
    s |= 0;
    s = (s + 0x6d2b79f5) | 0;
    let t = Math.imul(s ^ (s >>> 15), 1 | s);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// ----------------------------------------------------------------------------
// Lethality — single-shot P(kill) from a miss distance.
// Mirrors `pkillFromMiss()` in ManyOnMany.cpp:
//   Pssk = pk_max * exp(-0.5 * (miss / sigma)^2)
// i.e. the Gaussian / Carleton lethality model (a miss + an effective lethal
// radius `sigma_m`).
// ----------------------------------------------------------------------------

export function pkillFromMiss_m(
  miss_distance_m: number,
  sigma_m: number,
  pk_max: number,
): number {
  const sigma = sigma_m > 1e-9 ? sigma_m : 1e-9;
  const r = miss_distance_m / sigma;
  return pk_max * Math.exp(-0.5 * r * r);
}

// Cumulative kill probability of independent shots: 1 - prod(1 - Pssk_i).
// Mirrors `cumulativePk()` in ManyOnMany.cpp.
export function cumulativePk(pssk: number[]): number {
  let survive = 1.0;
  for (const p of pssk) survive *= 1.0 - p;
  return 1.0 - survive;
}

// ----------------------------------------------------------------------------
// Pairing miss-distance synthesis — the ONE part with no C++ counterpart we can
// call (it would need the physics engine). We build a deterministic miss for each
// interceptor x threat pairing from a coarse engagement geometry:
//
//   - a base miss equal to the sensor/lethality `sigma_m` (so an ideal head-on
//     shot scores ~pk_max),
//   - a crossing penalty that grows with the |interceptor - threat| index offset
//     (interceptors are "best matched" to the threat at their own index; off-
//     diagonal pairings have worse geometry — larger miss),
//   - a small seeded jitter so the matrix is not perfectly structured.
//
// The result feeds pkillFromMiss_m exactly as the C++ `res.miss_distance` does.
// ----------------------------------------------------------------------------

// Worst-case crossing penalty, in units of sigma_m: even a badly-matched pairing
// keeps a workable Pk (pk_max·exp(-0.5·1.2²) ≈ 0.49·pk_max), so surplus
// interceptors backing up a threat (raid/salvo) still contribute. The penalty
// saturates with index offset rather than growing without bound — a stand-in for
// "any interceptor can engage any threat, with geometry-dependent miss".
const MAX_CROSSING_SIGMA = 1.2;

export function pairingMiss_m(
  weapon: number,
  threat: number,
  sigma_m: number,
  rng: () => number,
): number {
  const offset = Math.abs(weapon - threat);
  // Saturating crossing-geometry penalty: tends to MAX_CROSSING_SIGMA·sigma_m as
  // the index mismatch grows (1 - exp(-offset/3) reaches ~0.95 by offset≈9).
  const crossing_m = MAX_CROSSING_SIGMA * sigma_m * (1 - Math.exp(-offset / 3));
  // Seeded jitter in [-0.15, +0.15] sigma, deterministic per call order.
  const jitter_m = (rng() - 0.5) * 0.3 * sigma_m;
  return Math.max(0, crossing_m + jitter_m);
}

// ----------------------------------------------------------------------------
// Pairing P(kill) matrix — mirrors `scorePairings()`. Weapon-major layout:
// matrix[weapon][threat]. Built once per campaign; the seeded RNG makes it
// reproducible for a given seed.
// ----------------------------------------------------------------------------

export interface PairingMatrix {
  ni: number;
  nt: number;
  // pk[weapon][threat] — single-shot P(kill).
  pk: number[][];
  // miss[weapon][threat] — synthesized miss distance [m] (for inspection/plots).
  miss_m: number[][];
}

export function scorePairings(
  ni: number,
  nt: number,
  sigma_m: number,
  pk_max: number,
  seed: number,
): PairingMatrix {
  const rng = makeRng(seed);
  const pk: number[][] = [];
  const miss_m: number[][] = [];
  for (let w = 0; w < ni; w++) {
    const pkRow: number[] = [];
    const missRow: number[] = [];
    for (let t = 0; t < nt; t++) {
      const m = pairingMiss_m(w, t, sigma_m, rng);
      missRow.push(m);
      pkRow.push(pkillFromMiss_m(m, sigma_m, pk_max));
    }
    pk.push(pkRow);
    miss_m.push(missRow);
  }
  return { ni, nt, pk, miss_m };
}

// ----------------------------------------------------------------------------
// Weapon-target assignment (WTA) — mirrors `assignWeapons()` with the greedy
// method: repeatedly commit the highest-P(kill) remaining (weapon, threat)
// pairing, one weapon per threat per wave. Deterministic: ties broken by lowest
// threat then weapon index (strictly-greater comparison keeps the first found).
//
// `available` = interceptor indices still in inventory; `threats` = threat indices
// still alive. Returns at most one assignment per threat and per weapon (one wave).
// ----------------------------------------------------------------------------

export interface WeaponAssignment {
  interceptor_index: number;
  threat_index: number;
  p_kill: number;
}

export function assignWeaponsGreedy(
  matrix: PairingMatrix,
  available: number[],
  threats: number[],
): WeaponAssignment[] {
  const result: WeaponAssignment[] = [];
  if (available.length === 0 || threats.length === 0) return result;

  const weaponUsed = new Array<boolean>(available.length).fill(false);
  const threatUsed = new Set<number>();
  const assignable = Math.min(available.length, threats.length);

  for (let picked = 0; picked < assignable; picked++) {
    let bestPk = -1.0;
    let bestWi = -1;
    let bestT = -1;
    for (let wi = 0; wi < available.length; wi++) {
      if (weaponUsed[wi]) continue;
      const w = available[wi];
      for (const t of threats) {
        if (threatUsed.has(t)) continue;
        const pk = matrix.pk[w][t];
        // Strictly-greater keeps the first (lowest-index) pairing on ties.
        if (pk > bestPk) {
          bestPk = pk;
          bestWi = wi;
          bestT = t;
        }
      }
    }
    if (bestWi < 0 || bestT < 0) break;
    weaponUsed[bestWi] = true;
    threatUsed.add(bestT);
    result.push({
      interceptor_index: available[bestWi],
      threat_index: bestT,
      p_kill: bestPk,
    });
  }
  // Stable, deterministic order by (threat, weapon).
  result.sort((a, b) =>
    a.threat_index !== b.threat_index
      ? a.threat_index - b.threat_index
      : a.interceptor_index - b.interceptor_index,
  );
  return result;
}

// ----------------------------------------------------------------------------
// BMC2 datalink degradation (issue #46) — an overlay this studio adds on top of
// the C++ Pk. A cued/command-link interceptor whose track updates arrive late
// (latency_s) or are lost (dropout_prob) flies on a staler estimate, so its
// effective single-shot P(kill) degrades. Model:
//
//   pk_eff = pk * (1 - dropout_prob) * exp(-latency_s / TAU_S)
//
// (1 - dropout_prob): a dropped command-link cue means the shot is uncued/wasted.
//   exp(-latency_s / TAU_S): stale-track degradation with a fixed link time
//   constant. Both factors are 1 at the ideal datalink (no latency, no dropout),
//   so this reduces to the unaltered C++ Pk when the link is perfect.
// ----------------------------------------------------------------------------

export const DATALINK_TAU_S = 2.0; // link staleness time constant [s]

export function degradePk(
  pk: number,
  latency_s: number,
  dropout_prob: number,
): number {
  const linkUp = 1.0 - Math.min(Math.max(dropout_prob, 0), 1);
  const stale = Math.exp(-Math.max(latency_s, 0) / DATALINK_TAU_S);
  return pk * linkUp * stale;
}

/** Apply the datalink degradation to every entry of a pairing matrix (copy). */
export function degradeMatrix(
  matrix: PairingMatrix,
  latency_s: number,
  dropout_prob: number,
): PairingMatrix {
  const pk = matrix.pk.map((row) =>
    row.map((p) => degradePk(p, latency_s, dropout_prob)),
  );
  return { ni: matrix.ni, nt: matrix.nt, pk, miss_m: matrix.miss_m };
}

// ----------------------------------------------------------------------------
// Campaign play-out — mirrors `runManyOnMany()` for the three doctrines, plus the
// deterministic (expected-value) rollup and the seeded Monte-Carlo rollup.
// ----------------------------------------------------------------------------

export type Doctrine = 'salvo' | 'shoot_look_shoot' | 'raid';

export interface CampaignParams {
  ni: number;
  nt: number;
  doctrine: Doctrine;
  shots_per_threat: number; // salvo
  max_waves: number; // shoot-look-shoot
  num_trials: number; // MC (1 => deterministic only)
  seed: number;
}

export interface ThreatOutcome {
  threat_index: number;
  shots_committed: number;
  cumulative_pk: number;
  killed: boolean; // deterministic assessment: cumulative_pk >= 0.5
}

export interface CampaignResult {
  num_interceptors: number;
  num_threats: number;
  doctrine: Doctrine;
  assignments: WeaponAssignment[];
  threats: ThreatOutcome[];
  interceptors_expended: number;
  leakers: number; // deterministic surviving-threat count
  expected_leakage: number; // sum of per-threat survival probability
  expected_kills: number;
  p_raid_annihilation: number; // product of per-threat kill probabilities
  mean_leakage: number; // MC mean surviving threats (== expected if num_trials<=1)
  mc_p_annihilation: number; // MC P(all killed) (== p_raid_annihilation if <=1)
}

export function runCampaign(
  matrix: PairingMatrix,
  p: CampaignParams,
): CampaignResult {
  const nt = p.nt;
  const r: CampaignResult = {
    num_interceptors: p.ni,
    num_threats: nt,
    doctrine: p.doctrine,
    assignments: [],
    threats: [],
    interceptors_expended: 0,
    leakers: 0,
    expected_leakage: 0,
    expected_kills: 0,
    p_raid_annihilation: 0,
    mean_leakage: 0,
    mc_p_annihilation: 0,
  };
  if (nt === 0) return r;

  // Per-threat accumulated single-shot P(kill)s across all committed shots.
  const threatShots: number[][] = Array.from({ length: nt }, () => []);
  const threatShotCount = new Array<number>(nt).fill(0);

  // Inventory of interceptor indices still available to fire.
  let inventory: number[] = [];
  for (let w = 0; w < p.ni; w++) inventory.push(w);

  const commit = (a: WeaponAssignment) => {
    r.assignments.push(a);
    threatShots[a.threat_index].push(a.p_kill);
    threatShotCount[a.threat_index] += 1;
    r.interceptors_expended += 1;
    inventory = inventory.filter((w) => w !== a.interceptor_index);
  };

  const allThreats = Array.from({ length: nt }, (_, t) => t);

  if (p.doctrine === 'salvo') {
    // Salvo: commit shots_per_threat interceptors to EACH threat (up to inventory).
    // Each round is one WTA wave (one weapon per threat); repeat shots rounds.
    const shots = Math.max(p.shots_per_threat, 1);
    for (let round = 0; round < shots && inventory.length > 0; round++) {
      const wave = assignWeaponsGreedy(matrix, inventory, allThreats);
      if (wave.length === 0) break;
      for (const a of wave) commit(a);
    }
  } else if (p.doctrine === 'shoot_look_shoot') {
    // SLS: each wave fires one weapon per SURVIVING threat (cumulative Pk < 0.5),
    // assess, re-engage survivors next wave, up to max_waves.
    const waves = Math.max(p.max_waves, 1);
    for (let wave = 0; wave < waves && inventory.length > 0; wave++) {
      const survivors: number[] = [];
      for (let t = 0; t < nt; t++) {
        if (cumulativePk(threatShots[t]) < 0.5) survivors.push(t);
      }
      if (survivors.length === 0) break;
      const assignment = assignWeaponsGreedy(matrix, inventory, survivors);
      if (assignment.length === 0) break;
      for (const a of assignment) commit(a);
    }
  } else {
    // Raid: finite inventory vs M threats. Repeat WTA waves until inventory is
    // exhausted so surplus interceptors back up the highest-value threats.
    while (inventory.length > 0) {
      const wave = assignWeaponsGreedy(matrix, inventory, allThreats);
      if (wave.length === 0) break;
      for (const a of wave) commit(a);
    }
  }

  // --- Per-threat outcomes + deterministic (expected-value) metrics ---
  for (let t = 0; t < nt; t++) {
    const cum = cumulativePk(threatShots[t]);
    const killed = cum >= 0.5;
    r.threats.push({
      threat_index: t,
      shots_committed: threatShotCount[t],
      cumulative_pk: cum,
      killed,
    });
    if (!killed) r.leakers += 1;
    r.expected_kills += cum;
    r.expected_leakage += 1.0 - cum;
  }
  // P(raid annihilation) = product of per-threat kill probabilities.
  r.p_raid_annihilation = 1.0;
  for (let t = 0; t < nt; t++) {
    r.p_raid_annihilation *= r.threats[t].cumulative_pk;
  }

  // --- Monte-Carlo rollup (num_trials > 1): sample each shot's Bernoulli kill.
  if (p.num_trials > 1) {
    const rng = makeRng(p.seed ^ 0x9e3779b9);
    let annihilations = 0;
    let totalLeakers = 0;
    for (let trial = 0; trial < p.num_trials; trial++) {
      let trialLeakers = 0;
      for (let t = 0; t < nt; t++) {
        let killed = false;
        for (const pssk of threatShots[t]) {
          if (rng() < pssk) {
            killed = true;
            break; // a threat needs only one successful shot
          }
        }
        if (!killed) trialLeakers += 1;
      }
      totalLeakers += trialLeakers;
      if (trialLeakers === 0) annihilations += 1;
    }
    r.mean_leakage = totalLeakers / p.num_trials;
    r.mc_p_annihilation = annihilations / p.num_trials;
  } else {
    r.mean_leakage = r.expected_leakage;
    r.mc_p_annihilation = r.p_raid_annihilation;
  }

  return r;
}
