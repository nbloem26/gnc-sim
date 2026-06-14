// gnc-sim — many-on-many engagement campaign (issue #45). See ManyOnMany.hpp for the contract.
//
// The campaign is built in three layers, each pure and deterministic:
//
//   1) scorePairings(): for every interceptor x threat pairing, synthesize a single-engagement
//      SimConfig (the interceptor's launch spec -> cfg.vehicle, the threat spec -> cfg.target) and
//      run the SAME runSimulation() that drives every other scenario. The analytic CPA miss
//      distance is turned into a single-shot P(kill) by a Gaussian lethality (Carleton) model
//        Pssk = pk_max * exp(-0.5 * (miss / pk_sigma_m)^2).
//      Because each pairing is an independent runSimulation, the per-engagement physics is reused
//      verbatim and the default path is untouched.
//
//   2) assignWeapons(): a deterministic weapon-target assignment (WTA) that maximizes expected
//      kills. Greedy commits the highest-marginal-Pk pairing first; the auction variant is a
//      Bertsekas-style ascending-price auction over the same Pk matrix. Both produce at most one
//      weapon per threat per call (one wave); the salvo doctrine calls it repeatedly.
//
//   3) runManyOnMany(): plays out the chosen doctrine (salvo / shoot-look-shoot / raid) on top of
//      the assignment, then rolls up campaign metrics: leakage, interceptors expended, and the
//      probability of raid annihilation. With num_trials>1 it additionally estimates mean leakage
//      and P(annihilation) by Monte-Carlo sampling each committed shot's Bernoulli kill from a
//      seeded project Rng (no std distribution -> native<->WASM identical).
#include "gncsim/scenario/ManyOnMany.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "gncsim/core/Rng.hpp"
#include "gncsim/scenario/Runner.hpp"

namespace gncsim {

namespace {

// Single-shot P(kill) from a pairing's miss distance via the Gaussian lethality (Carleton) model.
double pkillFromMiss(double miss_distance_m, double pk_sigma_m, double pk_max) {
  const double sigma = pk_sigma_m > 1e-9 ? pk_sigma_m : 1e-9;
  const double r = miss_distance_m / sigma;
  return pk_max * std::exp(-0.5 * r * r);
}

// Cumulative kill probability of independent shots: 1 - prod(1 - pssk_i).
double cumulativePk(const std::vector<double>& pssk) {
  double survive = 1.0;
  for (const double p : pssk) survive *= (1.0 - p);
  return 1.0 - survive;
}

// Build the per-pairing single-engagement SimConfig: copy the base config but override the vehicle
// (interceptor launch spec) and target (threat spec), and disable the campaign / Monte-Carlo blocks
// so the inner run is a single deterministic engagement.
SimConfig pairingConfig(const SimConfig& base, const ManyInterceptorSpec& weapon,
                        const ManyThreatSpec& threat) {
  SimConfig c = base;
  c.many_on_many.enabled = false;  // inner run is a plain single engagement
  c.monte_carlo.num_cases = 0;

  c.vehicle.pos0 = weapon.pos0_m;
  c.vehicle.launch_speed = weapon.launch_speed_mps;
  c.vehicle.launch_elevation_deg = weapon.launch_elevation_deg;
  c.vehicle.launch_azimuth_deg = weapon.launch_azimuth_deg;

  c.target.pos0 = threat.pos0_m;
  c.target.vel0 = threat.vel0_mps;
  c.target.maneuver = threat.maneuver;
  c.target.maneuver_g = threat.maneuver_g;
  c.target.maneuver_freq = threat.maneuver_freq_hz;
  return c;
}

// Look up the scored single-shot P(kill) for a given (interceptor, threat) pairing.
double pairPk(const std::vector<PairingPkill>& pairings, int n_threats, int weapon, int threat) {
  // pairings is laid out weapon-major: index = weapon * n_threats + threat.
  const std::size_t idx = static_cast<std::size_t>(weapon) * static_cast<std::size_t>(n_threats) +
                          static_cast<std::size_t>(threat);
  return (idx < pairings.size()) ? pairings[idx].p_kill : 0.0;
}

}  // namespace

std::vector<PairingPkill> scorePairings(const SimConfig& cfg) {
  const ManyOnManyConfig& mm = cfg.many_on_many;
  std::vector<PairingPkill> out;
  const int ni = static_cast<int>(mm.interceptors.size());
  const int nt = static_cast<int>(mm.threats.size());
  out.reserve(static_cast<std::size_t>(ni) * static_cast<std::size_t>(nt));

  for (int w = 0; w < ni; ++w) {
    for (int t = 0; t < nt; ++t) {
      const SimConfig pc = pairingConfig(cfg, mm.interceptors[static_cast<std::size_t>(w)],
                                         mm.threats[static_cast<std::size_t>(t)]);
      const SimResult res = runSimulation(pc);
      PairingPkill p;
      p.interceptor_index = w;
      p.threat_index = t;
      p.miss_distance_m = res.miss_distance;
      p.p_kill = pkillFromMiss(res.miss_distance, mm.pk_sigma_m, mm.pk_max);
      out.push_back(p);
    }
  }
  return out;
}

std::vector<WeaponAssignment> assignWeapons(const std::vector<PairingPkill>& pairings,
                                            const std::vector<int>& available,
                                            const std::vector<int>& threats,
                                            const std::string& method) {
  std::vector<WeaponAssignment> result;
  if (available.empty() || threats.empty() || pairings.empty()) return result;

  // Recover the matrix shape from the (weapon-major) pairing list: n_threats = max threat index
  // + 1.
  int n_threats = 0;
  for (const PairingPkill& p : pairings) n_threats = std::max(n_threats, p.threat_index + 1);
  if (n_threats == 0) return result;

  if (method == "auction") {
    // Bertsekas ascending-price auction: each available weapon bids for the threat that maximizes
    // (value - price); prices rise by the bidding gap + epsilon so the assignment converges to a
    // (near-)optimal max-weight matching. Value here is the pairing P(kill). Deterministic: bidders
    // and threats are iterated in index order, ties broken by lowest index. At most one weapon per
    // threat (and per weapon). Epsilon-scaled for guaranteed termination.
    std::vector<double> price(static_cast<std::size_t>(n_threats), 0.0);
    std::vector<int> threat_owner(static_cast<std::size_t>(n_threats), -1);  // weapon idx or -1
    std::vector<bool> is_threat(static_cast<std::size_t>(n_threats), false);
    for (const int t : threats) {
      if (t >= 0 && t < n_threats) is_threat[static_cast<std::size_t>(t)] = true;
    }
    const double eps = 1.0 / (static_cast<double>(threats.size()) + 1.0);

    std::vector<int> unassigned(available);  // weapons still needing a threat
    int guard = 0;
    const int max_iter = static_cast<int>(available.size()) * (n_threats + 1) * 64 + 64;
    while (!unassigned.empty() && guard++ < max_iter) {
      const int w = unassigned.back();
      // Best and second-best (value - price) over the still-eligible threats.
      double best_net = -1e18, second_net = -1e18;
      int best_t = -1;
      for (const int t : threats) {
        const double net = pairPk(pairings, n_threats, w, t) - price[static_cast<std::size_t>(t)];
        if (net > best_net) {
          second_net = best_net;
          best_net = net;
          best_t = t;
        } else if (net > second_net) {
          second_net = net;
        }
      }
      if (best_t < 0) {  // no eligible threat for this weapon: drop it
        unassigned.pop_back();
        continue;
      }
      unassigned.pop_back();
      // Bid: raise this threat's price by the (best - second) gap + epsilon.
      const double bid = (best_net - second_net) + eps;
      price[static_cast<std::size_t>(best_t)] += bid;
      const int prev = threat_owner[static_cast<std::size_t>(best_t)];
      threat_owner[static_cast<std::size_t>(best_t)] = w;
      if (prev >= 0) unassigned.push_back(prev);  // displaced weapon re-bids
    }

    for (int t = 0; t < n_threats; ++t) {
      const int w = threat_owner[static_cast<std::size_t>(t)];
      if (w >= 0 && is_threat[static_cast<std::size_t>(t)]) {
        WeaponAssignment a;
        a.interceptor_index = w;
        a.threat_index = t;
        a.p_kill = pairPk(pairings, n_threats, w, t);
        result.push_back(a);
      }
    }
    // Stable, deterministic order by (threat, weapon).
    std::sort(result.begin(), result.end(),
              [](const WeaponAssignment& x, const WeaponAssignment& y) {
                if (x.threat_index != y.threat_index) return x.threat_index < y.threat_index;
                return x.interceptor_index < y.interceptor_index;
              });
    return result;
  }

  // Greedy WTA (default): repeatedly commit the highest-P(kill) remaining (weapon, threat) pairing,
  // one weapon per threat. Deterministic: ties broken by lowest threat then weapon index. This
  // maximizes the sum of committed P(kill) under the one-per-threat, one-per-weapon constraint when
  // P(kill) is the marginal kill value (each threat gets at most one shot in a wave).
  std::vector<bool> weapon_used(available.size(), false);
  std::vector<bool> threat_used(static_cast<std::size_t>(n_threats), false);
  const int assignable = std::min(available.size(), threats.size());
  for (int picked = 0; picked < assignable; ++picked) {
    double best_pk = -1.0;
    int best_wi = -1, best_t = -1;  // best_wi indexes into `available`
    for (std::size_t wi = 0; wi < available.size(); ++wi) {
      if (weapon_used[wi]) continue;
      const int w = available[wi];
      for (const int t : threats) {
        if (threat_used[static_cast<std::size_t>(t)]) continue;
        const double pk = pairPk(pairings, n_threats, w, t);
        // Strictly-greater keeps the first (lowest-index) pairing on ties -> deterministic.
        if (pk > best_pk) {
          best_pk = pk;
          best_wi = static_cast<int>(wi);
          best_t = t;
        }
      }
    }
    if (best_wi < 0 || best_t < 0) break;
    weapon_used[static_cast<std::size_t>(best_wi)] = true;
    threat_used[static_cast<std::size_t>(best_t)] = true;
    WeaponAssignment a;
    a.interceptor_index = available[static_cast<std::size_t>(best_wi)];
    a.threat_index = best_t;
    a.p_kill = best_pk;
    result.push_back(a);
  }
  std::sort(result.begin(), result.end(), [](const WeaponAssignment& x, const WeaponAssignment& y) {
    if (x.threat_index != y.threat_index) return x.threat_index < y.threat_index;
    return x.interceptor_index < y.interceptor_index;
  });
  return result;
}

CampaignResult runManyOnMany(const SimConfig& cfg) {
  const ManyOnManyConfig& mm = cfg.many_on_many;
  CampaignResult r;
  r.num_interceptors = static_cast<int>(mm.interceptors.size());
  r.num_threats = static_cast<int>(mm.threats.size());
  r.doctrine = mm.doctrine;

  r.pairings = scorePairings(cfg);
  const int nt = r.num_threats;
  if (nt == 0) return r;

  // Per-threat accumulated single-shot P(kill)s across all committed shots.
  std::vector<std::vector<double>> threat_shots(static_cast<std::size_t>(nt));
  std::vector<int> threat_shot_count(static_cast<std::size_t>(nt), 0);

  // Inventory of interceptor indices still available to fire.
  std::vector<int> inventory;
  inventory.reserve(mm.interceptors.size());
  for (int w = 0; w < r.num_interceptors; ++w) inventory.push_back(w);

  auto commit = [&](const WeaponAssignment& a) {
    r.assignments.push_back(a);
    threat_shots[static_cast<std::size_t>(a.threat_index)].push_back(a.p_kill);
    ++threat_shot_count[static_cast<std::size_t>(a.threat_index)];
    ++r.interceptors_expended;
    // Remove the spent interceptor from inventory.
    inventory.erase(std::remove(inventory.begin(), inventory.end(), a.interceptor_index),
                    inventory.end());
  };

  if (mm.doctrine == "salvo") {
    // Salvo: commit shots_per_threat interceptors to EACH threat (up to inventory). Each "round" is
    // one WTA wave assigning one weapon per threat; repeat shots_per_threat rounds.
    const int shots = std::max(mm.shots_per_threat, 1);
    std::vector<int> all_threats(static_cast<std::size_t>(nt));
    for (int t = 0; t < nt; ++t) all_threats[static_cast<std::size_t>(t)] = t;
    for (int round = 0; round < shots && !inventory.empty(); ++round) {
      const auto wave = assignWeapons(r.pairings, inventory, all_threats, mm.wta_method);
      if (wave.empty()) break;
      for (const auto& a : wave) commit(a);
    }
  } else if (mm.doctrine == "shoot_look_shoot") {
    // Shoot-look-shoot: each wave fires one weapon per SURVIVING threat, the outcome is assessed,
    // and only the survivors are re-engaged in the next wave (up to max_waves). Assessment is the
    // deterministic per-threat cumulative P(kill) crossing 0.5 (a threat is "killed" once its
    // accumulated shots make a kill more likely than not). This is the EXPECTED-VALUE assessment;
    // the Monte-Carlo rollup below re-samples Bernoulli kills for P(annihilation).
    const int waves = std::max(mm.max_waves, 1);
    for (int wave = 0; wave < waves && !inventory.empty(); ++wave) {
      // Survivors = threats not yet assessed-killed.
      std::vector<int> survivors;
      for (int t = 0; t < nt; ++t) {
        if (cumulativePk(threat_shots[static_cast<std::size_t>(t)]) < 0.5) survivors.push_back(t);
      }
      if (survivors.empty()) break;
      const auto assignment = assignWeapons(r.pairings, inventory, survivors, mm.wta_method);
      if (assignment.empty()) break;
      for (const auto& a : assignment) commit(a);
    }
  } else {  // "raid"
    // Raid: a finite interceptor inventory defends against M threats. A single WTA pass commits
    // each available weapon to its best threat (one weapon per threat). Threats left unengaged — or
    // engaged with too low a P(kill) — leak. This is the defended-asset leakage-vs-inventory case.
    std::vector<int> all_threats(static_cast<std::size_t>(nt));
    for (int t = 0; t < nt; ++t) all_threats[static_cast<std::size_t>(t)] = t;
    // Allow multiple weapons per threat when inventory exceeds threats: repeat WTA rounds until the
    // inventory is exhausted, so surplus interceptors back up the highest-value threats.
    while (!inventory.empty()) {
      const auto wave = assignWeapons(r.pairings, inventory, all_threats, mm.wta_method);
      if (wave.empty()) break;
      for (const auto& a : wave) commit(a);
    }
  }

  // --- Per-threat outcomes + deterministic (expected-value) campaign metrics ---
  r.threats.resize(static_cast<std::size_t>(nt));
  for (int t = 0; t < nt; ++t) {
    ThreatOutcome& o = r.threats[static_cast<std::size_t>(t)];
    o.threat_index = t;
    o.shots_committed = threat_shot_count[static_cast<std::size_t>(t)];
    o.cumulative_pk = cumulativePk(threat_shots[static_cast<std::size_t>(t)]);
    o.killed = o.cumulative_pk >= 0.5;  // deterministic assessment
    if (!o.killed) ++r.leakers;
    r.expected_kills += o.cumulative_pk;
    r.expected_leakage += (1.0 - o.cumulative_pk);
  }
  // P(raid annihilation) = product of per-threat kill probabilities (independent threats).
  r.p_raid_annihilation = 1.0;
  for (int t = 0; t < nt; ++t) {
    r.p_raid_annihilation *= r.threats[static_cast<std::size_t>(t)].cumulative_pk;
  }

  // --- Monte-Carlo campaign rollup (num_trials > 1): sample each committed shot's Bernoulli kill
  // ---
  if (mm.num_trials > 1) {
    Rng rng(cfg.seed);
    int annihilations = 0;
    long total_leakers = 0;
    for (int trial = 0; trial < mm.num_trials; ++trial) {
      int trial_leakers = 0;
      for (int t = 0; t < nt; ++t) {
        bool killed = false;
        for (const double pssk : threat_shots[static_cast<std::size_t>(t)]) {
          if (rng.uniform(0.0, 1.0) < pssk) {
            killed = true;
            break;  // a threat needs only one successful shot
          }
        }
        if (!killed) ++trial_leakers;
      }
      total_leakers += trial_leakers;
      if (trial_leakers == 0) ++annihilations;
    }
    r.mean_leakage = static_cast<double>(total_leakers) / static_cast<double>(mm.num_trials);
    r.mc_p_annihilation = static_cast<double>(annihilations) / static_cast<double>(mm.num_trials);
  } else {
    r.mean_leakage = r.expected_leakage;
    r.mc_p_annihilation = r.p_raid_annihilation;
  }

  return r;
}

}  // namespace gncsim
