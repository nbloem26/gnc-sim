// gnc-sim — many-on-many engagement campaign (issue #45): N interceptors vs M threats,
// weapon-target assignment (WTA), and the salvo / shoot-look-shoot / raid doctrines. This is a
// SCENARIO-level orchestration layer that REUSES the per-engagement physics: every
// interceptor-vs-threat pairing is scored by running the same pure runSimulation() and converting
// the analytic CPA miss distance into a single-shot P(kill) via a Gaussian lethality (Carleton)
// model. Pure (no file I/O) so it runs identically native & WASM; deterministic given the seed
// (project Rng only — NO std distributions).
//
// Opt-in and additive: nothing here touches runSimulation, so the default single-engagement path is
// byte-identical. The CLI dispatches to runManyOnMany() when cfg.many_on_many.enabled.
#pragma once

#include <cstdint>
#include <vector>

#include "gncsim/core/Config.hpp"

namespace gncsim {

// Single-shot P(kill) for one interceptor-vs-threat pairing, plus the miss distance it came from.
struct PairingPkill {
  int interceptor_index = 0;  // index into cfg.many_on_many.interceptors
  int threat_index = 0;       // index into cfg.many_on_many.threats
  double miss_distance_m = 0.0;
  double p_kill = 0.0;  // single-shot P(kill) from the lethality model (0..pk_max)
};

// One weapon committed to one threat by the assignment.
struct WeaponAssignment {
  int interceptor_index = 0;
  int threat_index = 0;
  double p_kill = 0.0;  // the committed pairing's single-shot P(kill)
};

// Per-threat campaign outcome after the doctrine has played out.
struct ThreatOutcome {
  int threat_index = 0;
  int shots_committed = 0;     // interceptors fired at this threat across all waves
  double cumulative_pk = 0.0;  // 1 - prod(1 - Pssk_i) over the committed shots (expected kill prob)
  bool killed = false;         // deterministic rollup: cumulative_pk >= 0.5; or the MC majority
};

// The full campaign result: the scored pairing matrix, the assignment, per-threat outcomes, and the
// rolled-up campaign metrics (leakage, interceptors expended, P(raid annihilation)).
struct CampaignResult {
  int num_interceptors = 0;
  int num_threats = 0;
  std::string doctrine;

  std::vector<PairingPkill> pairings;         // every interceptor x threat pairing, scored
  std::vector<WeaponAssignment> assignments;  // weapons committed by the WTA (across all waves)
  std::vector<ThreatOutcome> threats;         // per-threat outcome

  int interceptors_expended = 0;     // total weapons fired
  int leakers = 0;                   // threats surviving the campaign (deterministic rollup)
  double expected_leakage = 0.0;     // expected number of surviving threats (sum of survival pk)
  double expected_kills = 0.0;       // expected number of threats killed
  double p_raid_annihilation = 0.0;  // probability ALL threats are killed (prod of per-threat pk)
  double mean_leakage = 0.0;         // Monte Carlo mean surviving threats (== expected_leakage if
                                     // num_trials <= 1)
  double mc_p_annihilation = 0.0;    // Monte Carlo P(annihilation) (== p_raid_annihilation if
                                     // num_trials <= 1)
};

// Build the full pairing P(kill) matrix: run runSimulation() once per interceptor-vs-threat pairing
// (the interceptor's launch spec drives cfg.vehicle, the threat spec drives cfg.target) and convert
// each miss to a single-shot P(kill). Deterministic; reuses the per-engagement physics unchanged.
std::vector<PairingPkill> scorePairings(const SimConfig& cfg);

// Deterministic weapon-target assignment maximizing expected kills given the pairing P(kill)
// matrix. `available` lists the interceptor indices still in inventory; `threats` lists the threat
// indices still alive. At most one weapon per threat per call (a wave); salvo commits this
// repeatedly. Method is greedy (highest-marginal-Pk first) or a Bertsekas-style auction — both
// deterministic.
std::vector<WeaponAssignment> assignWeapons(const std::vector<PairingPkill>& pairings,
                                            const std::vector<int>& available,
                                            const std::vector<int>& threats,
                                            const std::string& method);

// Run the whole many-on-many campaign: score pairings, assign weapons per the doctrine, play it
// out, and roll up the campaign metrics. Deterministic given cfg.seed.
CampaignResult runManyOnMany(const SimConfig& cfg);

}  // namespace gncsim
