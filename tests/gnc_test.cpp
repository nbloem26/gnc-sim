// gnc-sim — Phase 1 GNC tests: engagement geometry, Proportional Navigation properties, the
// alpha-beta Navigator, and basic autopilot bounds. World frame is ENU, SI units.
#include <cmath>

#include <gtest/gtest.h>

#include "gncsim/core/Config.hpp"
#include "gncsim/core/Types.hpp"
#include "gncsim/gnc/Gnc.hpp"
#include "gncsim/math/Quaternion.hpp"
#include "gncsim/math/Vector3.hpp"

using namespace gncsim;

namespace {

// Minimal EntityState at a position with a velocity (attitude/rate default to identity/zero).
EntityState entity(const Vector3& pos, const Vector3& vel) {
  EntityState s;
  s.pos = pos;
  s.vel = vel;
  return s;
}

GuidanceConfig pronavConfig(double N = 4.0, double max_accel = 300.0) {
  GuidanceConfig c;
  c.law = "pronav";
  c.nav_constant = N;
  c.max_accel = max_accel;
  return c;
}

}  // namespace

// ── A) Engagement geometry: head-on closing ─────────────────────────────────────────────────
TEST(Engagement, HeadOnClosingGeometry) {
  // Vehicle at origin moving +East; target 1000 m ahead on +East moving -East (toward vehicle).
  const EntityState veh = entity({0.0, 0.0, 0.0}, {300.0, 0.0, 0.0});
  const EntityState tgt = entity({1000.0, 0.0, 0.0}, {-200.0, 0.0, 0.0});

  const Engagement e = computeEngagement(veh, tgt);

  EXPECT_NEAR(e.range, 1000.0, 1e-9);
  // los_unit points from vehicle to target (+East).
  EXPECT_NEAR(e.los_unit.x, 1.0, 1e-12);
  // Closing: rel_vel = (-200) - (300) = -500 along +x; v_closing = -dot(rel_vel, los) = 500 > 0.
  EXPECT_GT(e.v_closing, 0.0);
  EXPECT_NEAR(e.v_closing, 500.0, 1e-9);
  // Purely radial approach => no LOS rotation.
  EXPECT_NEAR(e.los_rate, 0.0, 1e-12);
}

// ── B) Collision course: zero LOS rate => ~zero PN command ───────────────────────────────────
TEST(ProNav, CollisionCourseGivesZeroCommand) {
  // rel_vel anti-parallel to rel_pos: target closes straight down the line of sight. This is the
  // constant-bearing intercept geometry, so PN should command (essentially) no acceleration.
  const EntityState veh = entity({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
  const EntityState tgt = entity({2000.0, 1000.0, 500.0}, {-200.0, -100.0, -50.0});

  const Engagement e = computeEngagement(veh, tgt);
  EXPECT_GT(e.v_closing, 0.0);          // genuinely closing
  EXPECT_NEAR(e.los_rate, 0.0, 1e-12);  // the defining property

  const Vector3 a = proNavCommand(e, pronavConfig());
  EXPECT_NEAR(a.norm(), 0.0, 1e-9);
}

// ── C) Rotating LOS: non-zero command, perpendicular to LOS ──────────────────────────────────
TEST(ProNav, RotatingLosGivesPerpendicularCommand) {
  // Target offset so the line of sight rotates: vehicle flies +East, target sits off to the North
  // and drifts, producing a non-zero LOS rate.
  const EntityState veh = entity({0.0, 0.0, 0.0}, {300.0, 0.0, 0.0});
  const EntityState tgt = entity({1000.0, 800.0, 0.0}, {0.0, 200.0, 0.0});

  const Engagement e = computeEngagement(veh, tgt);
  EXPECT_GT(e.los_rate, 0.0);
  EXPECT_GT(e.v_closing, 0.0);

  const Vector3 a = proNavCommand(e, pronavConfig());
  EXPECT_GT(a.norm(), 0.0);
  // True PN acceleration is always perpendicular to the line of sight.
  EXPECT_NEAR(a.dot(e.los_unit), 0.0, 1e-6);
}

// ── D) Acceleration magnitude limit ─────────────────────────────────────────────────────────
TEST(ProNav, RespectsMaxAccelLimit) {
  // Closing fast with a strong lateral drift + large nav constant drives the raw command well
  // past the cap (and keeps v_closing > 0 so PN actually produces a command to clamp).
  const EntityState veh = entity({0.0, 0.0, 0.0}, {600.0, 0.0, 0.0});
  const EntityState tgt = entity({1500.0, 400.0, 0.0}, {-200.0, 300.0, 0.0});

  const Engagement e = computeEngagement(veh, tgt);
  ASSERT_GT(e.los_rate, 0.0);
  ASSERT_GT(e.v_closing, 0.0);

  const double max_accel = 250.0;
  const Vector3 a = proNavCommand(e, pronavConfig(/*N=*/8.0, max_accel));
  EXPECT_NEAR(a.norm(), max_accel, 1e-6);  // saturated exactly at the cap
}

// ── E) law == "none" => zero command ────────────────────────────────────────────────────────
TEST(ProNav, NoneLawGivesZeroCommand) {
  const EntityState veh = entity({0.0, 0.0, 0.0}, {300.0, 0.0, 0.0});
  const EntityState tgt = entity({1000.0, 800.0, 0.0}, {0.0, 200.0, 0.0});

  const Engagement e = computeEngagement(veh, tgt);
  ASSERT_GT(e.los_rate, 0.0);  // would otherwise produce a non-zero command

  GuidanceConfig cfg = pronavConfig();
  cfg.law = "none";
  const Vector3 a = proNavCommand(e, cfg);
  EXPECT_DOUBLE_EQ(a.norm(), 0.0);
}

TEST(ProNav, RecedingTargetGivesZeroCommand) {
  // v_closing <= 0 (diverging): no terminal-homing command even under the pronav law.
  const EntityState veh = entity({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
  const EntityState tgt = entity({1000.0, 200.0, 0.0}, {300.0, 50.0, 0.0});  // flying away

  const Engagement e = computeEngagement(veh, tgt);
  ASSERT_LE(e.v_closing, 0.0);
  const Vector3 a = proNavCommand(e, pronavConfig());
  EXPECT_DOUBLE_EQ(a.norm(), 0.0);
}

// ── F) Navigator: alpha-beta tracker converges on a constant-velocity track ──────────────────
TEST(Navigator, TracksConstantVelocityRelativeMotion) {
  const double dt = 0.01;
  Navigator nav(dt, /*alpha=*/0.5, /*beta=*/0.1);

  // Ground-truth relative track: p(t) = p0 + v0 * t, sampled at dt.
  const Vector3 p0{1000.0, -500.0, 200.0};
  const Vector3 v0{-150.0, 40.0, -10.0};

  // First update bootstraps position and zeros velocity.
  nav.update(p0);
  EXPECT_NEAR(nav.relPos().x, p0.x, 1e-12);
  EXPECT_NEAR(nav.relVel().norm(), 0.0, 1e-12);

  // Feed a handful of steps; the estimator should lock onto v0 and follow position.
  const int steps = 60;
  for (int k = 1; k <= steps; ++k) {
    const Vector3 p = p0 + v0 * (k * dt);
    nav.update(p);
  }

  // Velocity estimate converges near the true constant velocity.
  EXPECT_NEAR(nav.relVel().x, v0.x, 1.0);
  EXPECT_NEAR(nav.relVel().y, v0.y, 1.0);
  EXPECT_NEAR(nav.relVel().z, v0.z, 1.0);

  // Position estimate tracks the latest input closely.
  const Vector3 p_final = p0 + v0 * (steps * dt);
  EXPECT_NEAR(nav.relPos().x, p_final.x, 5.0);
  EXPECT_NEAR(nav.relPos().y, p_final.y, 5.0);
  EXPECT_NEAR(nav.relPos().z, p_final.z, 5.0);
}

// ── G) Autopilot: bounded fin telemetry and zero command at rest ─────────────────────────────
TEST(Autopilot, FinDeflectionStaysBounded) {
  ControlConfig cfg;
  cfg.kp = 8.0;
  cfg.kd = 2.5;
  cfg.max_fin_deflection = 0.35;
  Autopilot ap(cfg);

  EntityState s = entity({0.0, 0.0, 0.0}, {300.0, 0.0, 0.0});
  s.att = Quaternion{};  // nose along +East, aligned with velocity

  // A large lateral acceleration command demands a big attitude change -> saturated fins.
  const Vector3 accel_cmd{0.0, 5000.0, 0.0};
  Vector3 fin;
  ap.moment(s, accel_cmd, fin);

  EXPECT_LE(std::abs(fin.x), cfg.max_fin_deflection + 1e-12);
  EXPECT_LE(std::abs(fin.y), cfg.max_fin_deflection + 1e-12);
  EXPECT_LE(std::abs(fin.z), cfg.max_fin_deflection + 1e-12);
}

TEST(Autopilot, NoErrorNoCommandWhenAligned) {
  ControlConfig cfg;  // defaults
  Autopilot ap(cfg);

  // Nose aligned with velocity and zero command => desired == current nose => no moment.
  EntityState s = entity({0.0, 0.0, 0.0}, {300.0, 0.0, 0.0});
  s.att = Quaternion{};  // body +x -> world +East
  Vector3 fin;
  const Vector3 m = ap.moment(s, Vector3{0.0, 0.0, 0.0}, fin);

  EXPECT_NEAR(m.norm(), 0.0, 1e-9);
  EXPECT_NEAR(fin.norm(), 0.0, 1e-9);
}
