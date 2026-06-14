// gnc-sim — Phase 0 foundation tests: math (Vector3/Quaternion/RK4), config, RNG, serialization.
#include <algorithm>
#include <cmath>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "gncsim/core/Config.hpp"
#include "gncsim/core/Rng.hpp"
#include "gncsim/core/Serialize.hpp"
#include "gncsim/core/Types.hpp"
#include "gncsim/math/Integrators.hpp"
#include "gncsim/math/Quaternion.hpp"
#include "gncsim/math/Vector3.hpp"

using namespace gncsim;

TEST(Vector3, DotCrossNorm) {
  Vector3 a{1, 2, 3}, b{4, 5, 6};
  EXPECT_DOUBLE_EQ(a.dot(b), 32.0);
  Vector3 c = a.cross(b);
  EXPECT_DOUBLE_EQ(c.x, -3.0);
  EXPECT_DOUBLE_EQ(c.y, 6.0);
  EXPECT_DOUBLE_EQ(c.z, -3.0);
  EXPECT_DOUBLE_EQ(Vector3(3, 4, 0).norm(), 5.0);
}

TEST(Quaternion, RotatesXtoY) {
  Quaternion q = Quaternion::fromAxisAngle({0, 0, 1}, M_PI / 2);
  Vector3 r = q.rotate({1, 0, 0});
  EXPECT_NEAR(r.x, 0.0, 1e-12);
  EXPECT_NEAR(r.y, 1.0, 1e-12);
  EXPECT_NEAR(r.z, 0.0, 1e-12);
}

TEST(Quaternion, EulerRoundTrip) {
  Quaternion q = Quaternion::fromEuler(0.2, -0.5, 1.1);
  Vector3 e = q.toEuler();
  EXPECT_NEAR(e.x, 0.2, 1e-9);
  EXPECT_NEAR(e.y, -0.5, 1e-9);
  EXPECT_NEAR(e.z, 1.1, 1e-9);
}

TEST(Integrators, RK4ExponentialGrowth) {
  // dy/dt = y, y(0)=1 -> y(1)=e. RK4 should be accurate to ~1e-7 with small steps.
  double y = 1.0;
  const double dt = 1e-3;
  auto f = [](double, double yy) { return yy; };
  for (int i = 0; i < 1000; ++i) y = rk4Step(y, i * dt, dt, f);
  EXPECT_NEAR(y, std::exp(1.0), 1e-6);
}

TEST(Integrators, RK4MoreAccurateThanRK2) {
  auto f = [](double, double yy) { return yy; };
  const double dt = 0.05;
  double y4 = 1.0, y2 = 1.0;
  for (int i = 0; i < 20; ++i) {
    y4 = rk4Step(y4, i * dt, dt, f);
    y2 = rk2Step(y2, i * dt, dt, f);
  }
  const double truth = std::exp(1.0);
  EXPECT_LT(std::abs(y4 - truth), std::abs(y2 - truth));
}

TEST(Rng, DeterministicWithSeed) {
  Rng a(42), b(42);
  for (int i = 0; i < 100; ++i) EXPECT_DOUBLE_EQ(a.gaussian(), b.gaussian());
}

TEST(Config, ParsesAndDefaults) {
  const std::string txt = R"({
    "scenario":"homing","model":"6dof","seed":7,"dt":0.01,
    "guidance":{"nav_constant":4.0},
    "target":{"pos0":[1000,0,500]}
  })";
  SimConfig c = loadConfigFromString(txt);
  EXPECT_EQ(c.scenario, "homing");
  EXPECT_EQ(c.model, "6dof");
  EXPECT_EQ(c.seed, 7u);
  EXPECT_DOUBLE_EQ(c.dt, 0.01);
  EXPECT_DOUBLE_EQ(c.guidance.nav_constant, 4.0);
  EXPECT_DOUBLE_EQ(c.target.pos0.x, 1000.0);
  // Unspecified key keeps default.
  EXPECT_DOUBLE_EQ(c.env.g0, 9.80665);
}

TEST(Config, LaunchVelocity) {
  VehicleConfig v;
  v.launch_speed = 100.0;
  v.launch_elevation_deg = 90.0;  // straight up
  Vector3 vel = launchVelocity(v);
  EXPECT_NEAR(vel.z, 100.0, 1e-9);
  EXPECT_NEAR(vel.x, 0.0, 1e-9);
  EXPECT_NEAR(vel.y, 0.0, 1e-9);
}

TEST(Serialize, JsonRoundTripAndCsv) {
  SimResult r;
  r.scenario = "homing";
  r.model = "3dof";
  r.seed = 123;
  r.dt = 0.01;
  r.intercept = true;
  r.miss_distance = 1.5;
  Frame f0;
  f0.t = 0.0;
  f0.veh_pos = {0, 0, 0};
  f0.range = 100.0;
  Frame f1;
  f1.t = 0.01;
  f1.veh_pos = {1, 2, 3};
  f1.range = 50.0;
  r.frames = {f0, f1};

  const std::string js = toJsonString(r);
  auto parsed = nlohmann::json::parse(js);
  EXPECT_EQ(parsed["scenario"], "homing");
  EXPECT_TRUE(parsed["intercept"].get<bool>());
  EXPECT_EQ(parsed["series"]["t"].size(), 2u);
  EXPECT_DOUBLE_EQ(parsed["series"]["veh_x"][1].get<double>(), 1.0);

  auto files = toCsvFiles(r);
  ASSERT_TRUE(files.count("vehicle.csv"));
  EXPECT_NE(files["vehicle.csv"].find("t,x,y,z"), std::string::npos);
  // header + 2 data rows = 3 newlines.
  EXPECT_EQ(std::count(files["vehicle.csv"].begin(), files["vehicle.csv"].end(), '\n'), 3);

  auto manifest = nlohmann::json::parse(toManifestJson(r));
  EXPECT_EQ(manifest["num_frames"].get<int>(), 2);
}
