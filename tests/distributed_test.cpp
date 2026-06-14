// gnc-sim — distributed federation tests (issue #47).
//
// Covers the three required properties:
//   1) DIS Entity State / Track PDUs round-trip (encode -> decode == original within the DIS field
//      resolutions: positions/velocities/orientation are 32-bit floats on the wire).
//   2) A recorded federated run replays byte-identically (record -> replay -> re-record => same
//      bytes), and the snapshot stream is reconstructed exactly.
//   3) Two endpoints over an in-memory loopback transport exchange entity state and agree (no real
//      UDP port is bound, so the test is hermetic and cannot flake in CI).
#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "gncsim/core/Config.hpp"
#include "gncsim/interop/Dis.hpp"
#include "gncsim/interop/DisAdapter.hpp"
#include "gncsim/interop/Federation.hpp"
#include "gncsim/interop/MessageBus.hpp"
#include "gncsim/interop/Recording.hpp"
#include "gncsim/interop/Transport.hpp"
#include "gncsim/math/Quaternion.hpp"
#include "gncsim/scenario/Runner.hpp"

using namespace gncsim;
using namespace gncsim::interop;

namespace {

// DIS field resolution: world coordinates/velocity are 32-bit floats. A round-trip is exact to
// float precision, so compare within a relative+absolute float tolerance, not bit-equality.
void expectNearFloat(double got, double want, const char* what) {
  const double tol = 1e-3 + 1e-5 * std::fabs(want);
  EXPECT_NEAR(got, want, tol) << what;
}

EntityStateMsg makeEntity() {
  EntityStateMsg e;
  e.entity_id = 1;
  e.force_id = 1;
  e.entity_kind = 1;
  e.t_s = 3.25;
  e.pos_m = {1234.5, -678.25, 9012.75};
  e.vel_mps = {120.5, -33.25, 4.125};
  e.att = Quaternion::fromAxisAngle(Vector3{0, 0, 1}, 0.5);
  e.ang_vel_radps = {0.01, -0.02, 0.03};
  return e;
}

}  // namespace

// --- (1) DIS PDU round-trips ------------------------------------------------------------------

TEST(Distributed, EntityStatePduRoundTrip) {
  const EntityStateMsg orig = makeEntity();
  const auto pdu = dis::encodeEntityState(orig, /*exercise_id=*/7, /*site_id=*/2, /*app_id=*/3);

  // Header sanity: protocol version 5, Entity State type, length == buffer size.
  const dis::PduHeader h = dis::peekHeader(pdu);
  EXPECT_EQ(h.protocol_version, dis::kProtocolVersion);
  EXPECT_EQ(h.pdu_type, static_cast<std::uint8_t>(dis::PduType::kEntityState));
  EXPECT_EQ(h.exercise_id, 7);
  EXPECT_EQ(h.length, pdu.size());

  const EntityStateMsg got = dis::decodeEntityState(pdu);
  EXPECT_EQ(got.entity_id, orig.entity_id);
  EXPECT_EQ(got.force_id, orig.force_id);
  EXPECT_EQ(got.entity_kind, orig.entity_kind);
  EXPECT_DOUBLE_EQ(got.t_s, orig.t_s);  // t_s is f64 on the wire => exact
  expectNearFloat(got.pos_m.x, orig.pos_m.x, "pos.x");
  expectNearFloat(got.pos_m.y, orig.pos_m.y, "pos.y");
  expectNearFloat(got.pos_m.z, orig.pos_m.z, "pos.z");
  expectNearFloat(got.vel_mps.x, orig.vel_mps.x, "vel.x");
  expectNearFloat(got.vel_mps.y, orig.vel_mps.y, "vel.y");
  expectNearFloat(got.vel_mps.z, orig.vel_mps.z, "vel.z");

  // Orientation survives the quaternion -> Euler -> quaternion trip (up to sign, so compare the
  // rotation effect on a probe vector).
  const Vector3 probe{1.0, 2.0, 3.0};
  const Vector3 a = orig.att.rotate(probe);
  const Vector3 b = got.att.rotate(probe);
  expectNearFloat(b.x, a.x, "att probe x");
  expectNearFloat(b.y, a.y, "att probe y");
  expectNearFloat(b.z, a.z, "att probe z");
}

TEST(Distributed, TrackPduRoundTrip) {
  TrackMsg orig;
  orig.track_id = 4242;
  orig.t_s = 1.5;
  orig.pos_est_m = {500.0, 250.0, 100.0};
  orig.vel_est_mps = {-10.0, 5.0, 1.0};
  orig.quality = 0.875;

  const auto pdu = dis::encodeTrack(orig, /*exercise_id=*/1, /*site_id=*/1, /*app_id=*/1);
  EXPECT_EQ(dis::peekHeader(pdu).pdu_type,
            static_cast<std::uint8_t>(dis::PduType::kTrackDetection));

  const TrackMsg got = dis::decodeTrack(pdu);
  EXPECT_EQ(got.track_id, orig.track_id);
  EXPECT_DOUBLE_EQ(got.t_s, orig.t_s);
  expectNearFloat(got.pos_est_m.x, orig.pos_est_m.x, "track pos.x");
  expectNearFloat(got.vel_est_mps.y, orig.vel_est_mps.y, "track vel.y");
  expectNearFloat(got.quality, orig.quality, "quality");
}

TEST(Distributed, DecodeRejectsWrongType) {
  const auto entity_pdu = dis::encodeEntityState(makeEntity(), 1, 1, 1);
  EXPECT_THROW(dis::decodeTrack(entity_pdu), std::runtime_error);
  std::vector<std::uint8_t> truncated(entity_pdu.begin(), entity_pdu.begin() + 5);
  EXPECT_THROW(dis::decodeEntityState(truncated), std::exception);
}

// --- (2) Deterministic record + replay --------------------------------------------------------

TEST(Distributed, RecordReplayIsDeterministicAndByteIdentical) {
  SimConfig cfg = loadConfigFromString(R"({"scenario":"homing","model":"3dof"})");
  const SimResult result = runSimulation(cfg);
  const auto snaps = snapshotsFromResult(result);
  ASSERT_FALSE(snaps.empty());
  EXPECT_EQ(snaps.size(), result.frames.size());

  // Record run #1.
  auto rec1 = std::make_shared<FileRecorder>();
  MessageBus bus1;
  bus1.attach(rec1);
  bus1.start(/*exercise_id=*/1, /*site_id=*/1, /*app_id=*/1);
  for (const auto& s : snaps) bus1.publish(s);
  bus1.stop();
  const std::vector<std::uint8_t> bytes1 = rec1->bytes();
  ASSERT_FALSE(bytes1.empty());

  // Replay the bytes back to snapshots.
  const auto replayed = replay(bytes1);
  ASSERT_EQ(replayed.size(), snaps.size());
  for (std::size_t i = 0; i < snaps.size(); ++i) {
    EXPECT_EQ(replayed[i].step, snaps[i].step);
    EXPECT_DOUBLE_EQ(replayed[i].t_s, snaps[i].t_s);
    ASSERT_EQ(replayed[i].entities.size(), snaps[i].entities.size());
    for (std::size_t e = 0; e < snaps[i].entities.size(); ++e) {
      // Recording stores full f64 => replay is EXACT (not just within float resolution).
      EXPECT_EQ(replayed[i].entities[e].entity_id, snaps[i].entities[e].entity_id);
      EXPECT_DOUBLE_EQ(replayed[i].entities[e].pos_m.x, snaps[i].entities[e].pos_m.x);
      EXPECT_DOUBLE_EQ(replayed[i].entities[e].vel_mps.z, snaps[i].entities[e].vel_mps.z);
    }
  }

  // Re-record the replayed stream: bytes must be IDENTICAL (the determinism invariant).
  auto rec2 = std::make_shared<FileRecorder>();
  MessageBus bus2;
  bus2.attach(rec2);
  bus2.start(1, 1, 1);
  for (const auto& s : replayed) bus2.publish(s);
  bus2.stop();
  EXPECT_EQ(rec2->bytes(), bytes1) << "record -> replay -> re-record must be byte-identical";

  // And a second independent record of the SAME run reproduces the same bytes.
  auto rec3 = std::make_shared<FileRecorder>();
  MessageBus bus3;
  bus3.attach(rec3);
  bus3.start(1, 1, 1);
  for (const auto& s : snapshotsFromResult(runSimulation(cfg))) bus3.publish(s);
  bus3.stop();
  EXPECT_EQ(rec3->bytes(), bytes1) << "two records of the same run must be byte-identical";
}

// --- (3) Two endpoints exchange entity state over loopback (no real network) ------------------

TEST(Distributed, TwoEndpointsExchangeEntityStateOverLoopback) {
  // Endpoint A (the sim) sends; endpoint B (a peer federate) receives. Cross-wired loopback queues
  // model the wire with zero port binding.
  auto a_to_b = std::make_shared<LoopbackTransport>();
  LoopbackTransport b_inbox;
  a_to_b->connectTo(b_inbox);

  DisAdapter dis_pub(a_to_b, /*site_id=*/1, /*app_id=*/1);
  MessageBus bus;
  // attach() takes ownership; keep the publisher above for the assertion on pdusSent via the bus.
  bus.attach(std::shared_ptr<IFederateAdapter>(&dis_pub, [](IFederateAdapter*) {}));
  bus.start(/*exercise_id=*/9, /*site_id=*/1, /*app_id=*/1);

  SimConfig cfg = loadConfigFromString(R"({"scenario":"homing","model":"3dof"})");
  const auto snaps = snapshotsFromResult(runSimulation(cfg));
  ASSERT_FALSE(snaps.empty());

  // Publish just the first snapshot's entities and confirm the peer decodes them back identically
  // (within DIS float resolution).
  bus.publish(snaps.front());
  bus.stop();

  std::vector<EntityStateMsg> rx_entities;
  std::vector<TrackMsg> rx_tracks;
  drainDis(b_inbox, rx_entities, rx_tracks);

  ASSERT_EQ(rx_entities.size(), snaps.front().entities.size());
  for (std::size_t i = 0; i < rx_entities.size(); ++i) {
    EXPECT_EQ(rx_entities[i].entity_id, snaps.front().entities[i].entity_id);
    EXPECT_EQ(rx_entities[i].force_id, snaps.front().entities[i].force_id);
    expectNearFloat(rx_entities[i].pos_m.x, snaps.front().entities[i].pos_m.x, "rx pos.x");
    expectNearFloat(rx_entities[i].pos_m.y, snaps.front().entities[i].pos_m.y, "rx pos.y");
    expectNearFloat(rx_entities[i].vel_mps.x, snaps.front().entities[i].vel_mps.x, "rx vel.x");
  }
  EXPECT_EQ(dis_pub.pdusSent(), snaps.front().entities.size() + snaps.front().tracks.size());
}

// A full federated run streamed end-to-end: every entity PDU the sim emits is received by the peer.
TEST(Distributed, FullRunStreamsAllEntityPdus) {
  auto tx = std::make_shared<LoopbackTransport>();
  LoopbackTransport rx;
  tx->connectTo(rx);

  DisAdapter pub(tx);
  SimConfig cfg = loadConfigFromString(R"({"scenario":"homing","model":"3dof"})");
  const auto snaps = snapshotsFromResult(runSimulation(cfg));

  pub.onStart(1, 1, 1);
  std::size_t expected_entities = 0;
  for (const auto& s : snaps) {
    pub.publish(s);
    expected_entities += s.entities.size();
  }

  std::vector<EntityStateMsg> rx_entities;
  std::vector<TrackMsg> rx_tracks;
  drainDis(rx, rx_entities, rx_tracks);
  EXPECT_EQ(rx_entities.size(), expected_entities);
}
