// gnc-sim — interop: DIS (IEEE 1278.1) PDU encode/decode (issue #47).
//
// Std-only, big-endian (network byte order) codec built on ByteIo. The Entity State PDU layout
// below follows the IEEE 1278.1 field order for the fields we carry; reserved/padding fields are
// written as zeros and skipped on decode so the on-wire length is honoured. Positions, velocities,
// and Euler orientation use 32-bit floats (the DIS field resolution), so a round-trip is exact to
// float precision — asserted within that resolution by the test.
#include "gncsim/interop/Dis.hpp"

#include <cmath>
#include <stdexcept>

#include "gncsim/interop/ByteIo.hpp"

namespace gncsim::interop::dis {

namespace {

constexpr std::size_t kHeaderBytes = 12;

void writeHeader(ByteWriter& w, std::uint8_t exercise_id, PduType type, std::uint8_t family,
                 std::uint16_t length) {
  w.u8(kProtocolVersion);
  w.u8(exercise_id);
  w.u8(static_cast<std::uint8_t>(type));
  w.u8(family);
  w.u32(0);       // timestamp (opaque; deterministic zero so records are reproducible)
  w.u16(length);  // total PDU length
  w.u16(0);       // padding to 12-byte header
}

// Entity id triplet (site, application, entity) — the DIS EntityIdentifier record.
void writeEntityId(ByteWriter& w, std::uint16_t site, std::uint16_t app, std::uint16_t entity) {
  w.u16(site);
  w.u16(app);
  w.u16(entity);
}

}  // namespace

EulerAngles quaternionToEuler(const Quaternion& q) {
  const Quaternion n = q.normalized();
  EulerAngles e;
  // ZYX (psi=yaw, theta=pitch, phi=roll) from a scalar-first unit quaternion.
  const double sinp = 2.0 * (n.w * n.y - n.z * n.x);
  const double clamped = sinp > 1.0 ? 1.0 : (sinp < -1.0 ? -1.0 : sinp);
  e.theta_rad = static_cast<float>(std::asin(clamped));
  e.psi_rad = static_cast<float>(
      std::atan2(2.0 * (n.w * n.z + n.x * n.y), 1.0 - 2.0 * (n.y * n.y + n.z * n.z)));
  e.phi_rad = static_cast<float>(
      std::atan2(2.0 * (n.w * n.x + n.y * n.z), 1.0 - 2.0 * (n.x * n.x + n.y * n.y)));
  return e;
}

Quaternion eulerToQuaternion(const EulerAngles& e) {
  const double cy = std::cos(e.psi_rad * 0.5);
  const double sy = std::sin(e.psi_rad * 0.5);
  const double cp = std::cos(e.theta_rad * 0.5);
  const double sp = std::sin(e.theta_rad * 0.5);
  const double cr = std::cos(e.phi_rad * 0.5);
  const double sr = std::sin(e.phi_rad * 0.5);
  return Quaternion{cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                    cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy}
      .normalized();
}

std::vector<std::uint8_t> encodeEntityState(const EntityStateMsg& msg, std::uint8_t exercise_id,
                                            std::uint16_t site_id, std::uint16_t app_id) {
  // Body layout (after the 12-byte header):
  //   EntityID (6) | ForceID (1) | EntityKind (1) | t_s f64 (8)
  //   | location ENU x,y,z f32 (12) | velocity ENU x,y,z f32 (12)
  //   | orientation psi,theta,phi f32 (12) | angular vel x,y,z f32 (12)
  constexpr std::uint16_t kBodyBytes = 6 + 1 + 1 + 8 + 12 + 12 + 12 + 12;
  constexpr std::uint16_t kTotal = kHeaderBytes + kBodyBytes;

  ByteWriter w;
  writeHeader(w, exercise_id, PduType::kEntityState, /*family=*/1, kTotal);
  writeEntityId(w, site_id, app_id, static_cast<std::uint16_t>(msg.entity_id & 0xFFFF));
  w.u8(msg.force_id);
  w.u8(msg.entity_kind);
  w.f64(msg.t_s);
  w.f32(static_cast<float>(msg.pos_m.x));
  w.f32(static_cast<float>(msg.pos_m.y));
  w.f32(static_cast<float>(msg.pos_m.z));
  w.f32(static_cast<float>(msg.vel_mps.x));
  w.f32(static_cast<float>(msg.vel_mps.y));
  w.f32(static_cast<float>(msg.vel_mps.z));
  const EulerAngles eul = quaternionToEuler(msg.att);
  w.f32(eul.psi_rad);
  w.f32(eul.theta_rad);
  w.f32(eul.phi_rad);
  w.f32(static_cast<float>(msg.ang_vel_radps.x));
  w.f32(static_cast<float>(msg.ang_vel_radps.y));
  w.f32(static_cast<float>(msg.ang_vel_radps.z));
  return w.data();
}

EntityStateMsg decodeEntityState(const std::vector<std::uint8_t>& pdu) {
  ByteReader r(pdu);
  const std::uint8_t version = r.u8();
  if (version != kProtocolVersion) throw std::runtime_error("DIS: bad protocol version");
  r.u8();  // exercise id
  const std::uint8_t type = r.u8();
  if (type != static_cast<std::uint8_t>(PduType::kEntityState)) {
    throw std::runtime_error("DIS: not an Entity State PDU");
  }
  r.u8();   // family
  r.u32();  // timestamp
  r.u16();  // length
  r.u16();  // header padding

  r.u16();  // site
  r.u16();  // application
  EntityStateMsg msg;
  msg.entity_id = r.u16();
  msg.force_id = r.u8();
  msg.entity_kind = r.u8();
  msg.t_s = r.f64();
  msg.pos_m.x = r.f32();
  msg.pos_m.y = r.f32();
  msg.pos_m.z = r.f32();
  msg.vel_mps.x = r.f32();
  msg.vel_mps.y = r.f32();
  msg.vel_mps.z = r.f32();
  EulerAngles eul;
  eul.psi_rad = r.f32();
  eul.theta_rad = r.f32();
  eul.phi_rad = r.f32();
  msg.att = eulerToQuaternion(eul);
  msg.ang_vel_radps.x = r.f32();
  msg.ang_vel_radps.y = r.f32();
  msg.ang_vel_radps.z = r.f32();
  return msg;
}

std::vector<std::uint8_t> encodeTrack(const TrackMsg& msg, std::uint8_t exercise_id,
                                      std::uint16_t site_id, std::uint16_t app_id) {
  // Body: track id u32 (4) | t_s f64 (8) | pos x,y,z f32 (12) | vel x,y,z f32 (12) | quality f32
  // (4)
  constexpr std::uint16_t kBodyBytes = 4 + 8 + 12 + 12 + 4;
  constexpr std::uint16_t kTotal = kHeaderBytes + kBodyBytes;

  ByteWriter w;
  writeHeader(w, exercise_id, PduType::kTrackDetection, /*family=*/129, kTotal);
  writeEntityId(w, site_id, app_id, 0);
  w.u32(msg.track_id);
  w.f64(msg.t_s);
  w.f32(static_cast<float>(msg.pos_est_m.x));
  w.f32(static_cast<float>(msg.pos_est_m.y));
  w.f32(static_cast<float>(msg.pos_est_m.z));
  w.f32(static_cast<float>(msg.vel_est_mps.x));
  w.f32(static_cast<float>(msg.vel_est_mps.y));
  w.f32(static_cast<float>(msg.vel_est_mps.z));
  w.f32(static_cast<float>(msg.quality));
  return w.data();
}

TrackMsg decodeTrack(const std::vector<std::uint8_t>& pdu) {
  ByteReader r(pdu);
  if (r.u8() != kProtocolVersion) throw std::runtime_error("DIS: bad protocol version");
  r.u8();  // exercise id
  if (r.u8() != static_cast<std::uint8_t>(PduType::kTrackDetection)) {
    throw std::runtime_error("DIS: not a Track/Detection PDU");
  }
  r.u8();   // family
  r.u32();  // timestamp
  r.u16();  // length
  r.u16();  // header padding
  r.u16();  // site
  r.u16();  // application
  r.u16();  // entity (unused for tracks)

  TrackMsg msg;
  msg.track_id = r.u32();
  msg.t_s = r.f64();
  msg.pos_est_m.x = r.f32();
  msg.pos_est_m.y = r.f32();
  msg.pos_est_m.z = r.f32();
  msg.vel_est_mps.x = r.f32();
  msg.vel_est_mps.y = r.f32();
  msg.vel_est_mps.z = r.f32();
  msg.quality = r.f32();
  return msg;
}

PduHeader peekHeader(const std::vector<std::uint8_t>& pdu) {
  ByteReader r(pdu);
  PduHeader h;
  h.protocol_version = r.u8();
  h.exercise_id = r.u8();
  h.pdu_type = r.u8();
  h.protocol_family = r.u8();
  h.timestamp = r.u32();
  h.length = r.u16();
  return h;
}

}  // namespace gncsim::interop::dis
