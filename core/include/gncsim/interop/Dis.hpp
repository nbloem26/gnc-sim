/// @file Dis.hpp
/// @brief DIS (IEEE 1278.1) PDU encode/decode for the federation bus (issue #47).
///
/// The concrete, tested federation leg. DIS is a self-contained big-endian wire protocol with no
/// external RTI, so it is implementable with std-only code (`ByteIo.hpp`). We implement:
///
///  - the 12-byte **PDU Header** (protocol version, exercise id, PDU type, length, timestamp),
///  - the **Entity State PDU** (type 1) carrying entity id, force id, kind, ENU position, linear
///    velocity, and orientation (Euler angles, the DIS representation),
///  - a compact **Track/Detection PDU** (carried in the experimental PDU-type range, 129) for the
///    fused-track message.
///
/// Field resolutions match the protocol: DIS world coordinates and linear velocity are 32-bit
/// floats and orientation are 32-bit-float Euler angles, so an encode→decode round-trip is exact to
/// `float` precision, not `double`. The round-trip test asserts equality **within that field
/// resolution**, as the protocol mandates. The PDU body uses the sim's local ENU frame directly
/// (the geocentric mapping a real gateway would apply is an adapter concern; see
/// docs/DISTRIBUTED.md).
#pragma once

#include <cstdint>
#include <vector>

#include "gncsim/interop/ByteIo.hpp"
#include "gncsim/interop/Federation.hpp"
#include "gncsim/math/Quaternion.hpp"

namespace gncsim::interop::dis {

/// @brief DIS protocol version we emit (IEEE 1278.1-1995 == 5; a fixed, recognised value).
constexpr std::uint8_t kProtocolVersion = 5;

/// @brief PDU type ids we use (subset of the IEEE-1278 enumeration).
enum class PduType : std::uint8_t {
  kEntityState = 1,      ///< IEEE 1278.1 Entity State PDU
  kTrackDetection = 129  ///< experimental range (>= 129): our fused-track / detection PDU
};

/// @brief 12-byte DIS PDU header (common to every PDU).
struct PduHeader {
  std::uint8_t protocol_version = kProtocolVersion;
  std::uint8_t exercise_id = 0;
  std::uint8_t pdu_type = 0;
  std::uint8_t protocol_family = 1;  ///< 1 = Entity Information/Interaction
  std::uint32_t timestamp = 0;       ///< DIS timestamp (absolute/relative units; opaque here)
  std::uint16_t length = 0;          ///< total PDU length in bytes
};

/// @brief Euler-angle orientation (radians), the DIS spatial orientation representation.
struct EulerAngles {
  float psi_rad = 0.0F;    ///< heading
  float theta_rad = 0.0F;  ///< pitch
  float phi_rad = 0.0F;    ///< roll
};

/// @brief Convert a body→ENU quaternion to DIS Euler angles (ZYX: psi, theta, phi).
EulerAngles quaternionToEuler(const Quaternion& q);

/// @brief Convert DIS Euler angles back to a body→ENU quaternion.
Quaternion eulerToQuaternion(const EulerAngles& e);

/// @brief Encode an `EntityStateMsg` to a complete DIS Entity State PDU (header + body).
/// @param msg          the entity snapshot
/// @param exercise_id  DIS exercise id (from the bus `start()`)
/// @param site_id      simulation address site id (entity id high word)
/// @param app_id       simulation address application id
std::vector<std::uint8_t> encodeEntityState(const EntityStateMsg& msg, std::uint8_t exercise_id,
                                            std::uint16_t site_id, std::uint16_t app_id);

/// @brief Decode a DIS Entity State PDU back into an `EntityStateMsg`. Throws on a malformed
/// buffer.
EntityStateMsg decodeEntityState(const std::vector<std::uint8_t>& pdu);

/// @brief Encode a `TrackMsg` to our experimental Track/Detection PDU.
std::vector<std::uint8_t> encodeTrack(const TrackMsg& msg, std::uint8_t exercise_id,
                                      std::uint16_t site_id, std::uint16_t app_id);

/// @brief Decode our Track/Detection PDU back into a `TrackMsg`. Throws on a malformed buffer.
TrackMsg decodeTrack(const std::vector<std::uint8_t>& pdu);

/// @brief Read just the PDU header (peek the type to dispatch a received datagram).
PduHeader peekHeader(const std::vector<std::uint8_t>& pdu);

}  // namespace gncsim::interop::dis
