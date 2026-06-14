/// @file Recording.hpp
/// @brief Deterministic record + replay of a federated run (issue #47).
///
/// `FileRecorder` is an `IFederateAdapter` that serialises every bus snapshot to an in-memory byte
/// buffer in a fixed, endian-explicit format (`ByteIo.hpp`). The buffer can be written to a file
/// and re-read; `replay()` reconstructs the exact `StepSnapshot` sequence. Because the format is
/// big-endian and fully explicit (no host-layout, no float rounding — `double`s are stored as their
/// 64-bit bit pattern), the same run records to **byte-identical** bytes on any host, and
/// record→replay→re-record reproduces the same bytes. That is the determinism invariant the test
/// asserts.
///
/// A recorded session can be re-published through a fresh `MessageBus` (with, say, a DisAdapter
/// attached) to drive a federation from a canned run — replaying a federated exercise
/// deterministically.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "gncsim/interop/Federation.hpp"

namespace gncsim::interop {

/// @brief Magic + version stamped at the head of a recording (so a stale format is rejected).
constexpr char kRecordingMagic[4] = {'G', 'N', 'C', 'R'};
constexpr std::uint16_t kRecordingVersion = 1;

/// @brief Records bus snapshots to a replayable byte stream.
class FileRecorder : public IFederateAdapter {
 public:
  void onStart(std::uint16_t exercise_id, std::uint16_t site_id, std::uint16_t app_id) override;
  void publish(const StepSnapshot& snap) override;
  void onStop() override {}

  /// @brief The recorded bytes (header + every snapshot). Stable/deterministic for a given run.
  const std::vector<std::uint8_t>& bytes() const { return buf_; }

  /// @brief Write the recording to a file. (Driver-side I/O; not used by the pure core.)
  void writeToFile(const std::string& path) const;

 private:
  std::vector<std::uint8_t> buf_;
  std::uint64_t snapshot_count_ = 0;
  std::size_t count_field_pos_ = 0;  ///< byte offset of the snapshot-count field (patched on stop)
  bool header_written_ = false;
};

/// @brief Reconstruct the snapshot sequence from a recording buffer. Throws on a bad/truncated/
///        wrong-version buffer.
std::vector<StepSnapshot> replay(const std::vector<std::uint8_t>& recording);

/// @brief Read a recording file into a snapshot sequence.
std::vector<StepSnapshot> replayFile(const std::string& path);

}  // namespace gncsim::interop
