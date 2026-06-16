// gnc-sim — interop: deterministic record + replay of a federated run (issue #47).
//
// Record format (all big-endian via ByteIo, so byte-identical on any host):
//   magic "GNCR" (4) | version u16 (2) | exercise/site/app u16×3 (6) | snapshot_count u64 (8)
//   then per snapshot:
//     step u64 (8) | t_s f64 (8) | n_entities u32 (4) | n_tracks u32 (4)
//     entity×N:  id u32 | force u8 | kind u8 | t_s f64 | pos x,y,z f64 | vel x,y,z f64
//                | att w,x,y,z f64 | angvel x,y,z f64
//     track×M:   id u32 | t_s f64 | pos x,y,z f64 | vel x,y,z f64 | quality f64
//
// Entities/tracks store full f64 bit patterns (not the lossy DIS f32 fields), so replay reproduces
// the snapshot stream exactly — a recorded run replays byte-identically.
#include "gncsim/interop/Recording.hpp"

#include <fstream>
#include <stdexcept>

#include "gncsim/interop/ByteIo.hpp"

namespace gncsim::interop {

namespace {

void writeVec3(ByteWriter& w, const Vector3& v) {
  w.f64(v.x);
  w.f64(v.y);
  w.f64(v.z);
}

Vector3 readVec3(ByteReader& r) {
  Vector3 v;
  v.x = r.f64();
  v.y = r.f64();
  v.z = r.f64();
  return v;
}

}  // namespace

void FileRecorder::onStart(std::uint16_t exercise_id, std::uint16_t site_id, std::uint16_t app_id) {
  ByteWriter w;
  for (char c : kRecordingMagic) w.u8(static_cast<std::uint8_t>(c));
  w.u16(kRecordingVersion);
  w.u16(exercise_id);
  w.u16(site_id);
  w.u16(app_id);
  // Snapshot count is patched in onStop()/writeToFile(); remember where it lives.
  count_field_pos_ = w.size();
  w.u64(0);
  buf_ = w.data();
  snapshot_count_ = 0;
  header_written_ = true;
}

void FileRecorder::publish(const StepSnapshot& snap) {
  if (!header_written_) onStart(0, 0, 0);
  ByteWriter w;
  w.u64(snap.step);
  w.f64(snap.t_s);
  w.u32(static_cast<std::uint32_t>(snap.entities.size()));
  w.u32(static_cast<std::uint32_t>(snap.tracks.size()));
  for (const auto& e : snap.entities) {
    w.u32(e.entity_id);
    w.u8(e.force_id);
    w.u8(e.entity_kind);
    w.f64(e.t_s);
    writeVec3(w, e.pos_m);
    writeVec3(w, e.vel_mps);
    w.f64(e.att.w);
    w.f64(e.att.x);
    w.f64(e.att.y);
    w.f64(e.att.z);
    writeVec3(w, e.ang_vel_radps);
  }
  for (const auto& tr : snap.tracks) {
    w.u32(tr.track_id);
    w.f64(tr.t_s);
    writeVec3(w, tr.pos_est_m);
    writeVec3(w, tr.vel_est_mps);
    w.f64(tr.quality);
  }
  const auto& chunk = w.data();
  buf_.insert(buf_.end(), chunk.begin(), chunk.end());
  ++snapshot_count_;

  // Patch the snapshot-count field in place (big-endian) so the buffer is always self-consistent.
  std::uint64_t n = snapshot_count_;
  for (int i = 7; i >= 0; --i) {
    buf_[count_field_pos_ + static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(n & 0xFF);
    n >>= 8;
  }
}

void FileRecorder::writeToFile(const std::string& path) const {
  std::ofstream out(path, std::ios::binary);
  if (!out) throw std::runtime_error("FileRecorder: cannot write " + path);
  out.write(reinterpret_cast<const char*>(buf_.data()), static_cast<std::streamsize>(buf_.size()));
}

std::vector<StepSnapshot> replay(const std::vector<std::uint8_t>& recording) {
  ByteReader r(recording);
  for (char c : kRecordingMagic) {
    if (r.u8() != static_cast<std::uint8_t>(c)) throw std::runtime_error("replay: bad magic");
  }
  if (r.u16() != kRecordingVersion) throw std::runtime_error("replay: unsupported version");
  r.u16();  // exercise id
  r.u16();  // site id
  r.u16();  // app id
  const std::uint64_t count = r.u64();

  std::vector<StepSnapshot> out;
  out.reserve(count);
  for (std::uint64_t s = 0; s < count; ++s) {
    StepSnapshot snap;
    snap.step = r.u64();
    snap.t_s = r.f64();
    const std::uint32_t n_ent = r.u32();
    const std::uint32_t n_trk = r.u32();
    snap.entities.reserve(n_ent);
    for (std::uint32_t i = 0; i < n_ent; ++i) {
      EntityStateMsg e;
      e.entity_id = r.u32();
      e.force_id = r.u8();
      e.entity_kind = r.u8();
      e.t_s = r.f64();
      e.pos_m = readVec3(r);
      e.vel_mps = readVec3(r);
      e.att.w = r.f64();
      e.att.x = r.f64();
      e.att.y = r.f64();
      e.att.z = r.f64();
      e.ang_vel_radps = readVec3(r);
      snap.entities.push_back(e);
    }
    snap.tracks.reserve(n_trk);
    for (std::uint32_t i = 0; i < n_trk; ++i) {
      TrackMsg tr;
      tr.track_id = r.u32();
      tr.t_s = r.f64();
      tr.pos_est_m = readVec3(r);
      tr.vel_est_mps = readVec3(r);
      tr.quality = r.f64();
      snap.tracks.push_back(tr);
    }
    out.push_back(std::move(snap));
  }
  return out;
}

std::vector<StepSnapshot> replayFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("replayFile: cannot open " + path);
  std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(in)),
                                  std::istreambuf_iterator<char>());
  return replay(bytes);
}

}  // namespace gncsim::interop
