/// @file ByteIo.hpp
/// @brief Endian-explicit, std-only byte (de)serialisation primitives for the interop wire/record
///        formats (issue #47).
///
/// DIS (IEEE 1278) is a big-endian (network byte order) wire protocol; our record file reuses the
/// same primitives so encode/decode share one code path. Everything here is portable C++17 (no
/// `<arpa/inet.h>`, no `reinterpret_cast` punning of multi-byte integers): bytes are assembled and
/// read one at a time, so the output is identical on any host endianness — which is exactly what
/// the "same bytes on every machine" determinism requirement needs.
#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace gncsim::interop {

/// @brief Append-only big-endian byte writer.
class ByteWriter {
 public:
  void u8(std::uint8_t v) { buf_.push_back(v); }

  void u16(std::uint16_t v) {
    buf_.push_back(static_cast<std::uint8_t>((v >> 8) & 0xFF));
    buf_.push_back(static_cast<std::uint8_t>(v & 0xFF));
  }

  void u32(std::uint32_t v) {
    for (int s = 24; s >= 0; s -= 8) buf_.push_back(static_cast<std::uint8_t>((v >> s) & 0xFF));
  }

  void u64(std::uint64_t v) {
    for (int s = 56; s >= 0; s -= 8) buf_.push_back(static_cast<std::uint8_t>((v >> s) & 0xFF));
  }

  void i16(std::int16_t v) { u16(static_cast<std::uint16_t>(v)); }
  void i32(std::int32_t v) { u32(static_cast<std::uint32_t>(v)); }

  /// @brief IEEE-754 double, written as its big-endian 64-bit bit pattern (bit-exact round-trip).
  void f64(double v) {
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(v), "double must be 64-bit");
    std::memcpy(&bits, &v, sizeof(bits));
    u64(bits);
  }

  /// @brief IEEE-754 float, written as its big-endian 32-bit bit pattern.
  void f32(float v) {
    std::uint32_t bits = 0;
    static_assert(sizeof(bits) == sizeof(v), "float must be 32-bit");
    std::memcpy(&bits, &v, sizeof(bits));
    u32(bits);
  }

  void bytes(const std::uint8_t* p, std::size_t n) { buf_.insert(buf_.end(), p, p + n); }

  /// @brief Zero-pad to the requested total length (DIS PDUs are word-aligned).
  void padTo(std::size_t total) {
    while (buf_.size() < total) buf_.push_back(0);
  }

  const std::vector<std::uint8_t>& data() const { return buf_; }
  std::size_t size() const { return buf_.size(); }

 private:
  std::vector<std::uint8_t> buf_;
};

/// @brief Bounds-checked big-endian byte reader. Throws on underrun so a truncated/garbage buffer
///        fails loudly rather than reading past the end.
class ByteReader {
 public:
  ByteReader(const std::uint8_t* p, std::size_t n) : p_(p), n_(n) {}
  explicit ByteReader(const std::vector<std::uint8_t>& v) : p_(v.data()), n_(v.size()) {}

  std::uint8_t u8() {
    need(1);
    return p_[pos_++];
  }

  std::uint16_t u16() {
    need(2);
    const std::uint16_t v = static_cast<std::uint16_t>((static_cast<std::uint16_t>(p_[pos_]) << 8) |
                                                       static_cast<std::uint16_t>(p_[pos_ + 1]));
    pos_ += 2;
    return v;
  }

  std::uint32_t u32() {
    need(4);
    std::uint32_t v = 0;
    for (int i = 0; i < 4; ++i) v = (v << 8) | p_[pos_++];
    return v;
  }

  std::uint64_t u64() {
    need(8);
    std::uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v = (v << 8) | p_[pos_++];
    return v;
  }

  std::int16_t i16() { return static_cast<std::int16_t>(u16()); }
  std::int32_t i32() { return static_cast<std::int32_t>(u32()); }

  double f64() {
    const std::uint64_t bits = u64();
    double v = 0.0;
    std::memcpy(&v, &bits, sizeof(v));
    return v;
  }

  float f32() {
    const std::uint32_t bits = u32();
    float v = 0.0F;
    std::memcpy(&v, &bits, sizeof(v));
    return v;
  }

  /// @brief Skip `n` bytes (e.g. reserved/padding fields).
  void skip(std::size_t n) {
    need(n);
    pos_ += n;
  }

  std::size_t pos() const { return pos_; }
  std::size_t remaining() const { return n_ - pos_; }

 private:
  void need(std::size_t n) const {
    if (pos_ + n > n_) throw std::out_of_range("ByteReader: buffer underrun");
  }
  const std::uint8_t* p_;
  std::size_t n_;
  std::size_t pos_ = 0;
};

}  // namespace gncsim::interop
