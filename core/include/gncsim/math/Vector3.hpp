// gnc-sim — 3D vector. Header-only, vector-space ops for RK4 integration.
#pragma once

#include <cmath>

namespace gncsim {

struct Vector3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  constexpr Vector3() = default;
  constexpr Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

  // Vector-space ops (required by the RK4 integrator).
  constexpr Vector3 operator+(const Vector3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  constexpr Vector3 operator-(const Vector3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  constexpr Vector3 operator*(double s) const { return {x * s, y * s, z * s}; }
  constexpr Vector3 operator/(double s) const { return {x / s, y / s, z / s}; }
  constexpr Vector3 operator-() const { return {-x, -y, -z}; }

  Vector3& operator+=(const Vector3& o) { x += o.x; y += o.y; z += o.z; return *this; }
  Vector3& operator-=(const Vector3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
  Vector3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }

  double dot(const Vector3& o) const { return x * o.x + y * o.y + z * o.z; }
  Vector3 cross(const Vector3& o) const {
    return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
  }
  double norm() const { return std::sqrt(dot(*this)); }
  double normSq() const { return dot(*this); }
  Vector3 normalized() const {
    const double n = norm();
    return n > 0.0 ? (*this) / n : Vector3{};
  }
};

constexpr Vector3 operator*(double s, const Vector3& v) { return v * s; }

inline double distance(const Vector3& a, const Vector3& b) { return (a - b).norm(); }

}  // namespace gncsim
