/// @file Quaternion.hpp
/// @brief Unit quaternion for 6DOF attitude.
///
/// Hamilton convention, scalar-first @f$(w,x,y,z)@f$. `rotate()` maps a vector from the
/// body frame to the world (ENU) frame. Attitude kinematics @f$\dot q = \tfrac12 q\otimes\omega@f$;
/// see docs/THEORY.md §1.
#pragma once

#include <cmath>

#include "gncsim/math/Vector3.hpp"

namespace gncsim {

/// @brief Scalar-first unit quaternion @f$(w,x,y,z)@f$ rotating body→world (ENU).
struct Quaternion {
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  constexpr Quaternion() = default;
  constexpr Quaternion(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}

  double norm() const { return std::sqrt(w * w + x * x + y * y + z * z); }

  Quaternion normalized() const {
    const double n = norm();
    if (n <= 0.0) return Quaternion{};
    return {w / n, x / n, y / n, z / n};
  }

  // Hamilton product (this ⊗ o).
  Quaternion operator*(const Quaternion& o) const {
    return {
        w * o.w - x * o.x - y * o.y - z * o.z,
        w * o.x + x * o.w + y * o.z - z * o.y,
        w * o.y - x * o.z + y * o.w + z * o.x,
        w * o.z + x * o.y - y * o.x + z * o.w,
    };
  }

  Quaternion conjugate() const { return {w, -x, -y, -z}; }

  // Rotate a body-frame vector into the world frame.
  Vector3 rotate(const Vector3& v) const {
    const Quaternion q = normalized();
    const Quaternion p{0.0, v.x, v.y, v.z};
    const Quaternion r = q * p * q.conjugate();
    return {r.x, r.y, r.z};
  }

  static Quaternion fromAxisAngle(const Vector3& axis, double angle_rad) {
    const Vector3 a = axis.normalized();
    const double s = std::sin(angle_rad * 0.5);
    return {std::cos(angle_rad * 0.5), a.x * s, a.y * s, a.z * s};
  }

  // 3-2-1 (yaw-pitch-roll) Euler angles in radians.
  static Quaternion fromEuler(double roll, double pitch, double yaw) {
    const double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
    return Quaternion{
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    }
        .normalized();
  }

  // Returns (roll, pitch, yaw) in radians.
  Vector3 toEuler() const {
    const Quaternion q = normalized();
    const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    const double roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    sinp = sinp > 1.0 ? 1.0 : (sinp < -1.0 ? -1.0 : sinp);
    const double pitch = std::asin(sinp);

    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const double yaw = std::atan2(siny_cosp, cosy_cosp);
    return {roll, pitch, yaw};
  }

  // Quaternion derivative for body angular rate omega (rad/s). q_dot = 0.5 * q ⊗ [0, omega].
  Quaternion derivative(const Vector3& omega) const {
    const Quaternion wq{0.0, omega.x, omega.y, omega.z};
    const Quaternion d = (*this) * wq;
    return {0.5 * d.w, 0.5 * d.x, 0.5 * d.y, 0.5 * d.z};
  }
};

}  // namespace gncsim
