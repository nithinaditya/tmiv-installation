/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2020, ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _TMIV_COMMON_QUATERNION_H_
#define _TMIV_COMMON_QUATERNION_H_

#include "Matrix.h"
#include "Vector.h"

namespace TMIV::Common {
// Quaternion: q = w + ix + jy + kz with i^2 = j^2 + k^2 = -1
template <typename T> using Quaternion = stack::Vec4<T>;
using QuatF = Quaternion<float>;
using QuatD = Quaternion<double>;

// Quaternion product: a b
template <typename T1, typename T2>
auto operator*(const Quaternion<T1> &a, const Quaternion<T2> &b) {
  using R = std::common_type_t<T1, T2>;
  return Quaternion<R>{
      a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y(), // x
      a.w() * b.y() - a.x() * b.z() + a.y() * b.w() + a.z() * b.x(), // y
      a.w() * b.z() + a.x() * b.y() - a.y() * b.x() + a.z() * b.w(), // z
      a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z()  // w
  };
}

// Quaternion conjugent: q -> w - ix - jy - kz
template <typename T> auto conj(const Quaternion<T> &q) {
  return Quaternion<T>{-q.x(), -q.y(), -q.z(), q.w()};
}

// Unit quaternion test: ||q|| == 1
template <typename T, typename Tolerance = T>
auto normalized(const Quaternion<T> &q, Tolerance tol = static_cast<T>(1e-6F)) {
  return (norm(q) - 1) <= tol;
}

// Vector quaternion: v = (x, y, z) -> ix + jy + kz
template <typename T> auto quat(const stack::Vec3<T> &v) {
  return Quaternion<T>{v.x(), v.y(), v.z(), 0.F};
}

// Conjugate quaternion p by unit quaternion q: qpq^-1
//
// Precondition: ||q|| == 1
template <typename T1, typename T2> auto rotate(const Quaternion<T1> &p, const Quaternion<T2> &q) {
  assert(normalized(q));
  return q * p * conj(q);
}

// Rotate a vector by a unit quaternion
template <typename T1, typename T2> auto rotate(const stack::Vec3<T1> &v, const Quaternion<T2> &q) {
  const auto p = rotate(quat(v), q);
  using T3 = typename decltype(p)::value_type;
  return stack::Vec3<T3>{p.x(), p.y(), p.z()};
}

// Euler angles (yaw, pitch, roll) [rad] to quaternion conversion
//
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
template <typename T> auto euler2quat(const stack::Vec3<T> &eulerAngles) {
  using Q = Quaternion<T>;
  using std::cos;
  using std::sin;

  const auto y = eulerAngles[0]; // yaw rotation [rad]
  const auto p = eulerAngles[1]; // pitch rotation [rad]
  const auto r = eulerAngles[2]; // roll rotation [rad]

  constexpr auto zero = T{};
  constexpr auto half = T{0.5F};

  const auto qy = Q{zero, zero, sin(half * y), cos(half * y)};
  const auto qp = Q{zero, sin(half * p), zero, cos(half * p)};
  const auto qr = Q{sin(half * r), zero, zero, cos(half * r)};

  return qy * qp * qr;
}

template <typename T> auto quat2euler(const Quaternion<T> &q) {
  constexpr auto one = T{1.F};
  constexpr auto two = T{2.F};
  constexpr auto halfPi = static_cast<T>(M_PI2);

  const auto cYaw = sqr(q.x()) - sqr(q.y()) - sqr(q.z()) + sqr(q.w());
  const auto sYaw = two * (q.w() * q.z() + q.x() * q.y());
  const auto yaw = std::atan2(sYaw, cYaw);

  const auto sPitch = two * (q.w() * q.y() - q.z() * q.x());
  const auto pitch = std::abs(sPitch) < one ? std::asin(sPitch) : std::copysign(halfPi, sPitch);

  const auto cRoll = -sqr(q.x()) - sqr(q.y()) + sqr(q.z()) + sqr(q.w());
  const auto sRoll = two * (q.w() * q.x() + q.y() * q.z());
  const auto roll = std::atan2(sRoll, cRoll);

  return stack::Vec3<T>{yaw, pitch, roll};
}

// Unit quaternion (q) to rotation matrix (R) conversion
//
// The matrix R has the following property:
//
//    quat(Rv) == rotate(v, q))
//
// Precondition: ||q|| == 1
template <typename T> auto rotationMatrix(const Quaternion<T> &q) {
  assert(normalized(q));

  constexpr auto one = T{1};
  constexpr auto two = T{2};

  return stack::Mat3x3<T>{one - two * (q.y() * q.y() + q.z() * q.z()),  // R_xx
                          two * (q.x() * q.y() - q.z() * q.w()),        // R_xy
                          two * (q.z() * q.x() + q.y() * q.w()),        // R_xz
                          two * (q.x() * q.y() + q.z() * q.w()),        // R_yx
                          one - two * (q.z() * q.z() + q.x() * q.x()),  // R_yy
                          two * (q.y() * q.z() - q.x() * q.w()),        // R_yz
                          two * (q.z() * q.x() - q.y() * q.w()),        // R_zx
                          two * (q.y() * q.z() + q.x() * q.w()),        // R_zy
                          one - two * (q.x() * q.x() + q.y() * q.y())}; // R_zz
}

// Great-circle distance (aka orthodromic distance)
//
// https://en.wikipedia.org/wiki/Great-circle_distance
template <typename T1, typename T2>
auto greatCircleDistance(const Quaternion<T1> &q1, const Quaternion<T2> &q2) {
  using std::abs;
  using std::acos;
  using std::atan;

  using R = std::common_type_t<T1, T2>;
  const auto forward = stack::Vec3<R>{R(1), R(0), R(0)};
  const auto v1 = rotate(forward, q1);
  const auto v2 = rotate(forward, q2);
  const auto dot12 = dot(v1, v2);
  const auto cross12 = norm(cross(v1, v2));
  return dot12 > cross12 ? atan(cross12 / dot12) : acos(dot12);
}
} // namespace TMIV::Common

#endif
