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

#ifndef _TMIV_VIEWINGSPACE_SIGNEDDISTANCE_H_
#define _TMIV_VIEWINGSPACE_SIGNEDDISTANCE_H_

#include <TMIV/Common/Quaternion.h>
#include <TMIV/Common/Vector.h>
#include <TMIV/MivBitstream/ViewingSpace.h>

namespace TMIV::ViewingSpace {
//! \brief Represents a distance from the surface of a shape.
struct SignedDistance {
  float value;

  SignedDistance() : value(std::numeric_limits<float>::max()) {}
  explicit SignedDistance(float d) : value(d) {}
  explicit operator float() const { return value; }

  SignedDistance(const SignedDistance &other) = default;

  //! \brief Compute signed distance corresponding to union of shapes.
  auto operator+(const SignedDistance &other) const -> SignedDistance {
    return SignedDistance(std::min(value, other.value));
  }

  //! \brief Compute approximate signed distance corresponding to subtraction of shapes.
  auto operator-(const SignedDistance &other) const -> SignedDistance {
    return SignedDistance(std::max(value, -other.value));
  }

  //! \brief Compute signed distance corresponding to intersection of shapes.
  auto operator&(const SignedDistance &other) const -> SignedDistance {
    return SignedDistance(std::max(value, other.value));
  }

  auto operator+=(const SignedDistance &other) -> SignedDistance & {
    *this = (*this + other);
    return *this;
  }
  auto operator-=(const SignedDistance &other) -> SignedDistance & {
    *this = (*this - other);
    return *this;
  }
  auto operator&=(const SignedDistance &other) -> SignedDistance & {
    *this = (*this & other);
    return *this;
  }

  [[nodiscard]] auto isInside() const -> bool { return value < 0.f; }
  [[nodiscard]] auto isOutside() const -> bool { return !isInside(); }
};

//! \brief Compute signed distance between a point and a rotated cuboid primitive.
auto signedDistance(const MivBitstream::Cuboid &cuboid, const Common::QuatF &rotation,
                    const Common::Vec3f &point) -> SignedDistance;

//! \brief Compute signed distance between a point and a rotated spheroid primitive.
auto signedDistance(const MivBitstream::Spheroid &spheroid, const Common::QuatF &rotation,
                    const Common::Vec3f &point) -> SignedDistance;

//! \brief Compute signed distance between a point and a rotated half-space primitive.
auto signedDistance(const MivBitstream::Halfspace &halfspace, const Common::QuatF &rotation,
                    const Common::Vec3f &point) -> SignedDistance;

//! \brief Compute signed distance between a point and a PrimitiveShape.
auto signedDistance(const MivBitstream::PrimitiveShape &shape, const Common::Vec3f &point)
    -> SignedDistance;

} // namespace TMIV::ViewingSpace

#endif
