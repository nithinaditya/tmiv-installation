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

#include <TMIV/ViewingSpace/SignedDistance.h>

#include <TMIV/Common/LinAlg.h>

namespace TMIV::ViewingSpace {
// Based on https://www.iquilezles.org/www/articles/distfunctions/distfunctions.htm
auto signedDistance(const MivBitstream::Cuboid &cuboid, const Common::QuatF &rotation,
                    const Common::Vec3f &point) -> SignedDistance {
  assert(cuboid.center.has_value() && "Cuboid center must have a value assigned");
  const auto p = rotate(point - *cuboid.center, conj(rotation));
  const auto q = abs(p) - 0.5F * cuboid.size;
  return SignedDistance(norm(max(q, 0.F)) + std::min(std::max(q.x(), std::max(q.y(), q.z())), 0.F));
}

// Based on https://www.iquilezles.org/www/articles/distfunctions/distfunctions.htm
auto signedDistance(const MivBitstream::Spheroid &spheroid, const Common::QuatF &rotation,
                    const Common::Vec3f &point) -> SignedDistance {
  assert(spheroid.center.has_value() && "Spheroid center must have a value assigned");
  const auto p = rotate(point - *spheroid.center, conj(rotation));
  const auto r = spheroid.radius;
  const auto r2 = Common::Vec3f({r.x() * r.x(), r.y() * r.y(), r.z() * r.z()});
  float k0 = norm(Common::Vec3f({p.x() / r.x(), p.y() / r.y(), p.z() / r.z()}));
  float k1 = norm(Common::Vec3f({p.x() / r2.x(), p.y() / r2.y(), p.z() / r2.z()}));
  if (k1 < 1.0e-3F) {
    return SignedDistance(-std::min(r.x(), std::min(r.y(), r.z())));
  }
  return SignedDistance(k0 * (k0 - 1.0F) / k1);
}

auto signedDistance(const MivBitstream::Halfspace &halfspace, const Common::QuatF &rotation,
                    const Common::Vec3f &point) -> SignedDistance {
  const auto p = rotate(point, conj(rotation));
  return SignedDistance(dot(halfspace.normal, p) - halfspace.distance);
}

auto signedDistance(const MivBitstream::PrimitiveShape &shape, const Common::Vec3f &point)
    -> SignedDistance {
  const auto rotation = shape.rotation.value_or(Common::QuatF{0.F, 0.F, 0.F, 1.F});
  return std::visit([&](auto &&s) { return signedDistance(s, rotation, point); }, shape.primitive);
}

} // namespace TMIV::ViewingSpace
