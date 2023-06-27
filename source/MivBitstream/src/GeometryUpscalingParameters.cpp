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

#include <TMIV/MivBitstream/GeometryUpscalingParameters.h>

namespace TMIV::MivBitstream {
namespace {
constexpr float inferredErodeThreshold = 1.F;
constexpr unsigned inferredDeltaThreshold = 10;
constexpr std::uint8_t inferredMaxCurvature = 5;
} // namespace

auto operator<<(std::ostream &stream, GupType x) -> std::ostream & {
  if (x == GupType::HVR) {
    return stream << "HVR";
  }
  return stream << "[unknown:" << static_cast<int>(x) << ']';
}

auto GeometryUpscalingParameters::gup_type() const noexcept -> GupType { return m_gup_type; }

auto GeometryUpscalingParameters::gup_erode_threshold() const noexcept -> Common::Half {
  return gup_type() == GupType::HVR ? m_gup_erode_threshold : Common::Half(inferredErodeThreshold);
}

auto GeometryUpscalingParameters::gup_delta_threshold() const noexcept -> unsigned {
  return gup_type() == GupType::HVR ? m_gup_delta_threshold : inferredDeltaThreshold;
}

auto GeometryUpscalingParameters::gup_max_curvature() const noexcept -> std::uint8_t {
  return gup_type() == GupType::HVR ? m_gup_max_curvature : inferredMaxCurvature;
}

auto GeometryUpscalingParameters::gup_type(GupType value) noexcept
    -> GeometryUpscalingParameters & {
  m_gup_type = value;
  return *this;
}

auto GeometryUpscalingParameters::gup_erode_threshold(Common::Half value) noexcept
    -> GeometryUpscalingParameters & {
  VERIFY_BITSTREAM(gup_type() == GupType::HVR);
  m_gup_erode_threshold = value;
  return *this;
}

auto GeometryUpscalingParameters::gup_delta_threshold(unsigned value) noexcept
    -> GeometryUpscalingParameters & {
  VERIFY_BITSTREAM(gup_type() == GupType::HVR);
  m_gup_delta_threshold = value;
  return *this;
}

auto GeometryUpscalingParameters::gup_max_curvature(std::uint8_t value) noexcept
    -> GeometryUpscalingParameters & {
  VERIFY_BITSTREAM(gup_type() == GupType::HVR);
  m_gup_max_curvature = value;
  return *this;
}

auto operator<<(std::ostream &stream, const GeometryUpscalingParameters &x) -> std::ostream & {
  stream << "gup_type=" << x.gup_type() << '\n';
  if (x.gup_type() == GupType::HVR) {
    stream << "gup_erode_threshold=" << x.gup_erode_threshold() << '\n';
    stream << "gup_delta_threshold=" << x.gup_delta_threshold() << '\n';
    stream << "gup_max_curvature=" << static_cast<unsigned>(x.gup_max_curvature()) << '\n';
  }
  return stream;
}

auto GeometryUpscalingParameters::operator==(
    const GeometryUpscalingParameters &other) const noexcept -> bool {
  if (gup_type() != other.gup_type()) {
    return false;
  }
  if (gup_type() != GupType::HVR) {
    return true;
  }
  return gup_erode_threshold() == other.gup_erode_threshold() &&
         gup_delta_threshold() == other.gup_delta_threshold() &&
         gup_max_curvature() == other.gup_max_curvature();
}

auto GeometryUpscalingParameters::operator!=(
    const GeometryUpscalingParameters &other) const noexcept -> bool {
  return !operator==(other);
}

auto GeometryUpscalingParameters::decodeFrom(Common::InputBitstream &bitstream)
    -> GeometryUpscalingParameters {
  auto x = GeometryUpscalingParameters{};
  x.gup_type(bitstream.getUExpGolomb<GupType>());
  if (x.gup_type() == GupType::HVR) {
    x.gup_erode_threshold(bitstream.getFloat16());
    x.gup_delta_threshold(bitstream.getUExpGolomb<unsigned>());
    x.gup_max_curvature(bitstream.readBits<std::uint8_t>(3));
  }
  return x;
}

void GeometryUpscalingParameters::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putUExpGolomb(gup_type());
  if (gup_type() == GupType::HVR) {
    bitstream.putFloat16(Common::Half(gup_erode_threshold()));
    bitstream.putUExpGolomb(gup_delta_threshold());
    bitstream.writeBits(gup_max_curvature(), 3);
  }
}
} // namespace TMIV::MivBitstream
