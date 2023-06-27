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

#ifndef _TMIV_MIVBITSTREAM_TYPES_H_
#define _TMIV_MIVBITSTREAM_TYPES_H_

#include <cstdint>
#include <ostream>

namespace TMIV::MivBitstream {
enum class VuhUnitType : std::uint8_t {
  V3C_VPS,
  V3C_AD,
  V3C_OVD,
  V3C_GVD,
  V3C_AVD,
  V3C_PVD,
  V3C_CAD
};

constexpr auto operator<<(std::ostream &stream, const VuhUnitType x) -> std::ostream & {
  switch (x) {
  case VuhUnitType::V3C_VPS:
    return stream << "V3C_VPS";
  case VuhUnitType::V3C_AD:
    return stream << "V3C_AD";
  case VuhUnitType::V3C_OVD:
    return stream << "V3C_OVD";
  case VuhUnitType::V3C_GVD:
    return stream << "V3C_GVD";
  case VuhUnitType::V3C_AVD:
    return stream << "V3C_AVD";
  case VuhUnitType::V3C_PVD:
    return stream << "V3C_PVD";
  case VuhUnitType::V3C_CAD:
    return stream << "V3C_CAD";
  default:
    return stream << "[unknown:" << static_cast<int>(x) << "]";
  }
}

constexpr auto operator==(VuhUnitType vuh_unit_type, std::uint8_t underlying_value) noexcept
    -> bool {
  return static_cast<std::uint8_t>(vuh_unit_type) == underlying_value;
}

constexpr auto operator==(std::uint8_t underlying_value, VuhUnitType vuh_unit_type) noexcept
    -> bool {
  return operator==(vuh_unit_type, underlying_value);
}

} // namespace TMIV::MivBitstream

#endif
