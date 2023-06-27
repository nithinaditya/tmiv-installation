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

#ifndef _TMIV_COMMON_HALF_H_
#error "Include the .h, not the .hpp"
#endif

#include <cmath>
#include <sstream>

namespace TMIV::Common {
inline Half::operator float() const {
  const auto abs = m_code & 0x7FFFU;
  const auto sign = m_code == abs ? 1.F : -1.F;

  if (abs == 0) {
    return sign * 0.F;
  }

  const auto exponent = static_cast<int>((m_code & 0x7C00U) >> 10U);
  const auto mantissa = (m_code & 0x03FFU) | 0x400U;
  return sign * ldexp(static_cast<float>(mantissa), exponent - 25);
}

inline Half::Half(float value) {
  const auto x = std::abs(value);

  if (!std::isfinite(value) || x > 65504.F) {
    throw HalfError(value);
  }

  if (x < 0x1.p-14F) {
    m_code = 0; // the WD excludes subnormal numbers
  } else {
    int exponent{};
    const auto mantissa_f = std::frexp(x, &exponent);
    const auto mantissa = static_cast<unsigned>(std::lround(std::ldexp(mantissa_f, 11)));
    m_code =
        static_cast<uint16_t>((static_cast<unsigned>(exponent + 14) << 10) | (mantissa & 0x3FF));
  }

  if (std::signbit(value)) {
    m_code |= 0x8000;
  }
}

inline auto Half::decode(uint16_t code) -> Half {
  Half result;
  if ((code & 0x7FFFU) != 0 && ((code & 0x7C00U) == 0 || (code & 0x7C00U) == 0x7C00)) {
    throw HalfError(code);
  }
  result.m_code = code;
  return result;
}

inline auto Half::encode() const -> uint16_t { return m_code; }
} // namespace TMIV::Common
