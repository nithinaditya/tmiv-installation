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

#ifndef _TMIV_COMMON_BITSTREAM_H_
#error "Include the .h, not the .hpp"
#endif

#include <TMIV/Common/verify.h>

namespace TMIV::Common {
using uchar = std::make_unsigned_t<std::istream::char_type>;
constexpr unsigned charBits = std::numeric_limits<uchar>::digits;

template <typename Integer> auto InputBitstream::readBits(unsigned bits) -> Integer {
  while (m_size < bits) {
    VERIFY_BITSTREAM(m_size + charBits <= std::numeric_limits<uint64_t>::digits);
    VERIFY_BITSTREAM(m_stream.good());

    const auto value = m_stream.get();
    m_buffer = (m_buffer << charBits) | uchar(value);
    m_size += charBits;
  }

  m_size -= bits;
  auto value = m_buffer >> m_size;
  m_buffer &= (1 << m_size) - 1;

  VERIFY_BITSTREAM(static_cast<uint64_t>(Integer(value)) == value);
  return Integer(value);
}

template <typename Integer> auto InputBitstream::getUVar(uint64_t range) -> Integer {
  return readBits<Integer>(ceilLog2(range));
}

template <typename Integer> auto InputBitstream::getUExpGolomb() -> Integer {
  auto leadingBits = 0U;
  while (!getFlag()) {
    ++leadingBits;
  }
  const auto mask = (uint64_t{1} << leadingBits) - 1;
  const auto value = mask + readBits<uint64_t>(leadingBits);

  VERIFY_BITSTREAM(static_cast<uint64_t>(Integer(value)) == value);
  return Integer(value);
}

template <typename Integer> auto InputBitstream::getSExpGolomb() -> Integer {
  const auto codeNum = getUExpGolomb<uint64_t>();
  const auto absValue = static_cast<int64_t>((codeNum + 1) / 2);
  const auto value = (codeNum & 1) == 1 ? absValue : -absValue;

  VERIFY_BITSTREAM(static_cast<int64_t>(Integer(value)) == value);
  return Integer(value);
}

template <typename Integer> void OutputBitstream::writeBits(const Integer &value, unsigned bits) {
  if constexpr (std::is_signed_v<Integer>) {
    VERIFY_BITSTREAM(value >= 0);
  }
  writeBits_(std::make_unsigned_t<Integer>(value), bits);
}

template <typename Integer> void OutputBitstream::putUVar(const Integer &value, uint64_t range) {
  if constexpr (std::is_signed_v<Integer>) {
    VERIFY_BITSTREAM(value >= 0);
  }
  putUVar_(std::make_unsigned_t<Integer>(value), range);
}

template <typename Integer> void OutputBitstream::putUExpGolomb(const Integer &value) {
  if constexpr (std::is_signed_v<Integer>) {
    VERIFY_BITSTREAM(value >= 0);
  }
  putUExpGolomb_(std::make_unsigned_t<Integer>(value));
}
} // namespace TMIV::Common
