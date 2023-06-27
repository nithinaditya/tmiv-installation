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

#ifndef _TMIV_DECODER_NALUNITSEMANTICS_H_
#define _TMIV_DECODER_NALUNITSEMANTICS_H_

#include <TMIV/MivBitstream/NalUnit.h>

namespace TMIV::Decoder {
constexpr auto isAud(MivBitstream::NalUnitType nut) noexcept -> bool {
  return nut == MivBitstream::NalUnitType::NAL_AUD || nut == MivBitstream::NalUnitType::NAL_V3C_AUD;
}

constexpr auto isPrefixNalUnit(MivBitstream::NalUnitType nut) noexcept -> bool {
  return nut == MivBitstream::NalUnitType::NAL_ASPS || nut == MivBitstream::NalUnitType::NAL_AFPS ||
         nut == MivBitstream::NalUnitType::NAL_PREFIX_NSEI ||
         nut == MivBitstream::NalUnitType::NAL_PREFIX_ESEI ||
         nut == MivBitstream::NalUnitType::NAL_AAPS ||
         nut == MivBitstream::NalUnitType::NAL_CASPS ||
         nut == MivBitstream::NalUnitType::NAL_RSV_NACL_51 ||
         nut == MivBitstream::NalUnitType::NAL_RSV_NACL_52 ||
         nut == MivBitstream::NalUnitType::NAL_RSV_NACL_53 ||
         (MivBitstream::NalUnitType::NAL_UNSPEC_56 <= nut &&
          nut <= MivBitstream::NalUnitType::NAL_UNSPEC_59);
}

constexpr auto isAcl(MivBitstream::NalUnitType nut) noexcept -> bool {
  return nut <= MivBitstream::NalUnitType::NAL_RSV_ACL_35;
}

constexpr auto isCaf(MivBitstream::NalUnitType nut) noexcept -> bool {
  return (nut == MivBitstream::NalUnitType::NAL_CAF) ||
         (nut == MivBitstream::NalUnitType::NAL_IDR_CAF);
}

constexpr auto isSuffixNalUnit(MivBitstream::NalUnitType nut) noexcept -> bool {
  return nut == MivBitstream::NalUnitType::NAL_FD ||
         nut == MivBitstream::NalUnitType::NAL_SUFFIX_NSEI ||
         nut == MivBitstream::NalUnitType::NAL_SUFFIX_ESEI ||
         nut == MivBitstream::NalUnitType::NAL_RSV_NACL_54 ||
         nut == MivBitstream::NalUnitType::NAL_RSV_NACL_55 ||
         MivBitstream::NalUnitType::NAL_UNSPEC_60 <= nut;
}

constexpr auto isEos(MivBitstream::NalUnitType nut) noexcept -> bool {
  return nut == MivBitstream::NalUnitType::NAL_EOS;
}

constexpr auto isEob(MivBitstream::NalUnitType nut) noexcept -> bool {
  return nut == MivBitstream::NalUnitType::NAL_EOB;
}
} // namespace TMIV::Decoder

#endif
