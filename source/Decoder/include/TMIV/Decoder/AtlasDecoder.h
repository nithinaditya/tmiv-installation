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

#ifndef _TMIV_DECODER_ATLASDECODER_H_
#define _TMIV_DECODER_ATLASDECODER_H_

#include <TMIV/MivBitstream/AtlasFrameParameterSetRBSP.h>
#include <TMIV/MivBitstream/AtlasSequenceParameterSetRBSP.h>
#include <TMIV/MivBitstream/AtlasTileLayerRBSP.h>
#include <TMIV/MivBitstream/SeiRBSP.h>
#include <TMIV/MivBitstream/V3cUnit.h>

#include <functional>
#include <list>

namespace TMIV::Decoder {
using V3cUnitSource = std::function<std::optional<MivBitstream::V3cUnit>()>;

class AtlasDecoder {
public:
  AtlasDecoder() = default;
  explicit AtlasDecoder(V3cUnitSource source, const MivBitstream::V3cUnitHeader &vuh,
                        MivBitstream::V3cParameterSet vps, int32_t foc);

  struct AccessUnit {
    int32_t foc{};
    MivBitstream::AtlasSequenceParameterSetRBSP asps;
    MivBitstream::AtlasFrameParameterSetRBSP afps;
    MivBitstream::AtlasTileLayerRBSP atl;
  };

  auto operator()() -> std::optional<AccessUnit>;

private:
  auto decodeAsb() -> bool;
  auto decodeAu() -> AccessUnit;
  void decodePrefixNalUnit(AccessUnit &au, const MivBitstream::NalUnit &nu);
  void decodeAclNalUnit(AccessUnit &au, const MivBitstream::NalUnit &nu);
  static void decodeSuffixNalUnit(AccessUnit &au, const MivBitstream::NalUnit &nu);
  void decodeAsps(std::istream &stream);
  void decodeAfps(std::istream &stream);
  static void decodeSei(AccessUnit &au, std::istream &stream);
  static void decodeSeiMessage(AccessUnit &au, const MivBitstream::SeiMessage &message);

  V3cUnitSource m_source;
  MivBitstream::V3cUnitHeader m_vuh{MivBitstream::VuhUnitType::V3C_AD};
  MivBitstream::V3cParameterSet m_vps;

  std::list<MivBitstream::NalUnit> m_buffer;
  int32_t m_foc{};

  std::vector<MivBitstream::AtlasSequenceParameterSetRBSP> m_aspsV;
  std::vector<MivBitstream::AtlasFrameParameterSetRBSP> m_afpsV;
  unsigned m_maxAtlasFrmOrderCntLsb{};
};
} // namespace TMIV::Decoder

#endif
