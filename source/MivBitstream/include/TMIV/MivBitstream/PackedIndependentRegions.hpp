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

#ifndef _TMIV_MIVBITSTREAM_PACKEDINDEPENDENTREGIONS_H_
#error "Include the .h, not the .hpp"
#endif

#include <TMIV/Common/verify.h>

namespace TMIV::MivBitstream {
inline auto PackedIndependentRegions::pir_num_packed_frames_minus1(std::uint8_t value) -> auto & {
  m_pirPackedFrames = std::vector<PirPackedFrame>(value + 1U);
  return *this;
}
inline auto PackedIndependentRegions::pir_packed_frame_id(std::uint8_t j,
                                                          std::uint8_t value) noexcept -> auto & {
  m_pirPackedFrames[j].pir_packed_frame_id = value;
  return *this;
}
inline auto PackedIndependentRegions::pir_description_type_idc(std::uint8_t k,
                                                               std::uint8_t value) noexcept
    -> auto & {
  if (value == 0) {
    m_pirPackedFrames[k].regions = TileRegions(1U);
  } else if (value == 1) {
    m_pirPackedFrames[k].regions = subPicIds(1U);
  } else {
    VERIFY_V3CBITSTREAM(false); // Only defined for 0 and 1. Other values are reserved
  }
  return *this;
}
inline auto PackedIndependentRegions::pir_num_regions_minus1(std::uint8_t k, std::uint8_t value)
    -> auto & {
  if (pir_description_type_idc(k) == 0) {
    m_pirPackedFrames[k].regions = TileRegions(value + 1U);
  } else {
    m_pirPackedFrames[k].regions = subPicIds(value + 1U);
  }
  return *this;
}
inline auto PackedIndependentRegions::pir_top_left_tile_idx(std::uint8_t k, std::uint8_t i,
                                                            std::size_t value) noexcept -> auto & {
  auto &tileRegions = std::get<TileRegions>(m_pirPackedFrames[k].regions);
  tileRegions[i].pir_top_left_tile_idx = value;
  return *this;
}
inline auto PackedIndependentRegions::pir_bottom_right_tile_idx(std::uint8_t k, std::uint8_t i,
                                                                std::size_t value) noexcept
    -> auto & {
  auto &tileRegions = std::get<TileRegions>(m_pirPackedFrames[k].regions);
  tileRegions[i].pir_bottom_right_tile_idx = value;
  return *this;
}
inline auto PackedIndependentRegions::pir_subpic_id(std::uint8_t k, std::uint8_t i,
                                                    std::size_t value) noexcept -> auto & {
  auto &subPicId = std::get<subPicIds>(m_pirPackedFrames[k].regions);
  subPicId[i] = value;
  return *this;
}
} // namespace TMIV::MivBitstream
