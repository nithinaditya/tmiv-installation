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
#define _TMIV_MIVBITSTREAM_PACKEDINDEPENDENTREGIONS_H_

#include <TMIV/Common/Bitstream.h>

#include <optional>
#include <variant>
#include <vector>

namespace TMIV::MivBitstream {
struct TileRegion {
  auto operator==(const TileRegion &other) const noexcept -> bool {
    return (pir_top_left_tile_idx == other.pir_top_left_tile_idx) &&
           (pir_bottom_right_tile_idx == other.pir_bottom_right_tile_idx);
  }

  std::size_t pir_top_left_tile_idx{};
  std::size_t pir_bottom_right_tile_idx{};
};

using TileRegions = std::vector<TileRegion>;
using subPicIds = std::vector<std::size_t>;

struct PirPackedFrame {
  auto operator==(const PirPackedFrame &other) const noexcept -> bool {
    return (pir_packed_frame_id == other.pir_packed_frame_id) && (regions == other.regions);
  }

  std::uint8_t pir_packed_frame_id{};
  std::variant<TileRegions, subPicIds> regions{TileRegions(1U)};
};

// 23090-12: packed_independent_regions ( payloadSize )
class PackedIndependentRegions {
public:
  [[nodiscard]] auto pir_num_packed_frames_minus1() const noexcept -> std::uint8_t;
  [[nodiscard]] auto pir_packed_frame_id(std::uint8_t j) const noexcept -> std::uint8_t;
  [[nodiscard]] auto pir_description_type_idc(std::uint8_t k) const noexcept -> std::uint8_t;
  [[nodiscard]] auto pir_num_regions_minus1(std::uint8_t k) const -> std::uint8_t;
  [[nodiscard]] auto pir_top_left_tile_idx(std::uint8_t k, std::uint8_t i) const -> std::size_t;
  [[nodiscard]] auto pir_bottom_right_tile_idx(std::uint8_t k, std::uint8_t i) const -> std::size_t;
  [[nodiscard]] auto pir_subpic_id(std::uint8_t k, std::uint8_t i) const -> std::size_t;

  auto pir_num_packed_frames_minus1(std::uint8_t value) -> auto &;
  auto pir_packed_frame_id(std::uint8_t j, std::uint8_t value) noexcept -> auto &;
  auto pir_description_type_idc(std::uint8_t k, std::uint8_t value) noexcept -> auto &;
  auto pir_num_regions_minus1(std::uint8_t k, std::uint8_t value) -> auto &;
  auto pir_top_left_tile_idx(std::uint8_t k, std::uint8_t i, std::size_t value) noexcept -> auto &;
  auto pir_bottom_right_tile_idx(std::uint8_t k, std::uint8_t i, std::size_t value) noexcept
      -> auto &;
  auto pir_subpic_id(std::uint8_t k, std::uint8_t i, std::size_t value) noexcept -> auto &;

  friend auto operator<<(std::ostream &stream, const PackedIndependentRegions &x) -> std::ostream &;

  auto operator==(const PackedIndependentRegions &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> PackedIndependentRegions;
  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  std::vector<PirPackedFrame> m_pirPackedFrames{std::vector<PirPackedFrame>(1U)};
};

} // namespace TMIV::MivBitstream

#include "PackedIndependentRegions.hpp"

#endif
