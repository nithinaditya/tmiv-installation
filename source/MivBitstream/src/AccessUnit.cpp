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

#include <TMIV/MivBitstream/AccessUnit.h>

namespace TMIV::MivBitstream {
auto AtlasAccessUnit::frameSize() const noexcept -> Common::Vec2i {
  return Common::Vec2i{asps.asps_frame_width(), asps.asps_frame_height()};
}

auto AtlasAccessUnit::decGeoFrameSize(const V3cParameterSet &vps) const noexcept -> Common::Vec2i {
  if (vps.vps_miv_extension_present_flag()) {
    const auto &vme = vps.vps_miv_extension();
    if (vme.vme_geometry_scale_enabled_flag()) {
      const auto &asme = asps.asps_miv_extension();
      return Common::Vec2i{
          asps.asps_frame_width() / (asme.asme_geometry_scale_factor_x_minus1() + 1),
          asps.asps_frame_height() / (asme.asme_geometry_scale_factor_y_minus1() + 1)};
    }
  }
  return frameSize();
}

auto AtlasAccessUnit::decOccFrameSize(const V3cParameterSet &vps) const noexcept -> Common::Vec2i {
  if (vps.vps_miv_extension_present_flag()) {
    const auto &vme = vps.vps_miv_extension();
    if (!vme.vme_embedded_occupancy_enabled_flag() && vme.vme_occupancy_scale_enabled_flag()) {
      const auto &asme = asps.asps_miv_extension();
      const int codedUnpaddedOccupancyWidth =
          asps.asps_frame_width() / (asme.asme_occupancy_scale_factor_x_minus1() + 1);
      const int codedUnpadedOccupancyHeight =
          asps.asps_frame_height() / (asme.asme_occupancy_scale_factor_y_minus1() + 1);
      const int codedOccupancyWidth = codedUnpaddedOccupancyWidth + codedUnpaddedOccupancyWidth % 2;
      const int codedOccupancyHeight =
          codedUnpadedOccupancyHeight + codedUnpadedOccupancyHeight % 2;
      return Common::Vec2i{codedOccupancyWidth, codedOccupancyHeight};
    }
  }
  return frameSize();
}

auto AtlasAccessUnit::patchId(unsigned row, unsigned column) const -> std::uint16_t {
  const auto k = asps.asps_log2_patch_packing_block_size();
  return blockToPatchMap.getPlane(0)(row >> k, column >> k);
}

auto AccessUnit::sequenceConfig() const -> SequenceConfig {
  auto x = SequenceConfig{};

  x.contentName = "decoded";

  if (vui && vui->vui_timing_info_present_flag()) {
    x.frameRate = static_cast<double>(vui->vui_time_scale()) /
                  static_cast<double>(vui->vui_num_units_in_tick());
  }

  x.cameras.resize(viewParamsList.size());

  std::transform(viewParamsList.cbegin(), viewParamsList.cend(), x.cameras.begin(),
                 [](const ViewParams &vp) {
                   auto c = CameraConfig{};
                   c.viewParams = vp;
                   c.bitDepthColor = 10;
                   c.bitDepthDepth = 10;
                   return c;
                 });

  x.boundingBoxCenter = std::accumulate(
      viewParamsList.cbegin(), viewParamsList.cend(), Common::Vec3d{},
      [Z = 1. / static_cast<double>(viewParamsList.size())](const Common::Vec3d &init,
                                                            const ViewParams &vp) {
        return init + Common::Vec3d{Z * vp.ce.ce_view_pos_x(), Z * vp.ce.ce_view_pos_y(),
                                    Z * vp.ce.ce_view_pos_z()};
      });

  std::transform(viewParamsList.cbegin(), viewParamsList.cend(),
                 std::inserter(x.sourceCameraNames, x.sourceCameraNames.end()), [](const auto &vp) {
                   if (vp.name.empty()) {
                     throw std::runtime_error("The decoder needs to assign view names");
                   }
                   return vp.name;
                 });

  return x;
}

void requireAllPatchesWithinProjectionPlaneBounds(const ViewParamsList &vpl,
                                                  const PatchParamsList &ppl) {
  auto patchIndex = 0;

  for (const auto &pp : ppl) {
    const auto viewId = pp.atlasPatchProjectionId();

    static_assert(std::is_unsigned_v<decltype(viewId)>);

    if (viewId >= vpl.size()) {
      throw std::runtime_error("Patch has invalid view ID");
    }
    const auto &vp = vpl[viewId];

    const auto size_u = vp.ci.ci_projection_plane_width_minus1() + 1;
    const auto size_v = vp.ci.ci_projection_plane_height_minus1() + 1;

    const auto u_1 = pp.atlasPatch3dOffsetU();
    const auto v_1 = pp.atlasPatch3dOffsetV();
    const auto u_2 = u_1 + pp.atlasPatch3dSizeU();
    const auto v_2 = v_1 + pp.atlasPatch3dSizeV();

    if (u_1 < 0 || u_1 > u_2 || u_2 > size_u || v_1 < 0 || v_1 > v_2 || v_2 > size_v) {
      throw std::runtime_error(fmt::format(
          "Patch with index {} and projection ID {} is out of bounds", patchIndex, viewId));
    }

    ++patchIndex;
  }
}
} // namespace TMIV::MivBitstream
