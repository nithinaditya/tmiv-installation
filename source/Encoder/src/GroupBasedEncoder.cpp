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

#include <TMIV/Encoder/GroupBasedEncoder.h>

#include <algorithm>
#include <cassert>
#include <iostream>

namespace TMIV::Encoder {
namespace {
auto computeDominantAxis(std::vector<float> &Tx, std::vector<float> &Ty, std::vector<float> &Tz)
    -> int {
  const auto xMax = *max_element(Tx.begin(), Tx.end());
  const auto xMin = *min_element(Tx.begin(), Tx.end());
  const auto yMax = *max_element(Ty.begin(), Ty.end());
  const auto yMin = *min_element(Ty.begin(), Ty.end());
  const auto zMax = *max_element(Tz.begin(), Tz.end());
  const auto zMin = *min_element(Tz.begin(), Tz.end());

  const auto xRange = xMax - xMin;
  const auto yRange = yMax - yMin;
  const auto zRange = zMax - zMin;

  if (zRange >= xRange && zRange >= yRange) {
    return 2;
  }
  if (yRange >= xRange && yRange >= zRange) {
    return 1;
  }
  return 0;
}
} // namespace

GroupBasedEncoder::GroupBasedEncoder(const Common::Json &rootNode,
                                     const Common::Json &componentNode) {
  const auto numGroups_ = rootNode.require("numGroups").as<size_t>();

  while (m_encoders.size() < numGroups_) {
    m_encoders.emplace_back(rootNode, componentNode);
  }
}

void GroupBasedEncoder::prepareSequence(MivBitstream::EncoderParams params) {
  m_grouping = sourceSplitter(params);

  for (size_t groupId = 0; groupId != numGroups(); ++groupId) {
    m_encoders[groupId].prepareSequence(splitParams(groupId, params));
  }
}

void GroupBasedEncoder::prepareAccessUnit() {
  for (size_t groupId = 0; groupId != numGroups(); ++groupId) {
    m_encoders[groupId].prepareAccessUnit();
  }
}

void GroupBasedEncoder::pushFrame(Common::MVD16Frame views) {
  for (size_t groupId = 0; groupId != numGroups(); ++groupId) {
    std::cout << "Processing group " << groupId << ":\n";
    m_encoders[groupId].pushFrame(splitViews(groupId, views));
  }
}

auto GroupBasedEncoder::completeAccessUnit() -> const MivBitstream::EncoderParams & {
  auto perGroupParams = std::vector<const MivBitstream::EncoderParams *>(numGroups(), nullptr);

  for (size_t groupId = 0; groupId != numGroups(); ++groupId) {
    perGroupParams[groupId] = &m_encoders[groupId].completeAccessUnit();
  }

  return mergeParams(perGroupParams);
}

auto GroupBasedEncoder::popAtlas() -> Common::MVD10Frame {
  auto result = Common::MVD10Frame{};

  for (auto &encoder : m_encoders) {
    for (auto &atlas : encoder.popAtlas()) {
      result.push_back(std::move(atlas));
    }
  }

  return result;
}

auto GroupBasedEncoder::maxLumaSamplesPerFrame() const -> size_t {
  return std::accumulate(
      m_encoders.begin(), m_encoders.end(), size_t{},
      [](size_t sum, const auto &x) { return sum + x.maxLumaSamplesPerFrame(); });
}

auto GroupBasedEncoder::sourceSplitter(const MivBitstream::EncoderParams &params) -> Grouping {
  auto grouping = Grouping{};

  const auto &viewParamsList = params.viewParamsList;
  const auto numGroups = params.vme().group_mapping().gm_group_count();

  // Compute axial ranges and find the dominant one
  auto Tx = std::vector<float>{};
  auto Ty = std::vector<float>{};
  auto Tz = std::vector<float>{};
  for (size_t camIndex = 0; camIndex < viewParamsList.size(); camIndex++) {
    Tx.push_back(viewParamsList[camIndex].ce.ce_view_pos_x());
    Ty.push_back(viewParamsList[camIndex].ce.ce_view_pos_y());
    Tz.push_back(viewParamsList[camIndex].ce.ce_view_pos_z());
  }

  const auto dominantAxis = computeDominantAxis(Tx, Ty, Tz);

  // Select views per group
  auto viewsPool = std::vector<MivBitstream::ViewParams>{};
  auto viewsLabels = std::vector<uint8_t>{};
  auto viewsInGroup = std::vector<uint8_t>{};
  auto numViewsPerGroup = std::vector<int>{};

  for (size_t camIndex = 0; camIndex < viewParamsList.size(); camIndex++) {
    viewsPool.push_back(viewParamsList[camIndex]);
    viewsLabels.push_back(static_cast<uint8_t>(camIndex));
  }

  for (unsigned gIndex = 0; gIndex < numGroups; gIndex++) {
    viewsInGroup.clear();
    auto camerasInGroup = MivBitstream::ViewParamsList{};
    auto camerasOutGroup = MivBitstream::ViewParamsList{};
    if (gIndex + 1U < numGroups) {
      numViewsPerGroup.push_back(static_cast<int>(std::floor(viewParamsList.size() / numGroups)));
      int64_t maxElementIndex = 0;

      if (dominantAxis == 0) {
        maxElementIndex = max_element(Tx.begin(), Tx.end()) - Tx.begin();
      } else if (dominantAxis == 1) {
        maxElementIndex = max_element(Ty.begin(), Ty.end()) - Ty.begin();
      } else {
        maxElementIndex = max_element(Tz.begin(), Tz.end()) - Tz.begin();
      }

      const auto T0 = Common::Vec3f{Tx[maxElementIndex], Ty[maxElementIndex], Tz[maxElementIndex]};
      auto distance = std::vector<float>();
      distance.reserve(viewsPool.size());
      for (const auto &viewParams : viewsPool) {
        distance.push_back(norm(viewParams.ce.position() - T0));
      }

      // ascending order
      std::vector<size_t> sortedCamerasId(viewsPool.size());
      iota(sortedCamerasId.begin(), sortedCamerasId.end(), 0); // initalization
      std::sort(sortedCamerasId.begin(), sortedCamerasId.end(),
                [&distance](size_t i1, size_t i2) { return distance[i1] < distance[i2]; });
      for (int camIndex = 0; camIndex < numViewsPerGroup[gIndex]; camIndex++) {
        camerasInGroup.push_back(viewsPool[sortedCamerasId[camIndex]]);
      }

      // update the viewsPool
      Tx.clear();
      Ty.clear();
      Tz.clear();
      camerasOutGroup.clear();
      for (size_t camIndex = numViewsPerGroup[gIndex]; camIndex < viewsPool.size(); camIndex++) {
        camerasOutGroup.push_back(viewsPool[sortedCamerasId[camIndex]]);
        Tx.push_back(viewsPool[sortedCamerasId[camIndex]].ce.ce_view_pos_x());
        Ty.push_back(viewsPool[sortedCamerasId[camIndex]].ce.ce_view_pos_y());
        Tz.push_back(viewsPool[sortedCamerasId[camIndex]].ce.ce_view_pos_z());
      }

      std::cout << "Views selected for group " << gIndex << ": ";
      const auto *sep = "";
      for (size_t i = 0; i < camerasInGroup.size(); i++) {
        std::cout << sep << unsigned{viewsLabels[sortedCamerasId[i]]};
        viewsInGroup.push_back(viewsLabels[sortedCamerasId[i]]);
        sep = ", ";
      }
      std::cout << '\n';

      auto viewLabelsTemp = std::vector<uint8_t>{};
      for (size_t i = camerasInGroup.size(); i < viewsLabels.size(); i++) {
        viewLabelsTemp.push_back(viewsLabels[sortedCamerasId[i]]);
      }
      viewsLabels.assign(viewLabelsTemp.begin(), viewLabelsTemp.end());

      viewsPool = camerasOutGroup;
    } else {
      numViewsPerGroup.push_back(
          static_cast<int>((viewParamsList.size() -
                            (numGroups - 1) * std::floor(viewParamsList.size() / numGroups))));

      camerasInGroup.clear();
      std::copy(std::cbegin(viewsPool), std::cend(viewsPool), back_inserter(camerasInGroup));

      std::cout << "Views selected for group " << gIndex << ": ";
      const auto *sep = "";
      for (size_t i = 0; i < camerasInGroup.size(); i++) {
        std::cout << sep << int{viewsLabels[i]};
        viewsInGroup.push_back(viewsLabels[i]);
        sep = ", ";
      }
      std::cout << '\n';
    }
    for (const auto viewInGroup : viewsInGroup) {
      grouping.emplace_back(gIndex, viewInGroup);
    }
  }
  return grouping;
}

auto GroupBasedEncoder::splitParams(size_t groupId, const MivBitstream::EncoderParams &params) const
    -> MivBitstream::EncoderParams {
  // Independent metadata should work automatically. Just copy all metadata to all groups.
  auto result = params;
  result.viewParamsList.clear();

  // Only include the views that are part of this group
  for (const auto &[groupId_, viewId] : m_grouping) {
    if (groupId_ == groupId) {
      result.viewParamsList.push_back(params.viewParamsList[viewId]);
    }
  }

  return result;
}

auto GroupBasedEncoder::splitViews(size_t groupId, Common::MVD16Frame &views) const
    -> Common::MVD16Frame {
  auto result = Common::MVD16Frame{};

  // Only include the views that are part of this group
  for (const auto &[groupId_, viewId] : m_grouping) {
    if (groupId_ == groupId) {
      result.push_back(views[viewId]);
    }
  }

  return result;
}

auto GroupBasedEncoder::mergeVps(const std::vector<const MivBitstream::V3cParameterSet *> &vps)
    -> MivBitstream::V3cParameterSet {
  const auto atlasCount = accumulate(vps.cbegin(), vps.cend(), 0,
                                     [](int count, const MivBitstream::V3cParameterSet *vps) {
                                       return count + vps->vps_atlas_count_minus1() + 1;
                                     });

  auto x = MivBitstream::V3cParameterSet{};
  x.profile_tier_level(vps.front()->profile_tier_level())
      .vps_v3c_parameter_set_id(vps.front()->vps_v3c_parameter_set_id())
      .vps_atlas_count_minus1(static_cast<uint8_t>(atlasCount - 1))
      .vps_extension_present_flag(true)
      .vps_miv_extension_present_flag(true)
      .vps_miv_extension(vps.front()->vps_miv_extension());

  auto &gm = x.vps_miv_extension().group_mapping();
  gm.gm_group_count(static_cast<uint8_t>(vps.size()));

  uint8_t kOut = 0;
  uint8_t groupIndex = 0;

  for (const auto &v : vps) {
    assert(v->profile_tier_level() == x.profile_tier_level());
    assert(v->vps_v3c_parameter_set_id() == x.vps_v3c_parameter_set_id());
    assert(v->vps_miv_extension_present_flag());
    assert(v->vps_extension_6bits() == 0);
    assert(v->vps_miv_extension() == x.vps_miv_extension());

    for (uint8_t kIn = 0; kIn <= v->vps_atlas_count_minus1(); ++kIn, ++kOut) {
      const auto jIn = v->vps_atlas_id(kIn);
      const auto jOut = MivBitstream::AtlasId{kOut};
      x.vps_atlas_id(kOut, jOut);
      gm.gm_group_id(kOut, groupIndex);

      x.vps_frame_width(jOut, v->vps_frame_width(jIn));
      x.vps_frame_height(jOut, v->vps_frame_height(jIn));
      x.vps_map_count_minus1(jOut, v->vps_map_count_minus1(jIn));
      assert(x.vps_map_count_minus1(jOut) == 0);

      x.vps_auxiliary_video_present_flag(jOut, v->vps_auxiliary_video_present_flag(jIn));
      x.vps_occupancy_video_present_flag(jOut, v->vps_occupancy_video_present_flag(jIn));
      x.vps_geometry_video_present_flag(jOut, v->vps_geometry_video_present_flag(jIn));
      x.vps_attribute_video_present_flag(jOut, v->vps_attribute_video_present_flag(jIn));

      if (x.vps_occupancy_video_present_flag(jOut)) {
        x.occupancy_information(jOut) = v->occupancy_information(jIn);
      }
      if (x.vps_geometry_video_present_flag(jOut)) {
        x.geometry_information(jOut) = v->geometry_information(jIn);
      }
      if (x.vps_attribute_video_present_flag(jOut)) {
        x.attribute_information(jOut) = v->attribute_information(jIn);
      }
    }

    ++groupIndex;
  }

  assert(kOut == atlasCount);
  return x;
}

auto GroupBasedEncoder::mergeParams(
    const std::vector<const MivBitstream::EncoderParams *> &perGroupParams)
    -> const MivBitstream::EncoderParams & {
  // Start with first group
  m_params = *perGroupParams.front();
  m_params.patchParamsList.clear();

  // Merge V3C parameter sets
  std::vector<const MivBitstream::V3cParameterSet *> vps(perGroupParams.size());
  transform(std::begin(perGroupParams), std::end(perGroupParams), std::begin(vps),
            [](const auto &ivs) { return &ivs->vps; });
  m_params.vps = mergeVps(vps);

  // For each other group
  for (auto ivs = std::begin(perGroupParams) + 1; ivs != std::end(perGroupParams); ++ivs) {
    // Merge view parameters
    std::transform(std::begin((*ivs)->viewParamsList), std::end((*ivs)->viewParamsList),
                   back_inserter(m_params.viewParamsList),
                   [viewIdOffset = static_cast<uint16_t>(m_params.viewParamsList.size())](
                       MivBitstream::ViewParams vp) {
                     // Merging pruning graphs
                     if (vp.pp && !vp.pp->pp_is_root_flag()) {
                       for (uint16_t i = 0; i <= vp.pp->pp_num_parent_minus1(); ++i) {
                         vp.pp->pp_parent_id(i, vp.pp->pp_parent_id(i) + viewIdOffset);
                       }
                     }
                     return vp;
                   });

    // Merge viewing space
    assert(m_params.viewingSpace == (*ivs)->viewingSpace);
  }

  // Concatenate atlas access unit parameters
  for (const auto &params : perGroupParams) {
    for (const auto &atlas : params->atlas) {
      m_params.atlas.push_back(atlas);
    }
  }

  // Modify bit depth of pdu_projection_id
  for (auto &atlas : m_params.atlas) {
    atlas.asps.asps_extended_projection_enabled_flag(true).asps_max_number_projections_minus1(
        static_cast<uint16_t>(m_params.viewParamsList.size() - 1));
  }

  // Renumber atlas and view ID's
  uint16_t atlasIdOffset = 0;
  uint16_t viewIdOffset = 0;

  for (const auto *perGroupParam : perGroupParams) {
    // Copy patches in group order
    for (const auto &patch : perGroupParam->patchParamsList) {
      m_params.patchParamsList.push_back(patch);
      m_params.patchParamsList.back().atlasId = MivBitstream::AtlasId{
          static_cast<uint8_t>(perGroupParam->vps.indexOf(patch.atlasId) + atlasIdOffset)};
      m_params.patchParamsList.back().atlasPatchProjectionId(patch.atlasPatchProjectionId() +
                                                             viewIdOffset);
    }

    // Renumber atlases and views
    atlasIdOffset += static_cast<uint16_t>(perGroupParam->atlas.size());
    viewIdOffset += static_cast<uint16_t>(perGroupParam->viewParamsList.size());
  }

  return m_params;
}

} // namespace TMIV::Encoder
