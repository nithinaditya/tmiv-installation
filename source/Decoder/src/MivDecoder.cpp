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

#include <TMIV/Decoder/MivDecoder.h>

#include <TMIV/Common/verify.h>

#include <fmt/format.h>

#include <ctime>
#include <iostream>
#include <utility>

namespace TMIV::Decoder {
MivDecoder::MivDecoder(V3cUnitSource source) : m_inputBuffer{std::move(source)} {}

MivDecoder::~MivDecoder() {
  if (m_totalOccVideoDecodingTime > 0.) {
    std::cout << "Total ocupancy video sub bitstream decoding time: " << m_totalOccVideoDecodingTime
              << " s\n";
  }
  if (m_totalGeoVideoDecodingTime > 0.) {
    std::cout << "Total geometry video sub bitstream decoding time: " << m_totalGeoVideoDecodingTime
              << " s\n";
  }
  if (m_totalAttrVideoDecodingTime > 0.) {
    std::cout << "Total attribute video sub bitstream decoding time: "
              << m_totalAttrVideoDecodingTime << " s\n";
  }
}

void MivDecoder::setOccFrameServer(OccFrameServer value) { m_occFrameServer = std::move(value); }

void MivDecoder::setGeoFrameServer(GeoFrameServer value) { m_geoFrameServer = std::move(value); }

void MivDecoder::setTextureFrameServer(TextureFrameServer value) {
  m_textureFrameServer = std::move(value);
}

void MivDecoder::setTransparencyFrameServer(TransparencyFrameServer value) {
  m_transparencyFrameServer = std::move(value);
}

auto MivDecoder::operator()() -> std::optional<MivBitstream::AccessUnit> {
  m_au.irap = expectIrap();

  if (m_au.irap) {
    if (auto vps = decodeVps()) {
      m_au.vps = *vps;
      resetDecoder();
    } else {
      return std::nullopt;
    }
  }

  ++m_au.foc;

  if (!m_commonAtlasAu || m_commonAtlasAu->foc < m_au.foc) {
    m_commonAtlasAu = (*m_commonAtlasDecoder)();
  }
  if (m_commonAtlasAu && m_commonAtlasAu->foc == m_au.foc) {
    decodeCommonAtlas();
  }

  for (size_t k = 0; k <= m_au.vps.vps_atlas_count_minus1(); ++k) {
    if (!m_atlasAu[k] || m_atlasAu[k]->foc < m_au.foc) {
      m_atlasAu[k] = (*m_atlasDecoder[k])();
    }
    if (m_atlasAu[k] && m_atlasAu[k]->foc == m_au.foc) {
      decodeAtlas(k);
    }
  }

  if (decodeVideoSubBitstreams()) {
    // TODO(BK): This copies the video frames.
    return m_au;
  }
  return {};
}

auto MivDecoder::expectIrap() const -> bool { return !m_commonAtlasDecoder; }

auto MivDecoder::decodeVps() -> std::optional<MivBitstream::V3cParameterSet> {
  if (const auto &vu =
          m_inputBuffer(MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_VPS})) {
    return vu->v3c_unit_payload().v3c_parameter_set();
  }
  return std::nullopt;
}

void MivDecoder::resetDecoder() {
  summarizeVps();
  checkCapabilities();

  auto vuh = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_CAD};
  m_commonAtlasDecoder = std::make_unique<CommonAtlasDecoder>(
      [this, vuh]() { return m_inputBuffer(vuh); }, m_au.vps, m_au.foc);

  m_atlasDecoder.clear();
  m_atlasAu.assign(m_au.vps.vps_atlas_count_minus1() + size_t{1}, {});
  m_au.atlas.clear();
  m_occVideoDecoder.clear();
  m_geoVideoDecoder.clear();
  m_textureVideoDecoder.clear();
  m_transparencyVideoDecoder.clear();

  for (size_t k = 0; k <= m_au.vps.vps_atlas_count_minus1(); ++k) {
    const auto j = m_au.vps.vps_atlas_id(k);
    auto vuhCad = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_AD};
    vuhCad.vuh_atlas_id(j);
    m_atlasDecoder.push_back(std::make_unique<AtlasDecoder>(
        [this, vuhCad]() { return m_inputBuffer(vuhCad); }, vuhCad, m_au.vps, m_au.foc));
    m_au.atlas.emplace_back();

    if (m_au.vps.vps_occupancy_video_present_flag(j)) {
      auto vuhOvd = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_OVD};
      vuhOvd.vuh_v3c_parameter_set_id(m_au.vps.vps_v3c_parameter_set_id()).vuh_atlas_id(j);
      m_occVideoDecoder.push_back(startVideoDecoder(vuhOvd, m_totalOccVideoDecodingTime));
    } else {
      m_occVideoDecoder.push_back(nullptr);
    }

    if (m_au.vps.vps_geometry_video_present_flag(j)) {
      auto vuhGvd = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_GVD};
      vuhGvd.vuh_v3c_parameter_set_id(m_au.vps.vps_v3c_parameter_set_id()).vuh_atlas_id(j);
      m_geoVideoDecoder.push_back(startVideoDecoder(vuhGvd, m_totalGeoVideoDecodingTime));
    } else {
      m_geoVideoDecoder.push_back(nullptr);
    }

    bool attrTextureAbsent = true;
    bool attrTransparencyAbsent = true;
    if (m_au.vps.vps_attribute_video_present_flag(j)) {
      const auto &ai = m_au.vps.attribute_information(j);
      for (uint8_t i = 0; i < ai.ai_attribute_count(); ++i) {
        const auto type = ai.ai_attribute_type_id(i);
        auto vuhAvd = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_AVD};
        vuhAvd.vuh_v3c_parameter_set_id(m_au.vps.vps_v3c_parameter_set_id()).vuh_atlas_id(j);
        vuhAvd.vuh_attribute_index(i);
        if (type == MivBitstream::AiAttributeTypeId::ATTR_TEXTURE) {
          attrTextureAbsent = false;
          m_textureVideoDecoder.push_back(startVideoDecoder(vuhAvd, m_totalAttrVideoDecodingTime));
        } else if (type == MivBitstream::AiAttributeTypeId::ATTR_TRANSPARENCY) {
          attrTransparencyAbsent = false;
          m_transparencyVideoDecoder.push_back(
              startVideoDecoder(vuhAvd, m_totalAttrVideoDecodingTime));
        }
      }
    }
    if (attrTextureAbsent) {
      m_textureVideoDecoder.push_back(nullptr);
    }
    if (attrTransparencyAbsent) {
      m_transparencyVideoDecoder.push_back(nullptr);
    }
  }
}

auto MivDecoder::decodeVideoSubBitstreams() -> bool {
  auto result = std::array{false, false};

  for (size_t k = 0; k <= m_au.vps.vps_atlas_count_minus1(); ++k) {
    const auto j = m_au.vps.vps_atlas_id(k);

    if (m_au.vps.vps_occupancy_video_present_flag(j)) {
      result[static_cast<std::size_t>(decodeOccVideo(k))] = true;
    }
    if (m_au.vps.vps_geometry_video_present_flag(j)) {
      result[static_cast<std::size_t>(decodeGeoVideo(k))] = true;
    }

    // Note(FT): test the type of attribute to decode : texture AND/OR transparency
    for (auto attributeIndex = 0;
         attributeIndex < m_au.vps.attribute_information(j).ai_attribute_count();
         attributeIndex++) {
      if (m_au.vps.attribute_information(j).ai_attribute_type_id(attributeIndex) ==
          MivBitstream::AiAttributeTypeId::ATTR_TEXTURE) {
        result[static_cast<std::size_t>(decodeAttrTextureVideo(k))] = true;
      }
      if (m_au.vps.attribute_information(j).ai_attribute_type_id(attributeIndex) ==
          MivBitstream::AiAttributeTypeId::ATTR_TRANSPARENCY) {
        result[static_cast<std::size_t>(decodeAttrTransparencyVideo(k))] = true;
      }
    }
  }

  if (result[0U] && result[1U]) {
    throw std::runtime_error("One of the video streams is truncated");
  }
  return result[1U];
}

void MivDecoder::checkCapabilities() const {
  CONSTRAIN_PTL(m_au.vps.profile_tier_level().ptl_profile_codec_group_idc() ==
                MivBitstream::PtlProfileCodecGroupIdc::HEVC_Main10);
  CONSTRAIN_PTL(m_au.vps.profile_tier_level().ptl_profile_toolset_idc() ==
                    MivBitstream::PtlProfilePccToolsetIdc::MIV_Main ||
                m_au.vps.profile_tier_level().ptl_profile_toolset_idc() ==
                    MivBitstream::PtlProfilePccToolsetIdc::MIV_Extended ||
                m_au.vps.profile_tier_level().ptl_profile_toolset_idc() ==
                    MivBitstream::PtlProfilePccToolsetIdc::MIV_Geometry_Absent);
  CONSTRAIN_PTL(m_au.vps.profile_tier_level().ptl_profile_reconstruction_idc() ==
                MivBitstream::PtlProfileReconstructionIdc::MIV_Main);

  VERIFY_MIVBITSTREAM(m_au.vps.vps_miv_extension_present_flag());
  VERIFY_V3CBITSTREAM(m_au.vps.vps_extension_6bits() == 0);

  for (size_t k = 0; k <= m_au.vps.vps_atlas_count_minus1(); ++k) {
    const auto j = m_au.vps.vps_atlas_id(k);
    VERIFY_MIVBITSTREAM(m_au.vps.vps_map_count_minus1(j) == 0);
    VERIFY_MIVBITSTREAM(!m_au.vps.vps_auxiliary_video_present_flag(j));
  }
}

auto MivDecoder::startVideoDecoder(const MivBitstream::V3cUnitHeader &vuh, double &totalTime)
    -> std::unique_ptr<VideoDecoder::VideoServer> {
  std::string data;
  while (auto vu = m_inputBuffer(vuh)) {
    // TODO(BK): Let the video decoder pull V3C units. This implementation assumes the bitstream is
    // short enough to fit in memory. The reason for this shortcut is that the change requires
    // parsing of the Annex B byte stream, which can be easily done but it requires an additional
    // implementation effort.
    data += vu->v3c_unit_payload().video_sub_bitstream().data();
  }
  if (data.empty()) {
    return {}; // Out-of-band?
  }

  const double t0 = std::clock();
  auto server = std::make_unique<VideoDecoder::VideoServer>(
      VideoDecoder::IVideoDecoder::create(
          m_au.vps.profile_tier_level().ptl_profile_codec_group_idc()),
      data);
  server->wait();
  totalTime += (std::clock() - t0) / CLOCKS_PER_SEC;
  return server;
}

void MivDecoder::decodeCommonAtlas() {
  decodeViewParamsList();
  m_au.gup = m_commonAtlasAu->gup;
  m_au.vs = m_commonAtlasAu->vs;
  m_au.casps = m_commonAtlasAu->casps;
}

void MivDecoder::decodeViewParamsList() {
  const auto &caf = m_commonAtlasAu->caf;
  if (caf.caf_extension_present_flag() && caf.caf_miv_extension_present_flag()) {
    const auto &came = caf.caf_miv_extension();
    bool dqParamsPresentFlag = true;
    if (m_commonAtlasAu->casps.casps_extension_present_flag() &&
        m_commonAtlasAu->casps.casps_miv_extension_present_flag()) {
      dqParamsPresentFlag = m_commonAtlasAu->casps.casps_miv_extension()
                                .casme_depth_quantization_params_present_flag();
    }
    if (m_commonAtlasAu->irap) {
      decodeMvpl(came.miv_view_params_list(), dqParamsPresentFlag);
    } else {
      if (came.came_update_extrinsics_flag()) {
        decodeMvpue(came.miv_view_params_update_extrinsics());
      }
      if (came.came_update_intrinsics_flag()) {
        decodeMvpui(came.miv_view_params_update_intrinsics());
      }
      if (m_commonAtlasAu->casps.casps_miv_extension()
              .casme_depth_quantization_params_present_flag() &&
          came.came_update_depth_quantization_flag() && dqParamsPresentFlag) {
        decodeMvpudq(came.miv_view_params_update_depth_quantization());
      }
    }
  }

  if (m_commonAtlasAu->casps.casps_extension_present_flag() &&
      m_commonAtlasAu->casps.casps_miv_extension_present_flag()) {
    const auto &casme = m_commonAtlasAu->casps.casps_miv_extension();
    if (casme.casme_vui_params_present_flag()) {
      const auto &vui = casme.vui_parameters();
      VERIFY_MIVBITSTREAM(!m_au.vui || *m_au.vui == vui);
      m_au.vui = vui;
    }
  }
}

void MivDecoder::decodeMvpl(const MivBitstream::MivViewParamsList &mvpl, bool dqParamsPresentFlag) {
  m_au.viewParamsList.assign(mvpl.mvp_num_views_minus1() + size_t{1}, {});

  for (uint16_t viewId = 0; viewId <= mvpl.mvp_num_views_minus1(); ++viewId) {
    auto &vp = m_au.viewParamsList[viewId];
    vp.ce = mvpl.camera_extrinsics(viewId);
    vp.isInpainted = mvpl.mvp_inpaint_flag(viewId);
    vp.ci = mvpl.camera_intrinsics(viewId);
    if (dqParamsPresentFlag) {
      vp.dq = mvpl.depth_quantization(viewId);
    }
    if (mvpl.mvp_pruning_graph_params_present_flag()) {
      vp.pp = mvpl.pruning_parent(viewId);
    }

    vp.name = fmt::format("pv{:02}", viewId);
  }
}

void MivDecoder::decodeMvpue(const MivBitstream::MivViewParamsUpdateExtrinsics &mvpue) {
  for (uint16_t i = 0; i <= mvpue.mvpue_num_view_updates_minus1(); ++i) {
    m_au.viewParamsList[mvpue.mvpue_view_idx(i)].ce = mvpue.camera_extrinsics(i);
  }
}

void MivDecoder::decodeMvpui(const MivBitstream::MivViewParamsUpdateIntrinsics &mvpui) {
  for (uint16_t i = 0; i <= mvpui.mvpui_num_view_updates_minus1(); ++i) {
    m_au.viewParamsList[mvpui.mvpui_view_idx(i)].ci = mvpui.camera_intrinsics(i);
  }
}

void MivDecoder::decodeMvpudq(const MivBitstream::MivViewParamsUpdateDepthQuantization &mvpudq) {
  for (uint16_t i = 0; i <= mvpudq.mvpudq_num_view_updates_minus1(); ++i) {
    m_au.viewParamsList[mvpudq.mvpudq_view_idx(i)].dq = mvpudq.depth_quantization(i);
  }
}

void MivDecoder::decodeAtlas(size_t k) {
  m_au.atlas[k].asps = m_atlasAu[k]->asps;
  m_au.atlas[k].afps = m_atlasAu[k]->afps;
  const auto &ppl = decodePatchParamsList(k, m_au.atlas[k].patchParamsList);
  requireAllPatchesWithinProjectionPlaneBounds(m_au.viewParamsList, ppl);
  m_au.atlas[k].blockToPatchMap = decodeBlockToPatchMap(k, ppl);
}

// NOTE(BK): Combined implementation of two processes because there is only a single tile in MIV
// main profile:
//  * [WG 07 N 0003:9.2.6]   Decoding process of the block to patch map
//  * [WG 07 N 0003:9.2.7.2] Conversion of tile level blockToPatch information to atlas level
//                           blockToPatch information
auto MivDecoder::decodeBlockToPatchMap(size_t k, const MivBitstream::PatchParamsList &ppl) const
    -> Common::BlockToPatchMap {
  const auto &asps = m_au.atlas[k].asps;

  const std::int32_t log2PatchPackingBlockSize = asps.asps_log2_patch_packing_block_size();
  const auto patchPackingBlockSize = 1 << log2PatchPackingBlockSize;
  const auto offset = patchPackingBlockSize - 1;

  const auto atlasBlockToPatchMapWidth = (asps.asps_frame_width() + offset) / patchPackingBlockSize;
  const auto atlasBlockToPatchMapHeight =
      (asps.asps_frame_height() + offset) / patchPackingBlockSize;

  // All elements of TileBlockToPatchMap are first initialized to -1 as follows [9.2.6]
  auto btpm = Common::BlockToPatchMap{atlasBlockToPatchMapWidth, atlasBlockToPatchMapHeight};
  std::fill(btpm.getPlane(0).begin(), btpm.getPlane(0).end(), Common::unusedPatchId);

  // Then the AtlasBlockToPatchMap array is updated as follows:
  for (std::size_t p = 0; p < ppl.size(); ++p) {
    const std::size_t xOrg = ppl[p].atlasPatch2dPosX() / patchPackingBlockSize;
    const std::size_t yOrg = ppl[p].atlasPatch2dPosY() / patchPackingBlockSize;
    const std::size_t atlasPatchWidthBlk =
        (ppl[p].atlasPatch2dSizeX() + offset) / patchPackingBlockSize;
    const std::size_t atlasPatchHeightBlk =
        (ppl[p].atlasPatch2dSizeY() + offset) / patchPackingBlockSize;

    for (std::size_t y = 0; y < atlasPatchHeightBlk; ++y) {
      for (std::size_t x = 0; x < atlasPatchWidthBlk; ++x) {
        if (!asps.asps_patch_precedence_order_flag() ||
            btpm.getPlane(0)(yOrg + y, xOrg + x) == Common::unusedPatchId) {
          btpm.getPlane(0)(yOrg + y, xOrg + x) = static_cast<std::uint16_t>(p);
        }
      }
    }
  }

  return btpm;
}

auto MivDecoder::decodePatchParamsList(size_t k, MivBitstream::PatchParamsList &ppl) const
    -> const MivBitstream::PatchParamsList & {
  const auto &ath = m_atlasAu[k]->atl.atlas_tile_header();
  VERIFY_MIVBITSTREAM(ath.ath_type() == MivBitstream::AthType::I_TILE ||
                      ath.ath_type() == MivBitstream::AthType::SKIP_TILE);
  if (ath.ath_type() == MivBitstream::AthType::SKIP_TILE) {
    return ppl;
  }

  const auto &atdu = m_atlasAu[k]->atl.atlas_tile_data_unit();
  const auto &asps = m_atlasAu[k]->asps;
  const auto &afps = m_atlasAu[k]->afps;

  ppl.assign(atdu.atduTotalNumberOfPatches(), {});
  atdu.visit([&](size_t p, MivBitstream::AtduPatchMode /* unused */,
                 const MivBitstream::PatchInformationData &pid) {
    ppl[p] = MivBitstream::PatchParams::decodePdu(pid.patch_data_unit(), asps, afps, ath);
  });

  return ppl;
}

auto MivDecoder::decodeOccVideo(size_t k) -> bool {
  const double t0 = clock();

  if (m_occVideoDecoder[k]) {
    auto frame = m_occVideoDecoder[k]->getFrame();
    if (!frame) {
      return false;
    }
    m_au.atlas[k].decOccFrame = frame->as<Common::YUV400P10>();
    m_occVideoDecoder[k]->wait();
  } else if (m_occFrameServer) {
    m_au.atlas[k].decOccFrame = m_occFrameServer(m_au.vps.vps_atlas_id(k), m_au.foc,
                                                 m_au.atlas[k].decOccFrameSize(m_au.vps));
    if (m_au.atlas[k].decOccFrame.empty()) {
      return false;
    }
  } else {
    MIVBITSTREAM_ERROR("Out-of-band occupancy video data but no frame server provided");
  }

  m_totalOccVideoDecodingTime += (clock() - t0) / CLOCKS_PER_SEC;
  return true;
}

auto MivDecoder::decodeGeoVideo(size_t k) -> bool {
  const double t0 = clock();

  if (m_geoVideoDecoder[k]) {
    auto frame = m_geoVideoDecoder[k]->getFrame();
    if (!frame) {
      return false;
    }
    m_au.atlas[k].decGeoFrame = frame->as<Common::YUV400P10>();
    m_geoVideoDecoder[k]->wait();
  } else if (m_geoFrameServer) {
    m_au.atlas[k].decGeoFrame = m_geoFrameServer(m_au.vps.vps_atlas_id(k), m_au.foc,
                                                 m_au.atlas[k].decGeoFrameSize(m_au.vps));
    if (m_au.atlas[k].decGeoFrame.empty()) {
      return false;
    }
  } else {
    MIVBITSTREAM_ERROR("Out-of-band geometry video data but no frame server provided");
  }

  m_totalGeoVideoDecodingTime += (clock() - t0) / CLOCKS_PER_SEC;
  return true;
}

auto MivDecoder::decodeAttrTextureVideo(size_t k) -> bool {
  const double t0 = clock();

  if (m_textureVideoDecoder[k]) {
    auto frame = m_textureVideoDecoder[k]->getFrame();
    if (!frame) {
      return false;
    }
    m_au.atlas[k].attrFrame = frame->as<Common::YUV444P10>();
    m_textureVideoDecoder[k]->wait();
  } else if (m_textureFrameServer) {
    m_au.atlas[k].attrFrame =
        m_textureFrameServer(m_au.vps.vps_atlas_id(k), m_au.foc, m_au.atlas[k].frameSize());
    if (m_au.atlas[k].attrFrame.empty()) {
      return false;
    }
  } else {
    MIVBITSTREAM_ERROR("Out-of-band texture video data but no frame server provided");
  }

  m_totalAttrVideoDecodingTime += (clock() - t0) / CLOCKS_PER_SEC;
  return true;
}

auto MivDecoder::decodeAttrTransparencyVideo(size_t k) -> bool {
  const double t0 = clock();

  if (m_transparencyVideoDecoder[k]) {
    auto frame = m_transparencyVideoDecoder[k]->getFrame();
    if (!frame) {
      return false;
    }
    m_au.atlas[k].transparencyFrame = frame->as<Common::YUV400P10>();
    m_transparencyVideoDecoder[k]->wait();
  } else if (m_transparencyFrameServer) {
    m_au.atlas[k].transparencyFrame =
        m_transparencyFrameServer(m_au.vps.vps_atlas_id(k), m_au.foc, m_au.atlas[k].frameSize());
    if (m_au.atlas[k].transparencyFrame.empty()) {
      return false;
    }
  } else {
    MIVBITSTREAM_ERROR("Out-of-band transparency video data but no frame server provided");
  }

  m_totalAttrVideoDecodingTime += (clock() - t0) / CLOCKS_PER_SEC;
  return true;
}

void MivDecoder::summarizeVps() const {
  const auto &vps = m_au.vps;
  const auto &ptl = vps.profile_tier_level();

  std::cout << "V3C parameter set " << int{vps.vps_v3c_parameter_set_id()} << ":\n";
  std::cout << "  Tier " << static_cast<int>(ptl.ptl_tier_flag()) << ", " << ptl.ptl_level_idc()
            << ", codec group " << ptl.ptl_profile_codec_group_idc() << ", toolset "
            << ptl.ptl_profile_toolset_idc() << ", recon " << ptl.ptl_profile_reconstruction_idc()
            << ", decodes " << ptl.ptl_max_decodes_idc() << '\n';
  for (size_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
    const auto j = vps.vps_atlas_id(k);
    std::cout << "  Atlas " << j << ": " << vps.vps_frame_width(j) << " x "
              << vps.vps_frame_height(j);
    if (vps.vps_occupancy_video_present_flag(j)) {
      const auto &oi = vps.occupancy_information(j);
      std::cout << "; [OI: codec " << int{oi.oi_occupancy_codec_id()} << ", "
                << int{oi.oi_lossy_occupancy_compression_threshold()} << ", 2D "
                << (oi.oi_occupancy_2d_bit_depth_minus1() + 1) << ", align " << std::boolalpha
                << oi.oi_occupancy_MSB_align_flag() << ']';
    }
    if (vps.vps_geometry_video_present_flag(j)) {
      const auto &gi = vps.geometry_information(j);
      std::cout << "; [GI: codec " << int{gi.gi_geometry_codec_id()} << ", 2D "
                << (gi.gi_geometry_2d_bit_depth_minus1() + 1) << ", align " << std::boolalpha
                << gi.gi_geometry_MSB_align_flag() << ", 3D "
                << (gi.gi_geometry_3d_coordinates_bit_depth_minus1() + 1) << ']';
    }
    if (vps.vps_attribute_video_present_flag(j)) {
      const auto &ai = vps.attribute_information(j);
      std::cout << "; [AI: " << int{ai.ai_attribute_count()};
      for (uint8_t i = 0; i < ai.ai_attribute_count(); ++i) {
        std::cout << ", " << ai.ai_attribute_type_id(i) << ", codec "
                  << int{ai.ai_attribute_codec_id(i)} << ", dims "
                  << (ai.ai_attribute_dimension_minus1(i) + 1) << ", 2D "
                  << (ai.ai_attribute_2d_bit_depth_minus1(i) + 1) << ", align " << std::boolalpha
                  << ai.ai_attribute_MSB_align_flag(i);
        if (i + 1 == ai.ai_attribute_count()) {
          std::cout << "]";
        }
      }
    }
    std::cout << '\n';
  }
  const auto &vme = vps.vps_miv_extension();
  std::cout << ", geometry scaling " << std::boolalpha << vme.vme_geometry_scale_enabled_flag()
            << ", groups " << vme.group_mapping().gm_group_count() << ", embedded occupancy "
            << std::boolalpha << vme.vme_embedded_occupancy_enabled_flag() << ", occupancy scaling "
            << vme.vme_occupancy_scale_enabled_flag() << '\n';
}
} // namespace TMIV::Decoder
