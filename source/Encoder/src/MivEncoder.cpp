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

#include <TMIV/Encoder/MivEncoder.h>

#include <TMIV/Common/verify.h>
#include <TMIV/MivBitstream/SeiRBSP.h>

#include <sstream>
#include <vector>

namespace TMIV::Encoder {
MivEncoder::MivEncoder(std::ostream &stream) : m_stream{stream} {
  m_ssvh.encodeTo(m_stream);
  m_stream.flush();
}

void MivEncoder::writeAccessUnit(const MivBitstream::EncoderParams &params) {
  m_params = params;

  if (m_irap) {
    m_params.vps.profile_tier_level().ptl_max_decodes_idc(ptlMaxDecodesIdc());
    writeV3cUnit(MivBitstream::VuhUnitType::V3C_VPS, {}, m_params.vps);
    m_log2MaxFrmOrderCntLsbMinus4 =
        m_params.casps.casps_log2_max_common_atlas_frame_order_cnt_lsb_minus4();
  }

  m_frmOrderCntLsb = m_params.atlas.front().ath.ath_atlas_frm_order_cnt_lsb();
  VERIFY_MIVBITSTREAM(m_frmOrderCntLsb < maxFrmOrderCntLsb());

  // NOTE(#253): always write even for non-IRAP intra periods w/o view parameter updates
  //             to avoid frame order count ambiguity
  writeV3cUnit(MivBitstream::VuhUnitType::V3C_CAD, {}, commonAtlasSubBitstream());
  m_previouslySentMessages.viewParamsList = params.viewParamsList;

  for (uint8_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
    // Clause 7.4.5.3.2 of V-PCC DIS d85 [N19329]: AXPS regardless of atlas ID (and temporal ID)
    // share the same value space for AXPS ID
    auto &aau = m_params.atlas[k];
    aau.asps.asps_atlas_sequence_parameter_set_id(k);
    aau.afps.afps_atlas_frame_parameter_set_id(k);
    aau.afps.afps_atlas_sequence_parameter_set_id(k);
    aau.ath.ath_atlas_frame_parameter_set_id(k);

    writeV3cUnit(MivBitstream::VuhUnitType::V3C_AD, m_params.vps.vps_atlas_id(k),
                 atlasSubBitstream(k));
  }
  if (!m_params.randomAccess) {
    m_irap = false;
  }
}

auto MivEncoder::ptlMaxDecodesIdc() const -> MivBitstream::PtlMaxDecodesIdc {
  auto numDecodes = 0;
  for (uint8_t k = 0; k < m_params.vps.vps_atlas_count_minus1() + 1; ++k) {
    const auto j = m_params.vps.vps_atlas_id(k);
    numDecodes += static_cast<int>(m_params.vps.vps_auxiliary_video_present_flag(j));
    numDecodes += static_cast<int>(m_params.vps.vps_occupancy_video_present_flag(j));
    numDecodes += static_cast<int>(m_params.vps.vps_geometry_video_present_flag(j)) *
                  (m_params.vps.vps_map_count_minus1(j) + 1);
    if (m_params.vps.vps_attribute_video_present_flag(j)) {
      numDecodes += m_params.vps.attribute_information(j).ai_attribute_count() *
                    (m_params.vps.vps_map_count_minus1(j) + 1);
    }
  }
  if (numDecodes <= 1) {
    return MivBitstream::PtlMaxDecodesIdc::max_1;
  }
  if (numDecodes <= 2) {
    return MivBitstream::PtlMaxDecodesIdc::max_2;
  }
  if (numDecodes <= 3) {
    return MivBitstream::PtlMaxDecodesIdc::max_3;
  }
  if (numDecodes <= 4) {
    return MivBitstream::PtlMaxDecodesIdc::max_4;
  }
  if (numDecodes <= 6) {
    return MivBitstream::PtlMaxDecodesIdc::max_6;
  }
  if (numDecodes <= 12) {
    return MivBitstream::PtlMaxDecodesIdc::max_12;
  }
  if (numDecodes <= 16) {
    return MivBitstream::PtlMaxDecodesIdc::max_16;
  }
  if (numDecodes <= 24) {
    return MivBitstream::PtlMaxDecodesIdc::max_24;
  }
  if (numDecodes <= 24) {
    return MivBitstream::PtlMaxDecodesIdc::max_24;
  }
  if (numDecodes <= 32) {
    return MivBitstream::PtlMaxDecodesIdc::max_32;
  }
  return MivBitstream::PtlMaxDecodesIdc::unconstrained;
}

namespace {
template <typename Payload>
auto encodeSeiMessage(const Payload &payload, MivBitstream::PayloadType payloadType)
    -> MivBitstream::SeiMessage {
  std::ostringstream subStream;
  Common::OutputBitstream subBitstream{subStream};
  payload.encodeTo(subBitstream);
  return MivBitstream::SeiMessage{payloadType, subStream.str()};
}

void encodeSeiRbspToAsb(MivBitstream::AtlasSubBitstream &asb, const MivBitstream::SeiRBSP &seiRbsp,
                        const MivBitstream::NalUnitHeader &nuh) {
  std::ostringstream subStream;
  seiRbsp.encodeTo(subStream);
  asb.nal_units().emplace_back(nuh, subStream.str());
}

const auto nuhAsps = MivBitstream::NalUnitHeader{MivBitstream::NalUnitType::NAL_ASPS, 0, 1};
const auto nuhAfps = MivBitstream::NalUnitHeader{MivBitstream::NalUnitType::NAL_AFPS, 0, 1};
const auto nuhCasps = MivBitstream::NalUnitHeader{MivBitstream::NalUnitType::NAL_CASPS, 0, 1};
const auto nuhCaf = MivBitstream::NalUnitHeader{MivBitstream::NalUnitType::NAL_CAF, 0, 1};
const auto nuhCra = MivBitstream::NalUnitHeader{MivBitstream::NalUnitType::NAL_CRA, 0, 1};
const auto nuhIdr = MivBitstream::NalUnitHeader{MivBitstream::NalUnitType::NAL_IDR_N_LP, 0, 1};
const auto nuhIdrCaf = MivBitstream::NalUnitHeader{MivBitstream::NalUnitType::NAL_IDR_CAF, 0, 1};
const auto nuhPrefixNsei =
    MivBitstream::NalUnitHeader{MivBitstream::NalUnitType::NAL_PREFIX_NSEI, 0, 1};
} // namespace

auto MivEncoder::commonAtlasSubBitstream() -> MivBitstream::AtlasSubBitstream {
  auto asb = MivBitstream::AtlasSubBitstream{m_ssnh};

  encodePrefixSeiMessages(asb);

  if (m_irap) {
    writeNalUnit(asb, nuhCasps, m_params.casps);
    writeNalUnit(asb, nuhIdrCaf, commonAtlasFrame(), m_params.vps, nuhIdrCaf,
                 std::vector{m_params.casps}, maxFrmOrderCntLsb());
  } else {
    writeNalUnit(asb, nuhCaf, commonAtlasFrame(), m_params.vps, nuhCaf, std::vector{m_params.casps},
                 maxFrmOrderCntLsb());
  }

  return asb;
}

auto MivEncoder::commonAtlasFrame() const -> MivBitstream::CommonAtlasFrameRBSP {
  auto caf = MivBitstream::CommonAtlasFrameRBSP{};

  caf.caf_common_atlas_sequence_parameter_set_id(0)
      .caf_common_atlas_frm_order_cnt_lsb(m_frmOrderCntLsb)
      .caf_extension_present_flag(true)
      .caf_miv_extension_present_flag(true)
      .caf_miv_extension();
  auto &came = caf.caf_miv_extension();
  if (m_irap) {
    came.miv_view_params_list() = mivViewParamsList();
  } else {
    const auto &viewParamsList = m_previouslySentMessages.viewParamsList;
    VERIFY_MIVBITSTREAM(viewParamsList.size() == m_params.viewParamsList.size());
    came.came_update_extrinsics_flag(false);
    came.came_update_intrinsics_flag(false);
    came.came_update_depth_quantization_flag(false);
    for (size_t i = 0; i < viewParamsList.size(); ++i) {
      if (viewParamsList[i].ce != m_params.viewParamsList[i].ce) {
        came.came_update_extrinsics_flag(true);
      }
      if (viewParamsList[i].ci != m_params.viewParamsList[i].ci) {
        came.came_update_intrinsics_flag(true);
      }
      if (viewParamsList[i].dq != m_params.viewParamsList[i].dq) {
        came.came_update_depth_quantization_flag(true);
      }
    }
    if (came.came_update_extrinsics_flag()) {
      came.miv_view_params_update_extrinsics() = mivViewParamsUpdateExtrinsics();
    }
    if (came.came_update_intrinsics_flag()) {
      came.miv_view_params_update_intrinsics() = mivViewParamsUpdateIntrinsics();
    }
    if (came.came_update_depth_quantization_flag()) {
      came.miv_view_params_update_depth_quantization() = mivViewParamsUpdateDepthQuantization();
    }
  }
  return caf;
}

auto MivEncoder::mivViewParamsList() const -> MivBitstream::MivViewParamsList {
  auto mvpl = MivBitstream::MivViewParamsList{};
  const auto &vpl = m_params.viewParamsList;

  assert(!vpl.empty());
  mvpl.mvp_num_views_minus1(static_cast<uint16_t>(vpl.size() - 1));
  mvpl.mvp_intrinsic_params_equal_flag(
      std::all_of(vpl.begin(), vpl.end(), [&](const auto &x) { return x.ci == vpl.front().ci; }));
  mvpl.mvp_depth_quantization_params_equal_flag(
      std::all_of(vpl.begin(), vpl.end(), [&](const auto &x) { return x.dq == vpl.front().dq; }));
  mvpl.mvp_pruning_graph_params_present_flag(vpl.front().pp.has_value());

  for (uint16_t i = 0; i <= mvpl.mvp_num_views_minus1(); ++i) {
    const auto &vp = vpl[i];
    mvpl.camera_extrinsics(i) = vp.ce;
    mvpl.mvp_inpaint_flag(i, vp.isInpainted);

    if (i == 0 || !mvpl.mvp_intrinsic_params_equal_flag()) {
      mvpl.camera_intrinsics(i) = vp.ci;
    }
    if (i == 0 || !mvpl.mvp_depth_quantization_params_equal_flag()) {
      mvpl.depth_quantization(i) = vp.dq;
    }
    assert(vp.pp.has_value() == mvpl.mvp_pruning_graph_params_present_flag());
    if (vp.pp.has_value()) {
      mvpl.pruning_parent(i) = *vp.pp;
    }
  }

  mvpl.mvp_num_views_minus1(static_cast<uint16_t>(m_params.viewParamsList.size() - 1));
  for (uint8_t a = 0; a <= m_params.vps.vps_atlas_count_minus1(); ++a) {
    for (uint16_t v = 0; v <= mvpl.mvp_num_views_minus1(); ++v) {
      mvpl.mvp_view_enabled_in_atlas_flag(a, v, true);
      mvpl.mvp_view_complete_in_atlas_flag(a, v, m_params.viewParamsList[v].isBasicView);
    }
  }
  mvpl.mvp_explicit_view_id_flag(true);
  for (uint16_t v = 0; v <= mvpl.mvp_num_views_minus1(); ++v) {
    mvpl.mvp_view_id(v, v);
  }

  return mvpl;
}

auto MivEncoder::mivViewParamsUpdateExtrinsics() const
    -> MivBitstream::MivViewParamsUpdateExtrinsics {
  auto mvpue = MivBitstream::MivViewParamsUpdateExtrinsics{};
  auto viewIdx = std::vector<uint16_t>{};
  for (size_t v = 0; v < m_previouslySentMessages.viewParamsList.size(); ++v) {
    if (m_previouslySentMessages.viewParamsList[v].ce != m_params.viewParamsList[v].ce) {
      viewIdx.push_back(static_cast<uint16_t>(v));
    }
  }
  VERIFY_MIVBITSTREAM(!viewIdx.empty());
  mvpue.mvpue_num_view_updates_minus1(static_cast<uint16_t>(viewIdx.size() - 1));
  for (uint16_t i = 0; i <= mvpue.mvpue_num_view_updates_minus1(); ++i) {
    mvpue.mvpue_view_idx(i, viewIdx[i]);
    mvpue.camera_extrinsics(i) = m_params.viewParamsList[viewIdx[i]].ce;
  }
  return mvpue;
}

auto MivEncoder::mivViewParamsUpdateIntrinsics() const
    -> MivBitstream::MivViewParamsUpdateIntrinsics {
  auto mvpui = MivBitstream::MivViewParamsUpdateIntrinsics{};
  auto viewIdx = std::vector<uint16_t>{};
  for (size_t v = 0; v < m_previouslySentMessages.viewParamsList.size(); ++v) {
    if (m_previouslySentMessages.viewParamsList[v].ci != m_params.viewParamsList[v].ci) {
      viewIdx.push_back(static_cast<uint16_t>(v));
    }
  }
  VERIFY_MIVBITSTREAM(!viewIdx.empty());
  mvpui.mvpui_num_view_updates_minus1(static_cast<uint16_t>(viewIdx.size() - 1));
  for (uint16_t i = 0; i <= mvpui.mvpui_num_view_updates_minus1(); ++i) {
    mvpui.mvpui_view_idx(i, viewIdx[i]);
    mvpui.camera_intrinsics(i) = m_params.viewParamsList[viewIdx[i]].ci;
  }
  return mvpui;
}

auto MivEncoder::mivViewParamsUpdateDepthQuantization() const
    -> MivBitstream::MivViewParamsUpdateDepthQuantization {
  auto mvpudq = MivBitstream::MivViewParamsUpdateDepthQuantization{};
  auto viewIdx = std::vector<uint16_t>{};
  for (size_t v = 0; v < m_previouslySentMessages.viewParamsList.size(); ++v) {
    if (m_previouslySentMessages.viewParamsList[v].dq != m_params.viewParamsList[v].dq) {
      viewIdx.push_back(static_cast<uint16_t>(v));
    }
  }
  VERIFY_MIVBITSTREAM(!viewIdx.empty());
  mvpudq.mvpudq_num_view_updates_minus1(static_cast<uint16_t>(viewIdx.size() - 1));
  for (uint16_t i = 0; i <= mvpudq.mvpudq_num_view_updates_minus1(); ++i) {
    mvpudq.mvpudq_view_idx(i, viewIdx[i]);
    mvpudq.depth_quantization(i) = m_params.viewParamsList[viewIdx[i]].dq;
  }
  return mvpudq;
}

auto MivEncoder::atlasSubBitstream(size_t k) -> MivBitstream::AtlasSubBitstream {
  auto asb = MivBitstream::AtlasSubBitstream{m_ssnh};

  auto vuh = MivBitstream::V3cUnitHeader{MivBitstream::VuhUnitType::V3C_AD};
  vuh.vuh_atlas_id(m_params.vps.vps_atlas_id(k));

  const auto &aau = m_params.atlas[k];

  if (m_irap) {
    VERIFY_MIVBITSTREAM(m_log2MaxFrmOrderCntLsbMinus4 ==
                        aau.asps.asps_log2_max_atlas_frame_order_cnt_lsb_minus4());
    writeNalUnit(asb, nuhAsps, aau.asps, vuh, m_params.vps);
    writeNalUnit(asb, nuhAfps, aau.afps,
                 std::vector<MivBitstream::AtlasSequenceParameterSetRBSP>{aau.asps});
  }

  const auto aspsV = std::vector<MivBitstream::AtlasSequenceParameterSetRBSP>{aau.asps};
  const auto afpsV = std::vector<MivBitstream::AtlasFrameParameterSetRBSP>{aau.afps};
  const auto nuh = m_irap ? nuhIdr : nuhCra;
  writeNalUnit(asb, nuh, atlasTileLayer(k), nuh, aspsV, afpsV);

  return asb;
}

auto MivEncoder::atlasTileLayer(size_t k) const -> MivBitstream::AtlasTileLayerRBSP {
  auto patchData = MivBitstream::AtlasTileDataUnit::Vector{};
  patchData.reserve(m_params.patchParamsList.size());

  const auto &aau = m_params.atlas[k];
  const auto atlasId = m_params.vps.vps_atlas_id(k);

  for (const auto &pp : m_params.patchParamsList) {
    if (pp.atlasId == atlasId) {
      patchData.emplace_back(MivBitstream::AtduPatchMode::I_INTRA,
                             pp.encodePdu(aau.asps, aau.afps, aau.ath));
    }
  }

  auto x = MivBitstream::AtlasTileLayerRBSP{};
  x.atlas_tile_header() = aau.ath;
  x.atlas_tile_data_unit() = MivBitstream::AtlasTileDataUnit{patchData};
  return x;
}

template <typename Payload>
void MivEncoder::writeV3cUnit(MivBitstream::VuhUnitType vut, MivBitstream::AtlasId atlasId,
                              Payload &&payload) {
  auto vuh = MivBitstream::V3cUnitHeader{vut};
  if (vut != MivBitstream::VuhUnitType::V3C_VPS && vut != MivBitstream::VuhUnitType::V3C_CAD) {
    vuh.vuh_atlas_id(atlasId);
  }
  const auto vu = MivBitstream::V3cUnit{vuh, std::forward<Payload>(payload)};

  std::ostringstream substream;
  vu.encodeTo(substream);

  const auto ssvu = MivBitstream::SampleStreamV3cUnit{substream.str()};
  ssvu.encodeTo(m_stream, m_ssvh);
}

template <typename Payload, typename... Args>
void MivEncoder::writeNalUnit(MivBitstream::AtlasSubBitstream &asb, MivBitstream::NalUnitHeader nuh,
                              Payload &&payload, Args &&...args) {
  std::ostringstream substream1;
  payload.encodeTo(substream1, std::forward<Args>(args)...);
  asb.nal_units().emplace_back(nuh, substream1.str());
}

void MivEncoder::encodePrefixSeiMessages(MivBitstream::AtlasSubBitstream &asb) {
  std::vector<MivBitstream::SeiMessage> seiMessages{};
  if (m_params.viewingSpace.has_value() &&
      m_previouslySentMessages.viewingSpace != m_params.viewingSpace) {
    seiMessages.emplace_back(
        encodeSeiMessage(*m_params.viewingSpace, MivBitstream::PayloadType::viewing_space));
    m_previouslySentMessages.viewingSpace = m_params.viewingSpace;
  }

  if (!seiMessages.empty()) {
    MivBitstream::SeiRBSP seiRbsp{std::move(seiMessages)};
    encodeSeiRbspToAsb(asb, seiRbsp, nuhPrefixNsei);
  }
}
} // namespace TMIV::Encoder
