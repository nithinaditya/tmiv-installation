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

#include <TMIV/Common/verify.h>
#include <TMIV/MivBitstream/CommonAtlasFrameMivExtension.h>
#include <TMIV/MivBitstream/CommonAtlasSequenceParameterSetRBSP.h>

#include <cmath>

#include <fmt/ostream.h>

namespace TMIV::MivBitstream {
auto operator<<(std::ostream &stream, const CiCamType x) -> std::ostream & {
  switch (x) {
  case CiCamType::equirectangular:
    return stream << "equirectangular";
  case CiCamType::perspective:
    return stream << "perspective";
  case CiCamType::orthographic:
    return stream << "orthographic";
  default:
    MIVBITSTREAM_ERROR("Unknown cam type");
  }
}

auto CameraIntrinsics::projectionPlaneSize() const -> Common::Vec2i {
  return {ci_projection_plane_width_minus1() + 1, ci_projection_plane_height_minus1() + 1};
}

auto CameraIntrinsics::ci_erp_phi_min() const noexcept -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::equirectangular);
  VERIFY_MIVBITSTREAM(m_ci_erp_phi_min.has_value());
  return *m_ci_erp_phi_min;
}

auto CameraIntrinsics::ci_erp_phi_max() const noexcept -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::equirectangular);
  VERIFY_MIVBITSTREAM(m_ci_erp_phi_max.has_value());
  return *m_ci_erp_phi_max;
}

auto CameraIntrinsics::ci_erp_theta_min() const noexcept -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::equirectangular);
  VERIFY_MIVBITSTREAM(m_ci_erp_theta_min.has_value());
  return *m_ci_erp_theta_min;
}

auto CameraIntrinsics::ci_erp_theta_max() const noexcept -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::equirectangular);
  VERIFY_MIVBITSTREAM(m_ci_erp_theta_max.has_value());
  return *m_ci_erp_theta_max;
}

auto CameraIntrinsics::ci_perspective_focal_hor() const noexcept -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::perspective);
  VERIFY_MIVBITSTREAM(m_ci_perspective_focal_hor.has_value());
  return *m_ci_perspective_focal_hor;
}

auto CameraIntrinsics::ci_perspective_focal_ver() const noexcept -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::perspective);
  VERIFY_MIVBITSTREAM(m_ci_perspective_focal_ver.has_value());
  return *m_ci_perspective_focal_ver;
}

auto CameraIntrinsics::ci_perspective_center_hor() const noexcept -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::perspective);
  VERIFY_MIVBITSTREAM(m_ci_perspective_center_hor.has_value());
  return *m_ci_perspective_center_hor;
}

auto CameraIntrinsics::ci_perspective_center_ver() const noexcept -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::perspective);
  VERIFY_MIVBITSTREAM(m_ci_perspective_center_ver.has_value());
  return *m_ci_perspective_center_ver;
}

auto CameraIntrinsics::ci_ortho_width() const noexcept -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::orthographic);
  VERIFY_MIVBITSTREAM(m_ci_ortho_width.has_value());
  return *m_ci_ortho_width;
}

auto CameraIntrinsics::ci_ortho_height() const noexcept -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::orthographic);
  VERIFY_MIVBITSTREAM(m_ci_ortho_height.has_value());
  return *m_ci_ortho_height;
}

auto CameraIntrinsics::printTo(std::ostream &stream, uint16_t viewId) const -> std::ostream & {
  stream << "ci_cam_type[ " << viewId << " ]=" << ci_cam_type() << '\n';
  stream << "ci_projection_plane_width_minus1[ " << viewId
         << " ]=" << ci_projection_plane_width_minus1() << '\n';
  stream << "ci_projection_plane_height_minus1[ " << viewId
         << " ]=" << ci_projection_plane_height_minus1() << '\n';

  switch (ci_cam_type()) {
  case CiCamType::equirectangular:
    stream << "ci_erp_phi_min[ " << viewId << " ]=" << ci_erp_phi_min() << '\n';
    stream << "ci_erp_phi_max[ " << viewId << " ]=" << ci_erp_phi_max() << '\n';
    stream << "ci_erp_theta_min[ " << viewId << " ]=" << ci_erp_theta_min() << '\n';
    stream << "ci_erp_theta_max[ " << viewId << " ]=" << ci_erp_theta_max() << '\n';
    return stream;

  case CiCamType::perspective:
    stream << "ci_perspective_focal_hor[ " << viewId << " ]=" << ci_perspective_focal_hor() << '\n';
    stream << "ci_perspective_focal_ver[ " << viewId << " ]=" << ci_perspective_focal_ver() << '\n';
    stream << "ci_perspective_center_hor[ " << viewId << " ]=" << ci_perspective_center_hor()
           << '\n';
    stream << "ci_perspective_center_ver[ " << viewId << " ]=" << ci_perspective_center_ver()
           << '\n';
    return stream;

  case CiCamType::orthographic:
    stream << "ci_ortho_width[ " << viewId << " ]=" << ci_ortho_width() << '\n';
    stream << "ci_ortho_height[ " << viewId << " ]=" << ci_ortho_height() << '\n';
    return stream;

  default:
    MIVBITSTREAM_ERROR("Unknown cam type");
  }
}

auto CameraIntrinsics::decodeFrom(Common::InputBitstream &bitstream) -> CameraIntrinsics {
  auto x = CameraIntrinsics{};

  x.ci_cam_type(CiCamType(bitstream.getUint8()));
  x.ci_projection_plane_width_minus1(bitstream.getUint16());
  x.ci_projection_plane_height_minus1(bitstream.getUint16());

  switch (x.ci_cam_type()) {
  case CiCamType::equirectangular:
    x.ci_erp_phi_min(bitstream.getFloat32());
    x.ci_erp_phi_max(bitstream.getFloat32());
    x.ci_erp_theta_min(bitstream.getFloat32());
    x.ci_erp_theta_max(bitstream.getFloat32());
    return x;

  case CiCamType::perspective:
    x.ci_perspective_focal_hor(bitstream.getFloat32());
    x.ci_perspective_focal_ver(bitstream.getFloat32());
    x.ci_perspective_center_hor(bitstream.getFloat32());
    x.ci_perspective_center_ver(bitstream.getFloat32());
    return x;

  case CiCamType::orthographic:
    x.ci_ortho_width(bitstream.getFloat32());
    x.ci_ortho_height(bitstream.getFloat32());
    return x;

  default:
    MIVBITSTREAM_ERROR("Unknown cam type");
  }
}

void CameraIntrinsics::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putUint8(static_cast<uint8_t>(ci_cam_type()));
  bitstream.putUint16(ci_projection_plane_width_minus1());
  bitstream.putUint16(ci_projection_plane_height_minus1());

  switch (ci_cam_type()) {
  case CiCamType::equirectangular:
    bitstream.putFloat32(ci_erp_phi_min());
    bitstream.putFloat32(ci_erp_phi_max());
    bitstream.putFloat32(ci_erp_theta_min());
    bitstream.putFloat32(ci_erp_theta_max());
    return;

  case CiCamType::perspective:
    bitstream.putFloat32(ci_perspective_focal_hor());
    bitstream.putFloat32(ci_perspective_focal_ver());
    bitstream.putFloat32(ci_perspective_center_hor());
    bitstream.putFloat32(ci_perspective_center_ver());
    return;

  case CiCamType::orthographic:
    bitstream.putFloat32(ci_ortho_width());
    bitstream.putFloat32(ci_ortho_height());
    return;

  default:
    MIVBITSTREAM_ERROR("Unknown cam type");
  }
}

auto CameraExtrinsics::position() const noexcept -> Common::Vec3f {
  return {ce_view_pos_x(), ce_view_pos_y(), ce_view_pos_z()};
}

auto CameraExtrinsics::rotation() const noexcept -> Common::QuatF {
  const auto x = ce_view_quat_x();
  const auto y = ce_view_quat_y();
  const auto z = ce_view_quat_z();
  const auto w = std::sqrt(std::max(0.F, 1.F - x * x - y * y - z * z));

  return {x, y, z, w};
}

auto CameraExtrinsics::position(Common::Vec3f r) noexcept -> CameraExtrinsics & {
  ce_view_pos_x(r.x());
  ce_view_pos_y(r.y());
  ce_view_pos_z(r.z());
  return *this;
}

auto CameraExtrinsics::rotation(Common::QuatF q) noexcept -> CameraExtrinsics & {
  VERIFY_MIVBITSTREAM(normalized(q));
  ce_view_quat_x(q.x());
  ce_view_quat_y(q.y());
  ce_view_quat_z(q.z());
  return *this;
}

auto CameraExtrinsics::printTo(std::ostream &stream, uint16_t viewId) const -> std::ostream & {
  stream << "ce_view_pos_x[ " << viewId << " ]=" << ce_view_pos_x() << '\n';
  stream << "ce_view_pos_y[ " << viewId << " ]=" << ce_view_pos_y() << '\n';
  stream << "ce_view_pos_z[ " << viewId << " ]=" << ce_view_pos_z() << '\n';
  stream << "ce_view_quat_x[ " << viewId << " ]=" << ce_view_quat_x() << '\n';
  stream << "ce_view_quat_y[ " << viewId << " ]=" << ce_view_quat_y() << '\n';
  stream << "ce_view_quat_z[ " << viewId << " ]=" << ce_view_quat_z() << '\n';
  return stream;
}

auto CameraExtrinsics::decodeFrom(Common::InputBitstream &bitstream) -> CameraExtrinsics {
  auto x = CameraExtrinsics{};

  x.ce_view_pos_x(bitstream.getFloat32());
  x.ce_view_pos_y(bitstream.getFloat32());
  x.ce_view_pos_z(bitstream.getFloat32());
  x.ce_view_quat_x(bitstream.getFloat32());
  x.ce_view_quat_y(bitstream.getFloat32());
  x.ce_view_quat_z(bitstream.getFloat32());

  return x;
}

void CameraExtrinsics::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putFloat32(ce_view_pos_x());
  bitstream.putFloat32(ce_view_pos_y());
  bitstream.putFloat32(ce_view_pos_z());
  bitstream.putFloat32(ce_view_quat_x());
  bitstream.putFloat32(ce_view_quat_y());
  bitstream.putFloat32(ce_view_quat_z());
}

auto DepthQuantization::printTo(std::ostream &stream, uint16_t viewId) const -> std::ostream & {
  VERIFY_MIVBITSTREAM(dq_quantization_law() == 0);
  stream << "dq_quantization_law[ " << viewId << " ]=" << int{dq_quantization_law()}
         << "\ndq_norm_disp_low[ " << viewId << " ]=" << dq_norm_disp_low()
         << "\ndq_norm_disp_high[ " << viewId << " ]=" << dq_norm_disp_high()
         << "\ndq_depth_occ_map_threshold_default[ " << viewId
         << " ]=" << dq_depth_occ_map_threshold_default() << '\n';
  return stream;
}

auto DepthQuantization::decodeFrom(Common::InputBitstream &bitstream, const V3cParameterSet &vps)
    -> DepthQuantization {
  auto x = DepthQuantization{};

  const auto dq_quantization_law = bitstream.getUint8();
  VERIFY_MIVBITSTREAM(dq_quantization_law == 0);

  x.dq_norm_disp_low(bitstream.getFloat32());
  x.dq_norm_disp_high(bitstream.getFloat32());
  if (vps.vps_miv_extension().vme_embedded_occupancy_enabled_flag()) {
    x.dq_depth_occ_map_threshold_default(bitstream.getUExpGolomb<uint32_t>());
  }

  return x;
}

void DepthQuantization::encodeTo(Common::OutputBitstream &bitstream,
                                 const V3cParameterSet &vps) const {
  bitstream.putUint8(dq_quantization_law());
  bitstream.putFloat32(dq_norm_disp_low());
  bitstream.putFloat32(dq_norm_disp_high());
  if (vps.vps_miv_extension().vme_embedded_occupancy_enabled_flag()) {
    bitstream.putUExpGolomb(dq_depth_occ_map_threshold_default());
  }
}

PruningParents::PruningParents(std::vector<uint16_t> pp_parent_id)
    : m_pp_parent_id{std::move(pp_parent_id)} {}

auto PruningParents::pp_is_root_flag() const noexcept -> bool { return m_pp_parent_id.empty(); }

auto PruningParents::pp_num_parent_minus1() const noexcept -> uint16_t {
  VERIFY_MIVBITSTREAM(!pp_is_root_flag());
  return static_cast<uint16_t>(m_pp_parent_id.size() - 1);
}

auto PruningParents::pp_parent_id(uint16_t i) const noexcept -> uint16_t {
  VERIFY_MIVBITSTREAM(i < m_pp_parent_id.size());
  return m_pp_parent_id[i];
}

auto PruningParents::pp_parent_id(std::uint16_t i, std::uint16_t value) noexcept
    -> PruningParents & {
  VERIFY_MIVBITSTREAM(i < m_pp_parent_id.size());
  m_pp_parent_id[i] = value;
  return *this;
}

auto PruningParents::printTo(std::ostream &stream, uint16_t viewId) const -> std::ostream & {
  stream << "pp_is_root_flag[ " << viewId << " ]=" << std::boolalpha << pp_is_root_flag() << '\n';
  if (!pp_is_root_flag()) {
    stream << "pp_num_parent_minus1[ " << viewId << " ]=" << pp_num_parent_minus1() << '\n';
    for (auto i = 0; i <= pp_num_parent_minus1(); ++i) {
      stream << "pp_parent_id[ " << viewId << " ][ " << i << " ]=" << pp_parent_id(i) << '\n';
    }
  }
  return stream;
}

auto PruningParents::operator==(const PruningParents &other) const noexcept -> bool {
  return m_pp_parent_id == other.m_pp_parent_id;
}

auto PruningParents::operator!=(const PruningParents &other) const noexcept -> bool {
  return !operator==(other);
}

auto PruningParents::decodeFrom(Common::InputBitstream &bitstream, uint16_t mvp_num_views_minus1)
    -> PruningParents {
  const auto pp_is_root_flag = bitstream.getFlag();
  if (pp_is_root_flag) {
    return {};
  }

  const auto pp_num_parent_minus1 = bitstream.getUVar<size_t>(mvp_num_views_minus1);
  auto x = std::vector<uint16_t>(pp_num_parent_minus1 + 1);

  for (uint16_t &i : x) {
    i = bitstream.getUVar<uint16_t>(mvp_num_views_minus1 + uint64_t{1});
  }

  return PruningParents{x};
}

void PruningParents::encodeTo(Common::OutputBitstream &bitstream,
                              uint16_t mvp_num_views_minus1) const {
  bitstream.putFlag(pp_is_root_flag());
  if (!pp_is_root_flag()) {
    bitstream.putUVar(pp_num_parent_minus1(), mvp_num_views_minus1);
    for (uint16_t i = 0; i <= pp_num_parent_minus1(); ++i) {
      bitstream.putUVar(pp_parent_id(i), mvp_num_views_minus1 + uint64_t{1});
    }
  }
}

auto MivViewParamsList::mvp_num_views_minus1() const noexcept -> uint16_t {
  VERIFY_MIVBITSTREAM(!m_camera_extrinsics.empty());
  return static_cast<uint16_t>(m_camera_extrinsics.size() - 1);
}

auto MivViewParamsList::mvp_view_enabled_present_flag() const noexcept -> bool {
  return m_mvp_view_enabled_present_flag;
}

auto MivViewParamsList::mvp_view_enabled_in_atlas_flag(uint8_t atlasIdx,
                                                       uint16_t viewIdx) const noexcept -> bool {
  VERIFY_MIVBITSTREAM(mvp_view_enabled_present_flag());
  VERIFY_MIVBITSTREAM(atlasIdx < m_viewInAtlas.size());
  VERIFY_MIVBITSTREAM(viewIdx <= mvp_num_views_minus1());
  return m_viewInAtlas[atlasIdx][viewIdx].enabled;
}

auto MivViewParamsList::mvp_view_complete_in_atlas_flag(uint8_t atlasIdx,
                                                        uint16_t viewIdx) const noexcept -> bool {
  VERIFY_MIVBITSTREAM(mvp_view_enabled_in_atlas_flag(atlasIdx, viewIdx));
  VERIFY_MIVBITSTREAM(m_viewInAtlas[atlasIdx][viewIdx].complete.has_value());
  return *m_viewInAtlas[atlasIdx][viewIdx].complete;
}

auto MivViewParamsList::mvp_view_id(uint16_t viewIdx) const noexcept -> uint16_t {
  VERIFY_MIVBITSTREAM(viewIdx <= mvp_num_views_minus1());
  return m_mvp_view_id[viewIdx];
}

auto MivViewParamsList::mvp_inpaint_flag(std::uint16_t viewId) const noexcept -> bool {
  const auto viewIndex = viewIdToIndex(viewId);
  return m_mvpInpaintFlag[viewIndex];
}

auto MivViewParamsList::camera_extrinsics(uint16_t viewId) const noexcept
    -> const CameraExtrinsics & {
  VERIFY_MIVBITSTREAM(viewId < m_camera_extrinsics.size());
  return m_camera_extrinsics[viewId];
}

auto MivViewParamsList::camera_intrinsics(uint16_t viewId) const noexcept
    -> const CameraIntrinsics & {
  if (mvp_intrinsic_params_equal_flag()) {
    viewId = 0; // Convenience
  }
  VERIFY_MIVBITSTREAM(viewId < m_camera_intrinsics.size());
  return m_camera_intrinsics[viewId];
}

auto MivViewParamsList::depth_quantization(uint16_t viewId) const noexcept
    -> const DepthQuantization & {
  if (mvp_depth_quantization_params_equal_flag()) {
    viewId = 0; // Convenience
  }
  VERIFY_MIVBITSTREAM(viewId < m_depth_quantization.size());
  return m_depth_quantization[viewId];
}

auto MivViewParamsList::pruning_parent(uint16_t viewId) const noexcept -> const PruningParents & {
  VERIFY_MIVBITSTREAM(mvp_pruning_graph_params_present_flag());
  VERIFY_MIVBITSTREAM(viewId < m_pruning_parent.size());
  return m_pruning_parent[viewId];
}

auto MivViewParamsList::mvp_num_views_minus1(uint16_t value) noexcept -> MivViewParamsList & {
  for (auto &x : m_viewInAtlas) {
    x.resize(value + size_t{1});
  }
  m_camera_extrinsics.resize(value + size_t{1});
  m_mvpInpaintFlag.resize(value + size_t{1}, false);
  return *this;
}

auto MivViewParamsList::mvp_view_enabled_present_flag(bool value) noexcept -> MivViewParamsList & {
  m_mvp_view_enabled_present_flag = value;
  return *this;
}

auto MivViewParamsList::mvp_view_enabled_in_atlas_flag(uint8_t atlasIdx, uint16_t viewIdx,
                                                       bool value) noexcept -> MivViewParamsList & {
  mvp_view_enabled_present_flag(true);
  VERIFY_MIVBITSTREAM(viewIdx <= mvp_num_views_minus1());
  while (atlasIdx >= m_viewInAtlas.size()) {
    m_viewInAtlas.emplace_back(mvp_num_views_minus1() + 1);
  }
  m_viewInAtlas[atlasIdx][viewIdx].enabled = value;
  return *this;
}

auto MivViewParamsList::mvp_view_complete_in_atlas_flag(uint8_t atlasIdx, uint16_t viewIdx,
                                                        bool value) noexcept
    -> MivViewParamsList & {
  VERIFY_MIVBITSTREAM(mvp_view_enabled_in_atlas_flag(atlasIdx, viewIdx));
  m_viewInAtlas[atlasIdx][viewIdx].complete = value;
  return *this;
}

auto MivViewParamsList::mvp_explicit_view_id_flag(bool value) noexcept -> MivViewParamsList & {
  m_mvp_explicit_view_id_flag = value;
  return *this;
}

auto MivViewParamsList::mvp_view_id(uint16_t viewIdx, uint16_t viewId) noexcept
    -> MivViewParamsList & {
  VERIFY_MIVBITSTREAM(mvp_explicit_view_id_flag());
  if (m_mvp_view_id.size() < mvp_num_views_minus1() + size_t{1}) {
    m_mvp_view_id.resize(mvp_num_views_minus1() + size_t{1});
  }
  VERIFY_MIVBITSTREAM(viewIdx <= mvp_num_views_minus1());
  m_mvp_view_id[viewIdx] = viewId;
  return *this;
}

auto MivViewParamsList::mvp_inpaint_flag(uint16_t viewId, bool value) noexcept
    -> MivViewParamsList & {
  const auto viewIndex = viewIdToIndex(viewId);
  m_mvpInpaintFlag[viewIndex] = value;
  return *this;
}

auto MivViewParamsList::mvp_intrinsic_params_equal_flag(bool value) noexcept
    -> MivViewParamsList & {
  m_mvp_intrinsic_params_equal_flag = value;
  m_camera_intrinsics.resize(value ? 1U : m_camera_extrinsics.size());
  return *this;
}

auto MivViewParamsList::mvp_depth_quantization_params_equal_flag(bool value) noexcept
    -> MivViewParamsList & {
  m_mvp_depth_quantization_params_equal_flag = value;
  m_depth_quantization.resize(value ? 1U : m_camera_extrinsics.size());
  return *this;
}

auto MivViewParamsList::mvp_pruning_graph_params_present_flag(bool value) noexcept
    -> MivViewParamsList & {
  m_mvp_pruning_graph_params_present_flag = value;
  m_pruning_parent.resize(value ? m_camera_extrinsics.size() : 0U);
  return *this;
}

auto MivViewParamsList::camera_extrinsics(uint16_t viewId) noexcept -> CameraExtrinsics & {
  VERIFY_MIVBITSTREAM(viewId < m_camera_extrinsics.size());
  return m_camera_extrinsics[viewId];
}

auto MivViewParamsList::camera_intrinsics(uint16_t viewId) noexcept -> CameraIntrinsics & {
  VERIFY_MIVBITSTREAM(viewId < m_camera_intrinsics.size());
  return m_camera_intrinsics[viewId];
}

auto MivViewParamsList::depth_quantization(uint16_t viewId) noexcept -> DepthQuantization & {
  VERIFY_MIVBITSTREAM(viewId < m_depth_quantization.size());
  return m_depth_quantization[viewId];
}

auto MivViewParamsList::pruning_parent(uint16_t viewId) noexcept -> PruningParents & {
  VERIFY_MIVBITSTREAM(viewId < m_pruning_parent.size());
  return m_pruning_parent[viewId];
}

auto MivViewParamsList::viewIndexToId(std::uint16_t index) const -> std::uint16_t {
  VERIFY_MIVBITSTREAM(index <= mvp_num_views_minus1());
  if (mvp_explicit_view_id_flag()) {
    return mvp_view_id(index);
  }
  return index;
}

auto MivViewParamsList::viewIdToIndex(std::uint16_t id) const -> std::uint16_t {
  if (mvp_explicit_view_id_flag()) {
    const auto iter = std::find(m_mvp_view_id.cbegin(), m_mvp_view_id.cend(), id);
    VERIFY_MIVBITSTREAM(iter != m_mvp_view_id.cend());
    return static_cast<uint16_t>(std::distance(m_mvp_view_id.cbegin(), iter));
  }
  return id;
}

auto operator<<(std::ostream &stream, const MivViewParamsList &x) -> std::ostream & {
  stream << "mvp_num_views_minus1=" << x.mvp_num_views_minus1() << '\n';
  stream << "mvp_view_enabled_present_flag=" << std::boolalpha << x.mvp_view_enabled_present_flag()
         << '\n';

  if (x.mvp_view_enabled_present_flag()) {
    for (size_t a = 0; a < x.m_viewInAtlas.size(); ++a) {
      for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
        stream << "mvp_view_enabled_in_atlas_flag[ " << a << " ][ " << v << " ]=" << std::boolalpha
               << x.mvp_view_enabled_in_atlas_flag(static_cast<uint8_t>(a), v) << '\n';
        if (x.mvp_view_enabled_in_atlas_flag(static_cast<uint8_t>(a), v)) {
          stream << "mvp_view_complete_in_atlas_flag[ " << a << " ][ " << v
                 << " ]=" << std::boolalpha
                 << x.mvp_view_complete_in_atlas_flag(static_cast<uint8_t>(a), v) << '\n';
        }
      }
    }
  }

  stream << "mvp_explicit_view_id_flag=" << std::boolalpha << x.mvp_explicit_view_id_flag() << '\n';
  if (x.mvp_explicit_view_id_flag()) {
    for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
      stream << "mvp_view_id[ " << v << " ]=" << x.mvp_view_id(v) << '\n';
    }
  }

  for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
    const auto viewId = x.viewIndexToId(v);
    x.camera_extrinsics(v).printTo(stream, v);
    fmt::print(stream, "mvp_inpaint_flag[ {} ]={}\n", viewId, x.mvp_inpaint_flag(viewId));
  }

  stream << "mvp_intrinsic_params_equal_flag=" << std::boolalpha
         << x.mvp_intrinsic_params_equal_flag() << '\n';
  if (x.mvp_intrinsic_params_equal_flag()) {
    x.camera_intrinsics(0).printTo(stream, 0);
  } else {
    for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
      x.camera_intrinsics(v).printTo(stream, v);
    }
  }

  if (x.m_mvp_depth_quantization_params_equal_flag) {
    stream << "mvp_depth_quantization_params_equal_flag=" << std::boolalpha
           << x.mvp_depth_quantization_params_equal_flag() << '\n';
    if (x.mvp_depth_quantization_params_equal_flag()) {
      x.depth_quantization(0).printTo(stream, 0);
    } else {
      for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
        x.depth_quantization(v).printTo(stream, v);
      }
    }
  }

  stream << "mvp_pruning_graph_params_present_flag=" << std::boolalpha
         << x.mvp_pruning_graph_params_present_flag() << '\n';
  if (x.mvp_pruning_graph_params_present_flag()) {
    for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
      x.pruning_parent(v).printTo(stream, v);
    }
  }
  return stream;
}

auto MivViewParamsList::operator==(const MivViewParamsList &other) const noexcept -> bool {
  return m_viewInAtlas == other.m_viewInAtlas &&
         m_mvp_explicit_view_id_flag == other.m_mvp_explicit_view_id_flag &&
         m_mvp_view_id == other.m_mvp_view_id && m_camera_extrinsics == other.m_camera_extrinsics &&
         m_mvpInpaintFlag == other.m_mvpInpaintFlag &&
         m_mvp_intrinsic_params_equal_flag == other.m_mvp_intrinsic_params_equal_flag &&
         m_camera_intrinsics == other.m_camera_intrinsics &&
         m_mvp_depth_quantization_params_equal_flag ==
             other.m_mvp_depth_quantization_params_equal_flag &&
         m_depth_quantization == other.m_depth_quantization &&
         m_mvp_pruning_graph_params_present_flag == other.m_mvp_pruning_graph_params_present_flag &&
         m_pruning_parent == other.m_pruning_parent;
}

auto MivViewParamsList::operator!=(const MivViewParamsList &other) const noexcept -> bool {
  return !operator==(other);
}

auto MivViewParamsList::decodeFrom(Common::InputBitstream &bitstream, const V3cParameterSet &vps,
                                   const CommonAtlasSequenceParameterSetRBSP &casps)
    -> MivViewParamsList {
  auto x = MivViewParamsList{};

  x.mvp_num_views_minus1(bitstream.getUint16());
  x.mvp_view_enabled_present_flag(bitstream.getFlag());

  if (x.mvp_view_enabled_present_flag()) {
    for (uint8_t a = 0; a <= vps.vps_atlas_count_minus1(); ++a) {
      for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
        x.mvp_view_enabled_in_atlas_flag(a, v, bitstream.getFlag());
        if (x.mvp_view_enabled_in_atlas_flag(a, v)) {
          x.mvp_view_complete_in_atlas_flag(a, v, bitstream.getFlag());
        }
      }
    }
  }

  x.mvp_explicit_view_id_flag(bitstream.getFlag());
  if (x.mvp_explicit_view_id_flag()) {
    for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
      x.mvp_view_id(v, bitstream.getUint16());
    }
  }

  for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
    const auto viewId = x.viewIndexToId(v);
    x.camera_extrinsics(v) = CameraExtrinsics::decodeFrom(bitstream);
    x.mvp_inpaint_flag(viewId, bitstream.getFlag());
  }

  x.mvp_intrinsic_params_equal_flag(bitstream.getFlag());

  if (x.mvp_intrinsic_params_equal_flag()) {
    x.camera_intrinsics() = CameraIntrinsics::decodeFrom(bitstream);
  } else {
    for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
      x.camera_intrinsics(v) = CameraIntrinsics::decodeFrom(bitstream);
    }
  }

  if (casps.casps_miv_extension().casme_depth_quantization_params_present_flag()) {
    x.mvp_depth_quantization_params_equal_flag(bitstream.getFlag());

    if (x.mvp_depth_quantization_params_equal_flag()) {
      x.depth_quantization() = DepthQuantization::decodeFrom(bitstream, vps);
    } else {
      for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
        x.depth_quantization(v) = DepthQuantization::decodeFrom(bitstream, vps);
      }
    }
  }

  x.mvp_pruning_graph_params_present_flag(bitstream.getFlag());

  if (x.mvp_pruning_graph_params_present_flag()) {
    for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
      x.pruning_parent(v) = PruningParents::decodeFrom(bitstream, x.mvp_num_views_minus1());
    }
  }
  return x;
}

void MivViewParamsList::encodeTo(Common::OutputBitstream &bitstream, const V3cParameterSet &vps,
                                 const CommonAtlasSequenceParameterSetRBSP &casps) const {
  bitstream.putUint16(mvp_num_views_minus1());
  bitstream.putFlag(mvp_view_enabled_present_flag());

  if (mvp_view_enabled_present_flag()) {
    for (uint8_t a = 0; a <= vps.vps_atlas_count_minus1(); ++a) {
      for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
        bitstream.putFlag(mvp_view_enabled_in_atlas_flag(a, v));
        if (mvp_view_enabled_in_atlas_flag(a, v)) {
          bitstream.putFlag(mvp_view_complete_in_atlas_flag(a, v));
        }
      }
    }
  }

  bitstream.putFlag(mvp_explicit_view_id_flag());
  if (mvp_explicit_view_id_flag()) {
    for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
      bitstream.putUint16(mvp_view_id(v));
    }
  }

  for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
    const auto viewId = viewIndexToId(v);
    camera_extrinsics(v).encodeTo(bitstream);
    bitstream.putFlag(mvp_inpaint_flag(viewId));
  }

  bitstream.putFlag(mvp_intrinsic_params_equal_flag());

  if (mvp_intrinsic_params_equal_flag()) {
    camera_intrinsics().encodeTo(bitstream);
  } else {
    for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
      camera_intrinsics(v).encodeTo(bitstream);
    }
  }

  if (casps.casps_miv_extension().casme_depth_quantization_params_present_flag()) {
    bitstream.putFlag(mvp_depth_quantization_params_equal_flag());

    if (mvp_depth_quantization_params_equal_flag()) {
      depth_quantization().encodeTo(bitstream, vps);
    } else {
      for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
        depth_quantization(v).encodeTo(bitstream, vps);
      }
    }
  }
  bitstream.putFlag(mvp_pruning_graph_params_present_flag());

  if (mvp_pruning_graph_params_present_flag()) {
    for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
      pruning_parent(v).encodeTo(bitstream, mvp_num_views_minus1());
    }
  }
}

auto CommonAtlasFrameMivExtension::came_update_extrinsics_flag() const noexcept -> bool {
  VERIFY_MIVBITSTREAM(m_came_update_extrinsics_flag.has_value());
  return *m_came_update_extrinsics_flag;
}

auto CommonAtlasFrameMivExtension::came_update_intrinsics_flag() const noexcept -> bool {
  VERIFY_MIVBITSTREAM(m_came_update_intrinsics_flag.has_value());
  return *m_came_update_intrinsics_flag;
}

auto CommonAtlasFrameMivExtension::came_update_depth_quantization_flag() const noexcept -> bool {
  VERIFY_MIVBITSTREAM(m_came_update_depth_quantization_flag.has_value());
  return *m_came_update_depth_quantization_flag;
}

auto CommonAtlasFrameMivExtension::miv_view_params_list() const noexcept
    -> const MivViewParamsList & {
  VERIFY_MIVBITSTREAM(m_miv_view_params_list.has_value());
  return *m_miv_view_params_list;
}

auto CommonAtlasFrameMivExtension::miv_view_params_update_extrinsics() const noexcept
    -> const MivViewParamsUpdateExtrinsics & {
  VERIFY_MIVBITSTREAM(came_update_extrinsics_flag());
  VERIFY_MIVBITSTREAM(m_miv_view_params_update_extrinsics.has_value());
  return *m_miv_view_params_update_extrinsics;
}

auto CommonAtlasFrameMivExtension::miv_view_params_update_intrinsics() const noexcept
    -> const MivViewParamsUpdateIntrinsics & {
  VERIFY_MIVBITSTREAM(came_update_intrinsics_flag());
  VERIFY_MIVBITSTREAM(m_miv_view_params_update_intrinsics.has_value());
  return *m_miv_view_params_update_intrinsics;
}

auto CommonAtlasFrameMivExtension::miv_view_params_update_depth_quantization() const noexcept
    -> const MivViewParamsUpdateDepthQuantization & {
  VERIFY_MIVBITSTREAM(came_update_depth_quantization_flag());
  VERIFY_MIVBITSTREAM(m_miv_view_params_update_depth_quantization.has_value());
  return *m_miv_view_params_update_depth_quantization;
}

auto CommonAtlasFrameMivExtension::miv_view_params_list() noexcept -> MivViewParamsList & {
  if (!m_miv_view_params_list) {
    m_miv_view_params_list = MivViewParamsList{};
  }
  return *m_miv_view_params_list;
}

auto CommonAtlasFrameMivExtension::miv_view_params_update_extrinsics() noexcept
    -> MivViewParamsUpdateExtrinsics & {
  VERIFY_MIVBITSTREAM(came_update_extrinsics_flag());
  if (!m_miv_view_params_update_extrinsics) {
    m_miv_view_params_update_extrinsics = MivViewParamsUpdateExtrinsics{};
  }
  return *m_miv_view_params_update_extrinsics;
}

auto CommonAtlasFrameMivExtension::miv_view_params_update_intrinsics() noexcept
    -> MivViewParamsUpdateIntrinsics & {
  VERIFY_MIVBITSTREAM(came_update_intrinsics_flag());
  if (!m_miv_view_params_update_intrinsics) {
    m_miv_view_params_update_intrinsics = MivViewParamsUpdateIntrinsics{};
  }
  return *m_miv_view_params_update_intrinsics;
}

auto CommonAtlasFrameMivExtension::miv_view_params_update_depth_quantization() noexcept
    -> MivViewParamsUpdateDepthQuantization & {
  VERIFY_MIVBITSTREAM(came_update_depth_quantization_flag());
  if (!m_miv_view_params_update_depth_quantization) {
    m_miv_view_params_update_depth_quantization = MivViewParamsUpdateDepthQuantization{};
  }
  return *m_miv_view_params_update_depth_quantization;
}

auto CommonAtlasFrameMivExtension::came_update_extrinsics_flag(bool value) noexcept
    -> CommonAtlasFrameMivExtension & {
  m_came_update_extrinsics_flag = value;
  return *this;
}

auto CommonAtlasFrameMivExtension::came_update_intrinsics_flag(bool value) noexcept
    -> CommonAtlasFrameMivExtension & {
  m_came_update_intrinsics_flag = value;
  return *this;
}

auto CommonAtlasFrameMivExtension::came_update_depth_quantization_flag(bool value) noexcept
    -> CommonAtlasFrameMivExtension & {
  m_came_update_depth_quantization_flag = value;
  return *this;
}

auto operator<<(std::ostream &stream, const CommonAtlasFrameMivExtension &x) -> std::ostream & {
  if (x.m_miv_view_params_list) {
    stream << "miv_view_params_list=" << x.miv_view_params_list();
  } else {
    stream << "came_update_extrinsics_flag=" << std::boolalpha << x.came_update_extrinsics_flag()
           << '\n';
    stream << "came_update_intrinsics_flag=" << std::boolalpha << x.came_update_intrinsics_flag()
           << '\n';
    if (x.m_came_update_depth_quantization_flag) {
      stream << "came_update_depth_quantization_flag=" << std::boolalpha
             << x.came_update_depth_quantization_flag() << '\n';
    }
    if (x.came_update_extrinsics_flag()) {
      stream << x.miv_view_params_update_extrinsics();
    }
    if (x.came_update_intrinsics_flag()) {
      stream << x.miv_view_params_update_intrinsics();
    }
    if (x.m_came_update_depth_quantization_flag.value_or(false)) {
      stream << x.miv_view_params_update_depth_quantization();
    }
  }
  return stream;
}

auto CommonAtlasFrameMivExtension::operator==(
    const CommonAtlasFrameMivExtension &other) const noexcept -> bool {
  if (m_miv_view_params_list != other.m_miv_view_params_list) {
    return false;
  }
  if (m_miv_view_params_list.has_value()) {
    return true;
  }
  if (came_update_extrinsics_flag() != other.came_update_extrinsics_flag() ||
      came_update_intrinsics_flag() != other.came_update_intrinsics_flag() ||
      m_came_update_depth_quantization_flag != other.m_came_update_depth_quantization_flag) {
    return false;
  }
  if (came_update_extrinsics_flag() &&
      miv_view_params_update_extrinsics() != other.miv_view_params_update_extrinsics()) {
    return false;
  }
  if (came_update_intrinsics_flag() &&
      miv_view_params_update_intrinsics() != other.miv_view_params_update_intrinsics()) {
    return false;
  }
  if (m_came_update_depth_quantization_flag.value_or(false) &&
      miv_view_params_update_depth_quantization() !=
          other.miv_view_params_update_depth_quantization()) {
    return false;
  }

  return true;
}

auto CommonAtlasFrameMivExtension::operator!=(
    const CommonAtlasFrameMivExtension &other) const noexcept -> bool {
  return !operator==(other);
}

auto CommonAtlasFrameMivExtension::decodeFrom(Common::InputBitstream &bitstream,
                                              const V3cParameterSet &vps, const NalUnitHeader &nuh,
                                              const CommonAtlasSequenceParameterSetRBSP &casps)
    -> CommonAtlasFrameMivExtension {
  auto x = CommonAtlasFrameMivExtension{};

  if (nuh.nal_unit_type() == NalUnitType::NAL_IDR_CAF) {
    x.miv_view_params_list() = MivViewParamsList::decodeFrom(bitstream, vps, casps);
  } else {
    x.came_update_extrinsics_flag(bitstream.getFlag());
    x.came_update_intrinsics_flag(bitstream.getFlag());
    const auto &casme = casps.casps_miv_extension();

    if (casme.casme_depth_quantization_params_present_flag()) {
      x.came_update_depth_quantization_flag(bitstream.getFlag());
    }

    if (x.came_update_extrinsics_flag()) {
      x.miv_view_params_update_extrinsics() = MivViewParamsUpdateExtrinsics::decodeFrom(bitstream);
    }
    if (x.came_update_intrinsics_flag()) {
      x.miv_view_params_update_intrinsics() = MivViewParamsUpdateIntrinsics::decodeFrom(bitstream);
    }
    if (casme.casme_depth_quantization_params_present_flag() &&
        x.came_update_depth_quantization_flag()) {
      x.miv_view_params_update_depth_quantization() =
          MivViewParamsUpdateDepthQuantization::decodeFrom(bitstream, vps);
    }
  }

  return x;
}

void CommonAtlasFrameMivExtension::encodeTo(
    Common::OutputBitstream &bitstream, const V3cParameterSet &vps, const NalUnitHeader &nuh,
    const CommonAtlasSequenceParameterSetRBSP &casps) const {
  if (nuh.nal_unit_type() == NalUnitType::NAL_IDR_CAF) {
    miv_view_params_list().encodeTo(bitstream, vps, casps);
  } else {
    bitstream.putFlag(came_update_extrinsics_flag());
    bitstream.putFlag(came_update_intrinsics_flag());
    if (casps.casps_miv_extension().casme_depth_quantization_params_present_flag()) {
      bitstream.putFlag(came_update_depth_quantization_flag());
    }

    if (came_update_extrinsics_flag()) {
      miv_view_params_update_extrinsics().encodeTo(bitstream);
    }
    if (came_update_intrinsics_flag()) {
      miv_view_params_update_intrinsics().encodeTo(bitstream);
    }
    const auto &casme = casps.casps_miv_extension();
    if (casme.casme_depth_quantization_params_present_flag() &&
        came_update_depth_quantization_flag()) {
      miv_view_params_update_depth_quantization().encodeTo(bitstream, vps);
    }
  }
}

auto MivViewParamsList::mvp_depth_quantization_params_equal_flag() const noexcept -> bool {
  VERIFY_MIVBITSTREAM(m_mvp_depth_quantization_params_equal_flag.has_value());
  return *m_mvp_depth_quantization_params_equal_flag;
}

auto MivViewParamsUpdateExtrinsics::mvpue_num_view_updates_minus1() const noexcept -> uint16_t {
  return m_mvpue_num_view_updates_minus1;
}

auto MivViewParamsUpdateExtrinsics::mvpue_view_idx(const uint16_t i) const noexcept -> uint16_t {
  VERIFY_MIVBITSTREAM(i < m_mvpue_view_idx.size());
  return m_mvpue_view_idx[i];
}

auto MivViewParamsUpdateExtrinsics::camera_extrinsics(const uint16_t i) const noexcept
    -> const CameraExtrinsics & {
  VERIFY_MIVBITSTREAM(i < m_camera_extrinsics.size());
  return m_camera_extrinsics[i];
}
auto MivViewParamsUpdateExtrinsics::camera_extrinsics(const uint16_t i) noexcept
    -> CameraExtrinsics & {
  VERIFY_MIVBITSTREAM(i < m_camera_extrinsics.size());
  return m_camera_extrinsics[i];
}

auto MivViewParamsUpdateExtrinsics::mvpue_num_view_updates_minus1(const uint16_t value) noexcept
    -> MivViewParamsUpdateExtrinsics & {
  m_mvpue_num_view_updates_minus1 = value;
  m_mvpue_view_idx.resize(value + size_t{1});
  m_camera_extrinsics.resize(value + size_t{1});
  return *this;
}
auto MivViewParamsUpdateExtrinsics::mvpue_view_idx(const uint16_t i, const uint16_t value) noexcept
    -> MivViewParamsUpdateExtrinsics & {
  VERIFY_MIVBITSTREAM(i < m_camera_extrinsics.size());
  m_mvpue_view_idx[i] = value;
  return *this;
}

auto operator<<(std::ostream &stream, const MivViewParamsUpdateExtrinsics &x) -> std::ostream & {
  stream << "mvpue_num_view_updates_minus1=" << x.mvpue_num_view_updates_minus1() << '\n';
  for (uint16_t i = 0; i <= x.mvpue_num_view_updates_minus1(); ++i) {
    stream << "mvpue_view_idx[ " << i << " ]=" << x.mvpue_view_idx(i) << '\n';
    x.camera_extrinsics(i).printTo(stream, i);
  }
  return stream;
}

void MivViewParamsUpdateExtrinsics::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putUint16(mvpue_num_view_updates_minus1());
  for (uint16_t i = 0; i <= mvpue_num_view_updates_minus1(); ++i) {
    bitstream.putUint16(mvpue_view_idx(i));
    camera_extrinsics(i).encodeTo(bitstream);
  }
}

auto MivViewParamsUpdateExtrinsics::decodeFrom(Common::InputBitstream &bitstream)
    -> MivViewParamsUpdateExtrinsics {
  auto x = MivViewParamsUpdateExtrinsics{};
  x.mvpue_num_view_updates_minus1(bitstream.getUint16());
  for (uint16_t i = 0; i <= x.mvpue_num_view_updates_minus1(); ++i) {
    x.mvpue_view_idx(i, bitstream.getUint16());
    x.camera_extrinsics(i) = CameraExtrinsics::decodeFrom(bitstream);
  }
  return x;
}

auto MivViewParamsUpdateExtrinsics::operator==(
    const MivViewParamsUpdateExtrinsics &other) const noexcept -> bool {
  return m_mvpue_num_view_updates_minus1 == other.m_mvpue_num_view_updates_minus1 &&
         m_mvpue_view_idx == other.m_mvpue_view_idx &&
         m_camera_extrinsics == other.m_camera_extrinsics;
}

auto MivViewParamsUpdateExtrinsics::operator!=(
    const MivViewParamsUpdateExtrinsics &other) const noexcept -> bool {
  return !operator==(other);
}

auto MivViewParamsUpdateIntrinsics::mvpui_num_view_updates_minus1() const noexcept -> uint16_t {
  return m_mvpui_num_view_updates_minus1;
}

auto MivViewParamsUpdateIntrinsics::mvpui_view_idx(const uint16_t i) const noexcept -> uint16_t {
  VERIFY_MIVBITSTREAM(i < m_mvpui_view_idx.size());
  return m_mvpui_view_idx[i];
}

auto MivViewParamsUpdateIntrinsics::camera_intrinsics(const uint16_t i) const noexcept
    -> const CameraIntrinsics & {
  VERIFY_MIVBITSTREAM(i < m_camera_intrinsics.size());
  return m_camera_intrinsics[i];
}

auto MivViewParamsUpdateIntrinsics::camera_intrinsics(const uint16_t i) noexcept
    -> CameraIntrinsics & {
  VERIFY_MIVBITSTREAM(i < m_camera_intrinsics.size());
  return m_camera_intrinsics[i];
}

auto MivViewParamsUpdateIntrinsics::mvpui_num_view_updates_minus1(const uint16_t value) noexcept
    -> MivViewParamsUpdateIntrinsics & {
  m_mvpui_num_view_updates_minus1 = value;
  m_mvpui_view_idx.resize(m_mvpui_num_view_updates_minus1 + size_t{1});
  m_camera_intrinsics.resize(m_mvpui_num_view_updates_minus1 + size_t{1});
  return *this;
}

auto MivViewParamsUpdateIntrinsics::mvpui_view_idx(const uint16_t i, const uint16_t value) noexcept
    -> MivViewParamsUpdateIntrinsics & {
  VERIFY_MIVBITSTREAM(i < m_mvpui_view_idx.size());
  m_mvpui_view_idx[i] = value;
  return *this;
}

auto operator<<(std::ostream &stream, const MivViewParamsUpdateIntrinsics &x) -> std::ostream & {
  stream << "mvpui_num_view_updates_minus1=" << x.mvpui_num_view_updates_minus1() << '\n';
  for (uint16_t i = 0; i <= x.mvpui_num_view_updates_minus1(); ++i) {
    stream << "mvpui_view_idx[ " << i << " ]=" << x.mvpui_view_idx(i) << '\n';
    x.camera_intrinsics(i).printTo(stream, i);
  }
  return stream;
}

void MivViewParamsUpdateIntrinsics::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putUint16(mvpui_num_view_updates_minus1());
  for (uint16_t i = 0; i <= mvpui_num_view_updates_minus1(); ++i) {
    bitstream.putUint16(mvpui_view_idx(i));
    camera_intrinsics(i).encodeTo(bitstream);
  }
}

auto MivViewParamsUpdateIntrinsics::decodeFrom(Common::InputBitstream &bitstream)
    -> MivViewParamsUpdateIntrinsics {
  auto x = MivViewParamsUpdateIntrinsics{};
  x.mvpui_num_view_updates_minus1(bitstream.getUint16());
  for (uint16_t i = 0; i <= x.mvpui_num_view_updates_minus1(); ++i) {
    x.mvpui_view_idx(i, bitstream.getUint16());
    x.camera_intrinsics(i) = CameraIntrinsics::decodeFrom(bitstream);
  }
  return x;
}

auto MivViewParamsUpdateIntrinsics::operator==(
    const MivViewParamsUpdateIntrinsics &other) const noexcept -> bool {
  return m_mvpui_num_view_updates_minus1 == other.m_mvpui_num_view_updates_minus1 &&
         m_mvpui_view_idx == other.m_mvpui_view_idx &&
         m_camera_intrinsics == other.m_camera_intrinsics;
}

auto MivViewParamsUpdateIntrinsics::operator!=(
    const MivViewParamsUpdateIntrinsics &other) const noexcept -> bool {
  return !operator==(other);
}

auto MivViewParamsUpdateDepthQuantization::mvpudq_num_view_updates_minus1() const noexcept
    -> uint16_t {
  return m_mvpudq_num_view_updates_minus1;
}

auto MivViewParamsUpdateDepthQuantization::mvpudq_view_idx(const uint16_t i) const noexcept
    -> uint16_t {
  VERIFY_MIVBITSTREAM(i < m_mvpudq_view_idx.size());
  return m_mvpudq_view_idx[i];
}

auto MivViewParamsUpdateDepthQuantization::depth_quantization(const uint16_t i) const noexcept
    -> const DepthQuantization & {
  VERIFY_MIVBITSTREAM(i < m_depth_quantization.size());
  return m_depth_quantization[i];
}

auto MivViewParamsUpdateDepthQuantization::depth_quantization(const uint16_t i) noexcept
    -> DepthQuantization & {
  VERIFY_MIVBITSTREAM(i < m_depth_quantization.size());
  return m_depth_quantization[i];
}

auto MivViewParamsUpdateDepthQuantization::mvpudq_num_view_updates_minus1(
    const uint16_t value) noexcept -> MivViewParamsUpdateDepthQuantization & {
  m_mvpudq_num_view_updates_minus1 = value;
  m_mvpudq_view_idx.resize(m_mvpudq_num_view_updates_minus1 + size_t{1});
  m_depth_quantization.resize(m_mvpudq_num_view_updates_minus1 + size_t{1});
  return *this;
}
auto MivViewParamsUpdateDepthQuantization::mvpudq_view_idx(const uint16_t i,
                                                           const uint16_t value) noexcept
    -> MivViewParamsUpdateDepthQuantization & {
  VERIFY_MIVBITSTREAM(i < m_mvpudq_view_idx.size());
  m_mvpudq_view_idx[i] = value;
  return *this;
}

auto operator<<(std::ostream &stream, const MivViewParamsUpdateDepthQuantization &x)
    -> std::ostream & {
  stream << "mvpudq_num_view_updates_minus1=" << x.mvpudq_num_view_updates_minus1() << '\n';
  for (uint16_t i = 0; i <= x.mvpudq_num_view_updates_minus1(); ++i) {
    stream << "mvpudq_view_idx[ " << i << " ]=" << x.mvpudq_view_idx(i) << '\n';
    x.depth_quantization(i).printTo(stream, i);
  }
  return stream;
}

void MivViewParamsUpdateDepthQuantization::encodeTo(Common::OutputBitstream &bitstream,
                                                    const V3cParameterSet &vps) const {
  bitstream.putUint16(mvpudq_num_view_updates_minus1());
  for (uint16_t i = 0; i <= mvpudq_num_view_updates_minus1(); ++i) {
    bitstream.putUint16(mvpudq_view_idx(i));
    depth_quantization(i).encodeTo(bitstream, vps);
  }
}

auto MivViewParamsUpdateDepthQuantization::decodeFrom(Common::InputBitstream &bitstream,
                                                      const V3cParameterSet &vps)
    -> MivViewParamsUpdateDepthQuantization {
  auto x = MivViewParamsUpdateDepthQuantization{};
  x.mvpudq_num_view_updates_minus1(bitstream.getUint16());
  for (uint16_t i = 0; i <= x.mvpudq_num_view_updates_minus1(); ++i) {
    x.mvpudq_view_idx(i, bitstream.getUint16());
    x.depth_quantization(i) = DepthQuantization::decodeFrom(bitstream, vps);
  }
  return x;
}

auto MivViewParamsUpdateDepthQuantization::operator==(
    const MivViewParamsUpdateDepthQuantization &other) const noexcept -> bool {
  return m_mvpudq_num_view_updates_minus1 == other.m_mvpudq_num_view_updates_minus1 &&
         m_mvpudq_view_idx == other.m_mvpudq_view_idx &&
         m_depth_quantization == other.m_depth_quantization;
}

auto MivViewParamsUpdateDepthQuantization::operator!=(
    const MivViewParamsUpdateDepthQuantization &other) const noexcept -> bool {
  return !operator==(other);
}
} // namespace TMIV::MivBitstream
