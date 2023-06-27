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

#ifndef _TMIV_MIVBITSTREAM_COMMONATLASFRAMEMIVEXTENSION_H_
#error "Include the .h, not the .hpp"
#endif

namespace TMIV::MivBitstream {
constexpr auto CameraIntrinsics::ci_cam_type() const noexcept { return m_ci_cam_type; }

constexpr auto CameraIntrinsics::ci_projection_plane_width_minus1() const noexcept {
  return m_ci_projection_plane_width_minus1;
}

constexpr auto CameraIntrinsics::ci_projection_plane_height_minus1() const noexcept {
  return m_ci_projection_plane_height_minus1;
}

constexpr auto CameraIntrinsics::ci_cam_type(const CiCamType value) noexcept -> auto & {
  m_ci_cam_type = value;
  return *this;
}

constexpr auto
CameraIntrinsics::ci_projection_plane_width_minus1(const std::uint16_t value) noexcept -> auto & {
  m_ci_projection_plane_width_minus1 = value;
  return *this;
}

constexpr auto
CameraIntrinsics::ci_projection_plane_height_minus1(const std::uint16_t value) noexcept -> auto & {
  m_ci_projection_plane_height_minus1 = value;
  return *this;
}

constexpr auto CameraIntrinsics::ci_erp_phi_min(const float value) noexcept -> auto & {
  m_ci_erp_phi_min = value;
  return *this;
}

constexpr auto CameraIntrinsics::ci_erp_phi_max(const float value) noexcept -> auto & {
  m_ci_erp_phi_max = value;
  return *this;
}

constexpr auto CameraIntrinsics::ci_erp_theta_min(const float value) noexcept -> auto & {
  m_ci_erp_theta_min = value;
  return *this;
}

constexpr auto CameraIntrinsics::ci_erp_theta_max(const float value) noexcept -> auto & {
  m_ci_erp_theta_max = value;
  return *this;
}

constexpr auto CameraIntrinsics::ci_perspective_focal_hor(const float value) noexcept -> auto & {
  m_ci_perspective_focal_hor = value;
  return *this;
}

constexpr auto CameraIntrinsics::ci_perspective_focal_ver(const float value) noexcept -> auto & {
  m_ci_perspective_focal_ver = value;
  return *this;
}

constexpr auto CameraIntrinsics::ci_perspective_center_hor(const float value) noexcept -> auto & {
  m_ci_perspective_center_hor = value;
  return *this;
}

constexpr auto CameraIntrinsics::ci_perspective_center_ver(const float value) noexcept -> auto & {
  m_ci_perspective_center_ver = value;
  return *this;
}

constexpr auto CameraIntrinsics::ci_ortho_width(const float value) noexcept -> auto & {
  m_ci_ortho_width = value;
  return *this;
}

constexpr auto CameraIntrinsics::ci_ortho_height(const float value) noexcept -> auto & {
  m_ci_ortho_height = value;
  return *this;
}

template <typename F> decltype(auto) CameraIntrinsics::dispatch(F f) const {
  switch (ci_cam_type()) {
  case CiCamType::equirectangular:
    return f(Equirectangular{});
  case CiCamType::perspective:
    return f(Perspective{});
  case CiCamType::orthographic:
    return f(Orthographic{});
  default:
    abort();
  }
}

constexpr auto CameraIntrinsics::operator==(const CameraIntrinsics &other) const noexcept {
  return ci_cam_type() == other.ci_cam_type() &&
         ci_projection_plane_width_minus1() == other.ci_projection_plane_width_minus1() &&
         ci_projection_plane_height_minus1() == other.ci_projection_plane_height_minus1() &&
         m_ci_erp_phi_min == other.m_ci_erp_phi_min && m_ci_erp_phi_max == other.m_ci_erp_phi_max &&
         m_ci_erp_theta_min == other.m_ci_erp_theta_min &&
         m_ci_erp_theta_max == other.m_ci_erp_theta_max &&
         m_ci_perspective_focal_hor == other.m_ci_perspective_focal_hor &&
         m_ci_perspective_focal_ver == other.m_ci_perspective_focal_ver &&
         m_ci_perspective_center_hor == other.m_ci_perspective_center_hor &&
         m_ci_perspective_center_ver == other.m_ci_perspective_center_ver &&
         m_ci_ortho_width == other.m_ci_ortho_width && m_ci_ortho_height == other.m_ci_ortho_height;
}

constexpr auto CameraIntrinsics::operator!=(const CameraIntrinsics &other) const noexcept {
  return !operator==(other);
}

constexpr auto CameraExtrinsics::ce_view_pos_x() const noexcept { return m_ce_view_pos_x; }

constexpr auto CameraExtrinsics::ce_view_pos_y() const noexcept { return m_ce_view_pos_y; }

constexpr auto CameraExtrinsics::ce_view_pos_z() const noexcept { return m_ce_view_pos_z; }

constexpr auto CameraExtrinsics::ce_view_quat_x() const noexcept { return m_ce_view_quat_x; }

constexpr auto CameraExtrinsics::ce_view_quat_y() const noexcept { return m_ce_view_quat_y; }

constexpr auto CameraExtrinsics::ce_view_quat_z() const noexcept { return m_ce_view_quat_z; }

constexpr auto CameraExtrinsics::ce_view_pos_x(const float value) noexcept -> auto & {
  m_ce_view_pos_x = value;
  return *this;
}

constexpr auto CameraExtrinsics::ce_view_pos_y(const float value) noexcept -> auto & {
  m_ce_view_pos_y = value;
  return *this;
}

constexpr auto CameraExtrinsics::ce_view_pos_z(const float value) noexcept -> auto & {
  m_ce_view_pos_z = value;
  return *this;
}

constexpr auto CameraExtrinsics::ce_view_quat_x(const float value) noexcept -> auto & {
  m_ce_view_quat_x = value;
  return *this;
}

constexpr auto CameraExtrinsics::ce_view_quat_y(const float value) noexcept -> auto & {
  m_ce_view_quat_y = value;
  return *this;
}

constexpr auto CameraExtrinsics::ce_view_quat_z(const float value) noexcept -> auto & {
  m_ce_view_quat_z = value;
  return *this;
}

constexpr auto CameraExtrinsics::operator==(const CameraExtrinsics &other) const noexcept {
  return ce_view_pos_x() == other.ce_view_pos_x() && ce_view_pos_y() == other.ce_view_pos_y() &&
         ce_view_pos_z() == other.ce_view_pos_z() && ce_view_quat_x() == other.ce_view_quat_x() &&
         ce_view_quat_y() == other.ce_view_quat_y() && ce_view_quat_z() == other.ce_view_quat_z();
}

constexpr auto CameraExtrinsics::operator!=(const CameraExtrinsics &other) const noexcept {
  return !operator==(other);
}

constexpr auto DepthQuantization::dq_quantization_law() const noexcept { return uint8_t{}; }

constexpr auto DepthQuantization::dq_norm_disp_low() const noexcept { return m_dq_norm_disp_low; }

constexpr auto DepthQuantization::dq_norm_disp_high() const noexcept { return m_dq_norm_disp_high; }

constexpr auto DepthQuantization::dq_depth_occ_map_threshold_default() const noexcept {
  return m_dq_depth_occ_map_threshold_default;
}

constexpr auto DepthQuantization::dq_norm_disp_low(const float value) noexcept -> auto & {
  m_dq_norm_disp_low = value;
  return *this;
}

constexpr auto DepthQuantization::dq_norm_disp_high(const float value) noexcept -> auto & {
  m_dq_norm_disp_high = value;
  return *this;
}

constexpr auto
DepthQuantization::dq_depth_occ_map_threshold_default(const std::uint32_t value) noexcept
    -> auto & {
  m_dq_depth_occ_map_threshold_default = value;
  return *this;
}

constexpr auto DepthQuantization::operator==(const DepthQuantization &other) const noexcept {
  return dq_norm_disp_low() == other.dq_norm_disp_low() &&
         dq_norm_disp_high() == other.dq_norm_disp_high() &&
         dq_depth_occ_map_threshold_default() == other.dq_depth_occ_map_threshold_default();
}

constexpr auto DepthQuantization::operator!=(const DepthQuantization &other) const noexcept {
  return !operator==(other);
}

constexpr auto MivViewParamsList::mvp_explicit_view_id_flag() const noexcept {
  return m_mvp_explicit_view_id_flag;
}

constexpr auto MivViewParamsList::mvp_intrinsic_params_equal_flag() const noexcept {
  return m_mvp_intrinsic_params_equal_flag;
}

constexpr auto MivViewParamsList::mvp_pruning_graph_params_present_flag() const noexcept {
  return m_mvp_pruning_graph_params_present_flag;
}

constexpr auto MivViewParamsList::ViewInAtlas::operator==(const ViewInAtlas &other) const noexcept {
  return enabled == other.enabled && complete == other.complete;
}

constexpr auto MivViewParamsList::ViewInAtlas::operator!=(const ViewInAtlas &other) const noexcept {
  return !operator==(other);
}
} // namespace TMIV::MivBitstream
