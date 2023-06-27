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

#ifndef _TMIV_MIVBITSTREAM_SCENEOBJECTINFORMATION_H_
#error "Include the .h, not the .hpp"
#endif

namespace TMIV::MivBitstream {
constexpr auto SceneObjectInformation::soi_persistence_flag(const bool value) noexcept -> auto & {
  m_soi_persistence_flag = value;
  return *this;
}
constexpr auto SceneObjectInformation::soi_reset_flag(const bool value) noexcept -> auto & {
  m_soi_reset_flag = value;
  return *this;
}
constexpr auto SceneObjectInformation::soi_num_object_updates(const size_t value) noexcept
    -> auto & {
  m_temporary_soi_num_object_updates = value;
  return *this;
}
constexpr auto SceneObjectInformation::soi_simple_objects_flag(const bool value) noexcept
    -> auto & {
  m_soi_simple_objects_flag = value;
  return *this;
}
constexpr auto SceneObjectInformation::soi_object_label_present_flag(bool value) noexcept
    -> auto & {
  m_soi_object_label_present_flag = value;
  return *this;
}
constexpr auto SceneObjectInformation::soi_priority_present_flag(bool value) noexcept -> auto & {
  m_soi_priority_present_flag = value;
  return *this;
}
constexpr auto SceneObjectInformation::soi_object_hidden_present_flag(bool value) noexcept
    -> auto & {
  m_soi_object_hidden_present_flag = value;
  return *this;
}
constexpr auto SceneObjectInformation::soi_object_dependency_present_flag(bool value) noexcept
    -> auto & {
  m_soi_object_dependency_present_flag = value;
  return *this;
}
constexpr auto SceneObjectInformation::soi_visibility_cones_present_flag(bool value) noexcept
    -> auto & {
  m_soi_visibility_cones_present_flag = value;
  return *this;
}
constexpr auto SceneObjectInformation::soi_3d_bounding_box_present_flag(bool value) noexcept
    -> auto & {
  m_soi_3d_bounding_box_present_flag = value;
  return *this;
}
constexpr auto SceneObjectInformation::soi_collision_shape_present_flag(bool value) noexcept
    -> auto & {
  m_soi_collision_shape_present_flag = value;
  return *this;
}
constexpr auto SceneObjectInformation::soi_point_style_present_flag(bool value) noexcept -> auto & {
  m_soi_point_style_present_flag = value;
  return *this;
}
constexpr auto SceneObjectInformation::soi_material_id_present_flag(bool value) noexcept -> auto & {
  m_soi_material_id_present_flag = value;
  return *this;
}
constexpr auto SceneObjectInformation::soi_extension_present_flag(bool value) noexcept -> auto & {
  m_soi_extension_present_flag = value;
  return *this;
}
constexpr auto
SceneObjectInformation::soi_3d_bounding_box_scale_log2(const std::uint8_t value) noexcept
    -> auto & {
  m_soi_3d_bounding_box_scale_log2 = value;
  return *this;
}
constexpr auto
SceneObjectInformation::soi_log2_max_object_idx_updated_minus1(const std::uint8_t value) noexcept
    -> auto & {
  m_soi_log2_max_object_idx_updated_minus1 = value;
  return *this;
}
constexpr auto
SceneObjectInformation::soi_log2_max_object_dependency_idx(const std::uint8_t value) noexcept
    -> auto & {
  m_soi_log2_max_object_dependency_idx = value;
  return *this;
}
inline auto
SceneObjectInformation::setSceneObjectUpdates(std::vector<SceneObjectUpdate> &&updates) noexcept
    -> void {
  m_object_updates = std::move(updates);
  m_temporary_soi_num_object_updates.reset();
}

} // namespace TMIV::MivBitstream
