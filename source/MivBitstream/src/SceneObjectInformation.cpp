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
#include <TMIV/MivBitstream/SceneObjectInformation.h>

namespace TMIV::MivBitstream {
auto SceneObjectInformation::soi_persistence_flag() const noexcept -> bool {
  return m_soi_persistence_flag;
}
auto SceneObjectInformation::soi_reset_flag() const noexcept -> bool { return m_soi_reset_flag; }
auto SceneObjectInformation::soi_num_object_updates() const noexcept -> std::size_t {
  return m_temporary_soi_num_object_updates.value_or(m_object_updates.size());
}
auto SceneObjectInformation::soi_simple_objects_flag() const -> bool {
  VERIFY_V3CBITSTREAM(soi_num_object_updates() > 0 && m_soi_simple_objects_flag);
  return *m_soi_simple_objects_flag;
}
auto SceneObjectInformation::soi_object_label_present_flag() const -> bool {
  VERIFY_V3CBITSTREAM(soi_num_object_updates() > 0 && m_soi_simple_objects_flag);
  if (*m_soi_simple_objects_flag) {
    return false;
  }
  VERIFY_V3CBITSTREAM(m_soi_object_label_present_flag);
  return *m_soi_object_label_present_flag;
}
auto SceneObjectInformation::soi_priority_present_flag() const noexcept -> bool {
  VERIFY_V3CBITSTREAM(soi_num_object_updates() > 0 && m_soi_simple_objects_flag);
  if (*m_soi_simple_objects_flag) {
    return false;
  }
  VERIFY_V3CBITSTREAM(m_soi_priority_present_flag);
  return *m_soi_priority_present_flag;
}
auto SceneObjectInformation::soi_object_hidden_present_flag() const noexcept -> bool {
  VERIFY_V3CBITSTREAM(soi_num_object_updates() > 0 && m_soi_simple_objects_flag);
  if (*m_soi_simple_objects_flag) {
    return false;
  }
  VERIFY_V3CBITSTREAM(m_soi_object_hidden_present_flag);
  return *m_soi_object_hidden_present_flag;
}
auto SceneObjectInformation::soi_object_dependency_present_flag() const noexcept -> bool {
  VERIFY_V3CBITSTREAM(soi_num_object_updates() > 0 && m_soi_simple_objects_flag);
  if (*m_soi_simple_objects_flag) {
    return false;
  }
  VERIFY_V3CBITSTREAM(m_soi_object_dependency_present_flag);
  return *m_soi_object_dependency_present_flag;
}
auto SceneObjectInformation::soi_visibility_cones_present_flag() const noexcept -> bool {
  VERIFY_V3CBITSTREAM(soi_num_object_updates() > 0 && m_soi_simple_objects_flag);
  if (*m_soi_simple_objects_flag) {
    return false;
  }
  VERIFY_V3CBITSTREAM(m_soi_visibility_cones_present_flag);
  return *m_soi_visibility_cones_present_flag;
}
auto SceneObjectInformation::soi_3d_bounding_box_present_flag() const noexcept -> bool {
  VERIFY_V3CBITSTREAM(soi_num_object_updates() > 0 && m_soi_simple_objects_flag);
  if (*m_soi_simple_objects_flag) {
    return false;
  }
  VERIFY_V3CBITSTREAM(m_soi_3d_bounding_box_present_flag);
  return *m_soi_3d_bounding_box_present_flag;
}
auto SceneObjectInformation::soi_collision_shape_present_flag() const noexcept -> bool {
  VERIFY_V3CBITSTREAM(soi_num_object_updates() > 0 && m_soi_simple_objects_flag);
  if (*m_soi_simple_objects_flag) {
    return false;
  }
  VERIFY_V3CBITSTREAM(m_soi_collision_shape_present_flag);
  return *m_soi_collision_shape_present_flag;
}
auto SceneObjectInformation::soi_point_style_present_flag() const noexcept -> bool {
  VERIFY_V3CBITSTREAM(soi_num_object_updates() > 0 && m_soi_simple_objects_flag);
  if (*m_soi_simple_objects_flag) {
    return false;
  }
  VERIFY_V3CBITSTREAM(m_soi_point_style_present_flag);
  return *m_soi_point_style_present_flag;
}
auto SceneObjectInformation::soi_material_id_present_flag() const noexcept -> bool {
  VERIFY_V3CBITSTREAM(soi_num_object_updates() > 0 && m_soi_simple_objects_flag);
  if (*m_soi_simple_objects_flag) {
    return false;
  }
  VERIFY_V3CBITSTREAM(m_soi_material_id_present_flag);
  return *m_soi_material_id_present_flag;
}
auto SceneObjectInformation::soi_extension_present_flag() const noexcept -> bool {
  VERIFY_V3CBITSTREAM(soi_num_object_updates() > 0 && m_soi_simple_objects_flag);
  if (*m_soi_simple_objects_flag) {
    return false;
  }
  VERIFY_V3CBITSTREAM(m_soi_extension_present_flag);
  return *m_soi_extension_present_flag;
}
auto SceneObjectInformation::soi_3d_bounding_box_scale_log2() const noexcept -> std::uint8_t {
  VERIFY_V3CBITSTREAM(soi_num_object_updates() > 0 && soi_3d_bounding_box_present_flag() &&
                      m_soi_3d_bounding_box_scale_log2);
  return *m_soi_3d_bounding_box_scale_log2;
}
auto SceneObjectInformation::soi_log2_max_object_idx_updated_minus1() const noexcept
    -> std::uint8_t {
  VERIFY_V3CBITSTREAM(soi_num_object_updates() > 0);
  return m_soi_log2_max_object_idx_updated_minus1;
}
auto SceneObjectInformation::soi_log2_max_object_dependency_idx() const noexcept -> std::uint8_t {
  VERIFY_V3CBITSTREAM(soi_num_object_updates() > 0 && soi_object_dependency_present_flag() &&
                      m_soi_log2_max_object_dependency_idx);
  return *m_soi_log2_max_object_dependency_idx;
}
auto SceneObjectInformation::soi_object_idx(std::size_t i) const noexcept -> std::size_t {
  VERIFY_V3CBITSTREAM(soi_num_object_updates() > 0 && i < soi_num_object_updates());
  return m_object_updates[i].soi_object_idx;
}
auto SceneObjectInformation::soi_object_cancel_flag(std::size_t k) const noexcept -> bool {
  VERIFY_V3CBITSTREAM(soi_num_object_updates() > 0 && k < soi_num_object_updates());
  return m_object_updates[k].soi_object_cancel_flag;
}
auto SceneObjectInformation::soi_object_label_update_flag(std::size_t k) const noexcept -> bool {
  VERIFY_V3CBITSTREAM(isUpdateValid(k) && soi_object_label_present_flag() &&
                      m_object_updates[k].soi_object_label_update_flag);
  return *m_object_updates[k].soi_object_label_update_flag;
}
auto SceneObjectInformation::soi_object_label_idx(std::size_t k) const noexcept -> std::size_t {
  VERIFY_V3CBITSTREAM(isUpdateValid(k) && soi_object_label_update_flag(k) &&
                      m_object_updates[k].soi_object_label_idx);
  return *m_object_updates[k].soi_object_label_idx;
}
auto SceneObjectInformation::soi_priority_update_flag(std::size_t k) const noexcept -> bool {
  VERIFY_V3CBITSTREAM(isUpdateValid(k) && soi_priority_present_flag() &&
                      m_object_updates[k].soi_priority_update_flag);
  return *m_object_updates[k].soi_priority_update_flag;
}
auto SceneObjectInformation::soi_priority_value(std::size_t k) const noexcept -> std::uint8_t {
  VERIFY_V3CBITSTREAM(isUpdateValid(k) && soi_priority_present_flag() &&
                      soi_priority_update_flag(k) && m_object_updates[k].soi_priority_value);
  return *m_object_updates[k].soi_priority_value;
}
auto SceneObjectInformation::soi_object_hidden_flag(std::size_t k) const noexcept -> bool {
  VERIFY_V3CBITSTREAM(isUpdateValid(k) && soi_object_hidden_present_flag() &&
                      m_object_updates[k].soi_object_hidden_flag);
  return *m_object_updates[k].soi_object_hidden_flag;
}
auto SceneObjectInformation::soi_object_dependency_update_flag(std::size_t k) const noexcept
    -> bool {
  VERIFY_V3CBITSTREAM(isUpdateValid(k) && soi_object_dependency_present_flag() &&
                      m_object_updates[k].soi_object_dependency_update_flag);
  return *m_object_updates[k].soi_object_dependency_update_flag;
}
auto SceneObjectInformation::soi_object_num_dependencies(std::size_t k) const -> std::uint8_t {
  VERIFY_V3CBITSTREAM(isUpdateValid(k) && soi_object_dependency_present_flag());
  return static_cast<uint8_t>(m_object_updates[k].soi_object_dependency_idx.size());
}
auto SceneObjectInformation::soi_object_dependency_idx(std::size_t k, std::size_t j) const
    -> std::size_t {
  VERIFY_V3CBITSTREAM(isUpdateValid(k) && j < soi_object_num_dependencies(k) &&
                      soi_object_dependency_present_flag());
  return m_object_updates[k].soi_object_dependency_idx[j];
}
auto SceneObjectInformation::soi_visibility_cones_update_flag(std::size_t k) const noexcept
    -> bool {
  VERIFY_V3CBITSTREAM(isUpdateValid(k) && soi_visibility_cones_present_flag() &&
                      m_object_updates[k].soi_visibility_cones_update_flag);
  return *m_object_updates[k].soi_visibility_cones_update_flag;
}
auto SceneObjectInformation::soi_direction_x(std::size_t k) const -> std::int16_t {
  VERIFY_V3CBITSTREAM(isUpdateValid(k) && soi_visibility_cones_update_flag(k) &&
                      m_object_updates[k].soi_visibility_cones);
  return m_object_updates[k].soi_visibility_cones->soi_direction_x;
}
auto SceneObjectInformation::soi_direction_y(std::size_t k) const -> std::int16_t {
  VERIFY_V3CBITSTREAM(isUpdateValid(k) && soi_visibility_cones_update_flag(k) &&
                      m_object_updates[k].soi_visibility_cones);
  return m_object_updates[k].soi_visibility_cones->soi_direction_y;
}
auto SceneObjectInformation::soi_direction_z(std::size_t k) const -> std::int16_t {
  VERIFY_V3CBITSTREAM(isUpdateValid(k) && soi_visibility_cones_update_flag(k) &&
                      m_object_updates[k].soi_visibility_cones);
  return m_object_updates[k].soi_visibility_cones->soi_direction_z;
}
auto SceneObjectInformation::soi_angle(std::size_t k) const -> std::uint16_t {
  VERIFY_V3CBITSTREAM(isUpdateValid(k) && soi_visibility_cones_update_flag(k) &&
                      m_object_updates[k].soi_visibility_cones);
  return m_object_updates[k].soi_visibility_cones->soi_angle;
}
auto SceneObjectInformation::soi_3d_bounding_box_update_flag(std::size_t k) const noexcept -> bool {
  VERIFY_V3CBITSTREAM(isUpdateValid(k) && soi_3d_bounding_box_present_flag() &&
                      m_object_updates[k].soi_3d_bounding_box_update_flag);
  return *m_object_updates[k].soi_3d_bounding_box_update_flag;
}
auto SceneObjectInformation::soi_3d_bounding_box_x(std::size_t k) const -> std::size_t {
  VERIFY_V3CBITSTREAM(isBoundingBoxValid(k));
  return m_object_updates[k].soi_3d_bounding_box->soi_3d_bounding_box_x;
}
auto SceneObjectInformation::soi_3d_bounding_box_y(std::size_t k) const -> std::size_t {
  VERIFY_V3CBITSTREAM(isBoundingBoxValid(k));
  return m_object_updates[k].soi_3d_bounding_box->soi_3d_bounding_box_y;
}
auto SceneObjectInformation::soi_3d_bounding_box_z(std::size_t k) const -> std::size_t {
  VERIFY_V3CBITSTREAM(isBoundingBoxValid(k));
  return m_object_updates[k].soi_3d_bounding_box->soi_3d_bounding_box_z;
}
auto SceneObjectInformation::soi_3d_bounding_box_size_x(std::size_t k) const -> std::size_t {
  VERIFY_V3CBITSTREAM(isBoundingBoxValid(k));
  return m_object_updates[k].soi_3d_bounding_box->soi_3d_bounding_box_size_x;
}
auto SceneObjectInformation::soi_3d_bounding_box_size_y(std::size_t k) const -> std::size_t {
  VERIFY_V3CBITSTREAM(isBoundingBoxValid(k));
  return m_object_updates[k].soi_3d_bounding_box->soi_3d_bounding_box_size_y;
}
auto SceneObjectInformation::soi_3d_bounding_box_size_z(std::size_t k) const -> std::size_t {
  VERIFY_V3CBITSTREAM(isBoundingBoxValid(k));
  return m_object_updates[k].soi_3d_bounding_box->soi_3d_bounding_box_size_z;
}
auto SceneObjectInformation::soi_collision_shape_update_flag(std::size_t k) const noexcept -> bool {
  VERIFY_V3CBITSTREAM(isUpdateValid(k) && soi_collision_shape_present_flag() &&
                      m_object_updates[k].soi_collision_shape_update_flag);
  return *m_object_updates[k].soi_collision_shape_update_flag;
}
auto SceneObjectInformation::soi_collision_shape_id(std::size_t k) const noexcept -> std::uint16_t {
  VERIFY_V3CBITSTREAM(isUpdateValid(k) && soi_collision_shape_update_flag(k) &&
                      m_object_updates[k].soi_collision_shape_id);
  return *m_object_updates[k].soi_collision_shape_id;
}
auto SceneObjectInformation::soi_point_style_update_flag(std::size_t k) const noexcept -> bool {
  VERIFY_V3CBITSTREAM(isUpdateValid(k) && soi_point_style_present_flag() &&
                      m_object_updates[k].soi_point_style_update_flag);
  return *m_object_updates[k].soi_point_style_update_flag;
}
auto SceneObjectInformation::soi_point_shape_id(std::size_t k) const noexcept -> std::uint8_t {
  VERIFY_V3CBITSTREAM(isUpdateValid(k) && soi_point_style_update_flag(k) &&
                      m_object_updates[k].soi_point_shape_id);
  return *m_object_updates[k].soi_point_shape_id;
}
auto SceneObjectInformation::soi_point_size(std::size_t k) const noexcept -> std::uint16_t {
  VERIFY_V3CBITSTREAM(isUpdateValid(k) && soi_point_style_update_flag(k) &&
                      m_object_updates[k].soi_point_size);
  return *m_object_updates[k].soi_point_size;
}
auto SceneObjectInformation::soi_material_id_update_flag(std::size_t k) const noexcept -> bool {
  VERIFY_V3CBITSTREAM(isUpdateValid(k) && soi_material_id_present_flag() &&
                      m_object_updates[k].soi_material_id_update_flag);
  return *m_object_updates[k].soi_material_id_update_flag;
}
auto SceneObjectInformation::soi_material_id(std::size_t k) const noexcept -> std::uint16_t {
  VERIFY_V3CBITSTREAM(isUpdateValid(k) && soi_material_id_update_flag(k) &&
                      m_object_updates[k].soi_material_id);
  return *m_object_updates[k].soi_material_id;
}

[[nodiscard]] auto SceneObjectInformation::isUpdateValid(std::size_t k) const noexcept -> bool {
  return soi_num_object_updates() > 0 && k < soi_num_object_updates() && !soi_object_cancel_flag(k);
}
[[nodiscard]] auto SceneObjectInformation::isBoundingBoxValid(std::size_t k) const -> bool {
  return isUpdateValid(k) && soi_3d_bounding_box_present_flag() &&
         soi_3d_bounding_box_update_flag(k) && m_object_updates[k].soi_3d_bounding_box;
}

namespace {
auto putFlag(std::ostream &stream, std::string &&fieldName, bool flagValue) -> std::ostream & {
  return stream << fieldName << "=" << std::boolalpha << flagValue << "\n";
}
auto putIndexedFlag(std::ostream &stream, std::string &&fieldName, std::size_t index,
                    bool flagValue) -> std::ostream & {
  return stream << fieldName << "(" << index << ")=" << std::boolalpha << flagValue << "\n";
}
auto putUnsigned(std::ostream &stream, std::string &&fieldName, std::size_t flagValue)
    -> std::ostream & {
  return stream << fieldName << "=" << flagValue << "\n";
}
auto putIndexedUnsigned(std::ostream &stream, std::string &&fieldName, std::size_t index,
                        std::size_t flagValue) -> std::ostream & {
  return stream << fieldName << "(" << index << ")=" << flagValue << "\n";
}
} // namespace

auto operator<<(std::ostream &stream, const SceneObjectInformation &x) -> std::ostream & {
  putFlag(stream, "soi_persistence_flag", x.soi_persistence_flag());
  putFlag(stream, "soi_reset_flag", x.soi_reset_flag());
  putUnsigned(stream, "soi_num_object_updates", x.soi_num_object_updates());
  if (x.soi_num_object_updates() > 0) {
    putFlag(stream, "soi_simple_objects_flag", x.soi_simple_objects_flag());
    putFlag(stream, "soi_object_label_present_flag", x.soi_object_label_present_flag());
    putFlag(stream, "soi_priority_present_flag", x.soi_priority_present_flag());
    putFlag(stream, "soi_object_hidden_present_flag", x.soi_object_hidden_present_flag());
    putFlag(stream, "soi_object_dependency_present_flag", x.soi_object_dependency_present_flag());
    putFlag(stream, "soi_visibility_cones_present_flag", x.soi_visibility_cones_present_flag());
    putFlag(stream, "soi_3d_bounding_box_present_flag", x.soi_3d_bounding_box_present_flag());
    putFlag(stream, "soi_collision_shape_present_flag", x.soi_collision_shape_present_flag());
    putFlag(stream, "soi_point_style_present_flag", x.soi_point_style_present_flag());
    putFlag(stream, "soi_material_id_present_flag", x.soi_material_id_present_flag());
    putFlag(stream, "soi_extension_present_flag", x.soi_extension_present_flag());
    if (x.soi_3d_bounding_box_present_flag()) {
      putUnsigned(stream, "soi_3d_bounding_box_scale_log2", x.soi_3d_bounding_box_scale_log2());
    }
    putUnsigned(stream, "soi_log2_max_object_idx_updated_minus1",
                x.soi_log2_max_object_idx_updated_minus1());
    if (x.soi_object_dependency_present_flag()) {
      putUnsigned(stream, "soi_log2_max_object_dependency_idx",
                  x.soi_log2_max_object_dependency_idx());
    }
    for (std::size_t i = 0; i < x.soi_num_object_updates(); ++i) {
      const auto k = static_cast<unsigned>(x.soi_object_idx(i));
      putUnsigned(stream, "soi_object_idx", k);
      putIndexedFlag(stream, "soi_object_cancel_flag", k, x.soi_object_cancel_flag(k));
      if (!x.soi_object_cancel_flag(k)) {
        if (x.soi_object_label_present_flag()) {
          putIndexedFlag(stream, "soi_object_label_update_flag", k,
                         x.soi_object_label_update_flag(k));
          if (x.soi_object_label_update_flag(k)) {
            putIndexedUnsigned(stream, "soi_object_label_idx", k, x.soi_object_label_idx(k));
          }
        }
        if (x.soi_priority_present_flag()) {
          putIndexedFlag(stream, "soi_priority_update_flag", k, x.soi_priority_update_flag(k));
          if (x.soi_priority_update_flag(k)) {
            putIndexedUnsigned(stream, "soi_priority_value", k, x.soi_priority_value(k));
          }
        }
        if (x.soi_object_hidden_present_flag()) {
          putIndexedFlag(stream, "soi_object_hidden_flag", k, x.soi_object_hidden_flag(k));
        }
        if (x.soi_object_dependency_present_flag()) {
          putIndexedFlag(stream, "soi_object_dependency_update_flag", k,
                         x.soi_object_dependency_update_flag(k));
          if (x.soi_object_dependency_update_flag(k)) {
            putIndexedUnsigned(stream, "soi_object_num_dependencies", k,
                               x.soi_object_num_dependencies(k));
            for (std::size_t j = 0; j < x.soi_object_num_dependencies(k); ++j) {
              stream << "soi_object_dependency_idx(" << k
                     << ")=" << static_cast<unsigned>(x.soi_object_dependency_idx(k, j)) << "\n";
            }
          }
        }
        if (x.soi_visibility_cones_present_flag()) {
          putIndexedFlag(stream, "soi_visibility_cones_update_flag", k,
                         x.soi_visibility_cones_update_flag(k));
          if (x.soi_visibility_cones_update_flag(k)) {
            stream << "soi_direction_x(" << k << ")=" << x.soi_direction_x(k) << "\n";
            stream << "soi_direction_y(" << k << ")=" << x.soi_direction_y(k) << "\n";
            stream << "soi_direction_z(" << k << ")=" << x.soi_direction_z(k) << "\n";
            putIndexedUnsigned(stream, "soi_angle", k, x.soi_angle(k));
          }
        }
        if (x.soi_3d_bounding_box_present_flag()) {
          putIndexedFlag(stream, "soi_3d_bounding_box_update_flag", k,
                         x.soi_3d_bounding_box_update_flag(k));
          if (x.soi_3d_bounding_box_update_flag(k)) {
            putIndexedUnsigned(stream, "soi_3d_bounding_box_x", k, x.soi_3d_bounding_box_x(k));
            putIndexedUnsigned(stream, "soi_3d_bounding_box_y", k, x.soi_3d_bounding_box_y(k));
            putIndexedUnsigned(stream, "soi_3d_bounding_box_z", k, x.soi_3d_bounding_box_z(k));
            putIndexedUnsigned(stream, "soi_3d_bounding_box_size_x", k,
                               x.soi_3d_bounding_box_size_x(k));
            putIndexedUnsigned(stream, "soi_3d_bounding_box_size_y", k,
                               x.soi_3d_bounding_box_size_y(k));
            putIndexedUnsigned(stream, "soi_3d_bounding_box_size_z", k,
                               x.soi_3d_bounding_box_size_z(k));
          }
        }
        if (x.soi_collision_shape_present_flag()) {
          putIndexedFlag(stream, "soi_collision_shape_update_flag", k,
                         x.soi_collision_shape_update_flag(k));
          if (x.soi_collision_shape_update_flag(k)) {
            putIndexedUnsigned(stream, "soi_collision_shape_id", k, x.soi_collision_shape_id(k));
          }
        }
        if (x.soi_point_style_present_flag()) {
          putIndexedFlag(stream, "soi_point_style_update_flag", k,
                         x.soi_point_style_update_flag(k));
          if (x.soi_point_style_update_flag(k)) {
            putIndexedUnsigned(stream, "soi_point_shape_id", k, x.soi_point_shape_id(k));
            putIndexedUnsigned(stream, "soi_point_size", k, x.soi_point_size(k));
          }
        }
        if (x.soi_material_id_present_flag()) {
          putIndexedFlag(stream, "soi_material_id_update_flag", k,
                         x.soi_material_id_update_flag(k));
          if (x.soi_material_id_update_flag(k)) {
            putIndexedUnsigned(stream, "soi_material_id", k, x.soi_material_id(k));
          }
        }
      }
    }
  }
  return stream;
}

auto SceneObjectInformation::operator==(const SceneObjectInformation &other) const noexcept
    -> bool {
  return (m_soi_persistence_flag == other.m_soi_persistence_flag) &&
         (m_soi_reset_flag == other.m_soi_reset_flag) &&
         (m_temporary_soi_num_object_updates == other.m_temporary_soi_num_object_updates) &&
         (m_soi_simple_objects_flag == other.m_soi_simple_objects_flag) &&
         (m_soi_object_label_present_flag == other.m_soi_object_label_present_flag) &&
         (m_soi_priority_present_flag == other.m_soi_priority_present_flag) &&
         (m_soi_object_hidden_present_flag == other.m_soi_object_hidden_present_flag) &&
         (m_soi_object_dependency_present_flag == other.m_soi_object_dependency_present_flag) &&
         (m_soi_visibility_cones_present_flag == other.m_soi_visibility_cones_present_flag) &&
         (m_soi_3d_bounding_box_present_flag == other.m_soi_3d_bounding_box_present_flag) &&
         (m_soi_collision_shape_present_flag == other.m_soi_collision_shape_present_flag) &&
         (m_soi_point_style_present_flag == other.m_soi_point_style_present_flag) &&
         (m_soi_material_id_present_flag == other.m_soi_material_id_present_flag) &&
         (m_soi_extension_present_flag == other.m_soi_extension_present_flag) &&
         (m_soi_3d_bounding_box_scale_log2 == other.m_soi_3d_bounding_box_scale_log2) &&
         (m_soi_log2_max_object_idx_updated_minus1 ==
          other.m_soi_log2_max_object_idx_updated_minus1) &&
         (m_soi_log2_max_object_dependency_idx == other.m_soi_log2_max_object_dependency_idx) &&
         (m_object_updates == other.m_object_updates);
}

auto SceneObjectInformation::decodeFrom(Common::InputBitstream &bitstream)
    -> SceneObjectInformation {
  SceneObjectInformation result{};
  result.soi_persistence_flag(bitstream.getFlag());
  result.soi_reset_flag(bitstream.getFlag());

  result.soi_num_object_updates(bitstream.getUExpGolomb<std::size_t>());
  if (result.soi_num_object_updates() > 0) {
    result.soi_simple_objects_flag(bitstream.getFlag());
    if (!result.soi_simple_objects_flag()) {
      result.soi_object_label_present_flag(bitstream.getFlag());
      result.soi_priority_present_flag(bitstream.getFlag());
      result.soi_object_hidden_present_flag(bitstream.getFlag());
      result.soi_object_dependency_present_flag(bitstream.getFlag());
      result.soi_visibility_cones_present_flag(bitstream.getFlag());
      result.soi_3d_bounding_box_present_flag(bitstream.getFlag());
      result.soi_collision_shape_present_flag(bitstream.getFlag());
      result.soi_point_style_present_flag(bitstream.getFlag());
      result.soi_material_id_present_flag(bitstream.getFlag());
      result.soi_extension_present_flag(bitstream.getFlag());
    } else {
      result.soi_object_label_present_flag(false);
      result.soi_priority_present_flag(false);
      result.soi_object_hidden_present_flag(false);
      result.soi_object_dependency_present_flag(false);
      result.soi_visibility_cones_present_flag(false);
      result.soi_3d_bounding_box_present_flag(false);
      result.soi_collision_shape_present_flag(false);
      result.soi_point_style_present_flag(false);
      result.soi_material_id_present_flag(false);
      result.soi_extension_present_flag(false);
    }
    if (result.soi_3d_bounding_box_present_flag()) {
      result.soi_3d_bounding_box_scale_log2(bitstream.readBits<std::uint8_t>(5));
    }
    result.soi_log2_max_object_idx_updated_minus1(bitstream.readBits<std::uint8_t>(5));
    if (result.soi_object_dependency_present_flag()) {
      result.soi_log2_max_object_dependency_idx(bitstream.readBits<std::uint8_t>(5));
    }
  }
  std::vector<SceneObjectUpdate> updates(result.soi_num_object_updates());
  for (std::size_t i = 0; i < result.soi_num_object_updates(); ++i) {
    updates[i].soi_object_idx =
        bitstream.readBits<std::size_t>(result.soi_log2_max_object_idx_updated_minus1() + 1);
    const auto k = updates[i].soi_object_idx;
    auto &currentObjectUpdate = updates[k];
    currentObjectUpdate.soi_object_cancel_flag = bitstream.getFlag();
    if (!currentObjectUpdate.soi_object_cancel_flag) {
      if (result.soi_object_label_present_flag()) {
        currentObjectUpdate.soi_object_label_update_flag = bitstream.getFlag();
        if (currentObjectUpdate.soi_object_label_update_flag.value()) {
          currentObjectUpdate.soi_object_label_idx = bitstream.getUExpGolomb<std::size_t>();
        }
      }
      if (result.soi_priority_present_flag()) {
        currentObjectUpdate.soi_priority_update_flag = bitstream.getFlag();
        if (currentObjectUpdate.soi_priority_update_flag.value()) {
          currentObjectUpdate.soi_priority_value = bitstream.readBits<std::uint8_t>(4);
        }
      }
      if (result.soi_object_hidden_present_flag()) {
        currentObjectUpdate.soi_object_hidden_flag = bitstream.getFlag();
      }
      if (result.soi_object_dependency_present_flag()) {
        currentObjectUpdate.soi_object_dependency_update_flag = bitstream.getFlag();
        if (currentObjectUpdate.soi_object_dependency_update_flag.value()) {
          currentObjectUpdate.soi_object_dependency_idx =
              std::vector<std::size_t>(bitstream.readBits<std::size_t>(4));
          for (auto &soi_object_dependency_index : currentObjectUpdate.soi_object_dependency_idx) {
            soi_object_dependency_index =
                bitstream.readBits<std::size_t>(result.soi_log2_max_object_dependency_idx());
          }
        }
      }
      if (result.soi_visibility_cones_present_flag()) {
        currentObjectUpdate.soi_visibility_cones_update_flag = bitstream.getFlag();
        if (currentObjectUpdate.soi_visibility_cones_update_flag.value()) {
          currentObjectUpdate.soi_visibility_cones = SoiVisibilityCones{};
          auto &cones = currentObjectUpdate.soi_visibility_cones.value();
          cones.soi_direction_x = bitstream.readBits<std::int16_t>(16);
          cones.soi_direction_y = bitstream.readBits<std::int16_t>(16);
          cones.soi_direction_z = bitstream.readBits<std::int16_t>(16);
          cones.soi_angle = bitstream.getUint16();
        }
      }
      if (result.soi_3d_bounding_box_present_flag()) {
        currentObjectUpdate.soi_3d_bounding_box_update_flag = bitstream.getFlag();
        if (currentObjectUpdate.soi_3d_bounding_box_update_flag.value()) {
          currentObjectUpdate.soi_3d_bounding_box = Soi3dBoundingBox{};
          auto &box = currentObjectUpdate.soi_3d_bounding_box.value();
          box.soi_3d_bounding_box_x = bitstream.getUExpGolomb<std::size_t>();
          box.soi_3d_bounding_box_y = bitstream.getUExpGolomb<std::size_t>();
          box.soi_3d_bounding_box_z = bitstream.getUExpGolomb<std::size_t>();
          box.soi_3d_bounding_box_size_x = bitstream.getUExpGolomb<std::size_t>();
          box.soi_3d_bounding_box_size_y = bitstream.getUExpGolomb<std::size_t>();
          box.soi_3d_bounding_box_size_z = bitstream.getUExpGolomb<std::size_t>();
        }
      }
      if (result.soi_collision_shape_present_flag()) {
        currentObjectUpdate.soi_collision_shape_update_flag = bitstream.getFlag();
        if (currentObjectUpdate.soi_collision_shape_update_flag.value()) {
          currentObjectUpdate.soi_collision_shape_id = bitstream.readBits<std::uint16_t>(16);
        }
      }
      if (result.soi_point_style_present_flag()) {
        currentObjectUpdate.soi_point_style_update_flag = bitstream.getFlag();
        if (currentObjectUpdate.soi_point_style_update_flag.value()) {
          currentObjectUpdate.soi_point_shape_id = bitstream.readBits<std::uint8_t>(8);
          currentObjectUpdate.soi_point_size = bitstream.readBits<std::uint16_t>(16);
        }
      }
      if (result.soi_material_id_present_flag()) {
        currentObjectUpdate.soi_material_id_update_flag = bitstream.getFlag();
        if (currentObjectUpdate.soi_material_id_update_flag.value()) {
          currentObjectUpdate.soi_material_id = bitstream.getUint16();
        }
      }
    }
  }
  result.setSceneObjectUpdates(std::move(updates));
  return result;
}

void SceneObjectInformation::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putFlag(soi_persistence_flag());
  bitstream.putFlag(soi_reset_flag());
  bitstream.putUExpGolomb(soi_num_object_updates());
  if (soi_num_object_updates() > 0) {
    bitstream.putFlag(soi_simple_objects_flag());
    if (!soi_simple_objects_flag()) {
      bitstream.putFlag(soi_object_label_present_flag());
      bitstream.putFlag(soi_priority_present_flag());
      bitstream.putFlag(soi_object_hidden_present_flag());
      bitstream.putFlag(soi_object_dependency_present_flag());
      bitstream.putFlag(soi_visibility_cones_present_flag());
      bitstream.putFlag(soi_3d_bounding_box_present_flag());
      bitstream.putFlag(soi_collision_shape_present_flag());
      bitstream.putFlag(soi_point_style_present_flag());
      bitstream.putFlag(soi_material_id_present_flag());
      bitstream.putFlag(soi_extension_present_flag());
    }
    if (soi_3d_bounding_box_present_flag()) {
      bitstream.writeBits(soi_3d_bounding_box_scale_log2(), 5);
    }
    bitstream.writeBits(soi_log2_max_object_idx_updated_minus1(), 5);
    if (soi_object_dependency_present_flag()) {
      bitstream.writeBits(soi_log2_max_object_dependency_idx(), 5);
    }
    for (std::size_t i = 0; i < soi_num_object_updates(); ++i) {
      bitstream.writeBits(soi_object_idx(i), soi_log2_max_object_idx_updated_minus1() + 1);
      const auto k = soi_object_idx(i);
      bitstream.putFlag(soi_object_cancel_flag(k));
      if (!soi_object_cancel_flag(k)) {
        if (soi_object_label_present_flag()) {
          bitstream.putFlag(soi_object_label_update_flag(k));
          if (soi_object_label_update_flag(k)) {
            bitstream.putUExpGolomb(soi_object_label_idx(k));
          }
        }
        if (soi_priority_present_flag()) {
          bitstream.putFlag(soi_priority_update_flag(k));
          if (soi_priority_update_flag(k)) {
            bitstream.writeBits(soi_priority_value(k), 4);
          }
        }
        if (soi_object_hidden_present_flag()) {
          bitstream.putFlag(soi_object_hidden_flag(k));
        }
        if (soi_object_dependency_present_flag()) {
          bitstream.putFlag(soi_object_dependency_update_flag(k));
          if (soi_object_dependency_update_flag(k)) {
            bitstream.writeBits(soi_object_num_dependencies(k), 4);
            for (std::uint8_t j = 0; j < soi_object_num_dependencies(k); ++j) {
              bitstream.writeBits(soi_object_dependency_idx(k, j),
                                  soi_log2_max_object_dependency_idx());
            }
          }
        }
        if (soi_visibility_cones_present_flag()) {
          bitstream.putFlag(soi_visibility_cones_update_flag(k));
          if (soi_visibility_cones_update_flag(k)) {
            bitstream.writeBits(soi_direction_x(k), 16);
            bitstream.writeBits(soi_direction_y(k), 16);
            bitstream.writeBits(soi_direction_z(k), 16);
            bitstream.writeBits(soi_angle(k), 16);
          }
        }
        if (soi_3d_bounding_box_present_flag()) {
          bitstream.putFlag(soi_3d_bounding_box_update_flag(k));
          if (soi_3d_bounding_box_update_flag(k)) {
            bitstream.putUExpGolomb(soi_3d_bounding_box_x(k));
            bitstream.putUExpGolomb(soi_3d_bounding_box_y(k));
            bitstream.putUExpGolomb(soi_3d_bounding_box_z(k));
            bitstream.putUExpGolomb(soi_3d_bounding_box_size_x(k));
            bitstream.putUExpGolomb(soi_3d_bounding_box_size_y(k));
            bitstream.putUExpGolomb(soi_3d_bounding_box_size_z(k));
          }
        }
        if (soi_collision_shape_present_flag()) {
          bitstream.putFlag(soi_collision_shape_update_flag(k));
          if (soi_collision_shape_update_flag(k)) {
            bitstream.writeBits(soi_collision_shape_id(k), 16);
          }
        }
        if (soi_point_style_present_flag()) {
          bitstream.putFlag(soi_point_style_update_flag(k));
          if (soi_point_style_update_flag(k)) {
            bitstream.writeBits(soi_point_shape_id(k), 8);
            bitstream.writeBits(soi_point_size(k), 16);
          }
        }
        if (soi_material_id_present_flag()) {
          bitstream.putFlag(soi_material_id_update_flag(k));
          if (soi_material_id_update_flag(k)) {
            bitstream.putUint16(soi_material_id(k));
          }
        }
      }
    }
  }
}
} // namespace TMIV::MivBitstream
