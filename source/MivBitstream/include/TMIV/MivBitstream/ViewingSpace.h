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

#ifndef _TMIV_MIVBITSTREAM_VIEWINGSPACE_H_
#define _TMIV_MIVBITSTREAM_VIEWINGSPACE_H_

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/Json.h>
#include <TMIV/Common/Quaternion.h>
#include <TMIV/Common/Vector.h>

#include <TMIV/MivBitstream/ViewParamsList.h>

#include <iosfwd>
#include <optional>
#include <variant>
#include <vector>

namespace TMIV::MivBitstream {
using Common::InputBitstream;
using Common::OutputBitstream;

enum class PrimitiveShapeType {
  cuboid = 0,
  spheroid = 1,
  halfspace = 2
}; // 23090-12: primitive_shape_type[ e ]

enum class PrimitiveShapeOperation {
  add = 0,
  interpolate = 1
}; // 23090-12: primitive_shape_operation[ e ]

enum class ElementaryShapeOperation {
  add = 0,
  subtract = 1,
  intersect = 2
}; // 23090-12: vs_elementary_shape_operation[ e ]

// 23090-12: cuboid_primitive( e, s )
struct Cuboid {
  Cuboid() = default;
  Cuboid(Common::Vec3f _center, Common::Vec3f _size) : center{_center}, size{_size} {}

  [[nodiscard]] auto cp_center_x() const noexcept -> float {
    VERIFY_BITSTREAM(center);
    return center->x();
  }
  [[nodiscard]] auto cp_center_y() const noexcept -> float {
    VERIFY_BITSTREAM(center);
    return center->y();
  }
  [[nodiscard]] auto cp_center_z() const noexcept -> float {
    VERIFY_BITSTREAM(center);
    return center->z();
  }
  [[nodiscard]] auto cp_size_x() const noexcept -> float { return size.x(); }
  [[nodiscard]] auto cp_size_y() const noexcept -> float { return size.y(); }
  [[nodiscard]] auto cp_size_z() const noexcept -> float { return size.z(); }

  std::optional<Common::Vec3f> center{};
  Common::Vec3f size{};

  friend auto operator<<(std::ostream &stream, const Cuboid &cuboid) -> std::ostream &;
  auto operator==(const Cuboid &other) const -> bool;
  auto operator!=(const Cuboid &other) const -> bool { return !operator==(other); }

  static auto decodeFrom(Common::InputBitstream &stream, bool cameraInferred) -> Cuboid;
  void encodeTo(Common::OutputBitstream &stream, bool cameraInferred) const;

  static auto loadFromJson(const Common::Json &node, bool inferredView) -> Cuboid;
};

// 23090-12: sphere_primitive( e, s )
struct Spheroid {
  Spheroid() = default;
  explicit Spheroid(Common::Vec3f _radius) : radius{_radius} {}
  Spheroid(Common::Vec3f _center, Common::Vec3f _radius) : center{_center}, radius{_radius} {}

  [[nodiscard]] auto sp_center_x() const noexcept -> float {
    VERIFY_BITSTREAM(center);
    return center->x();
  }
  [[nodiscard]] auto sp_center_y() const noexcept -> float {
    VERIFY_BITSTREAM(center);
    return center->y();
  }
  [[nodiscard]] auto sp_center_z() const noexcept -> float {
    VERIFY_BITSTREAM(center);
    return center->z();
  }
  [[nodiscard]] auto sp_radius_x() const noexcept -> float { return radius.x(); }
  [[nodiscard]] auto sp_radius_y() const noexcept -> float { return radius.y(); }
  [[nodiscard]] auto sp_radius_z() const noexcept -> float { return radius.z(); }

  std::optional<Common::Vec3f> center{};
  Common::Vec3f radius{};

  friend auto operator<<(std::ostream &stream, const Spheroid &spheroid) -> std::ostream &;
  auto operator==(const Spheroid &other) const -> bool;
  auto operator!=(const Spheroid &other) const -> bool { return !operator==(other); }

  static auto decodeFrom(Common::InputBitstream &stream, bool cameraInferred) -> Spheroid;
  void encodeTo(Common::OutputBitstream &stream, bool cameraInferred) const;

  static auto loadFromJson(const Common::Json &node, bool inferredView) -> Spheroid;
};

// 23090-12: halfspace_primitive( e, s )
struct Halfspace {
  [[nodiscard]] auto hp_normal_x() const noexcept -> float { return normal.x(); }
  [[nodiscard]] auto hp_normal_y() const noexcept -> float { return normal.y(); }
  [[nodiscard]] auto hp_normal_z() const noexcept -> float { return normal.z(); }
  [[nodiscard]] auto hp_distance() const noexcept -> float { return distance; }

  Common::Vec3f normal{};
  float distance{};

  friend auto operator<<(std::ostream &stream, const Halfspace &halfspace) -> std::ostream &;
  auto operator==(const Halfspace &other) const -> bool;
  auto operator!=(const Halfspace &other) const -> bool { return !operator==(other); }

  static auto decodeFrom(Common::InputBitstream &stream) -> Halfspace;
  void encodeTo(Common::OutputBitstream &stream, bool inferredView) const;

  static auto loadFromJson(const Common::Json &node, bool inferredView) -> Halfspace;
};

struct PrimitiveShape {
  std::variant<Cuboid, Spheroid, Halfspace> primitive{};

  [[nodiscard]] auto shapeType() const -> PrimitiveShapeType;

  std::optional<float> guardBandSize{};

  std::optional<Common::QuatF> rotation{};

  struct ViewingDirectionConstraint {
    std::optional<float> guardBandDirectionSize{};
    Common::QuatF directionRotation{0.F, 0.F, 0.F, 1.F};
    float yawRange{360.F};
    float pitchRange{180.F};

    auto operator==(const ViewingDirectionConstraint &other) const -> bool;
    auto operator!=(const ViewingDirectionConstraint &other) const -> bool {
      return !operator==(other);
    }
  };
  std::optional<ViewingDirectionConstraint> viewingDirectionConstraint{};

  friend auto operator<<(std::ostream &stream, const PrimitiveShape &shape) -> std::ostream &;
  auto operator==(const PrimitiveShape &other) const -> bool;
  auto operator!=(const PrimitiveShape &other) const -> bool { return !operator==(other); }

  static auto loadFromJson(const Common::Json &node, bool inferredView) -> PrimitiveShape;
};

inline auto PrimitiveShape::shapeType() const -> PrimitiveShapeType {
  assert(primitive.index() != std::variant_npos);
  if (std::holds_alternative<Cuboid>(primitive)) {
    return PrimitiveShapeType::cuboid;
  }
  if (std::holds_alternative<Spheroid>(primitive)) {
    return PrimitiveShapeType::spheroid;
  }
  if (std::holds_alternative<Halfspace>(primitive)) {
    return PrimitiveShapeType::halfspace;
  }
  abort();
}

using PrimitiveShapeVector = std::vector<PrimitiveShape>;

// 23090-12: elementary_shape( e )
struct ElementaryShape {
  [[nodiscard]] auto es_num_primitive_shapes_minus1() const noexcept -> std::uint8_t;
  [[nodiscard]] constexpr auto es_primitive_shape_operation() const noexcept
      -> PrimitiveShapeOperation;
  [[nodiscard]] auto es_guard_band_present_flag() const noexcept -> bool;
  [[nodiscard]] auto es_primitive_orientation_present_flag() const noexcept -> bool;
  [[nodiscard]] auto es_viewing_direction_constraint_present_flag() const noexcept -> bool;
  [[nodiscard]] auto es_camera_inferred_flag() const noexcept -> bool;
  [[nodiscard]] auto es_view_idx(int s) const noexcept -> unsigned;
  [[nodiscard]] auto es_primitive_shape_type(int s) const noexcept -> PrimitiveShapeType;
  [[nodiscard]] auto es_guard_band_size(int s) const noexcept -> float;
  [[nodiscard]] auto es_primitive_shape_quat_x(int s) const noexcept -> float;
  [[nodiscard]] auto es_primitive_shape_quat_y(int s) const noexcept -> float;
  [[nodiscard]] auto es_primitive_shape_quat_z(int s) const noexcept -> float;
  [[nodiscard]] auto es_guard_band_direction_size(int s) const noexcept -> float;
  [[nodiscard]] auto es_primitive_shape_viewing_direction_quat_x_center(int s) const noexcept
      -> float;
  [[nodiscard]] auto es_primitive_shape_viewing_direction_quat_y_center(int s) const noexcept
      -> float;
  [[nodiscard]] auto es_primitive_shape_viewing_direction_quat_z_center(int s) const noexcept
      -> float;
  [[nodiscard]] auto es_primitive_shape_viewing_direction_yaw_range(int s) const noexcept -> float;
  [[nodiscard]] auto es_primitive_shape_viewing_direction_pitch_range(int s) const noexcept
      -> float;

  PrimitiveShapeVector primitives{};
  PrimitiveShapeOperation primitiveOperation{};
  std::vector<int> inferringViews{};

  friend auto operator<<(std::ostream &stream, const ElementaryShape &shape) -> std::ostream &;
  auto operator==(const ElementaryShape &other) const -> bool;
  auto operator!=(const ElementaryShape &other) const -> bool { return !operator==(other); }

  static auto decodeFrom(Common::InputBitstream &stream) -> ElementaryShape;
  void encodeTo(Common::OutputBitstream &stream) const;

  static auto loadFromJson(const Common::Json &node, const Common::Json &config) -> ElementaryShape;
};

struct ElementaryShapeAndOperation {
  ElementaryShapeAndOperation() = default;
  ElementaryShapeAndOperation(ElementaryShapeOperation o, ElementaryShape e)
      : elementary_shape{std::move(e)}, elementary_shape_operation{o} {}

  auto operator==(const ElementaryShapeAndOperation &other) const -> bool {
    return (elementary_shape == other.elementary_shape) &&
           (elementary_shape_operation == other.elementary_shape_operation);
  }

  ElementaryShape elementary_shape{};
  ElementaryShapeOperation elementary_shape_operation{};
};

using ElementaryShapeVector = std::vector<ElementaryShapeAndOperation>;

// 23090-12: viewing_space( )
struct ViewingSpace {
  [[nodiscard]] auto vs_num_elementary_shapes_minus1() const noexcept -> std::size_t;
  [[nodiscard]] auto vs_elementary_shape_operation(std::size_t e) const noexcept
      -> ElementaryShapeOperation;
  [[nodiscard]] auto elementary_shape(std::size_t e) const noexcept -> ElementaryShape;

  ElementaryShapeVector elementaryShapes{};

  friend auto operator<<(std::ostream &stream, const ViewingSpace &viewingSpace) -> std::ostream &;
  auto operator==(const ViewingSpace &other) const -> bool;
  auto operator!=(const ViewingSpace &other) const -> bool { return !operator==(other); }

  static auto decodeFrom(Common::InputBitstream &stream) -> ViewingSpace;
  void encodeTo(Common::OutputBitstream &stream) const;

  static auto loadFromJson(const Common::Json &node, const Common::Json &config) -> ViewingSpace;
};

} // namespace TMIV::MivBitstream

#endif
