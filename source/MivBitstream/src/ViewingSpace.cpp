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

#include <TMIV/MivBitstream/ViewingSpace.h>

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/Common.h>
#include <TMIV/Common/Half.h>
#include <TMIV/Common/verify.h>

#include <algorithm>

namespace TMIV::MivBitstream {
using Common::Half;
using Common::Json;
using Common::QuatF;

static auto decodeRotation(Common::InputBitstream &stream) -> Common::QuatF {
  Common::QuatF q;
  q.x() = stream.getFloat16();
  q.y() = stream.getFloat16();
  q.z() = stream.getFloat16();
  q.w() = 0.F;
  q.w() = std::sqrt(1.F - norm2(q));
  return q;
}

static auto equalRotation(const Common::QuatF &a, const Common::QuatF &b) -> bool {
  assert(normalized(a, 1.0E-3F) && normalized(b, 1.0E-3F));
  const float d = dot(a, b);
  return d > 0.9999F;
}

auto ViewingSpace::vs_num_elementary_shapes_minus1() const noexcept -> std::size_t {
  VERIFY_MIVBITSTREAM(!elementaryShapes.empty());
  return elementaryShapes.size() - 1U;
}

auto ViewingSpace::vs_elementary_shape_operation(std::size_t e) const noexcept
    -> ElementaryShapeOperation {
  VERIFY_MIVBITSTREAM(e <= vs_num_elementary_shapes_minus1());
  return elementaryShapes[e].elementary_shape_operation;
}

auto ViewingSpace::elementary_shape(std::size_t e) const noexcept -> ElementaryShape {
  VERIFY_MIVBITSTREAM(e <= vs_num_elementary_shapes_minus1());
  return elementaryShapes[e].elementary_shape;
}

auto operator<<(std::ostream &stream, const ViewingSpace &viewingSpace) -> std::ostream & {
  stream << "Viewing space:" << std::endl;
  for (const auto &s : viewingSpace.elementaryShapes) {
    stream << (s.elementary_shape_operation == ElementaryShapeOperation::add ? "add "
                                                                             : "subtract ");
    stream << '(' << s.elementary_shape << ')' << std::endl;
  }
  return stream;
}

auto ViewingSpace::operator==(const ViewingSpace &other) const -> bool {
  return (elementaryShapes == other.elementaryShapes);
}

auto ViewingSpace::decodeFrom(Common::InputBitstream &stream) -> ViewingSpace {
  ViewingSpace vs;
  auto numShapes = stream.getUExpGolomb<size_t>() + 1;
  vs.elementaryShapes.reserve(numShapes);
  for (size_t i = 0; i < numShapes; ++i) {
    const auto op = stream.readBits<ElementaryShapeOperation>(2);
    const auto shape = ElementaryShape::decodeFrom(stream);
    vs.elementaryShapes.emplace_back(op, shape);
  }
  return vs;
}

void ViewingSpace::encodeTo(Common::OutputBitstream &stream) const {
  stream.putUExpGolomb(vs_num_elementary_shapes_minus1());
  for (const auto &shape : elementaryShapes) {
    stream.writeBits(shape.elementary_shape_operation, 2);
    shape.elementary_shape.encodeTo(stream);
  }
}

auto operator<<(std::ostream &stream, const ElementaryShape &elementaryShape) -> std::ostream & {
  stream << (elementaryShape.primitiveOperation == PrimitiveShapeOperation::interpolate
                 ? "interpolate"
                 : "add");
  for (const auto &p : elementaryShape.primitives) {
    stream << ", ";
    stream << p;
  }
  return stream;
}

auto ElementaryShape::es_num_primitive_shapes_minus1() const noexcept -> std::uint8_t {
  return static_cast<std::uint8_t>(primitives.size() - 1U);
}

constexpr auto ElementaryShape::es_primitive_shape_operation() const noexcept
    -> PrimitiveShapeOperation {
  return primitiveOperation;
}

auto ElementaryShape::es_guard_band_present_flag() const noexcept -> bool {
  return std::any_of(primitives.cbegin(), primitives.cend(), [](const PrimitiveShape &shape) {
    return shape.guardBandSize.has_value() ||
           (shape.viewingDirectionConstraint.has_value() &&
            shape.viewingDirectionConstraint->guardBandDirectionSize.has_value());
  });
}

auto ElementaryShape::es_primitive_orientation_present_flag() const noexcept -> bool {
  return std::any_of(primitives.cbegin(), primitives.cend(),
                     [](const PrimitiveShape &shape) { return shape.rotation.has_value(); });
}

auto ElementaryShape::es_viewing_direction_constraint_present_flag() const noexcept -> bool {
  return std::any_of(primitives.cbegin(), primitives.cend(), [](const PrimitiveShape &shape) {
    return shape.viewingDirectionConstraint.has_value();
  });
}

auto ElementaryShape::es_camera_inferred_flag() const noexcept -> bool {
  return !inferringViews.empty();
}

auto ElementaryShape::es_view_idx(int s) const noexcept -> unsigned {
  return static_cast<unsigned>(inferringViews[s]);
}

auto ElementaryShape::es_primitive_shape_type(int s) const noexcept -> PrimitiveShapeType {
  return primitives[s].shapeType();
}

auto ElementaryShape::es_guard_band_size(int s) const noexcept -> float {
  VERIFY_MIVBITSTREAM(es_guard_band_present_flag());
  return primitives[s].guardBandSize.value_or(0.F);
}

auto ElementaryShape::es_primitive_shape_quat_x(int s) const noexcept -> float {
  VERIFY_MIVBITSTREAM(es_primitive_orientation_present_flag());
  return primitives[s].rotation->x();
}

auto ElementaryShape::es_primitive_shape_quat_y(int s) const noexcept -> float {
  VERIFY_MIVBITSTREAM(es_primitive_orientation_present_flag());
  return primitives[s].rotation->y();
}

auto ElementaryShape::es_primitive_shape_quat_z(int s) const noexcept -> float {
  VERIFY_MIVBITSTREAM(es_primitive_orientation_present_flag());
  return primitives[s].rotation->z();
}

auto ElementaryShape::es_guard_band_direction_size(int s) const noexcept -> float {
  VERIFY_MIVBITSTREAM(es_guard_band_present_flag());
  const auto viewing_direction_constraint = primitives[s].viewingDirectionConstraint.value_or(
      PrimitiveShape::ViewingDirectionConstraint());
  return viewing_direction_constraint.guardBandDirectionSize.value_or(0.F);
}
auto ElementaryShape::es_primitive_shape_viewing_direction_quat_x_center(int s) const noexcept
    -> float {
  VERIFY_MIVBITSTREAM(es_viewing_direction_constraint_present_flag());
  const auto viewing_direction_constraint = primitives[s].viewingDirectionConstraint.value_or(
      PrimitiveShape::ViewingDirectionConstraint());
  return viewing_direction_constraint.directionRotation.x();
}
auto ElementaryShape::es_primitive_shape_viewing_direction_quat_y_center(int s) const noexcept
    -> float {
  VERIFY_MIVBITSTREAM(es_viewing_direction_constraint_present_flag());
  const auto viewing_direction_constraint = primitives[s].viewingDirectionConstraint.value_or(
      PrimitiveShape::ViewingDirectionConstraint());
  return viewing_direction_constraint.directionRotation.y();
}
auto ElementaryShape::es_primitive_shape_viewing_direction_quat_z_center(int s) const noexcept
    -> float {
  VERIFY_MIVBITSTREAM(es_viewing_direction_constraint_present_flag());
  const auto viewing_direction_constraint = primitives[s].viewingDirectionConstraint.value_or(
      PrimitiveShape::ViewingDirectionConstraint());
  return viewing_direction_constraint.directionRotation.z();
}

auto ElementaryShape::es_primitive_shape_viewing_direction_yaw_range(int s) const noexcept
    -> float {
  VERIFY_MIVBITSTREAM(es_viewing_direction_constraint_present_flag());
  const auto viewing_direction_constraint = primitives[s].viewingDirectionConstraint.value_or(
      PrimitiveShape::ViewingDirectionConstraint());
  return viewing_direction_constraint.yawRange;
}

auto ElementaryShape::es_primitive_shape_viewing_direction_pitch_range(int s) const noexcept
    -> float {
  VERIFY_MIVBITSTREAM(es_viewing_direction_constraint_present_flag());
  const auto viewing_direction_constraint = primitives[s].viewingDirectionConstraint.value_or(
      PrimitiveShape::ViewingDirectionConstraint());
  return viewing_direction_constraint.pitchRange;
}

auto ElementaryShape::operator==(const ElementaryShape &other) const -> bool {
  return (primitives == other.primitives) && (inferringViews == other.inferringViews);
}

auto ElementaryShape::decodeFrom(Common::InputBitstream &stream) -> ElementaryShape {
  ElementaryShape elementaryShape;
  const auto numPrimitives = stream.readBits<size_t>(8) + 1;
  elementaryShape.primitiveOperation = stream.readBits<PrimitiveShapeOperation>(2);
  const auto guardBandPresent = stream.getFlag();
  const auto orientationPresent = stream.getFlag();
  const auto directionConstraintPresent = stream.getFlag();
  const auto cameraInferred = stream.getFlag();
  elementaryShape.primitives.reserve(numPrimitives);
  for (std::size_t i = 0; i < numPrimitives; ++i) {
    TMIV::Common::Vec3f c{};
    if (cameraInferred) {
      elementaryShape.inferringViews.push_back(static_cast<int>(stream.getUint16()));
    }
    PrimitiveShape primitiveShape;
    const auto shapeType = stream.readBits<PrimitiveShapeType>(2);
    switch (shapeType) {
    case PrimitiveShapeType::cuboid:
      primitiveShape.primitive = Cuboid::decodeFrom(stream, cameraInferred);
      break;
    case PrimitiveShapeType::spheroid:
      primitiveShape.primitive = Spheroid::decodeFrom(stream, cameraInferred);
      break;
    case PrimitiveShapeType::halfspace:
      primitiveShape.primitive = Halfspace::decodeFrom(stream);
      break;
    default:
      abort();
    }
    if (guardBandPresent) {
      primitiveShape.guardBandSize = stream.getFloat16();
    }
    if (orientationPresent) {
      if (!cameraInferred) {
        primitiveShape.rotation = decodeRotation(stream);
      }
    }
    if (directionConstraintPresent) {
      auto vdc = PrimitiveShape::ViewingDirectionConstraint();
      if (guardBandPresent) {
        vdc.guardBandDirectionSize = stream.getFloat16();
      }
      if (!cameraInferred) {
        vdc.directionRotation = decodeRotation(stream);
      }
      vdc.yawRange = stream.getFloat16();
      vdc.pitchRange = stream.getFloat16();
      primitiveShape.viewingDirectionConstraint = vdc;
    }
    elementaryShape.primitives.emplace_back(primitiveShape);
  }
  return elementaryShape;
}

void ElementaryShape::encodeTo(Common::OutputBitstream &stream) const {
  VERIFY_MIVBITSTREAM(!primitives.empty());
  stream.writeBits(es_num_primitive_shapes_minus1(), 8);
  stream.writeBits(es_primitive_shape_operation(), 2);
  stream.putFlag(es_guard_band_present_flag());
  stream.putFlag(es_primitive_orientation_present_flag());
  stream.putFlag(es_viewing_direction_constraint_present_flag());
  stream.putFlag(es_camera_inferred_flag());
  for (int s = 0; s <= es_num_primitive_shapes_minus1(); s++) {
    const auto &p = primitives[static_cast<PrimitiveShapeVector::size_type>(s)];
    if (es_camera_inferred_flag()) {
      stream.putUint16(static_cast<std::uint16_t>(es_view_idx(s)));
    }
    stream.writeBits(es_primitive_shape_type(s), 2);
    visit([&](const auto &x) { x.encodeTo(stream, es_camera_inferred_flag()); }, p.primitive);
    if (es_guard_band_present_flag()) {
      stream.putFloat16(Common::Half(es_guard_band_size(s)));
    }
    if (es_primitive_orientation_present_flag()) {
      if (!es_camera_inferred_flag()) {
        stream.putFloat16(Common::Half(es_primitive_shape_quat_x(s)));
        stream.putFloat16(Common::Half(es_primitive_shape_quat_y(s)));
        stream.putFloat16(Common::Half(es_primitive_shape_quat_z(s)));
      }
    }
    if (es_viewing_direction_constraint_present_flag()) {
      if (es_guard_band_present_flag()) {
        stream.putFloat16(Common::Half(es_guard_band_direction_size(s)));
      }
      if (!es_camera_inferred_flag()) {
        stream.putFloat16(Common::Half(es_primitive_shape_viewing_direction_quat_x_center(s)));
        stream.putFloat16(Common::Half(es_primitive_shape_viewing_direction_quat_y_center(s)));
        stream.putFloat16(Common::Half(es_primitive_shape_viewing_direction_quat_z_center(s)));
      }
      stream.putFloat16(Common::Half(es_primitive_shape_viewing_direction_yaw_range(s)));
      stream.putFloat16(Common::Half(es_primitive_shape_viewing_direction_pitch_range(s)));
    }
  }
}

auto operator<<(std::ostream &stream, const PrimitiveShape &shape) -> std::ostream & {
  visit([&](const auto &x) { stream << x; }, shape.primitive);
  if (shape.guardBandSize.has_value()) {
    stream << " guardband " << shape.guardBandSize.value();
  }
  if (shape.rotation.has_value()) {
    stream << " rotation " << shape.rotation.value();
  }
  if (shape.viewingDirectionConstraint.has_value()) {
    const auto &vdc = shape.viewingDirectionConstraint.value();
    stream << " direction rotation " << vdc.directionRotation << " yaw range " << vdc.yawRange
           << " pitch range " << vdc.pitchRange << "+/-" << 0.5F * vdc.pitchRange;
    if (vdc.guardBandDirectionSize.has_value()) {
      stream << " guardband " << vdc.guardBandDirectionSize.value();
    }
  }
  return stream;
}

auto PrimitiveShape::operator==(const PrimitiveShape &other) const -> bool {
  if (primitive != other.primitive) {
    return false;
  }
  if (guardBandSize != other.guardBandSize) {
    return false;
  }
  if (!equalRotation(rotation.value_or(Common::QuatF{0.F, 0.F, 0.F, 1.F}),
                     other.rotation.value_or(Common::QuatF{0.F, 0.F, 0.F, 1.F}))) {
    return false;
  }
  if (viewingDirectionConstraint != other.viewingDirectionConstraint) {
    return false;
  }
  return true;
}

auto PrimitiveShape::ViewingDirectionConstraint::operator==(
    const ViewingDirectionConstraint &other) const -> bool {
  if (guardBandDirectionSize != other.guardBandDirectionSize) {
    return false;
  }
  if (!equalRotation(directionRotation, other.directionRotation)) {
    return false;
  }
  if (yawRange != other.yawRange || pitchRange != other.pitchRange) {
    return false;
  }
  return true;
}

auto operator<<(std::ostream &stream, const Cuboid &cuboid) -> std::ostream & {
  stream << "cuboid ";
  if (cuboid.center) {
    stream << cuboid.center.value();
  } else {
    stream << "without center assigned";
  }
  return stream << " size " << cuboid.size;
}

auto Cuboid::operator==(const Cuboid &other) const -> bool {
  return center == other.center && size == other.size;
}

auto Cuboid::decodeFrom(Common::InputBitstream &stream, bool cameraInferred) -> Cuboid {
  Cuboid cuboid;
  if (!cameraInferred) {
    cuboid.center = Common::Vec3f{};
    cuboid.center->x() = stream.getFloat16();
    cuboid.center->y() = stream.getFloat16();
    cuboid.center->z() = stream.getFloat16();
  }
  cuboid.size.x() = stream.getFloat16();
  cuboid.size.y() = stream.getFloat16();
  cuboid.size.z() = stream.getFloat16();
  return cuboid;
}

void Cuboid::encodeTo(Common::OutputBitstream &stream, bool cameraInferred) const {
  if (!cameraInferred) {
    stream.putFloat16(Common::Half(cp_center_x()));
    stream.putFloat16(Common::Half(cp_center_y()));
    stream.putFloat16(Common::Half(cp_center_z()));
  }
  stream.putFloat16(Common::Half(cp_size_x()));
  stream.putFloat16(Common::Half(cp_size_y()));
  stream.putFloat16(Common::Half(cp_size_z()));
}

auto operator<<(std::ostream &stream, const Spheroid &spheroid) -> std::ostream & {
  stream << "spheroid ";
  if (spheroid.center) {
    stream << spheroid.center.value();
  } else {
    stream << "without center assigned";
  }
  return stream << " radius " << spheroid.radius;
}

auto Spheroid::operator==(const Spheroid &other) const -> bool {
  return center == other.center && radius == other.radius;
}

auto Spheroid::decodeFrom(Common::InputBitstream &stream, bool cameraInferred) -> Spheroid {
  Spheroid spheroid;
  if (!cameraInferred) {
    spheroid.center = Common::Vec3f{};
    spheroid.center->x() = stream.getFloat16();
    spheroid.center->y() = stream.getFloat16();
    spheroid.center->z() = stream.getFloat16();
  }
  spheroid.radius.x() = stream.getFloat16();
  spheroid.radius.y() = stream.getFloat16();
  spheroid.radius.z() = stream.getFloat16();
  return spheroid;
}

void Spheroid::encodeTo(Common::OutputBitstream &stream, bool cameraInferred) const {
  if (!cameraInferred) {
    stream.putFloat16(Common::Half(sp_center_x()));
    stream.putFloat16(Common::Half(sp_center_y()));
    stream.putFloat16(Common::Half(sp_center_z()));
  }
  stream.putFloat16(Common::Half(sp_radius_x()));
  stream.putFloat16(Common::Half(sp_radius_y()));
  stream.putFloat16(Common::Half(sp_radius_z()));
}

auto operator<<(std::ostream &stream, const Halfspace &halfspace) -> std::ostream & {
  stream << "halfspace " << halfspace.normal << " distance " << halfspace.distance;
  return stream;
}

auto Halfspace::operator==(const Halfspace &other) const -> bool {
  return normal == other.normal && distance == other.distance;
}

auto Halfspace::decodeFrom(Common::InputBitstream &stream) -> Halfspace {
  Halfspace plane;
  plane.normal.x() = stream.getFloat16();
  plane.normal.y() = stream.getFloat16();
  plane.normal.z() = stream.getFloat16();
  plane.distance = stream.getFloat16();
  return plane;
}

void Halfspace::encodeTo(Common::OutputBitstream &stream, bool /*cameraInferred*/) const {
  stream.putFloat16(Common::Half(hp_normal_x()));
  stream.putFloat16(Common::Half(hp_normal_y()));
  stream.putFloat16(Common::Half(hp_normal_z()));
  stream.putFloat16(Common::Half(hp_distance()));
}

auto ViewingSpace::loadFromJson(const Common::Json &node, const Common::Json &config)
    -> ViewingSpace {
  auto parseOperation = [](const std::string &str) -> auto {
    if (str == "add") {
      return ElementaryShapeOperation::add;
    }
    if (str == "subtract") {
      return ElementaryShapeOperation::subtract;
    };
    if (str == "intersect") {
      return ElementaryShapeOperation::intersect;
    }
    throw std::runtime_error("Invalid elementary shape operation in the metadata JSON file");
  };

  ViewingSpace viewingSpace{};
  const auto &elementaryShapes = node.require("ElementaryShapes").as<Json::Array>();

  for (const auto &elementaryShape : elementaryShapes) {
    viewingSpace.elementaryShapes.emplace_back(
        parseOperation(elementaryShape.require("ElementaryShapeOperation").as<std::string>()),
        ElementaryShape::loadFromJson(elementaryShape.require("ElementaryShape"), config));
  }

  // consolidate the optional values across all primitives in each elementary shape
  for (auto &elementaryShape : viewingSpace.elementaryShapes) {
    bool guardBandPresent{};
    bool rotationPresent{};
    bool directionConstraintPresent{};
    for (auto &primitive : elementaryShape.elementary_shape.primitives) {
      guardBandPresent |= primitive.guardBandSize.has_value();
      rotationPresent |= primitive.rotation.has_value();
      if (primitive.viewingDirectionConstraint.has_value()) {
        directionConstraintPresent |= true;
        guardBandPresent |=
            primitive.viewingDirectionConstraint.value().guardBandDirectionSize.has_value();
      }
    }
    for (auto &primitive : elementaryShape.elementary_shape.primitives) {
      if (guardBandPresent && !primitive.guardBandSize.has_value()) {
        primitive.guardBandSize = 0.F;
      }
      if (rotationPresent && !primitive.rotation.has_value()) {
        primitive.rotation = Common::QuatF();
      }
      if (directionConstraintPresent) {
        if (!primitive.viewingDirectionConstraint.has_value()) {
          primitive.viewingDirectionConstraint = PrimitiveShape::ViewingDirectionConstraint();
        }
        if (guardBandPresent &&
            !primitive.viewingDirectionConstraint.value().guardBandDirectionSize.has_value()) {
          primitive.viewingDirectionConstraint.value().guardBandDirectionSize = 0.F;
        }
      }
    }
  }
  return viewingSpace;
}

auto ElementaryShape::loadFromJson(const Common::Json &node, const Common::Json &config)
    -> ElementaryShape {
  auto parseOperation = [](const std::string &str) -> auto {
    if (str == "add") {
      return PrimitiveShapeOperation::add;
    }
    if (str == "interpolate") {
      return PrimitiveShapeOperation::interpolate;
    };
    throw std::runtime_error("Invalid primitive shape operation in the metadata JSON file");
  };

  ElementaryShape elementaryShape{};

  // added for m52412 inferred_views implementation
  bool inferredView = false;
  if (const auto &subsubnode = node.optional("InferringViews")) {
    // TODO(BK): Use ID instead of index? Not sure how to fix this code.
    const auto sourceCameraNames = config.require("SourceCameraNames").asVector<std::string>();
    std::vector<std::string> views = subsubnode.asVector<std::string>();
    for (const auto &v : views) {
      size_t idx = 0;
      for (; idx < sourceCameraNames.size(); idx++) {
        if (v == sourceCameraNames[idx]) {
          break;
        }
      }
      if (idx == sourceCameraNames.size()) {
        throw std::runtime_error("Invalid inferred view in the metadata JSON file");
      }
      elementaryShape.inferringViews.push_back(static_cast<int>(idx));
    }
    inferredView = true;
  }

  // primitive shape operation
  elementaryShape.primitiveOperation =
      parseOperation(node.require("PrimitiveShapeOperation").as<std::string>());

  // primitive shapes
  const auto &primitiveShapes = node.require("PrimitiveShapes").as<Json::Array>();
  for (const auto &primitiveShape : primitiveShapes) {
    elementaryShape.primitives.push_back(
        PrimitiveShape::loadFromJson(primitiveShape, inferredView));
  }

  // check consistency
  if (inferredView &&
      (elementaryShape.primitives.size() != elementaryShape.inferringViews.size())) {
    throw std::runtime_error(
        "Incompatible number of inferring views and primitive shapes in the metadata JSON file");
  }

  return elementaryShape;
}

auto PrimitiveShape::loadFromJson(const Common::Json &node, bool inferredView) -> PrimitiveShape {
  PrimitiveShape primitiveShape{};
  const auto &shapeType = node.require("PrimitiveShapeType").as<std::string>();
  if (shapeType == "Cuboid") {
    primitiveShape.primitive = Cuboid{{}, {}};
  }
  if (shapeType == "cuboid") {
    primitiveShape.primitive = Cuboid::loadFromJson(node, inferredView);
  }
  if (shapeType == "spheroid") {
    primitiveShape.primitive = Spheroid::loadFromJson(node, inferredView);
  }
  if (shapeType == "halfspace") {
    primitiveShape.primitive = Halfspace::loadFromJson(node, inferredView);
  }
  if (const auto &subnode = node.optional("GuardBandSize")) {
    primitiveShape.guardBandSize = subnode.as<float>();
  }
  if (const auto &subnode = node.optional("Rotation")) {
    primitiveShape.rotation = Common::euler2quat(Common::radperdeg * subnode.asVec<float, 3>());
  }
  if (const auto &subnode = node.optional("ViewingDirectionConstraint")) {
    primitiveShape.viewingDirectionConstraint = PrimitiveShape::ViewingDirectionConstraint();
    if (const auto &subsubnode = subnode.optional("GuardBandDirectionSize")) {
      primitiveShape.viewingDirectionConstraint.value().guardBandDirectionSize =
          subsubnode.as<float>();
    }
    if (!inferredView) {
      const float directionYaw = subnode.require("YawCenter").as<float>();
      const float directionPitch = subnode.require("PitchCenter").as<float>();
      primitiveShape.viewingDirectionConstraint.value().directionRotation =
          Common::euler2quat(Common::radperdeg * Common::Vec3f{directionYaw, directionPitch, 0.F});
    }
    primitiveShape.viewingDirectionConstraint.value().yawRange =
        subnode.require("YawRange").as<float>();
    primitiveShape.viewingDirectionConstraint.value().pitchRange =
        subnode.require("PitchRange").as<float>();
  }
  return primitiveShape;
}

auto Cuboid::loadFromJson(const Common::Json &node, bool inferredView) -> Cuboid {
  Cuboid cuboid;
  if (!inferredView) {
    cuboid.center = node.require("Center").asVec<float, 3>();
  }
  cuboid.size = node.require("Size").asVec<float, 3>();
  return cuboid;
}

auto Spheroid::loadFromJson(const Common::Json &node, bool inferredView) -> Spheroid {
  Spheroid spheroid;
  if (!inferredView) {
    spheroid.center = node.require("Center").asVec<float, 3>();
  }
  spheroid.radius = node.require("Radius").asVec<float, 3>();
  return spheroid;
}

auto Halfspace::loadFromJson(const Common::Json &node, bool /*inferredView*/) -> Halfspace {
  Halfspace plane;
  plane.normal = node.require("Normal").asVec<float, 3>();
  plane.distance = node.require("Distance").as<float>();
  return plane;
}

} // namespace TMIV::MivBitstream
