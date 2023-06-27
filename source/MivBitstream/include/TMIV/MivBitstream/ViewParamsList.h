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

#ifndef _TMIV_MIVBITSTREAM_VIEWPARAMSLIST_H_
#define _TMIV_MIVBITSTREAM_VIEWPARAMSLIST_H_

#include <TMIV/MivBitstream/CommonAtlasFrameMivExtension.h>

#include <TMIV/Common/Json.h>
#include <TMIV/Common/Vector.h>

namespace TMIV::MivBitstream {
struct ViewParams {
  CameraIntrinsics ci;
  CameraExtrinsics ce;
  DepthQuantization dq;
  std::optional<PruningParents> pp;

  ViewParams() = default;
  ViewParams(const CameraIntrinsics &ci, const CameraExtrinsics &ce, const DepthQuantization &dq,
             std::optional<PruningParents> pp, std::string name)
      : ci{ci}, ce{ce}, dq{dq}, pp{std::move(pp)}, name{std::move(name)} {}

  // Not in the specification. Just to improve screen output
  std::string name{};

  // Not part of the bitstream. Does the depth map have invalid/non-occupied?
  bool hasOccupancy{};

  // Is this a basic view or an additional view?
  bool isBasicView{true};

  // Is this view a regular view (e.g. captured by a camera) or the result of an inpainting process?
  bool isInpainted{};

  // Number of layers in MPI. Not in the specification, but needed to handle MPI.
  int nbMpiLayers{1};

  auto printTo(std::ostream &stream, std::uint16_t viewId) const -> std::ostream &;
  auto operator==(const ViewParams &other) const -> bool;
  auto operator!=(const ViewParams &other) const -> bool { return !operator==(other); }

  // Load a single (source) camera from a JSON metadata file (RVS 3.x format)
  //
  // The parameter is a an item of the viewParamsList node (a JSON object).
  explicit ViewParams(const Common::Json &node);

  explicit operator Common::Json() const;
};

// Data type that corresponds to camera_params_list of specification
struct ViewParamsList : public std::vector<ViewParams> {
  ViewParamsList() = default;
  explicit ViewParamsList(std::vector<ViewParams> viewParamsList);

  // Size or name of each view as a vector
  [[nodiscard]] auto viewSizes() const -> Common::SizeVector;
  [[nodiscard]] auto viewNames() const -> std::vector<std::string>;

  friend auto operator<<(std::ostream &stream, const ViewParamsList &viewParamsList)
      -> std::ostream &;
  auto operator==(const ViewParamsList &other) const -> bool;
  auto operator!=(const ViewParamsList &other) const -> bool { return !operator==(other); }

  // Load (source) camera parameters from a JSON metadata file (RVS 3.x format)
  static auto loadFromJson(const Common::Json &node, const std::vector<std::string> &names)
      -> ViewParamsList;
};
} // namespace TMIV::MivBitstream

#endif
