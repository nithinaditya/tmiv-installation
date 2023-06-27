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

#include <TMIV/Renderer/SubBlockCuller.h>

#include <TMIV/Common/Factory.h>
#include <TMIV/MivBitstream/DepthOccupancyTransform.h>
#include <cassert>
#include <iostream>

#include "TMIV/Renderer/Engine.h"
#include <TMIV/Renderer/reprojectPoints.h>

namespace TMIV::Renderer {
SubBlockCuller::SubBlockCuller(const Common::Json & /*rootNode*/,
                               const Common::Json & /*componentNode*/) {}

auto choosePatch(const MivBitstream::PatchParams &patch,
                 const MivBitstream::ViewParamsList &cameras,
                 const MivBitstream::ViewParams &target) -> bool {
  const auto &camera = cameras[patch.atlasPatchProjectionId()];
  const auto R_t = AffineTransform(cameras[patch.atlasPatchProjectionId()].ce, target.ce);

  auto uv = std::array<Common::Vec2f, 4>{};
  auto xy_v = std::array<Common::Vec2f, 8>{};
  const auto w = static_cast<float>(patch.atlasPatch3dSizeU());
  const auto h = static_cast<float>(patch.atlasPatch3dSizeV());
  uv[0].x() = static_cast<float>(patch.atlasPatch3dOffsetU());
  uv[0].y() = static_cast<float>(patch.atlasPatch3dOffsetV());
  uv[1] = uv[0] + Common::Vec2f{w, 0};
  uv[2] = uv[0] + Common::Vec2f{0, h};
  uv[3] = uv[0] + Common::Vec2f{w, h};

  // Using Camera depth
  constexpr auto depthBitDepth = Common::Depth10Frame::getBitDepth();
  const auto depthTransform = MivBitstream::DepthTransform{camera.dq, patch, depthBitDepth};
  const auto patch_dep_near = depthTransform.expandDepth(Common::maxLevel(depthBitDepth));
  const auto patch_dep_far = depthTransform.expandDepth(0);

  for (int i = 0; i < 4; i++) {
    const auto xyz = R_t(unprojectVertex(uv[i], patch_dep_near, camera.ci));
    const auto rayAngle = Common::angle(xyz, xyz - R_t.translation());
    SceneVertexDescriptor v;
    v.position = xyz;
    v.rayAngle = rayAngle;
    auto pix = target.ci.dispatch([&](auto camType) {
      Engine<camType> engine{target.ci};
      return engine.projectVertex(v);
    });
    xy_v[i] = pix.position;
  }
  for (int i = 0; i < 4; i++) {
    const auto xyz = R_t(unprojectVertex(uv[i], patch_dep_far, camera.ci));
    const auto rayAngle = Common::angle(xyz, xyz - R_t.translation());
    SceneVertexDescriptor v;
    v.position = xyz;
    v.rayAngle = rayAngle;
    auto pix = target.ci.dispatch([&](auto camType) {
      Engine<camType> engine{target.ci};
      return engine.projectVertex(v);
    });
    // wangbin modify
    xy_v[i + 4] = pix.position;
  }

  // TODO(BK): Magic constant assumes meters
  float xy_v_xmax = -1000;
  float xy_v_xmin = 65536;

  // TODO(BK): Magic constant assumes meters
  float xy_v_ymax = -1000;
  float xy_v_ymin = 65536;

  for (auto &i : xy_v) {
    if (i[0] > xy_v_xmax) {
      xy_v_xmax = i[0];
    }
    if (i[0] < xy_v_xmin) {
      xy_v_xmin = i[0];
    }
    if (i[1] > xy_v_ymax) {
      xy_v_ymax = i[1];
    }
    if (i[1] < xy_v_ymin) {
      xy_v_ymin = i[1];
    }
  }
  return !(xy_v_xmin > target.ci.projectionPlaneSize().x() || xy_v_xmax < 0 || xy_v_ymax < 0 ||
           xy_v_ymin > target.ci.projectionPlaneSize().y() ||
           (xy_v_xmin != xy_v_xmin && xy_v_xmax != xy_v_xmax && xy_v_ymin != xy_v_ymin &&
            xy_v_ymax != xy_v_ymax));
}

auto divideInBlocks(const MivBitstream::PatchParams &patch) {
  // The size of the sub-block is fixed for now
  constexpr std::int32_t blockSize = 128;

  const auto gridWidth = (patch.atlasPatch2dSizeX() + blockSize - 1) / blockSize;
  const auto gridHeight = (patch.atlasPatch2dSizeY() + blockSize - 1) / blockSize;
  MivBitstream::PatchParamsList subblock(gridWidth * static_cast<size_t>(gridHeight), patch);

  for (int32_t blockY = 0; blockY < gridHeight; ++blockY) {
    for (int32_t blockX = 0; blockX < gridWidth; ++blockX) {
      auto &b = subblock[blockY * static_cast<size_t>(gridWidth) + blockX];

      const auto x1 = blockX * blockSize;
      const auto y1 = blockY * blockSize;
      const auto x2 = std::min(x1 + blockSize, patch.atlasPatch2dSizeX());
      const auto y2 = std::min(y1 + blockSize, patch.atlasPatch2dSizeY());

      b.atlasPatch2dPosX(patch.atlasPatch2dPosX() + x1);
      b.atlasPatch2dPosY(patch.atlasPatch2dPosY() + y1);
      b.atlasPatch2dSizeX(x2 - x1);
      b.atlasPatch2dSizeY(y2 - y1);

      assert(patch.atlasPatchOrientationIndex() ==
             MivBitstream::FlexiblePatchOrientation::FPO_NULL);
      b.atlasPatch3dOffsetU(b.atlasPatch3dOffsetU() + x1);
      b.atlasPatch3dOffsetV(b.atlasPatch3dOffsetV() + y1);
    }
  }
  return subblock;
}

auto SubBlockCuller::filterBlockToPatchMap(const MivBitstream::AccessUnit &frame,
                                           const MivBitstream::AtlasAccessUnit &atlas,
                                           const MivBitstream::ViewParams &viewportParams) const
    -> Common::BlockToPatchMap {
  auto result = atlas.blockToPatchMap;

  for (size_t patchIdx = 0; patchIdx < atlas.patchParamsList.size(); ++patchIdx) {
    const auto &patch = atlas.patchParamsList[patchIdx];
    const auto &view = frame.viewParamsList[patch.atlasPatchProjectionId()];

    if (patch.atlasPatch3dSizeU() == view.ci.ci_projection_plane_width_minus1() + 1 &&
        patch.atlasPatch3dSizeV() == view.ci.ci_projection_plane_height_minus1() + 1 &&
        patch.atlasPatchOrientationIndex() == MivBitstream::FlexiblePatchOrientation::FPO_NULL) {
      for (const auto &block : divideInBlocks(patch)) {
        if (!choosePatch(block, frame.viewParamsList, viewportParams)) {
          inplaceErasePatch(result, block, static_cast<uint16_t>(patchIdx), atlas.asps);
        }
      }
    } else {
      if (!choosePatch(patch, frame.viewParamsList, viewportParams)) {
        inplaceErasePatch(result, patch, static_cast<uint16_t>(patchIdx), atlas.asps);
      }
    }
  }
  return result;
}

void SubBlockCuller::inplaceErasePatch(Common::BlockToPatchMap &patchMap,
                                       const MivBitstream::PatchParams &patch, uint16_t patchId,
                                       const MivBitstream::AtlasSequenceParameterSetRBSP &asps) {
  const auto patchPackingBlockSize = 1U << asps.asps_log2_patch_packing_block_size();
  const auto firstX = patch.atlasPatch2dPosX() / patchPackingBlockSize;
  const auto firstY = patch.atlasPatch2dPosY() / patchPackingBlockSize;
  const auto lastX = firstX + patch.atlasPatch2dSizeX() / patchPackingBlockSize;
  const auto lastY = firstY + patch.atlasPatch2dSizeY() / patchPackingBlockSize;

  for (auto y = firstY; y < lastY; ++y) {
    for (auto x = firstX; x < lastX; ++x) {
      if (patchMap.getPlane(0)(y, x) == patchId) {
        patchMap.getPlane(0)(y, x) = Common::unusedPatchId;
      }
    }
  }
}

} // namespace TMIV::Renderer
