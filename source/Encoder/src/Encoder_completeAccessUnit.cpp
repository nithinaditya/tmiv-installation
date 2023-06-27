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

#include <TMIV/Encoder/Encoder.h>

#include <cassert>
#include <iostream>

namespace TMIV::Encoder {
namespace {
constexpr auto textureBitDepth = Common::TextureFrame::getBitDepth();
constexpr auto textureMaxVal = (1 << textureBitDepth) - 1;
constexpr auto textureMedVal = 1 << (textureBitDepth - 1);

struct PatchStats {
  PatchStats() = default;
  PatchStats(int64_t _minVal) : minVal{_minVal} {}
  int64_t minVal{0}, maxVal{0}, sumVal{0}, cntVal{0};
};

void adaptPatchStatsToTexture(std::array<PatchStats, 3> &patchStats,
                              const Common::TextureDepth16Frame &view,
                              Common::TextureDepthFrame<Common::YUV400P16> &atlas,
                              const Common::Vec2i &pView, const Common::Vec2i &pAtlas) {
  // Y
  atlas.texture.getPlane(0)(pAtlas.y(), pAtlas.x()) =
      view.texture.getPlane(0)(pView.y(), pView.x());

  patchStats[0].sumVal += atlas.texture.getPlane(0)(pAtlas.y(), pAtlas.x());
  patchStats[0].cntVal += 1;

  if (patchStats[0].minVal > atlas.texture.getPlane(0)(pAtlas.y(), pAtlas.x())) {
    patchStats[0].minVal = atlas.texture.getPlane(0)(pAtlas.y(), pAtlas.x());
  }
  if (patchStats[0].maxVal < atlas.texture.getPlane(0)(pAtlas.y(), pAtlas.x())) {
    patchStats[0].maxVal = atlas.texture.getPlane(0)(pAtlas.y(), pAtlas.x());
  }

  // UV
  if ((pView.x() % 2) == 0 && (pView.y() % 2) == 0) {
    for (int p = 1; p < 3; ++p) {
      atlas.texture.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2) =
          view.texture.getPlane(p)(pView.y() / 2, pView.x() / 2);

      patchStats[p].sumVal += atlas.texture.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2);
      patchStats[p].cntVal += 1;

      if (patchStats[p].minVal > atlas.texture.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2)) {
        patchStats[p].minVal = atlas.texture.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2);
      }
      if (patchStats[p].maxVal < atlas.texture.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2)) {
        patchStats[p].maxVal = atlas.texture.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2);
      }
    }
  }
}
} // namespace

void Encoder::scaleGeometryDynamicRange() {
  assert(m_config.dynamicDepthRange);
  const auto lowDepthQuality = m_params.casme().casme_depth_low_quality_flag();
  const auto numOfFrames = m_transportViews.size();
  const auto numOfViews = m_transportViews[0].size();

  for (size_t v = 0; v < numOfViews; v++) {
    int minDepthMapValWithinGOP = 65535;
    int maxDepthMapValWithinGOP = 0;

    for (size_t f = 0; f < numOfFrames; f++) {
      for (const auto geometry : m_transportViews[f][v].depth.getPlane(0)) {
        if (geometry < minDepthMapValWithinGOP) {
          minDepthMapValWithinGOP = geometry;
        }
        if (geometry > maxDepthMapValWithinGOP) {
          maxDepthMapValWithinGOP = geometry;
        }
      }
    }

    for (size_t f = 0; f < numOfFrames; f++) {
      for (auto &geometry : m_transportViews[f][v].depth.getPlane(0)) {
        geometry = static_cast<uint16_t>(
            (geometry + 0.5 - minDepthMapValWithinGOP) /
            (static_cast<double>(maxDepthMapValWithinGOP) - minDepthMapValWithinGOP) * 65535.0);
        if (lowDepthQuality) {
          geometry /= 2;
        }
      }
    }
    const double normDispHighOrig = m_transportParams.viewParamsList[v].dq.dq_norm_disp_high();
    const double normDispLowOrig = m_transportParams.viewParamsList[v].dq.dq_norm_disp_low();

    double normDispHigh =
        maxDepthMapValWithinGOP / 65535.0 * (normDispHighOrig - normDispLowOrig) + normDispLowOrig;
    const double normDispLow =
        minDepthMapValWithinGOP / 65535.0 * (normDispHighOrig - normDispLowOrig) + normDispLowOrig;

    if (lowDepthQuality) {
      normDispHigh = 2 * normDispHigh - normDispLow;
    }

    m_params.viewParamsList[v].dq.dq_norm_disp_high(static_cast<float>(normDispHigh));
    m_params.viewParamsList[v].dq.dq_norm_disp_low(static_cast<float>(normDispLow));
  }
}

auto Encoder::completeAccessUnit() -> const MivBitstream::EncoderParams & {
  m_aggregator->completeAccessUnit();
  const auto &aggregatedMask = m_aggregator->getAggregatedMask();

  updateAggregationStatistics(aggregatedMask);

  if (m_config.dynamicDepthRange) {
    scaleGeometryDynamicRange();
  }

  if (0 < m_params.maxEntityId) {
    m_packer->updateAggregatedEntityMasks(m_aggregatedEntityMask);
  }

  m_params.patchParamsList = m_packer->pack(m_params.atlasSizes(), aggregatedMask,
                                            m_transportParams.viewParamsList, m_config.blockSize);

  m_params = m_geometryQuantizer->setOccupancyParams(m_params);

  constructVideoFrames();

  const auto &paramsQuantized = m_geometryQuantizer->transformParams(m_params);
  const auto &paramsScaled = m_geometryDownscaler.transformParams(paramsQuantized);
  return paramsScaled;
}

void Encoder::updateAggregationStatistics(const Common::MaskList &aggregatedMask) {
  const auto lumaSamplesPerFrame = std::accumulate(
      aggregatedMask.begin(), aggregatedMask.end(), size_t{}, [](size_t sum, const auto &mask) {
        return sum + 2 * std::count_if(mask.getPlane(0).begin(), mask.getPlane(0).end(),
                                       [](auto x) { return x > 0; });
      });
  std::cout << "Aggregated luma samples per frame is " << (1e-6 * lumaSamplesPerFrame) << "M\n";
  m_maxLumaSamplesPerFrame = std::max(m_maxLumaSamplesPerFrame, lumaSamplesPerFrame);
}

// TODO(BK): Avoid functions like this one that are too long and/or poorly named
void Encoder::calculateAttributeOffset(
    std::vector<std::array<std::array<int64_t, 4>, 3>> patchAttrOffsetValuesFullGOP) {

  for (std::uint8_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
    auto &asme = m_params.atlas[k].asme();
    asme.asme_patch_attribute_offset_enabled_flag(m_config.attributeOffsetFlag);
    if (!m_config.attributeOffsetFlag) {
      return;
    }
    asme.asme_patch_attribute_offset_bit_depth_minus1(m_config.attributeOffsetBitCount - 1);
  }

  const auto bitShift = calculatePatchAttrOffsetValuesFullGOP(patchAttrOffsetValuesFullGOP);

  std::vector<std::vector<std::vector<int>>> btpm = calculateBtpm();

  adaptBtpmToPatchCount(btpm);

  for (auto &videoFrame : m_videoFrameBuffer) {
    for (std::uint8_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
      auto &atlas = videoFrame[k];

      for (int y = 0; y < atlas.texture.getHeight(); ++y) {
        for (int x = 0; x < atlas.texture.getWidth(); ++x) {
          const auto patchIndex = btpm[k][y / m_config.blockSize][x / m_config.blockSize];

          // TODO(BK): Avoid comparing depth with 0
          if (patchIndex == Common::unusedPatchId ||
              (atlas.depth.getPlane(0)(y, x) == 0 &&
               !m_params
                    .viewParamsList[m_params.patchParamsList[patchIndex].atlasPatchProjectionId()]
                    .isBasicView)) {
            continue;
          }
          const auto &pp = m_params.patchParamsList[patchIndex];
          const auto offset = pp.atlasPatchAttributeOffset();
          atlas.texture.getPlane(0)(y, x) -= ((offset.x() << bitShift) - textureMedVal);
          if (y % 2 == 0 && x % 2 == 0) {
            atlas.texture.getPlane(1)(y / 2, x / 2) -= ((offset.y() << bitShift) - textureMedVal);
            atlas.texture.getPlane(2)(y / 2, x / 2) -= ((offset.z() << bitShift) - textureMedVal);
          }
        }
      }
    }
  }
}

void Encoder::adaptBtpmToPatchCount(std::vector<std::vector<std::vector<int>>> &btpm) const {
  int patchCnt = 0;
  for (const auto &patch : m_params.patchParamsList) {

    size_t atlasId = m_params.vps.indexOf(patch.atlasId);

    const auto &currentAtlas = m_videoFrameBuffer[0][atlasId];
    int AH = currentAtlas.texture.getHeight() / m_config.blockSize;
    int AW = currentAtlas.texture.getWidth() / m_config.blockSize;

    int w = patch.atlasPatch3dSizeU();
    int h = patch.atlasPatch3dSizeV();
    int xM = patch.atlasPatch3dOffsetU();
    int yM = patch.atlasPatch3dOffsetV();

    for (int dyAligned = 0; dyAligned < h; dyAligned += m_config.blockSize) {
      for (int dxAligned = 0; dxAligned < w; dxAligned += m_config.blockSize) {

        for (int dy = dyAligned; dy < dyAligned + m_config.blockSize; dy++) {
          for (int dx = dxAligned; dx < dxAligned + m_config.blockSize; dx++) {

            Common::Vec2i pView = {xM + dx, yM + dy};
            Common::Vec2i pAtlas = patch.viewToAtlas(pView);

            int ay = pAtlas.y() / m_config.blockSize;
            int ax = pAtlas.x() / m_config.blockSize;

            if (ay < 0 || ax < 0 || ay >= AH || ax >= AW || pAtlas.y() % m_config.blockSize != 0 ||
                pAtlas.x() % m_config.blockSize != 0) {
              continue;
            }

            // std::cout << ay << "\t" << ax << "\n";
            btpm[atlasId][ay][ax] = patchCnt;
          }
        }
      }
    }

    patchCnt++;
  }
}

auto Encoder::calculateBtpm() const -> std::vector<std::vector<std::vector<int>>> {
  std::vector<std::vector<std::vector<int>>> btpm;
  for (uint8_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
    const auto &currentAtlas = m_videoFrameBuffer[0][k];
    int AH = currentAtlas.texture.getHeight() / m_config.blockSize;
    int AW = currentAtlas.texture.getWidth() / m_config.blockSize;
    std::vector<std::vector<int>> tmphw;
    for (int h = 0; h < AH; h++) {
      std::vector<int> tmpw;
      tmpw.reserve(AW);
      for (int w = 0; w < AW; w++) {
        tmpw.push_back(65535);
      }
      tmphw.push_back(tmpw);
    }
    btpm.push_back(tmphw);
  }
  return btpm;
}

auto Encoder::calculatePatchAttrOffsetValuesFullGOP(
    std::vector<std::array<std::array<int64_t, 4>, 3>> &patchAttrOffsetValuesFullGOP) -> int {
  const auto bitShift = textureBitDepth - m_config.attributeOffsetBitCount;

  for (size_t p = 0; p != m_params.patchParamsList.size(); ++p) {
    for (int c = 0; c < 3; c++) {
      if (patchAttrOffsetValuesFullGOP[p][c][3] > 0) {
        patchAttrOffsetValuesFullGOP[p][c][2] /= patchAttrOffsetValuesFullGOP[p][c][3];
      } else {
        patchAttrOffsetValuesFullGOP[p][c][2] = 0;
      }
      patchAttrOffsetValuesFullGOP[p][c][2] -= textureMedVal; // offset

      if (patchAttrOffsetValuesFullGOP[p][c][0] - patchAttrOffsetValuesFullGOP[p][c][2] < 0) {
        patchAttrOffsetValuesFullGOP[p][c][2] = patchAttrOffsetValuesFullGOP[p][c][0];
      } else if (patchAttrOffsetValuesFullGOP[p][c][1] - patchAttrOffsetValuesFullGOP[p][c][2] >
                 int64_t(textureMaxVal)) {
        patchAttrOffsetValuesFullGOP[p][c][2] =
            patchAttrOffsetValuesFullGOP[p][c][1] - textureMaxVal;
      }

      patchAttrOffsetValuesFullGOP[p][c][2] += textureMedVal;
      patchAttrOffsetValuesFullGOP[p][c][2] >>= bitShift;
    }

    m_params.patchParamsList[p].atlasPatchAttributeOffset(
        {static_cast<uint16_t>(patchAttrOffsetValuesFullGOP[p][0][2]),
         static_cast<uint16_t>(patchAttrOffsetValuesFullGOP[p][1][2]),
         static_cast<uint16_t>(patchAttrOffsetValuesFullGOP[p][2][2])});
  }
  return bitShift;
}

void Encoder::constructVideoFrames() {
  int frameId = 0;

  auto patchAttrOffsetValuesFullGOP = std::vector<std::array<std::array<int64_t, 4>, 3>>{};

  if (m_config.attributeOffsetFlag) {
    for (int p = 0; p < int(m_params.patchParamsList.size()); p++) {
      std::array<std::array<int64_t, 4>, 3> tmp{};
      for (int c = 0; c < 3; c++) {
        tmp[c][0] = textureMaxVal;
        tmp[c][1] = 0;
        tmp[c][2] = 0;
        tmp[c][3] = 0;
      }
      patchAttrOffsetValuesFullGOP.push_back(tmp);
    }
  }

  const auto &vps = m_params.vps;

  for (const auto &views : m_transportViews) {
    Common::MVD16Frame atlasList;

    for (size_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
      auto &frame = atlasList.emplace_back();
      const auto j = vps.vps_atlas_id(k);
      const auto frameWidth = vps.vps_frame_width(j);
      const auto frameHeight = vps.vps_frame_height(j);

      if (m_config.haveTexture) {
        frame.texture.resize(frameWidth, frameHeight);
        frame.texture.fillNeutral();
      }

      if (m_config.haveGeometry) {
        frame.depth.resize(frameWidth, frameHeight);
        frame.depth.fillZero();
      }

      if (vps.vps_occupancy_video_present_flag(j)) {
        int occFrameWidth = frameWidth;
        int occFrameHeight = frameHeight;

        const auto &asme = m_params.atlas[k].asme();
        if (!asme.asme_embedded_occupancy_enabled_flag() &&
            asme.asme_occupancy_scale_enabled_flag()) {
          occFrameWidth /= asme.asme_occupancy_scale_factor_x_minus1() + 1;
          occFrameHeight /= asme.asme_occupancy_scale_factor_y_minus1() + 1;
        }
        frame.occupancy.resize(Common::align(occFrameWidth, 2), Common::align(occFrameHeight, 2));
        frame.occupancy.fillZero();
      }
    }

    int patchCnt = 0;
    auto patchAttrOffsetValues1Frame = std::array<std::array<int64_t, 4>, 3>{};

    for (const auto &patch : m_params.patchParamsList) {
      const auto &view = views[patch.atlasPatchProjectionId()];

      const auto k = m_params.vps.indexOf(patch.atlasId);
      if (0 < m_params.atlas[k].asme().asme_max_entity_id()) {
        Common::MVD16Frame tempViews;
        tempViews.push_back(view);
        const auto &entityViews = entitySeparator(tempViews, *patch.atlasPatchEntityId());
        patchAttrOffsetValues1Frame = writePatchInAtlas(patch, entityViews[0], atlasList, frameId);
      } else {
        patchAttrOffsetValues1Frame = writePatchInAtlas(patch, view, atlasList, frameId);
      }

      if (m_config.attributeOffsetFlag) {
        for (int c = 0; c < 3; c++) {
          if (patchAttrOffsetValuesFullGOP[patchCnt][c][0] > patchAttrOffsetValues1Frame[c][0]) {
            patchAttrOffsetValuesFullGOP[patchCnt][c][0] = patchAttrOffsetValues1Frame[c][0];
          }
          if (patchAttrOffsetValuesFullGOP[patchCnt][c][1] < patchAttrOffsetValues1Frame[c][1]) {
            patchAttrOffsetValuesFullGOP[patchCnt][c][1] = patchAttrOffsetValues1Frame[c][1];
          }
          patchAttrOffsetValuesFullGOP[patchCnt][c][2] += patchAttrOffsetValues1Frame[c][2];
          patchAttrOffsetValuesFullGOP[patchCnt][c][3] += patchAttrOffsetValues1Frame[c][3];
        }
        patchCnt++;
      }
    }
    m_videoFrameBuffer.push_back(std::move(atlasList));
    ++frameId;
  }
  calculateAttributeOffset(patchAttrOffsetValuesFullGOP);
}

auto Encoder::writePatchInAtlas(const MivBitstream::PatchParams &patchParams,
                                const Common::TextureDepth16Frame &view, Common::MVD16Frame &frame,
                                int frameId) -> std::array<std::array<int64_t, 4>, 3> {

  const auto k = m_params.vps.indexOf(patchParams.atlasId);
  auto &atlas = frame[k];

  // TODO(BK): It would be better if atlasPatch... functions are std::int32_t or std::int64_t
  const auto sizeU = static_cast<std::int32_t>(patchParams.atlasPatch3dSizeU());
  const auto sizeV = static_cast<std::int32_t>(patchParams.atlasPatch3dSizeV());
  const auto posU = static_cast<std::int32_t>(patchParams.atlasPatch3dOffsetU());
  const auto posV = static_cast<std::int32_t>(patchParams.atlasPatch3dOffsetV());

  const auto &inViewParams = m_transportParams.viewParamsList[patchParams.atlasPatchProjectionId()];
  const auto &outViewParams = m_params.viewParamsList[patchParams.atlasPatchProjectionId()];

  std::array<PatchStats, 3> patchStats{};
  std::fill(patchStats.begin(), patchStats.end(), PatchStats{textureMaxVal});

  assert(0 <= posU && posU + sizeU <= inViewParams.ci.ci_projection_plane_width_minus1() + 1);
  assert(0 <= posV && posV + sizeV <= inViewParams.ci.ci_projection_plane_height_minus1() + 1);

  for (int vBlock = 0; vBlock < sizeV; vBlock += m_config.blockSize) {
    for (int uBlock = 0; uBlock < sizeU; uBlock += m_config.blockSize) {
      bool isAggregatedMaskBlockNonEmpty = false;
      for (int v = vBlock; v < vBlock + m_config.blockSize && v < sizeV; v++) {
        for (int u = uBlock; u < uBlock + m_config.blockSize && u < sizeU; u++) {
          const auto viewId = patchParams.atlasPatchProjectionId();
          if (m_nonAggregatedMask[viewId](v + posV, u + posU)[frameId]) {
            isAggregatedMaskBlockNonEmpty = true;
            break;
          }
        }
        if (isAggregatedMaskBlockNonEmpty) {
          break;
        }
      }
      int yOcc = 0;
      int xOcc = 0;
      for (int v = vBlock; v < vBlock + m_config.blockSize && v < sizeV; ++v) {
        for (int u = uBlock; u < uBlock + m_config.blockSize && u < sizeU; ++u) {
          const auto pView = Common::Vec2i{posU + u, posV + v};
          const auto pAtlas = patchParams.viewToAtlas(pView);

          const auto &asme = m_params.atlas[k].asme();
          if (!asme.asme_embedded_occupancy_enabled_flag() &&
              asme.asme_occupancy_scale_enabled_flag()) {
            yOcc = pAtlas.y() / (asme.asme_occupancy_scale_factor_y_minus1() + 1);
            xOcc = pAtlas.x() / (asme.asme_occupancy_scale_factor_x_minus1() + 1);
          } else {
            yOcc = pAtlas.y();
            xOcc = pAtlas.x();
          }

          if (!isAggregatedMaskBlockNonEmpty && m_config.haveGeometry) {
            adaptAtlas(patchParams, atlas, yOcc, xOcc, pView, pAtlas);
            continue;
          }

          if (m_config.haveTexture) {
            adaptPatchStatsToTexture(patchStats, view, atlas, pView, pAtlas);
          }

          // Depth
          if (m_config.haveGeometry) {
            auto depth = view.depth.getPlane(0)(pView.y(), pView.x());
            // TODO(BK): We need to stop using depth == 0 as a special value. This is bug-prone.
            if (depth == 0 && !inViewParams.hasOccupancy && outViewParams.hasOccupancy &&
                asme.asme_max_entity_id() == 0) {
              depth = 1; // Avoid marking valid depth as invalid
            }
            atlas.depth.getPlane(0)(pAtlas.y(), pAtlas.x()) = depth;
            if (depth > 0 && m_params.vps.vps_occupancy_video_present_flag(patchParams.atlasId)) {
              atlas.occupancy.getPlane(0)(yOcc, xOcc) = 1;
            }
          }
        }
      }
    }
  }

  std::array<std::array<int64_t, 4>, 3> ret{};
  for (int c = 0; c < 3; c++) {
    ret[c][0] = patchStats[c].minVal;
    ret[c][1] = patchStats[c].maxVal;
    ret[c][2] = patchStats[c].sumVal;
    ret[c][3] = patchStats[c].cntVal;
  }
  return ret;
}

void Encoder::adaptAtlas(const MivBitstream::PatchParams &patchParams,
                         Common::TextureDepthFrame<Common::YUV400P16> &atlas, int yOcc, int xOcc,
                         const Common::Vec2i &pView, const Common::Vec2i &pAtlas) const {
  atlas.depth.getPlane(0)(pAtlas.y(), pAtlas.x()) = 0;
  if (m_params.vps.vps_occupancy_video_present_flag(patchParams.atlasId)) {
    atlas.occupancy.getPlane(0)(yOcc, xOcc) = 0;
  }
  if (m_config.haveTexture) {
    atlas.texture.getPlane(0)(pAtlas.y(), pAtlas.x()) = textureMedVal;
    if ((pView.x() % 2) == 0 && (pView.y() % 2) == 0) {
      atlas.texture.getPlane(1)(pAtlas.y() / 2, pAtlas.x() / 2) = textureMedVal;
      atlas.texture.getPlane(2)(pAtlas.y() / 2, pAtlas.x() / 2) = textureMedVal;
    }
  }
}

} // namespace TMIV::Encoder
