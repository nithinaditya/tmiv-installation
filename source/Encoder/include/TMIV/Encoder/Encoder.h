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

#ifndef _TMIV_ENCODER_ENCODER_H_
#define _TMIV_ENCODER_ENCODER_H_

#include <TMIV/Aggregator/IAggregator.h>
#include <TMIV/Common/Json.h>
#include <TMIV/Encoder/GeometryDownscaler.h>
#include <TMIV/Encoder/IEncoder.h>
#include <TMIV/GeometryQuantizer/IGeometryQuantizer.h>
#include <TMIV/Packer/IPacker.h>
#include <TMIV/Pruner/IPruner.h>
#include <TMIV/ViewOptimizer/IViewOptimizer.h>

#include <bitset>
#include <deque>
#include <memory>

namespace TMIV::Encoder {
class Encoder : public IEncoder {
public:
  Encoder(const Common::Json & /*rootNode*/, const Common::Json & /*componentNode*/);
  Encoder(const Encoder &) = delete;
  Encoder(Encoder &&) = default;
  auto operator=(const Encoder &) -> Encoder & = delete;
  auto operator=(Encoder &&) -> Encoder & = default;
  ~Encoder() override = default;

  void prepareSequence(MivBitstream::EncoderParams params) override;
  void prepareAccessUnit() override;
  void pushFrame(Common::MVD16Frame sourceViews) override;
  auto completeAccessUnit() -> const MivBitstream::EncoderParams & override;
  auto popAtlas() -> Common::MVD10Frame override;
  [[nodiscard]] auto maxLumaSamplesPerFrame() const -> std::size_t override;

private: // Encoder_prepareSequence.cpp
  [[nodiscard]] auto
  calculateNominalAtlasFrameSizes(const MivBitstream::EncoderParams &params) const
      -> Common::SizeVector;
  [[nodiscard]] auto calculateViewGridSize(const MivBitstream::EncoderParams &params) const
      -> Common::Vec2i;
  auto vuiParameters() const -> MivBitstream::VuiParameters;
  void setGiGeometry3dCoordinatesBitdepthMinus1();
  void enableOccupancyPerView();
  void prepareIvau();
  auto log2FocLsbMinus4() const -> std::uint8_t;
  auto patchSizeQuantizers() const -> Common::Vec2i;

private: // Encoder_prepareAccessUnit.cpp
  void resetNonAggregatedMask();

private: // Encoder_pushFrame.cpp
  void pushSingleEntityFrame(Common::MVD16Frame sourceViews);
  void updateNonAggregatedMask(const Common::MVD16Frame &transportViews,
                               const Common::MaskList &masks);
  void pushMultiEntityFrame(Common::MVD16Frame sourceViews);
  static auto entitySeparator(const Common::MVD16Frame &transportViews, uint16_t entityId)
      -> Common::MVD16Frame;
  static auto yuvSampler(const Common::EntityMapList &in)
      -> std::vector<Common::Frame<Common::YUV420P16>>;
  static void mergeMasks(Common::MaskList &mergedMasks, Common::MaskList masks);
  static void updateMasks(const Common::MVD16Frame &views, Common::MaskList &masks);
  void aggregateEntityMasks(Common::MaskList &masks, std::uint16_t entityId);

private: // Encoder_completeAccessUnit.cpp
  void scaleGeometryDynamicRange();
  void updateAggregationStatistics(const Common::MaskList &aggregatedMask);
  void constructVideoFrames();
  void calculateAttributeOffset(
      std::vector<std::array<std::array<int64_t, 4>, 3>> patchAttrOffsetValuesFullGOP);
  auto calculatePatchAttrOffsetValuesFullGOP(
      std::vector<std::array<std::array<int64_t, 4>, 3>> &patchAttrOffsetValuesFullGOP) -> int;
  auto calculateBtpm() const -> std::vector<std::vector<std::vector<int>>>;
  void adaptBtpmToPatchCount(std::vector<std::vector<std::vector<int>>> &btpm) const;
  auto writePatchInAtlas(const MivBitstream::PatchParams &patchParams,
                         const Common::TextureDepth16Frame &view, Common::MVD16Frame &frame,
                         int frameId) -> std::array<std::array<int64_t, 4>, 3>;
  void adaptAtlas(const MivBitstream::PatchParams &patchParams,
                  Common::TextureDepthFrame<Common::YUV400P16> &atlas, int yOcc, int xOcc,
                  const Common::Vec2i &pView, const Common::Vec2i &pAtlas) const;

private:
  struct Configuration {
    Configuration(const Common::Json & /*rootNode*/, const Common::Json & /*componentNode*/);
    int intraPeriod;
    int blockSize{};
    Common::Vec2i blockSizeDepthQualityDependent;
    double maxLumaSampleRate{};
    int maxLumaPictureSize{};
    double maxBlockRate{};
    int maxBlocksPerAtlas{};
    int maxAtlases{};
    bool haveTexture;
    bool haveGeometry;
    bool haveOccupancy;
    bool oneViewPerAtlasFlag;
    std::vector<Common::Vec2i> overrideAtlasFrameSizes{};
    bool geometryScaleEnabledFlag;
    int dilationIter;
    Common::Vec2i entityEncRange;
    bool dynamicDepthRange;
    bool attributeOffsetFlag;
    int attributeOffsetBitCount;
  };

  // Encoder_popFrame.cpp
  void incrementFoc();

  // Encoder sub-components
  std::unique_ptr<ViewOptimizer::IViewOptimizer> m_viewOptimizer;
  std::unique_ptr<Pruner::IPruner> m_pruner;
  std::unique_ptr<Aggregator::IAggregator> m_aggregator;
  std::unique_ptr<Packer::IPacker> m_packer;
  std::unique_ptr<GeometryQuantizer::IGeometryQuantizer> m_geometryQuantizer;
  GeometryDownscaler m_geometryDownscaler;

  Configuration m_config;

  // View-optimized encoder input
  MivBitstream::EncoderParams m_transportParams;
  std::vector<Common::MVD16Frame> m_transportViews;

  // Encoder output (ready for HM)
  MivBitstream::EncoderParams m_params;
  std::deque<Common::MVD16Frame> m_videoFrameBuffer;

  // Mask aggregation state
  static constexpr auto maxIntraPeriod = 32;
  using NonAggregatedMask = Common::Mat<std::bitset<maxIntraPeriod>>;
  std::vector<NonAggregatedMask> m_nonAggregatedMask;
  std::vector<Common::MaskList> m_aggregatedEntityMask;
  std::size_t m_maxLumaSamplesPerFrame{};
};
} // namespace TMIV::Encoder

#endif
