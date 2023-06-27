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

#ifndef _TMIV_ENCODER_MPIENCODER_H_
#define _TMIV_ENCODER_MPIENCODER_H_

#include <TMIV/Common/Json.h>
#include <TMIV/Encoder/IMpiEncoder.h>
#include <TMIV/Packer/IPacker.h>

#include <deque>
#include <memory>

namespace TMIV::Encoder {
class MpiEncoder : public IMpiEncoder {
public:
  static constexpr auto maxIntraPeriod = 32;

private:
  // Parameters
  int m_intraPeriod{};
  Common::Vec2i m_blockSizeDepthQualityDependent;
  std::vector<Common::Vec2i> m_overrideAtlasFrameSizes{};
  unsigned m_textureDilation{};
  unsigned m_transparencyDynamic{};

  // Attributes
  std::unique_ptr<Packer::IPacker> m_packer;
  int m_blockSize{};
  std::size_t m_maxLumaSamplesPerFrame{};
  MivBitstream::EncoderParams m_params;

public:
  MpiEncoder(const Common::Json &rootNode, const Common::Json &componentNode);
  MpiEncoder(const MpiEncoder &) = delete;
  MpiEncoder(MpiEncoder &&) = default;
  auto operator=(const MpiEncoder &) -> MpiEncoder & = delete;
  auto operator=(MpiEncoder &&) -> MpiEncoder & = default;
  ~MpiEncoder() override = default;

  void prepareSequence(MivBitstream::EncoderParams params) override;
  auto processAccessUnit(int firstFrameId, int lastFrameId)
      -> const MivBitstream::EncoderParams & override;
  auto popAtlas(int frameId) -> Common::MVD10Frame override;
  [[nodiscard]] auto maxLumaSamplesPerFrame() const -> std::size_t override {
    return m_maxLumaSamplesPerFrame;
  }

private:
  auto vuiParameters() const -> MivBitstream::VuiParameters;
  void setGiGeometry3dCoordinatesBitdepthMinus1();
  void prepareIvau();
  auto log2FocLsbMinus4() const -> std::uint8_t;
  void incrementFoc();
  void writePatchInAtlas(const MivBitstream::PatchParams &patchParams,
                         const Common::TextureDepth10Frame &view, Common::MVD10Frame &atlas) const;
};

} // namespace TMIV::Encoder

#endif
