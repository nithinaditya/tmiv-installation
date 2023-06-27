/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
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
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
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

#include <TMIV/VideoDecoder/HmVideoDecoder.h>

#include <TLibCommon/TComList.h>
#include <TLibCommon/TComPicYuv.h>
#include <TLibDecoder/AnnexBread.h>
#include <TLibDecoder/NALread.h>
#include <TLibDecoder/TDecTop.h>

namespace TMIV::VideoDecoder {
// This implementation is based on TAppDec.cpp (HM 16.16) with all optional parameters locked to
// default values and without fields.
class HmVideoDecoder::Impl {
public:
  void decode(std::istream &stream) {
    int poc{};
    TComList<TComPic *> *pcListPic = nullptr;

    InputByteStream bytestream(stream);

    // create & initialize internal classes
    m_cTDecTop.create();
    m_cTDecTop.init();
    m_cTDecTop.setDecodedPictureHashSEIEnabled(1);

    // set the last displayed POC correctly for skip forward.
    m_iPOCLastDisplay += m_iSkipFrame;

    // reconstruction file not yet opened. (must be performed after SPS is seen)
    bool loopFiltered = false;

    // main decoder loop
    while (!!stream) {
      /* location serves to work around a design fault in the decoder, whereby
       * the process of reading a new slice that is the first slice of a new frame
       * requires the TDecTop::decode() method to be called again with the same
       * nal unit. */
      std::streampos location = stream.tellg();
      AnnexBStats stats = AnnexBStats();

      InputNALUnit nalu;
      byteStreamNALUnit(bytestream, nalu.getBitstream().getFifo(), stats);

      // call actual decoding function
      bool bNewPicture = false;
      if (nalu.getBitstream().getFifo().empty()) {
        std::cout << "Warning: Attempt to decode an empty NAL unit\n";
      } else {
        read(nalu);
        bNewPicture = m_cTDecTop.decode(nalu, m_iSkipFrame, m_iPOCLastDisplay);
        if (bNewPicture) {
          stream.clear();
          stream.seekg(location - streamoff(3));
          bytestream.reset();
        }
      }

      if ((bNewPicture || !stream || nalu.m_nalUnitType == NAL_UNIT_EOS) &&
          !m_cTDecTop.getFirstSliceInSequence()) {
        if (!loopFiltered || stream) {
          m_cTDecTop.executeLoopFilters(poc, pcListPic);
        }
        loopFiltered = (nalu.m_nalUnitType == NAL_UNIT_EOS);
        if (nalu.m_nalUnitType == NAL_UNIT_EOS) {
          m_cTDecTop.setFirstSliceInSequence(true);
        }
      } else if ((bNewPicture || !stream || nalu.m_nalUnitType == NAL_UNIT_EOS) &&
                 m_cTDecTop.getFirstSliceInSequence()) {
        m_cTDecTop.setFirstSliceInPicture(true);
      }

      if (pcListPic != nullptr) {
        if (m_outputBitDepth.front() == 0) {
          const auto &recon = pcListPic->front()->getPicSym()->getSPS().getBitDepths().recon;
          std::copy(std::cbegin(recon), std::cend(recon), std::begin(m_outputBitDepth));
        }

        if (bNewPicture) {
          xWriteOutput(*pcListPic, nalu.m_temporalId);
        }
        if ((bNewPicture || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA) &&
            m_cTDecTop.getNoOutputPriorPicsFlag()) {
          m_cTDecTop.checkNoOutputPriorPics(pcListPic);
          m_cTDecTop.setNoOutputPriorPicsFlag(false);
        }
        if (bNewPicture && (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
                            nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
                            nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_N_LP ||
                            nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_RADL ||
                            nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_LP)) {
          xFlushOutput(*pcListPic);
        }
        if (nalu.m_nalUnitType == NAL_UNIT_EOS) {
          xWriteOutput(*pcListPic, nalu.m_temporalId);
          m_cTDecTop.setFirstSliceInPicture(false);
        }
        // write reconstruction to file -- for additional bumping as defined in C.5.2.3
        if (!bNewPicture && nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_TRAIL_N &&
            nalu.m_nalUnitType <= NAL_UNIT_RESERVED_VCL31) {
          xWriteOutput(*pcListPic, nalu.m_temporalId);
        }
      }
    }

    xFlushOutput(*pcListPic);

    // TODO(BK): It's either double delete or leaking memory. Easy to fix by putting a reference
    //           count in initROM/destroyROM, but the intention was not to modify HM.
    // m_cTDecTop.deletePicBuffer();
    m_cTDecTop.destroy();
  }

  void addFrameListener(FrameListener listener) { m_frameListeners.push_back(std::move(listener)); }

private:
  void xWriteOutput(TComList<TComPic *> &pcListPic, unsigned /*tId*/) {
    if (pcListPic.empty()) {
      return;
    }

    int numPicsNotYetDisplayed = 0;
    int dpbFullness = 0;
    const auto &activeSPS = pcListPic.front()->getPicSym()->getSPS();
    const auto maxNrSublayers = activeSPS.getMaxTLayers();
    const auto numReorderPicsHighestTid = activeSPS.getNumReorderPics(maxNrSublayers - 1);
    const auto maxDecPicBufferingHighestTid = activeSPS.getMaxDecPicBuffering(maxNrSublayers - 1);

    for (const auto *pcPic : pcListPic) {
      if (pcPic->getOutputMark() && pcPic->getPOC() > m_iPOCLastDisplay) {
        numPicsNotYetDisplayed++;
        dpbFullness++;
      } else if (pcPic->getSlice(0)->isReferenced()) {
        dpbFullness++;
      }
    }

    for (auto *pcPic : pcListPic) {
      if (pcPic->getOutputMark() && pcPic->getPOC() > m_iPOCLastDisplay &&
          (numPicsNotYetDisplayed > numReorderPicsHighestTid ||
           dpbFullness > maxDecPicBufferingHighestTid)) {
        numPicsNotYetDisplayed--;
        if (!pcPic->getSlice(0)->isReferenced()) {
          dpbFullness--;
        }

        xWritePicture(*pcPic);
      }
    }
  }

  void xFlushOutput(TComList<TComPic *> &pcListPic) {
    if (pcListPic.empty()) {
      return;
    }

    for (auto *pcPic : pcListPic) {
      if (pcPic->getOutputMark()) {
        xWritePicture(*pcPic);
      }
      if (pcPic != nullptr) {
        pcPic->destroy();
        delete pcPic; // NOLINT(cppcoreguidelines-owning-memory)
      }
    }

    pcListPic.clear();
    m_iPOCLastDisplay = -MAX_INT;
  }

  auto anyFrame(TComPicYuv &comPicYuv) const -> Common::AnyFrame {
    auto x = Common::AnyFrame{};

    assert(comPicYuv.getNumberValidComponents() <= x.planes.size());

    for (const auto componentId : {COMPONENT_Y, COMPONENT_Cb, COMPONENT_Cr}) {
      if (componentId < comPicYuv.getNumberValidComponents()) {
        const auto k = int{componentId};
        const auto width = comPicYuv.getWidth(componentId);
        const auto height = comPicYuv.getHeight(componentId);

        x.bitdepth[k] = m_outputBitDepth[toChannelType(componentId)];
        x.planes[k].resize(static_cast<size_t>(height), static_cast<size_t>(width));

        const auto *row = comPicYuv.getAddr(componentId);

        for (int i = 0; i < height; ++i) {
          // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
          std::copy(row, row + width, x.planes[k].row_begin(i));
          // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
          row += comPicYuv.getStride(componentId);
        }
      }
    }

    return x;
  }

  void xWritePicture(TComPic &comPic) {
    if (!m_frameListeners.empty()) {
      // Copy into Common::AnyFrame
      auto *comPicYuv = comPic.getPicYuvRec();
      assert(comPicYuv);
      const auto picture = anyFrame(*comPicYuv);

      // Invoke all listeners
      for (const auto &listener : m_frameListeners) {
        listener(picture);
      }
    }

    // update POC of display order
    m_iPOCLastDisplay = comPic.getPOC();

    // erase non-referenced comPic in the reference comPic list after display
    if (!comPic.getSlice(0)->isReferenced() && comPic.getReconMark()) {
      comPic.setReconMark(false);

      // mark it should be extended later
      comPic.getPicYuvRec()->setBorderExtension(false);
    }

    comPic.setOutputMark(false);
  }

  TDecTop m_cTDecTop{};

  std::vector<FrameListener> m_frameListeners;

  int m_iPOCLastDisplay{-MAX_INT};
  int m_iSkipFrame{};
  std::array<int, MAX_NUM_CHANNEL_TYPE> m_outputBitDepth{};
}; // namespace TMIV::VideoDecoder

HmVideoDecoder::HmVideoDecoder() : m_impl{new Impl{}} {}

HmVideoDecoder::~HmVideoDecoder() = default;

void HmVideoDecoder::decode(std::istream &stream) { m_impl->decode(stream); }

void HmVideoDecoder::addFrameListener(FrameListener listener) {
  return m_impl->addFrameListener(std::move(listener));
}
} // namespace TMIV::VideoDecoder
