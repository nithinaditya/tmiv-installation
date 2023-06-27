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

#include <TMIV/Renderer/Inpainter.h>

#include <TMIV/Common/Common.h>

#include <cmath>

namespace TMIV::Renderer {
namespace {
constexpr auto depthBlendingThreshold = 655.36; // 1% of bit depth

enum class InpaintingType { horizontal, vertical, omnidirectional };

void perform2WayInpainting(Common::Texture444Depth16Frame &yuvd, InpaintingType inpaintingType,
                           const Common::Mat<int> &nonEmptyNeighbor1,
                           const Common::Mat<int> &nonEmptyNeighbor2,
                           const Common::Mat<int> &mapERP2Cassini = Common::Mat<int>()) {
  auto &Y = yuvd.first.getPlane(0);
  auto &U = yuvd.first.getPlane(1);
  auto &V = yuvd.first.getPlane(2);
  auto &D = yuvd.second.getPlane(0);

  const int width = static_cast<int>(Y.width());
  const int height = static_cast<int>(Y.height());

  for (int h = 0, pp = 0; h < height; h++) {
    for (int w = 0; w < width; w++, pp++) {
      if (D(h, w) != 0) {
        continue;
      }

      bool use1 = false;
      bool use2 = false;

      int w0 = 0;
      int h0 = 0;
      int w1 = 0;
      int h1 = 0;
      int w2 = 0;
      int h2 = 0;

      if (inpaintingType == InpaintingType::omnidirectional) {
        bool pointExistsInCassini = mapERP2Cassini(h, w) != -1;
        if (!pointExistsInCassini) {
          continue;
        }

        h0 = mapERP2Cassini(h, w) / width; // current pixel in Cassini
                                           // projection
        w0 = mapERP2Cassini(h, w) % width;

        h1 = nonEmptyNeighbor1(h0, w0) / width; // left neighbor in Cassini projection
        w1 = nonEmptyNeighbor1(h0, w0) % width;

        h2 = nonEmptyNeighbor2(h0, w0) / width; // right neighbor in Cassini projection
        w2 = nonEmptyNeighbor2(h0, w0) % width;
      } else {  // perspective
        h0 = h; // current pixel
        w0 = w;

        h1 = inpaintingType == InpaintingType::vertical
                 ? h0
                 : nonEmptyNeighbor1(h0, w0); // left or top neighbor
        w1 = inpaintingType == InpaintingType::vertical ? nonEmptyNeighbor1(h0, w0) : w0;

        h2 = inpaintingType == InpaintingType::vertical
                 ? h0
                 : nonEmptyNeighbor2(h0, w0); // right or bottom neighbor
        w2 = inpaintingType == InpaintingType::vertical ? nonEmptyNeighbor2(h0, w0) : w0;
      }

      if (nonEmptyNeighbor1(h0, w0) != -1) {
        if (nonEmptyNeighbor2(h0, w0) != -1) {
          const auto farthestDepth = D(h1, w1) < D(h2, w2) ? D(h1, w1) : D(h2, w2);
          if (D(h1, w1) - farthestDepth <= depthBlendingThreshold) {
            use1 = true;
          }
          if (D(h2, w2) - farthestDepth <= depthBlendingThreshold) {
            use2 = true;
          }
        } else {
          use1 = true;
        }
      } else {
        if (nonEmptyNeighbor2(h0, w0) != -1) {
          use2 = true;
        } else {
          continue;
        }
      }

      if (use1) {
        if (use2) {
          auto dist1 = std::sqrt(static_cast<float>((h - h1) * (h - h1) + (w - w1) * (w - w1)));
          auto dist2 = std::sqrt(static_cast<float>((h - h2) * (h - h2) + (w - w2) * (w - w2)));
          const auto sumdist = dist1 + dist2;
          const auto weight1 = dist2 / sumdist;
          const auto weight2 = dist1 / sumdist;

          Y(h, w) = static_cast<int>(static_cast<float>(Y(h1, w1)) * weight1);
          U(h, w) = static_cast<int>(static_cast<float>(U(h1, w1)) * weight1);
          V(h, w) = static_cast<int>(static_cast<float>(V(h1, w1)) * weight1);
          D(h, w) = static_cast<int>(static_cast<float>(D(h1, w1)) * weight1);

          Y(h, w) += static_cast<int>(static_cast<float>(Y(h2, w2)) * weight2);
          U(h, w) += static_cast<int>(static_cast<float>(U(h2, w2)) * weight2);
          V(h, w) += static_cast<int>(static_cast<float>(V(h2, w2)) * weight2);
          D(h, w) += static_cast<int>(static_cast<float>(D(h2, w2)) * weight2);

          if (D(h, w) == 0) {
            D(h, w) = static_cast<int>(static_cast<float>(D(h1, w1)) * weight1 +
                                       static_cast<float>(D(h2, w2)) * weight2);
          }
        } else {
          Y(h, w) = Y(h1, w1);
          U(h, w) = U(h1, w1);
          V(h, w) = V(h1, w1);
          D(h, w) = D(h1, w1);
        }
      } else {
        if (use2) {
          Y(h, w) = Y(h2, w2);
          U(h, w) = U(h2, w2);
          V(h, w) = V(h2, w2);
          D(h, w) = D(h2, w2);
        } else {
          continue;
        }
      }

    } // w
  }   // h
}

void fillVerticalCracks(Common::Texture444Depth16Frame &yuvd) {
  auto &Y = yuvd.first.getPlane(0);
  auto &U = yuvd.first.getPlane(1);
  auto &V = yuvd.first.getPlane(2);
  auto &D = yuvd.second.getPlane(0);

  const int width = static_cast<int>(Y.width());
  const int height = static_cast<int>(Y.height());

  // fill vertical cracks
  for (int h = 0; h < height; h++) {
    for (int w = 1; w < width - 1; w++) {
      if (D(h, w) == 0 && D(h, w - 1) != 0 && D(h, w + 1) != 0) {
        Y(h, w) = (Y(h, w - 1) + Y(h, w + 1)) / 2;
        U(h, w) = (U(h, w - 1) + U(h, w + 1)) / 2;
        V(h, w) = (V(h, w - 1) + V(h, w + 1)) / 2;
        D(h, w) = (D(h, w - 1) + D(h, w + 1)) / 2;
      }
    }
  }
}

void inpaintOmnidirectionalView(Common::Texture444Depth16Frame &yuvd,
                                double fullOmniRangePercentage) {
  auto &Y = yuvd.first.getPlane(0);
  auto &D = yuvd.second.getPlane(0);

  const int width = static_cast<int>(Y.width());
  const int height = static_cast<int>(Y.height());

  Common::Mat<uint8_t> isHole;
  isHole.resize(height, width);

  Common::Mat<int> nonEmptyNeighborL;
  nonEmptyNeighborL.resize(height, width);

  Common::Mat<int> nonEmptyNeighborR;
  nonEmptyNeighborR.resize(height, width);

  Common::Mat<int> mapERP2Cassini;
  mapERP2Cassini.resize(height, width);

  Common::Mat<int> mapCassini2ERP;
  mapCassini2ERP.resize(height, width);

  for (int h = 0; h < height; h++) {
    for (int w = 0; w < width; w++) {
      nonEmptyNeighborL(h, w) = -1;
      nonEmptyNeighborR(h, w) = -1;
      mapERP2Cassini(h, w) = -1;
      mapCassini2ERP(h, w) = -1;
      isHole(h, w) = 1;
    }
  }

  int width2 = width / 2;
  int height2 = height / 2;
  for (int h = 0; h < height; h++) {
    auto oldH = h - height2;
    for (int w = 0; w < width; w++) {
      auto oldPP = h * width + w;

      auto oldW = w - width2;
      auto tmpH = std::sqrt(height * h - h * h);
      if (tmpH / height2 > fullOmniRangePercentage) {
        tmpH = height2 * fullOmniRangePercentage;
      }
      auto newW = oldW * tmpH / height2;
      newW += width2;

      auto tmpW = std::sqrt(width * newW - newW * newW);
      auto newH = oldH * width2 / tmpW;
      newH += height2;

      auto iNewH = std::lround(newH);
      auto iNewW = std::lround(newW);

      if (iNewH < 0 || iNewH >= height) {
        continue;
      }

      auto newPP = iNewH * width + iNewW;

      mapERP2Cassini(h, w) = newPP;
      if (isHole(iNewH, iNewW) == 1) {
        mapCassini2ERP(iNewH, iNewW) = oldPP;
      }
      if (D(h, w) != 0) {
        isHole(iNewH, iNewW) = 0; // 1 if hole
      }
    }
  }

  // analysis from top-left

  for (int h = 0; h < height; h++) {
    for (int w = 0; w < width; w++) {
      nonEmptyNeighborL(h, w) = mapCassini2ERP(h, w);
      if (isHole(h, w) != 0) {
        if (w > 0) {
          nonEmptyNeighborL(h, w) = nonEmptyNeighborL(h, w - 1);
        } else {
          nonEmptyNeighborL(h, w) = -1;
        }
      }

    } // w
  }   // h

  // analysis from bottom-right

  for (int h = height - 1; h >= 0; h--) {
    for (int w = width - 1; w >= 0; w--) {
      nonEmptyNeighborR(h, w) = mapCassini2ERP(h, w);
      if (isHole(h, w) != 0) {
        if (w < width - 1) {
          nonEmptyNeighborR(h, w) = nonEmptyNeighborR(h, w + 1);
        } else {
          nonEmptyNeighborR(h, w) = -1;
        }
      }

    } // w
  }   // h

  // inpainting

  perform2WayInpainting(yuvd, InpaintingType::omnidirectional, nonEmptyNeighborL, nonEmptyNeighborR,
                        mapERP2Cassini);
}

void inpaintPerspectiveView(Common::Texture444Depth16Frame &yuvd) {
  auto &D = yuvd.second.getPlane(0);

  const int width = static_cast<int>(D.width());
  const int height = static_cast<int>(D.height());

  Common::Mat<int> nonEmptyNeighborL;
  nonEmptyNeighborL.resize(height, width);

  Common::Mat<int> nonEmptyNeighborR;
  nonEmptyNeighborR.resize(height, width);

  Common::Mat<int> nonEmptyNeighborT;
  nonEmptyNeighborT.resize(height, width);

  Common::Mat<int> nonEmptyNeighborB;
  nonEmptyNeighborB.resize(height, width);

  // analysis from top-left

  for (int h = 0; h < height; h++) {
    for (int w = 0; w < width; w++) {
      nonEmptyNeighborL(h, w) = w;

      if (D(h, w) == 0) {
        if (w > 0) {
          nonEmptyNeighborL(h, w) = nonEmptyNeighborL(h, w - 1);
        } else {
          nonEmptyNeighborL(h, w) = -1;
        }
      }
    }
  }

  // analysis from bottom-right

  for (int h = height - 1; h >= 0; h--) {
    for (int w = width - 1; w >= 0; w--) {
      nonEmptyNeighborR(h, w) = w;

      if (D(h, w) == 0) {
        if (w < width - 1) {
          nonEmptyNeighborR(h, w) = nonEmptyNeighborR(h, w + 1);
        } else {
          nonEmptyNeighborR(h, w) = -1;
        }
      }
    }
  }

  // horizontal inpainting

  perform2WayInpainting(yuvd, InpaintingType::vertical, nonEmptyNeighborL, nonEmptyNeighborR);

  // analysis from top-left

  for (int h = 0; h < height; h++) {
    for (int w = 0; w < width; w++) {
      nonEmptyNeighborT(h, w) = h;

      if (D(h, w) == 0) {
        if (h > 0) {
          nonEmptyNeighborT(h, w) = nonEmptyNeighborT(h - 1, w);
        } else {
          nonEmptyNeighborT(h, w) = -1;
        }
      }
    }
  }

  // analysis from bottom-right

  for (int h = height - 1; h >= 0; h--) {
    for (int w = width - 1; w >= 0; w--) {
      nonEmptyNeighborB(h, w) = h;

      if (D(h, w) == 0) {
        if (h < height - 1) {
          nonEmptyNeighborB(h, w) = nonEmptyNeighborB(h, w + 1);
        } else {
          nonEmptyNeighborB(h, w) = -1;
        }
      }
    }
  }

  // vertical inpainting

  perform2WayInpainting(yuvd, InpaintingType::horizontal, nonEmptyNeighborT, nonEmptyNeighborB);
}

void inplaceInpaint_impl(Common::Texture444Depth16Frame &yuvd,
                         const MivBitstream::ViewParams &meta) {
  fillVerticalCracks(yuvd);

  if (meta.ci.ci_cam_type() == MivBitstream::CiCamType::equirectangular) {
    const auto fullOmniRangePercentage =
        (meta.ci.ci_erp_phi_max() - meta.ci.ci_erp_phi_min()) / Common::fullCycle;
    inpaintOmnidirectionalView(yuvd, fullOmniRangePercentage);
  }

  inpaintPerspectiveView(yuvd);
}
} // namespace

Inpainter::Inpainter(const Common::Json & /*rootNode*/, const Common::Json & /*componentNode*/) {}

void Inpainter::inplaceInpaint(Common::Texture444Depth16Frame &viewport,
                               const MivBitstream::ViewParams &metadata) const {
  inplaceInpaint_impl(viewport, metadata);
}
} // namespace TMIV::Renderer
