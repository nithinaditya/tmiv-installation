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

#ifndef _TMIV_PACKER_MAXRECTPIP_H_
#define _TMIV_PACKER_MAXRECTPIP_H_

#include <TMIV/Packer/Cluster.h>
#include <list>

namespace TMIV::Packer {
class MaxRectPiP {
public:
  class Output {
  protected:
    int m_x = 0;
    int m_y = 0;
    bool m_isRotated = false;

  public:
    void set(int x, int y, bool isRotated) {
      m_x = x;
      m_y = y;
      m_isRotated = isRotated;
    }
    [[nodiscard]] auto x() const -> int { return m_x; }
    [[nodiscard]] auto y() const -> int { return m_y; }
    [[nodiscard]] auto isRotated() const -> bool { return m_isRotated; }
  };
  using OccupancyMap = TMIV::Common::Mat<std::uint8_t>;

private:
  class Rectangle {
  protected:
    int m_x0 = 0;
    int m_y0 = 0;
    int m_x1 = 0;
    int m_y1 = 0;

  public:
    Rectangle() = default;
    Rectangle(int x0, int y0, int x1, int y1) : m_x0(x0), m_y0(y0), m_x1(x1), m_y1(y1) {}
    [[nodiscard]] auto left() const -> int { return m_x0; }
    [[nodiscard]] auto right() const -> int { return m_x1; }
    [[nodiscard]] auto bottom() const -> int { return m_y0; }
    [[nodiscard]] auto top() const -> int { return m_y1; }
    [[nodiscard]] auto width() const -> int { return (m_x1 - m_x0 + 1); }
    [[nodiscard]] auto height() const -> int { return (m_y1 - m_y0 + 1); }
    [[nodiscard]] auto split(int w, int h) const -> std::vector<Rectangle>;
    [[nodiscard]] auto remove(const Rectangle &r) const -> std::vector<Rectangle>;
    [[nodiscard]] auto isInside(const Rectangle &r) const -> bool;
    [[nodiscard]] auto getShortSideFitScore(int w, int h) const -> float;
    [[nodiscard]] auto getArea() const -> int { return (width() * height()); }
  };

protected:
  int m_width = 0, m_height = 0, m_alignment = 0;
  std::list<Rectangle> m_F;
  bool m_pip = true;
  OccupancyMap m_occupancyMap;

public:
  MaxRectPiP(int w, int h, int a, bool pip);
  auto push(const Cluster &c, const ClusteringMap &clusteringMap, Output &packerOutput) -> bool;

protected:
  void updateOccupancyMap(const Cluster &c, const ClusteringMap &clusteringMap,
                          const Output &packerOutput);
  auto pushInUsedSpace(int w, int h, bool isBasicView, Output &packerOutput) -> bool;
  auto pushInFreeSpace(int w, int h, bool isBasicView, Output &packerOutput) -> bool;
};
} // namespace TMIV::Packer

#endif
