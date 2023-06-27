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

#include "MaxRectPiP.h"
#include <TMIV/Common/LinAlg.h>

namespace TMIV::Packer {
constexpr auto occupied = uint8_t{128};

////////////////////////////////////////////////////////////////////////////////
auto MaxRectPiP::Rectangle::split(int w, int h) const -> std::vector<MaxRectPiP::Rectangle> {
  std::vector<Rectangle> out;

  if (h < height()) {
    out.emplace_back(m_x0, m_y0 + h, m_x1, m_y1);
  }

  if (w < width()) {
    out.emplace_back(m_x0 + w, m_y0, m_x1, m_y1);
  }

  return out;
}

auto MaxRectPiP::Rectangle::remove(const Rectangle &r) const -> std::vector<MaxRectPiP::Rectangle> {
  std::vector<Rectangle> out;

  if (!((r.m_x1 <= m_x0) || (m_x1 <= r.m_x0) || (r.m_y1 <= m_y0) || (m_y1 <= r.m_y0))) {
    // Left part
    if (m_x0 < r.m_x0) {
      out.emplace_back(m_x0, m_y0, r.m_x0 - 1, m_y1);
    }

    // Right part
    if (r.m_x1 < m_x1) {
      out.emplace_back(r.m_x1 + 1, m_y0, m_x1, m_y1);
    }

    // Bottom part
    if (m_y0 < r.m_y0) {
      out.emplace_back(m_x0, m_y0, m_x1, r.m_y0 - 1);
    }

    // Top part
    if (r.m_y1 < m_y1) {
      out.emplace_back(m_x0, r.m_y1 + 1, m_x1, m_y1);
    }
  }

  return out;
}

auto MaxRectPiP::Rectangle::isInside(const Rectangle &r) const -> bool {
  return ((r.m_x0 <= m_x0) && (m_x0 <= r.m_x1) && (r.m_x0 <= m_x1) && (m_x1 <= r.m_x1) &&
          (r.m_y0 <= m_y0) && (m_y0 <= r.m_y1) && (r.m_y0 <= m_y1) && (m_y1 <= r.m_y1));
}

auto MaxRectPiP::Rectangle::getShortSideFitScore(int w, int h) const -> float {
  int dw = width() - w;
  int dh = height() - h;

  if ((0 <= dw) && (0 <= dh)) {
    return static_cast<float>(std::min(dw, dh));
  }
  { return std::numeric_limits<float>::max(); }
}

//////////////////////////////////////////////////////////////
MaxRectPiP::MaxRectPiP(int w, int h, int a, bool pip)
    : m_width(w), m_height(h), m_alignment(a), m_pip(pip) {
  // Maps
  auto wa = static_cast<unsigned>(w / a);
  auto ha = static_cast<unsigned>(h / a);

  m_occupancyMap.resize({ha, wa});
  std::fill(m_occupancyMap.begin(), m_occupancyMap.end(), uint8_t{});

  // Push full rectangle
  m_F.emplace_back(0, 0, w - 1, h - 1);
}

auto MaxRectPiP::push(const Cluster &c, const ClusteringMap &clusteringMap, Output &packerOutput)
    -> bool {
  int w = Common::align(c.width(), m_alignment);
  int h = Common::align(c.height(), m_alignment);

  if ((m_pip && pushInUsedSpace(w, h, c.isBasicView(), packerOutput)) ||
      pushInFreeSpace(w, h, c.isBasicView(), packerOutput)) {
    // Update occupancy map
    if (m_pip) {
      updateOccupancyMap(c, clusteringMap, packerOutput);
    }

    return true;
  }
  { return false; }
}

void MaxRectPiP::updateOccupancyMap(const Cluster &c, const ClusteringMap &clusteringMap,
                                    const MaxRectPiP::Output &packerOutput) {

  const auto &clusteringBuffer = clusteringMap.getPlane(0);
  bool isRotated = packerOutput.isRotated();
  int w = c.width();
  int h = c.height();
  int w_align = Common::align(c.width(), m_alignment);
  int h_align = Common::align(c.height(), m_alignment);
  // overflow
  Common::Vec2i patchOverflow = Common::Vec2i({c.jmin(), c.imin()}) +
                                Common::Vec2i({w_align, h_align}) - clusteringMap.getSize();

  // Step #0 (in atlas)
  Common::Vec2i q0 = {packerOutput.x(), packerOutput.y()};
  int XMin = q0.x() / m_alignment;
  int XLast = (q0.x() + (isRotated ? h : w) - 1) / m_alignment + 1;
  int YMin = q0.y() / m_alignment;
  int YLast = (q0.y() + (isRotated ? w : h) - 1) / m_alignment + 1;

  for (auto Y = YMin; Y < YLast; Y++) {
    std::fill(m_occupancyMap.row_begin(Y) + XMin, m_occupancyMap.row_begin(Y) + XLast, occupied);
  }

  // Step #1 (in projection)
  Common::Vec2i p0 = {c.jmin(), c.imin()};
  if (patchOverflow.x() > 0) {
    p0.x() -= patchOverflow.x();
  }
  if (patchOverflow.y() > 0) {
    p0.y() -= patchOverflow.y();
  }
  int xMin = std::max(0, p0.x());
  int xMax = p0.x() + w - 1;
  int yMin = std::max(0, p0.y());
  int yMax = p0.y() + h - 1;

  auto p2q = [isRotated, w_align, p0, q0](const Common::Vec2i p) {
    return isRotated ? (q0 + Common::Vec2i({p.y() - p0.y(), (w_align - 1) - (p.x() - p0.x())}))
                     : (q0 + (p - p0));
  };

  for (auto y = yMin; y <= yMax; y++) {
    for (auto x = xMin; x <= xMax; x++) {
      if (clusteringBuffer(y, x) == c.getClusterId()) {
        Common::Vec2i q =
            p2q(Common::Vec2i({static_cast<int>(x), static_cast<int>(y)})) / m_alignment;
        m_occupancyMap(q.y(), q.x()) = 0;
      }
    }
  }
}

auto MaxRectPiP::pushInUsedSpace(int w, int h, bool isBasicView, MaxRectPiP::Output &packerOutput)
    -> bool {
  int W = w / m_alignment;
  int H = h / m_alignment;

  auto isGoodCandidate = [this](int xmin, int xmax, int ymin, int ymax) -> bool {
    if ((xmax < static_cast<int>(m_occupancyMap.width())) &&
        (ymax < static_cast<int>(m_occupancyMap.height()))) {
      for (int y = ymin; y <= ymax; y++) {
        for (int x = xmin; x <= xmax; x++) {
          if (m_occupancyMap(y, x) != occupied) {
            return false;
          }
        }
      }

      return true;
    }
    { return false; }
  };

  for (auto Y = 0; Y < static_cast<int>(m_occupancyMap.height()); ++Y) {
    for (auto X = 0; X < static_cast<int>(m_occupancyMap.width()); ++X) {
      // Without Rotation
      if (isGoodCandidate(X, X + W - 1, Y, Y + H - 1)) {
        packerOutput.set(X * m_alignment, Y * m_alignment, false);
        return true;
      }

      // With Rotation
      if (!isBasicView && isGoodCandidate(X, X + H - 1, Y, Y + W - 1)) {
        packerOutput.set(X * m_alignment, Y * m_alignment, true);
        return true;
      }
    }
  }

  return false;
}

auto MaxRectPiP::pushInFreeSpace(int w, int h, bool isBasicView, MaxRectPiP::Output &packerOutput)
    -> bool {
  // Select best free rectangles that fit current patch (BSSF criterion)
  auto best_iter = m_F.cend();
  float best_score = std::numeric_limits<float>::max();
  bool best_rotation = false;

  for (auto iter = m_F.cbegin(); iter != m_F.cend(); iter++) {
    const Rectangle &r = *iter;
    float score_regular = r.getShortSideFitScore(w, h);
    float score_rotated = r.getShortSideFitScore(h, w);

    if ((isBasicView || score_regular <= score_rotated) && (score_regular < best_score)) {
      best_iter = iter;
      best_score = score_regular;
      best_rotation = false;
    } else if (!isBasicView && score_rotated < best_score) {
      best_iter = iter;
      best_score = score_rotated;
      best_rotation = true;
    }
  }

  if (best_iter == m_F.cend()) {
    return false;
  }

  // Update free rectangles
  Rectangle B(best_iter->left(), best_iter->bottom(),
              best_iter->left() + ((best_rotation ? h : w) - 1),
              best_iter->bottom() + ((best_rotation ? w : h) - 1));

  // Split current free rectangle
  std::vector<Rectangle> splitted = best_iter->split(B.width(), B.height());
  std::move(splitted.begin(), splitted.end(), back_inserter(m_F));

  m_F.erase(best_iter);

  // Intersection with existing free rectangles
  std::vector<std::list<Rectangle>::const_iterator> intersected;
  auto last = m_F.cend();

  for (auto iter = m_F.cbegin(); iter != last; iter++) {
    std::vector<Rectangle> intersecting = iter->remove(B);

    if (!intersecting.empty()) {
      intersected.push_back(iter);
      std::move(intersecting.begin(), intersecting.end(), back_inserter(m_F));
    }
  }

  for (const auto &iter : intersected) {
    m_F.erase(iter);
  }

  // Degenerated free rectangles
  std::vector<std::list<Rectangle>::const_iterator> degenerated;

  for (auto iter1 = m_F.cbegin(); iter1 != m_F.cend(); iter1++) {
    for (auto iter2 = m_F.cbegin(); iter2 != m_F.cend(); iter2++) {
      if ((iter1 != iter2) && (iter1->isInside(*iter2))) {
        degenerated.push_back(iter1);
        break;
      }
    }
  }

  for (const auto &iter : degenerated) {
    m_F.erase(iter);
  }

  // Update output
  packerOutput.set(B.left(), B.bottom(), best_rotation);

  return true;
}
} // namespace TMIV::Packer
