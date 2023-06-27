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

#ifndef _TMIV_RENDERER_MPIRASTERIZER_H_
#define _TMIV_RENDERER_MPIRASTERIZER_H_

#include "Engine.h"

#include <array>

namespace TMIV::Renderer {

template <typename... T> using PixelAttributes = std::tuple<T...>;

template <typename... T> class MpiRasterizer {
public:
  using Exception = std::logic_error;
  using AttributeMaps = std::tuple<std::vector<T>...>;
  using FragmentShader =
      std::function<void(const ViewportPosition2D &, const std::array<float, 3> &,
                         const std::array<PixelAttributes<T...>, 3> &)>;

  // Construct a rasterizer with specified size.
  // The number of strips for concurrent processing is
  // chosen based on image size and hardware concurrency.
  MpiRasterizer(Common::Vec2i size);

  // Construct a rasterizer with specified size and specify
  //  the number of strips for concurrent processing.
  MpiRasterizer(Common::Vec2i size, int numStrips);

  // Submit a batch of triangles
  //
  // The batch is stored within the Rasterizer for later processing.
  // Multiple batches may be submitted sequentially.
  void submit(const ImageVertexDescriptorList &vertices, AttributeMaps attributes,
              const TriangleDescriptorList &triangles);

  // Raster all submitted batches
  void run(const FragmentShader &fragmentShader);

private:
  struct Strip {
    // Strip dimensions
    const int i1{};
    const int i2{};
    const int cols{};

    [[nodiscard]] constexpr auto rows() const -> int { return i2 - i1; }

    // Batches of triangles to be processed
    std::vector<TriangleDescriptorList> batches;
  };

  // Information of each batch that is shared between strips
  //
  // Note that m_batches.size() == m_strips[].batches.size()
  struct Batch {
    const ImageVertexDescriptorList vertices;
    const AttributeMaps attributes;
  };

  using Size = Common::Mat<float>::tuple_type;

  void submitTriangle(TriangleDescriptor descriptor, const Batch &batch);
  void rasterTriangle(TriangleDescriptor descriptor, const Batch &batch, Strip &strip,
                      const FragmentShader &fragmentShader);
  void clearBatches();

  const Size m_size{};
  float m_dk_di{}; // i for row, j for column and k for strip index
  std::vector<Strip> m_strips;
  std::vector<Batch> m_batches;
};
} // namespace TMIV::Renderer

#include "MpiRasterizer.hpp"

#endif
