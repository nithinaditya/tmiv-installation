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
#error "Include the .h, not the .hpp"
#endif

#include <TMIV/Common/Common.h>

#include <algorithm>
#include <cmath>
#include <future>
#include <thread>

namespace TMIV::Renderer {
namespace detail {
// Calculate a number of strips that should ensure there is enough work but
// avoids having too many triangles in multiple strips
//
// Example: 8 hyper cores, 2048 rows ==> 128 strips of 16 rows each
inline auto MpiNumStrips(int rows) -> int {
  const double hw = std::thread::hardware_concurrency();
  const int maximum = (rows + 3) / 4;
  if (maximum <= hw) {
    return maximum;
  }
  return static_cast<int>(std::lround(sqrt(hw * maximum)));
}

template <typename M0, typename... M>
auto MpiFetchAttributes(int index, const std::tuple<M0, M...> &attributes) {
  std::tuple<typename M0::value_type, typename M::value_type...> result;
  std::get<0>(result) = std::get<0>(attributes)[index];
  if constexpr (sizeof...(M) >= 1) {
    std::get<1>(result) = std::get<1>(attributes)[index];
  }
  if constexpr (sizeof...(M) >= 2) {
    std::get<2>(result) = std::get<2>(attributes)[index];
  }
  if constexpr (sizeof...(M) >= 3) {
    std::get<3>(result) = std::get<3>(attributes)[index];
  }
  static_assert(sizeof...(M) <= 3);
  return result;
}

inline auto MpiFetchAttributes(int /* index */, const std::tuple<> &
                               /* attributes */) -> std::tuple<> {
  return {};
}
} // namespace detail

template <typename... T>
MpiRasterizer<T...>::MpiRasterizer(Common::Vec2i size)
    : MpiRasterizer{size, detail::MpiNumStrips(size.y())} {}

template <typename... T>
MpiRasterizer<T...>::MpiRasterizer(Common::Vec2i size, int numStrips)
    : m_size{unsigned(size.y()), unsigned(size.x())} {
  assert(size.x() >= 0 && size.y() >= 0);
  assert(numStrips > 0);
  m_strips.reserve(numStrips);
  for (int n = 0; n < numStrips; ++n) {
    const int i1 = size.y() * n / numStrips;
    const int i2 = size.y() * (n + 1) / numStrips;
    m_strips.push_back({i1, i2, size.x(), {}});
  }
  m_dk_di = static_cast<float>(numStrips) / static_cast<float>(size.y());
}

template <typename... T>
void MpiRasterizer<T...>::submit(const ImageVertexDescriptorList &vertices,
                                 AttributeMaps attributes,
                                 const TriangleDescriptorList &triangles) {
  m_batches.push_back(Batch{vertices, move(attributes)});
  for (auto &strip : m_strips) {
    strip.batches.emplace_back();
  }
  for (auto triangle : triangles) {
    submitTriangle(triangle, m_batches.back());
  }
}

template <typename... T> void MpiRasterizer<T...>::run(const FragmentShader &fragmentShader) {
  std::vector<std::future<void>> work;
  work.reserve(m_strips.size());

  // Launch all work
  for (auto &strip : m_strips) {
    work.push_back(std::async( // Strips in parallel
        [this, &strip, &fragmentShader]() {
          for (size_t i = 0; i < m_batches.size(); ++i) { // Batches in sequence
            for (auto triangle : strip.batches[i]) {
              rasterTriangle(triangle, m_batches[i], strip, fragmentShader);
            }
          }
        }));
  }

  // Synchronize on completion
  for (auto &item : work) {
    item.get();
  }

  // Deallocate
  clearBatches();
}

template <typename... T>
void MpiRasterizer<T...>::submitTriangle(TriangleDescriptor descriptor, const Batch &batch) {
  const auto K = static_cast<int>(m_strips.size());
  auto k1 = K;
  auto k2 = 0;

  for (auto n : descriptor.indices) {
    const auto y = batch.vertices[n].position.y();
    if (std::isnan(y)) {
      return;
    }
    const auto k = y * m_dk_di;
    k1 = std::min(k1, static_cast<int>(std::floor(k)));
    k2 = std::max(k2, static_cast<int>(std::ceil(k)) + 1);
  }

  // Cull
  k1 = std::max(0, k1);
  k2 = std::min(K, k2);

  for (int k = k1; k < k2; ++k) {
    m_strips[k].batches.back().push_back(descriptor);
  }
}

// Switch to fixed-point vertices to correctly handle edge points
namespace mpi_fixed_point {
using intfp = std::int_fast32_t;
using Vec2fp = Common::stack::Vec2<intfp>;

constexpr const auto bits = intfp{4};
constexpr const auto eps = intfp{1};
constexpr const auto one = eps << bits;
constexpr const auto half = one / intfp{2};

inline auto fixed(float x) -> intfp {
  using std::ldexp;
  return static_cast<intfp>(static_cast<int>(std::floor(0.5F + ldexp(x, bits))));
}
inline auto fixed(Common::Vec2f v) -> Vec2fp { return {fixed(v.x()), fixed(v.y())}; }
inline auto fixed(int x) -> intfp { return static_cast<intfp>(x) << bits; }
inline auto fpfloor(intfp x) -> int { return static_cast<int>(x >> bits); }
inline auto fpceil(intfp x) -> int { return fpfloor(x + one - eps); }
inline void swap_vec2fp(Vec2fp &p1, Vec2fp &p2) {
  auto p3 = p1;
  p1 = p2;
  p2 = p3;
}
} // namespace mpi_fixed_point

template <typename... T>
void MpiRasterizer<T...>::rasterTriangle(TriangleDescriptor descriptor, const Batch &batch,
                                         Strip &strip, const FragmentShader &fragmentShader) {
  using namespace mpi_fixed_point;
  using std::ldexp;
  using std::max;
  using std::min;

  const auto n0 = descriptor.indices[0];
  const auto n1 = descriptor.indices[1];
  const auto n2 = descriptor.indices[2];

  // Image coordinate within strip
  const auto stripOffset = Vec2fp{0, fixed(strip.i1)};
  const Vec2fp uv0 = fixed(batch.vertices[n0].position) - stripOffset;
  Vec2fp uv1 = fixed(batch.vertices[n1].position) - stripOffset;
  Vec2fp uv2 = fixed(batch.vertices[n2].position) - stripOffset;

  // Determine triangle bounding box
  const auto u1 = max(0, fpfloor(min({uv0.x(), uv1.x(), uv2.x()})));
  const auto u2 = min(strip.cols, 1 + fpceil(max({uv0.x(), uv1.x(), uv2.x()})));

  if (u1 >= u2) {
    return; // Cull
  }
  const auto v1 = max(0, fpfloor(min({uv0.y(), uv1.y(), uv2.y()})));
  const auto v2 = min(strip.rows(), 1 + fpceil(max({uv0.y(), uv1.y(), uv2.y()})));

  if (v1 >= v2) {
    return; // Cull
  }

  // Determine (unclipped) parallelogram area
  auto area = (uv1.y() - uv2.y()) * (uv0.x() - uv2.x()) + (uv2.x() - uv1.x()) * (uv0.y() - uv2.y());

  // Fetch multiple attributes (e.g. color)
  auto a1 = detail::MpiFetchAttributes(n1, batch.attributes);
  auto a2 = detail::MpiFetchAttributes(n2, batch.attributes);

  // Switch to positively oriented triangle if necessary
  if (area < 0) {
    area = std::abs(area);
    swap_vec2fp(uv1, uv2);
    a1.swap(a2);
  }

  const std::array<decltype(a1), 3> pixelAttributes{
      detail::MpiFetchAttributes(n0, batch.attributes), a1, a2};

  const auto inv_area = 1.F / static_cast<float>(area);

  // For each pixel in the bounding box
  for (int v = v1; v < v2; ++v) {
    for (int u = u1; u < u2; ++u) {
      // Calculate the Barycentric coordinate of the pixel center (x +
      // 1/2, y + 1/2)
      const auto X0 = (uv1.y() - uv2.y()) * (fixed(u) - uv2.x() + half) +
                      (uv2.x() - uv1.x()) * (fixed(v) - uv2.y() + half);
      if (X0 < 0) {
        continue;
      }
      const auto X1 = (uv2.y() - uv0.y()) * (fixed(u) - uv2.x() + half) +
                      (uv0.x() - uv2.x()) * (fixed(v) - uv2.y() + half);
      if (X1 < 0) {
        continue;
      }
      const auto X2 = area - X0 - X1;
      if (X2 < 0) {
        continue;
      }

      const std::array<float, 3> weights{inv_area * static_cast<float>(X0),
                                         inv_area * static_cast<float>(X1),
                                         inv_area * static_cast<float>(X2)};

      const ViewportPosition2D viewport{u, strip.i1 + v};

      fragmentShader(viewport, weights, pixelAttributes);
    }
  }
}

template <typename... T> void MpiRasterizer<T...>::clearBatches() {
  for (auto &strip : m_strips) {
    strip.batches.clear();
  }
  m_batches.clear();
}
} // namespace TMIV::Renderer
