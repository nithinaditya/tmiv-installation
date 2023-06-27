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

#ifndef _TMIV_RENDERER_RASTERIZER_H_
#error "Include the .h, not the .hpp"
#endif

#include "blend.h"
#include <TMIV/Common/Common.h>

#include <cmath>
#include <future>
#include <thread>

namespace TMIV::Renderer {
namespace detail {
// Calculate a number of strips that should ensure there is enough work but
// avoids having too many triangles in multiple strips
//
// Example: 8 hyper cores, 2048 rows ==> 128 strips of 16 rows each
inline auto numStrips(int rows) -> int {
  const double hw = std::thread::hardware_concurrency();
  const int maximum = (rows + 3) / 4;
  if (maximum <= hw) {
    return maximum;
  }
  return static_cast<int>(std::lround(sqrt(hw * maximum)));
}

template <typename M0, typename... M>
auto fetchAttributes(int index, const std::tuple<M0, M...> &attributes) {
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

inline auto fetchAttributes(int /* index */, const std::tuple<> &
                            /* attributes */) -> std::tuple<> {
  return {};
}
} // namespace detail

template <typename... T>
Rasterizer<T...>::Rasterizer(Pixel pixel, Common::Vec2i size)
    : Rasterizer{pixel, size, detail::numStrips(size.y())} {}

template <typename... T>
Rasterizer<T...>::Rasterizer(Pixel pixel, Common::Vec2i size, int numStrips)
    : m_pixel{pixel}, m_size{static_cast<size_t>(size.y()), static_cast<size_t>(size.x())} {
  assert(size.x() >= 0 && size.y() >= 0);
  assert(numStrips > 0);
  m_strips.reserve(numStrips);
  for (int n = 0; n < numStrips; ++n) {
    const auto i1 = size.y() * n / numStrips;
    const auto i2 = size.y() * (n + 1) / numStrips;
    auto accumulator =
        std::vector<Accumulator>{static_cast<size_t>(i2 - i1) * static_cast<size_t>(size.x())};
    m_strips.push_back({i1, i2, size.x(), {}, std::move(accumulator)});
  }
  m_dk_di = static_cast<float>(numStrips) / static_cast<float>(size.y());
}

template <typename... T>
void Rasterizer<T...>::submit(const ImageVertexDescriptorList &vertices, AttributeMaps attributes,
                              const TriangleDescriptorList &triangles) {
  m_batches.push_back(Batch{vertices, std::move(attributes)});
  for (auto &strip : m_strips) {
    strip.batches.emplace_back();
  }
  for (auto triangle : triangles) {
    submitTriangle(triangle, m_batches.back());
  }
}

template <typename... T> void Rasterizer<T...>::run() {
  std::vector<std::future<void>> work;
  work.reserve(m_strips.size());

  // Launch all work
  for (auto &strip : m_strips) {
    work.push_back(std::async( // Strips in parallel
        [this, &strip]() {
          for (size_t i = 0; i < m_batches.size(); ++i) { // Batches in sequence
            for (auto triangle : strip.batches[i]) {
              rasterTriangle(triangle, m_batches[i], strip);
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

template <typename... T> auto Rasterizer<T...>::depth() const -> Common::Mat<float> {
  Common::Mat<float> matrix(m_size);
  auto i_matrix = std::begin(matrix);
  visit([&i_matrix](const Value &x) {
    *i_matrix++ = x.depth();
    return true;
  });
  assert(i_matrix == std::end(matrix));
  return matrix;
}

template <typename... T> auto Rasterizer<T...>::normDisp() const -> Common::Mat<float> {
  Common::Mat<float> matrix(m_size);
  auto i_matrix = std::begin(matrix);
  visit([&i_matrix](const Value &x) {
    *i_matrix++ = x.normDisp;
    return true;
  });
  assert(i_matrix == std::end(matrix));
  return matrix;
}

template <typename... T> auto Rasterizer<T...>::normWeight() const -> Common::Mat<float> {
  Common::Mat<float> matrix(m_size);
  auto i_matrix = std::begin(matrix);
  visit([&i_matrix](const Value &x) {
    *i_matrix++ = x.normWeight;
    return true;
  });
  assert(i_matrix == std::end(matrix));
  return matrix;
}

template <typename... T>
template <size_t I>
[[nodiscard]] auto Rasterizer<T...>::attribute() const
    -> Common::Mat<std::tuple_element_t<I, Attributes>> {
  Common::Mat<std::tuple_element_t<I, Attributes>> matrix(m_size);
  auto i_matrix = std::begin(matrix);
  visit([&i_matrix](const Value &x) {
    *i_matrix++ = std::get<I>(x.attributes());
    return true;
  });
  assert(i_matrix == std::end(matrix));
  return matrix;
}

template <typename... T>
template <class Visitor>
void Rasterizer<T...>::visit(Visitor visitor) const {
  if (!m_batches.empty()) {
    throw Exception{"The Rasterizer does not allow frame buffer access when "
                    "work is queued."};
  }
  for (const auto &strip : m_strips) {
    for (const auto &accumulator : strip.matrix) {
      if (!visitor(m_pixel.average(accumulator))) {
        return;
      }
    }
  }
}

template <typename... T>
void Rasterizer<T...>::submitTriangle(TriangleDescriptor descriptor, const Batch &batch) {
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
namespace fixed_point {
using intfp = std::int_fast32_t;
using Vec2fp = Common::stack::Vec2<intfp>;

constexpr const auto bits = intfp{4};
constexpr const auto eps = intfp{1};
constexpr const auto one = eps << bits;
constexpr const auto half = one / intfp{2};

inline auto fixed(float x) -> intfp {
  using std::ldexp;
  return static_cast<intfp>(std::floor(0.5F + ldexp(x, bits)));
}
inline auto fixed(Common::Vec2f v) -> Vec2fp { return {fixed(v.x()), fixed(v.y())}; }
inline auto fixed(int x) -> intfp { return static_cast<intfp>(x) << bits; }
inline auto fpfloor(intfp x) -> int { return static_cast<int>(x >> bits); }
inline auto fpceil(intfp x) -> int { return fpfloor(x + one - eps); }
} // namespace fixed_point

template <typename... T>
void Rasterizer<T...>::rasterTriangle(TriangleDescriptor descriptor, const Batch &batch,
                                      Strip &strip) {
  using namespace fixed_point;
  using std::ldexp;
  using std::max;
  using std::min;

  const auto n0 = descriptor.indices[0];
  const auto n1 = descriptor.indices[1];
  const auto n2 = descriptor.indices[2];

  // Image coordinate within strip
  const auto stripOffset = Vec2fp{0, fixed(strip.i1)};
  const auto uv0 = fixed(batch.vertices[n0].position) - stripOffset;
  const auto uv1 = fixed(batch.vertices[n1].position) - stripOffset;
  const auto uv2 = fixed(batch.vertices[n2].position) - stripOffset;

  // Determine triangle bounding box
  const auto u1 = std::max(0, fpfloor(std::min({uv0.x(), uv1.x(), uv2.x()})));
  const auto u2 = std::min(strip.cols, 1 + fpceil(std::max({uv0.x(), uv1.x(), uv2.x()})));
  if (u1 >= u2) {
    return; // Cull
  }
  const auto v1 = std::max(0, fpfloor(std::min({uv0.y(), uv1.y(), uv2.y()})));
  const auto v2 = std::min(strip.rows(), 1 + fpceil(std::max({uv0.y(), uv1.y(), uv2.y()})));
  if (v1 >= v2) {
    return; // Cull
  }

  // Determine (unclipped) parallelogram area
  const auto area =
      (uv1.y() - uv2.y()) * (uv0.x() - uv2.x()) + (uv2.x() - uv1.x()) * (uv0.y() - uv2.y());
  if (area <= 0) {
    return; // Cull
  }
  const auto area_f = ldexp(static_cast<float>(area), -2 * bits);
  const auto inv_area = 1.F / static_cast<float>(area);

  // Calculate feature values for determining blending weights
  const auto stretching = 0.5F * area_f / descriptor.area;
  const auto rayAngle = (1 / 3.F) * (batch.vertices[n0].rayAngle + batch.vertices[n1].rayAngle +
                                     batch.vertices[n2].rayAngle);

  // Fetch normalized disparity values (diopters)
  const auto d0 = 1.F / batch.vertices[n0].depth;
  const auto d1 = 1.F / batch.vertices[n1].depth;
  const auto d2 = 1.F / batch.vertices[n2].depth;

  // Fetch multiple attributes (e.g. color)
  const auto a0 = detail::fetchAttributes(n0, batch.attributes);
  const auto a1 = detail::fetchAttributes(n1, batch.attributes);
  const auto a2 = detail::fetchAttributes(n2, batch.attributes);

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

      const auto w0 = inv_area * static_cast<float>(X0);
      const auto w1 = inv_area * static_cast<float>(X1);
      const auto w2 = inv_area * static_cast<float>(X2);

      // Barycentric interpolation of normalized disparity and attributes
      // (e.g. color)
      const auto d = w0 * d0 + w1 * d1 + w2 * d2;
      const auto a = blendAttributes(w0, a0, w1, a1, w2, a2);

      // Blend pixel
      assert(v * strip.cols + u < static_cast<int>(strip.matrix.size()));
      auto &P = strip.matrix[v * strip.cols + u];

      auto p = m_pixel.construct(a, d, rayAngle, stretching);
      if (X0 == 0 || X1 == 0 || X2 == 0) {
        // Count edge points half assuming there is an adjacent triangle
        p.normWeight *= 0.5F;
      }
      P = m_pixel.blend(P, p);
    }
  }
}

template <typename... T> void Rasterizer<T...>::clearBatches() {
  for (auto &strip : m_strips) {
    strip.batches.clear();
  }
  m_batches.clear();
}
} // namespace TMIV::Renderer
