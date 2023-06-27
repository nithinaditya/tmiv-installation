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

#ifndef _TMIV_COMMON_FRAME_H_
#define _TMIV_COMMON_FRAME_H_

#include <TMIV/Common/Matrix.h>
#include <TMIV/Common/Vector.h>

#include <cassert>
#include <cstdint>
#include <istream>
#include <ostream>
#include <variant>

namespace TMIV::Common {
class YUV400P8 {};
class YUV400P10 {};
class YUV400P16 {};
class YUV420P8 {};
class YUV420P10 {};
class YUV420P16 {};
class YUV444P8 {};
class YUV444P10 {};
class YUV444P16 {};

namespace detail {
template <typename FORMAT> struct PixelFormatHelper {};
} // namespace detail

template <typename FORMAT> constexpr auto neutralColor() {
  return detail::PixelFormatHelper<FORMAT>::neutralColor();
}

template <typename FORMAT> class Frame {
public:
  using base_type = typename detail::PixelFormatHelper<FORMAT>::base_type;
  using plane_type = heap::Matrix<base_type>;

private:
  static constexpr auto numberOfPlanes = detail::PixelFormatHelper<FORMAT>::numberOfPlanes;
  static constexpr auto bitDepth = detail::PixelFormatHelper<FORMAT>::bitDepth;
  int32_t m_width{};
  int32_t m_height{};
  std::array<plane_type, numberOfPlanes> m_planes{};

public:
  Frame() = default;
  explicit Frame(int32_t w, int32_t h);

  [[nodiscard]] auto empty() const noexcept;

  void resize(int32_t w, int32_t h);

  [[nodiscard]] auto getPlanes() -> auto &;
  [[nodiscard]] auto getPlanes() const -> const auto &;
  [[nodiscard]] auto getPlane(int index) const -> const auto &;
  [[nodiscard]] auto getPlane(int index) -> auto &;
  [[nodiscard]] auto getWidth() const;
  [[nodiscard]] auto getHeight() const;
  [[nodiscard]] auto getSize() const;
  [[nodiscard]] auto getMemorySize() const;
  [[nodiscard]] auto getDiskSize() const;
  [[nodiscard]] static constexpr auto getNumberOfPlanes();
  [[nodiscard]] static constexpr auto getBitDepth();

  void read(std::istream &stream);
  void dump(std::ostream &stream) const;

  // Reset all samples to zero
  // NOTE(BK): samples are already set to zero on construction
  void fillZero();

  // Set all samples to the neutral color
  void fillNeutral();

  // Set all samples to one
  void fillOne();

  // Set invalid samples to the neutral color
  template <typename OTHER_FORMAT, typename = std::enable_if<std::is_same_v<FORMAT, YUV444P10>>>
  void fillInvalidWithNeutral(const Frame<OTHER_FORMAT> &depth);

  [[nodiscard]] static constexpr auto neutralColor();
};

auto yuv420p(const Frame<YUV444P8> &frame) -> Frame<YUV420P8>;
auto yuv420p(const Frame<YUV444P10> &frame) -> Frame<YUV420P10>;
auto yuv420p(const Frame<YUV444P16> &frame) -> Frame<YUV420P16>;

auto yuv444p(const Frame<YUV420P8> &frame) -> Frame<YUV444P8>;
auto yuv444p(const Frame<YUV420P10> &frame) -> Frame<YUV444P10>;
auto yuv444p(const Frame<YUV420P16> &frame) -> Frame<YUV444P16>;

// A type that can carry a large variation of possible frame types
//
// TODO(BK): Consider using AnyFrame for IO library
struct AnyFrame {
  // Convert to any specific format
  template <typename FORMAT> auto as() const -> Frame<FORMAT>;

  static constexpr auto maxPlanes = 4;
  std::array<Mat<uint32_t>, maxPlanes> planes{};
  std::array<uint8_t, maxPlanes> bitdepth{};
};
} // namespace TMIV::Common

#include "Frame.hpp"

namespace TMIV::Common {
using TextureFrame = Frame<YUV420P10>;
using Texture444Frame = Frame<YUV444P10>; // The renderer uses 4:4:4 internally
using Depth10Frame = Frame<YUV400P10>;    // Decoder side
using Depth16Frame = Frame<YUV400P16>;    // Encoder side
using Occupancy10Frame = Frame<YUV400P10>;
using Transparency10Frame = Frame<YUV400P10>;
using Mask = Frame<YUV400P8>;
using BlockToPatchMap = Frame<YUV400P16>;
const auto unusedPatchId = UINT16_MAX;
using EntityMap = Frame<YUV400P16>;

template <typename FORMAT> struct TextureDepthFrame {
  TextureFrame texture;
  Frame<FORMAT> depth;
  EntityMap entities{};
  Occupancy10Frame occupancy{};
  Transparency10Frame transparency{};

  TextureDepthFrame() = default;
  TextureDepthFrame(TextureFrame texture_, Frame<FORMAT> depth_)
      : texture{std::move(texture_)}, depth{std::move(depth_)} {}
  TextureDepthFrame(TextureFrame texture_, Frame<FORMAT> depth_, Occupancy10Frame occupancy_)
      : texture{std::move(texture_)}, depth{std::move(depth_)}, occupancy{std::move(occupancy_)} {}
  TextureDepthFrame(TextureFrame texture_, Frame<FORMAT> depth_, Occupancy10Frame occupancy_,
                    Transparency10Frame transparency_)
      : texture{std::move(texture_)}
      , depth{std::move(depth_)}
      , occupancy{std::move(occupancy_)}
      , transparency{std::move(transparency_)} {}
};
using TextureDepth10Frame = TextureDepthFrame<YUV400P10>;
using TextureDepth16Frame = TextureDepthFrame<YUV400P16>;
using Texture444Depth10Frame = std::pair<Texture444Frame, Depth10Frame>;
using Texture444Depth16Frame = std::pair<Texture444Frame, Depth16Frame>;

using EntityMapList = std::vector<EntityMap>;
template <typename FORMAT> using MVDFrame = std::vector<TextureDepthFrame<FORMAT>>;
using MVD10Frame = MVDFrame<YUV400P10>;
using MVD16Frame = MVDFrame<YUV400P16>;
using MaskList = std::vector<Mask>;

// Expand a YUV 4:4:4 10-bit texture to packed 4:4:4 32-bit float texture with
// linear transfer and nearest interpolation for chroma
auto expandTexture(const Frame<YUV444P10> &inYuv) -> Mat<Vec3f>;

// Expand a YUV 4:2:0 10-bit texture to 32-bit float luma map with linear transfer
auto expandLuma(const Frame<YUV420P10> &inYuv) -> Mat<float>;

// Quantize a packed 4:4:4 32-bit float texture as YUV 4:4:4 10-bit texture with
// linear transfer and area interpolation for chroma
auto quantizeTexture(const Mat<Vec3f> &in) -> Frame<YUV444P10>;
} // namespace TMIV::Common

#endif
