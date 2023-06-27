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

#ifndef _TMIV_RENDERER_BLEND_H_
#define _TMIV_RENDERER_BLEND_H_

#include <tuple>

namespace TMIV::Renderer {
// Blend two arithmetic tensors of fixed size
template <typename T> static auto blendValues(float w_a, T a, float w_b, T b) -> T;

// Blend three arithmetic tensors of fixed size
template <typename T> auto blendValues(float w_a, T a, float w_b, T b, float w_c, T c) -> T;

// Blend the attributes of two pixels
template <typename T0, typename... T>
auto blendAttributes(float w_a, const std::tuple<T0, T...> &a, float w_b,
                     const std::tuple<T0, T...> &b) -> std::tuple<T0, T...>;

inline auto blendAttributes(float /* w_a */, const std::tuple<> & /* a */, float /* w_b */,
                            const std::tuple<> & /* b */) {
  return std::tuple{};
}

// Blend the attributes of three pixels
template <typename T0, typename... T>
auto blendAttributes(float w_a, const std::tuple<T0, T...> &a, float w_b,
                     const std::tuple<T0, T...> &b, float w_c, const std::tuple<T0, T...> &c)
    -> std::tuple<T0, T...>;

inline auto blendAttributes(float /* w_a */, const std::tuple<> & /* a */, float /* w_b */,
                            const std::tuple<> & /* b */, float /* w_c */,
                            const std::tuple<> & /* c */) {
  return std::tuple{};
}
} // namespace TMIV::Renderer

#include "blend.hpp"

#endif
