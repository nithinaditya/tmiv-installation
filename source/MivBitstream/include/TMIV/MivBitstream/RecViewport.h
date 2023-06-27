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

#ifndef _TMIV_MIVBITSTREAM_RECVIEWPORT_H_
#define _TMIV_MIVBITSTREAM_RECVIEWPORT_H_

#include <TMIV/Common/Bitstream.h>
#include <optional>
#include <vector>

namespace TMIV::MivBitstream {
// 23090-12: rec_viewport()
class RecViewport {
public:
  RecViewport() = default;
  explicit RecViewport(std::uint16_t value1, bool value2 = true);

  [[nodiscard]] auto rec_viewport_id() const noexcept -> std::uint16_t;
  [[nodiscard]] auto rec_viewport_cancel_flag() const noexcept -> bool;
  [[nodiscard]] auto rec_viewport_persistence_flag() const noexcept -> bool;
  [[nodiscard]] auto rec_viewport_center_view_flag() const noexcept -> bool;
  [[nodiscard]] auto rec_viewport_left_view_flag() const noexcept -> bool;
  [[nodiscard]] auto rec_viewport_pos_x() const noexcept -> float;
  [[nodiscard]] auto rec_viewport_pos_y() const noexcept -> float;
  [[nodiscard]] auto rec_viewport_pos_z() const noexcept -> float;
  [[nodiscard]] auto rec_viewport_quat_x() const noexcept -> float;
  [[nodiscard]] auto rec_viewport_quat_y() const noexcept -> float;
  [[nodiscard]] auto rec_viewport_quat_z() const noexcept -> float;
  [[nodiscard]] auto rec_viewport_hor_range() const noexcept -> float;
  [[nodiscard]] auto rec_viewport_ver_range() const noexcept -> float;

  constexpr auto rec_viewport_id(const std::uint16_t value) noexcept -> auto &;
  constexpr auto rec_viewport_cancel_flag(const bool value) noexcept -> auto &;
  constexpr auto rec_viewport_persistence_flag(const bool value) noexcept -> auto &;
  constexpr auto rec_viewport_center_view_flag(const bool value) noexcept -> auto &;
  constexpr auto rec_viewport_left_view_flag(const bool value) noexcept -> auto &;
  constexpr auto rec_viewport_pos_x(const float value) noexcept -> auto &;
  constexpr auto rec_viewport_pos_y(const float value) noexcept -> auto &;
  constexpr auto rec_viewport_pos_z(const float value) noexcept -> auto &;
  constexpr auto rec_viewport_quat_x(const float value) noexcept -> auto &;
  constexpr auto rec_viewport_quat_y(const float value) noexcept -> auto &;
  constexpr auto rec_viewport_quat_z(const float value) noexcept -> auto &;
  constexpr auto rec_viewport_hor_range(const float value) noexcept -> auto &;
  constexpr auto rec_viewport_ver_range(const float value) noexcept -> auto &;

  friend auto operator<<(std::ostream &stream, const RecViewport &x) -> std::ostream &;

  auto operator==(const RecViewport &other) const noexcept -> bool;
  auto operator!=(const RecViewport &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> RecViewport;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  std::uint16_t m_rec_viewport_id{};
  bool m_rec_viewport_cancel_flag{};
  std::optional<bool> m_rec_viewport_persistence_flag{};
  std::optional<bool> m_rec_viewport_center_view_flag{};
  std::optional<bool> m_rec_viewport_left_view_flag{};
  std::optional<float> m_rec_viewport_pos_x{};
  std::optional<float> m_rec_viewport_pos_y{};
  std::optional<float> m_rec_viewport_pos_z{};
  std::optional<float> m_rec_viewport_quat_x{};
  std::optional<float> m_rec_viewport_quat_y{};
  std::optional<float> m_rec_viewport_quat_z{};
  std::optional<float> m_rec_viewport_hor_range{};
  std::optional<float> m_rec_viewport_ver_range{};
};
} // namespace TMIV::MivBitstream

#include "RecViewport.hpp"

#endif
