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

#ifndef _TMIV_MIVBITSTREAM_VUIPARAMETERS_H_
#error "Include the .h, not the .hpp"
#endif

namespace TMIV::MivBitstream {
constexpr auto CoordinateSystemParameters::cas_forward_axis() const noexcept {
  return m_cas_forward_axis;
}

constexpr auto CoordinateSystemParameters::cas_delta_left_axis_minus1() const noexcept {
  return m_cas_delta_left_axis_minus1;
}

constexpr auto CoordinateSystemParameters::cas_forward_sign() const noexcept {
  return m_cas_forward_sign;
}

constexpr auto CoordinateSystemParameters::cas_left_sign() const noexcept {
  return m_cas_left_sign;
}

constexpr auto CoordinateSystemParameters::cas_up_sign() const noexcept { return m_cas_up_sign; }

constexpr auto CoordinateSystemParameters::cas_forward_axis(std::uint8_t value) noexcept -> auto & {
  m_cas_forward_axis = value;
  return *this;
}

constexpr auto CoordinateSystemParameters::cas_delta_left_axis_minus1(std::uint8_t value) noexcept
    -> auto & {
  m_cas_delta_left_axis_minus1 = value;
  return *this;
}

constexpr auto CoordinateSystemParameters::cas_forward_sign(bool value) noexcept -> auto & {
  m_cas_forward_sign = value;
  return *this;
}

constexpr auto CoordinateSystemParameters::cas_left_sign(bool value) noexcept -> auto & {
  m_cas_left_sign = value;
  return *this;
}

constexpr auto CoordinateSystemParameters::cas_up_sign(bool value) noexcept -> auto & {
  m_cas_up_sign = value;
  return *this;
}

constexpr auto CoordinateSystemParameters::isOmafCas() const noexcept {
  return cas_forward_axis() == 0 && cas_delta_left_axis_minus1() == 0 && cas_forward_sign() &&
         cas_left_sign() && cas_up_sign();
}

constexpr auto
CoordinateSystemParameters::operator==(const CoordinateSystemParameters &other) const noexcept {
  return cas_forward_axis() == other.cas_forward_axis() &&
         cas_delta_left_axis_minus1() == other.cas_delta_left_axis_minus1() &&
         cas_forward_sign() == other.cas_forward_sign() &&
         cas_left_sign() == other.cas_left_sign() && cas_up_sign() == other.cas_up_sign();
}

constexpr auto
CoordinateSystemParameters::operator!=(const CoordinateSystemParameters &other) const noexcept {
  return !operator==(other);
}

constexpr auto VuiParameters::vui_timing_info_present_flag() const noexcept {
  return m_vui_timing_info_present_flag;
}

constexpr auto VuiParameters::vui_bitstream_restriction_present_flag() const noexcept {
  return m_vui_bitstream_restriction_present_flag;
}

constexpr auto VuiParameters::vui_coordinate_system_parameters_present_flag() const noexcept {
  return m_vui_coordinate_system_parameters_present_flag;
}

constexpr auto VuiParameters::vui_unit_in_metres_flag() const noexcept {
  return m_vui_unit_in_metres_flag;
}

constexpr auto VuiParameters::vui_display_box_info_present_flag() const noexcept {
  return m_vui_display_box_info_present_flag;
}

constexpr auto VuiParameters::vui_anchor_point_present_flag() const noexcept {
  return m_vui_anchor_point_present_flag;
}

constexpr auto VuiParameters::vui_timing_info_present_flag(bool value) noexcept -> auto & {
  m_vui_timing_info_present_flag = value;
  return *this;
}

constexpr auto VuiParameters::vui_bitstream_restriction_present_flag(bool value) noexcept
    -> auto & {
  m_vui_bitstream_restriction_present_flag = value;
  return *this;
}

constexpr auto VuiParameters::vui_coordinate_system_parameters_present_flag(bool value) noexcept
    -> auto & {
  m_vui_coordinate_system_parameters_present_flag = value;
  return *this;
}

constexpr auto VuiParameters::vui_unit_in_metres_flag(bool value) noexcept -> auto & {
  m_vui_unit_in_metres_flag = value;
  return *this;
}

constexpr auto VuiParameters::vui_display_box_info_present_flag(bool value) noexcept -> auto & {
  m_vui_display_box_info_present_flag = value;
  return *this;
}

constexpr auto VuiParameters::vui_anchor_point_present_flag(bool value) noexcept -> auto & {
  m_vui_anchor_point_present_flag = value;
  return *this;
}
} // namespace TMIV::MivBitstream
