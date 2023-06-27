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

#ifndef _TMIV_MIVBITSTREAM_SEIRBSP_H_
#define _TMIV_MIVBITSTREAM_SEIRBSP_H_

#include <TMIV/Common/Bitstream.h>

#include <vector>

namespace TMIV::MivBitstream {
enum class PayloadType : std::uint16_t {
  buffering_period,
  atlas_frame_timing,
  filler_payload,
  user_data_registered_itu_t_t35,
  user_data_unregistered,
  recovery_point,
  no_display,
  time_code,
  sei_manifest,
  sei_prefix_indication,
  active_sub_bitstreams,
  component_codec_mapping,
  scene_object_information,
  object_label_information,
  patch_information,
  volumetric_rectangle_information,
  atlas_object_association,
  viewport_camera_parameters,
  viewport_position,
  packed_independent_regions,
  attribute_transformation_params = 64, // V-PCC
  occupancy_synthesis,
  geometry_smoothing,
  attribute_smoothing,
  viewing_space = 128, // MIV
  viewing_space_handling,
  geometry_upscaling_parameters,
  rec_viewport // TODO(christoph_bachhuber) remove once removed from part 12 specs
};

auto operator<<(std::ostream &stream, PayloadType pt) -> std::ostream &;

// 23090-5: sei_message()
class SeiMessage {
public:
  SeiMessage() = default;
  SeiMessage(PayloadType pt, std::string payload);

  [[nodiscard]] auto payloadType() const noexcept -> PayloadType;
  [[nodiscard]] auto payloadSize() const noexcept -> std::size_t;
  [[nodiscard]] auto payload() const noexcept -> const std::string &;

  friend auto operator<<(std::ostream &stream, const SeiMessage &x) -> std::ostream &;

  auto operator==(const SeiMessage &other) const noexcept -> bool;
  auto operator!=(const SeiMessage &other) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream) -> SeiMessage;

  void encodeTo(std::ostream &stream) const;

private:
  PayloadType m_payloadType{};
  std::string m_payload;
};

// 23090-5: sei_rbsp()
class SeiRBSP {
public:
  SeiRBSP() = default;
  explicit SeiRBSP(std::vector<SeiMessage> messages);

  [[nodiscard]] constexpr auto messages() const noexcept -> const auto & { return m_messages; }
  constexpr auto messages() noexcept -> auto & { return m_messages; }

  friend auto operator<<(std::ostream &stream, const SeiRBSP &x) -> std::ostream &;

  auto operator==(const SeiRBSP &other) const noexcept -> bool;
  auto operator!=(const SeiRBSP &other) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream) -> SeiRBSP;

  void encodeTo(std::ostream &stream) const;

private:
  std::vector<SeiMessage> m_messages;
};
} // namespace TMIV::MivBitstream

#endif
