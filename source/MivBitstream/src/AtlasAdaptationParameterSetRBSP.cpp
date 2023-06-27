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

#include <TMIV/MivBitstream/AtlasAdaptationParameterSetRBSP.h>

#include <TMIV/Common/verify.h>

#include <cmath>

namespace TMIV::MivBitstream {
auto operator<<(std::ostream &stream, const AapsVpccExtension & /* x */) -> std::ostream & {
  stream << "aaps_vpcc_camera_parameters_present_flag=false\n";
  return stream;
}

auto AapsVpccExtension::decodeFrom(Common::InputBitstream &bitstream) -> AapsVpccExtension {
  const auto aaps_vpcc_camera_parameters_present_flag = bitstream.getFlag();
  LIMITATION(!aaps_vpcc_camera_parameters_present_flag);
  return {};
}

void AapsVpccExtension::encodeTo(Common::OutputBitstream &bitstream) {
  const auto aaps_vpcc_camera_parameters_present_flag = false;
  bitstream.putFlag(aaps_vpcc_camera_parameters_present_flag);
}

auto AapsMivExtension::vui_parameters() const noexcept -> const VuiParameters & {
  VERIFY_MIVBITSTREAM(aame_vui_params_present_flag());
  VERIFY_MIVBITSTREAM(m_vui_parameters.has_value());
  return *m_vui_parameters;
}

auto AapsMivExtension::vui_parameters(const VuiParameters &value) noexcept -> AapsMivExtension & {
  VERIFY_MIVBITSTREAM(aame_vui_params_present_flag());
  m_vui_parameters = value;
  return *this;
}

auto operator<<(std::ostream &stream, const AapsMivExtension &x) -> std::ostream & {
  stream << "aame_omaf_v1_compatible_flag=" << std::boolalpha << x.aame_omaf_v1_compatible_flag()
         << '\n';
  stream << "aame_vui_params_present_flag=" << std::boolalpha << x.aame_vui_params_present_flag()
         << '\n';
  if (x.aame_vui_params_present_flag()) {
    stream << x.vui_parameters();
  }
  return stream;
}

auto AapsMivExtension::operator==(const AapsMivExtension &other) const noexcept -> bool {
  return aame_omaf_v1_compatible_flag() == other.aame_omaf_v1_compatible_flag();
}

auto AapsMivExtension::operator!=(const AapsMivExtension &other) const noexcept -> bool {
  return !operator==(other);
}

auto AapsMivExtension::decodeFrom(Common::InputBitstream &bitstream) -> AapsMivExtension {
  auto x = AapsMivExtension{};
  x.aame_omaf_v1_compatible_flag(bitstream.getFlag());
  x.aame_vui_params_present_flag(bitstream.getFlag());
  if (x.aame_vui_params_present_flag()) {
    x.vui_parameters(VuiParameters::decodeFrom(bitstream, nullptr));
  }
  return x;
}

void AapsMivExtension::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putFlag(aame_omaf_v1_compatible_flag());
  bitstream.putFlag(aame_vui_params_present_flag());
  if (aame_vui_params_present_flag()) {
    vui_parameters().encodeTo(bitstream, nullptr);
  }
}

auto AtlasAdaptationParameterSetRBSP::aaps_log2_max_atlas_frame_order_cnt_lsb_minus4()
    const noexcept -> uint8_t {
  VERIFY_V3CBITSTREAM(aaps_log2_max_afoc_present_flag());
  VERIFY_V3CBITSTREAM(m_aaps_log2_max_atlas_frame_order_cnt_lsb_minus4.has_value());
  return *m_aaps_log2_max_atlas_frame_order_cnt_lsb_minus4;
}

auto AtlasAdaptationParameterSetRBSP::aaps_vpcc_extension() const noexcept
    -> const AapsVpccExtension & {
  VERIFY_V3CBITSTREAM(aaps_vpcc_extension_present_flag());
  VERIFY_V3CBITSTREAM(m_aaps_vpcc_extension.has_value());
  return *m_aaps_vpcc_extension;
}

auto AtlasAdaptationParameterSetRBSP::aapsExtensionData() const noexcept
    -> const std::vector<bool> & {
  VERIFY_V3CBITSTREAM(aaps_extension_7bits() != 0);
  VERIFY_V3CBITSTREAM(m_aapsExtensionData.has_value());
  return *m_aapsExtensionData;
}

auto AtlasAdaptationParameterSetRBSP::aaps_log2_max_atlas_frame_order_cnt_lsb_minus4(
    std::uint8_t value) noexcept -> AtlasAdaptationParameterSetRBSP & {
  VERIFY_V3CBITSTREAM(aaps_log2_max_afoc_present_flag());
  m_aaps_log2_max_atlas_frame_order_cnt_lsb_minus4 = value;
  return *this;
}

auto AtlasAdaptationParameterSetRBSP::aaps_vpcc_extension_present_flag(bool value) noexcept
    -> AtlasAdaptationParameterSetRBSP & {
  VERIFY_V3CBITSTREAM(aaps_extension_present_flag());
  m_aaps_vpcc_extension_present_flag = value;
  return *this;
}

auto AtlasAdaptationParameterSetRBSP::aaps_extension_7bits(std::uint8_t value) noexcept
    -> AtlasAdaptationParameterSetRBSP & {
  VERIFY_V3CBITSTREAM(aaps_extension_present_flag());
  m_aaps_extension_7bits = value;
  return *this;
}

auto AtlasAdaptationParameterSetRBSP::aaps_vpcc_extension(const AapsVpccExtension &value) noexcept
    -> AtlasAdaptationParameterSetRBSP & {
  VERIFY_V3CBITSTREAM(aaps_vpcc_extension_present_flag());
  m_aaps_vpcc_extension = value;
  return *this;
}

auto AtlasAdaptationParameterSetRBSP::aapsExtensionData(std::vector<bool> value) noexcept
    -> AtlasAdaptationParameterSetRBSP & {
  VERIFY_V3CBITSTREAM(aaps_extension_7bits() != 0);
  m_aapsExtensionData = std::move(value);
  return *this;
}

auto operator<<(std::ostream &stream, const AtlasAdaptationParameterSetRBSP &x) -> std::ostream & {
  stream << "aaps_atlas_adaptation_parameter_set_id="
         << int{x.aaps_atlas_adaptation_parameter_set_id()} << '\n';
  stream << "aaps_log2_max_afoc_present_flag=" << std::boolalpha
         << x.aaps_log2_max_afoc_present_flag() << '\n';
  if (x.aaps_log2_max_afoc_present_flag()) {
    stream << "aaps_log2_max_atlas_frame_order_cnt_lsb_minus4="
           << int{x.aaps_log2_max_atlas_frame_order_cnt_lsb_minus4()} << '\n';
  }
  stream << "aaps_extension_present_flag=" << std::boolalpha << x.aaps_extension_present_flag()
         << '\n';
  if (x.aaps_extension_present_flag()) {
    stream << "aaps_vpcc_extension_present_flag=" << std::boolalpha
           << x.aaps_vpcc_extension_present_flag() << '\n';
    stream << "aaps_extension_7bits=" << int{x.aaps_extension_7bits()} << '\n';
  }
  if (x.aaps_vpcc_extension_present_flag()) {
    stream << x.aaps_vpcc_extension();
  }
  if (x.aaps_extension_7bits() != 0U) {
    for (auto bit : x.aapsExtensionData()) {
      stream << "aaps_extension_data_flag=" << std::boolalpha << bit << '\n';
    }
  }
  return stream;
}

auto AtlasAdaptationParameterSetRBSP::operator==(
    const AtlasAdaptationParameterSetRBSP &other) const noexcept -> bool {
  if (aaps_atlas_adaptation_parameter_set_id() != other.aaps_atlas_adaptation_parameter_set_id() ||
      aaps_log2_max_afoc_present_flag() != other.aaps_log2_max_afoc_present_flag() ||
      aaps_extension_present_flag() != other.aaps_extension_present_flag() ||
      aaps_vpcc_extension_present_flag() != other.aaps_vpcc_extension_present_flag() ||
      aaps_extension_7bits() != other.aaps_extension_7bits()) {
    return false;
  }
  if (aaps_log2_max_afoc_present_flag() &&
      aaps_log2_max_atlas_frame_order_cnt_lsb_minus4() !=
          other.aaps_log2_max_atlas_frame_order_cnt_lsb_minus4()) {
    return false;
  }
  if (aaps_vpcc_extension_present_flag() && aaps_vpcc_extension() != other.aaps_vpcc_extension()) {
    return false;
  }
  if (aaps_extension_7bits() != 0 && aapsExtensionData() != other.aapsExtensionData()) {
    return false;
  }
  return true;
}

auto AtlasAdaptationParameterSetRBSP::operator!=(
    const AtlasAdaptationParameterSetRBSP &other) const noexcept -> bool {
  return !operator==(other);
}

auto AtlasAdaptationParameterSetRBSP::decodeFrom(std::istream &stream)
    -> AtlasAdaptationParameterSetRBSP {
  Common::InputBitstream bitstream{stream};

  auto x = AtlasAdaptationParameterSetRBSP{};

  x.aaps_atlas_adaptation_parameter_set_id(bitstream.getUExpGolomb<uint8_t>());
  x.aaps_log2_max_afoc_present_flag(bitstream.getFlag());

  if (x.aaps_log2_max_afoc_present_flag()) {
    x.aaps_log2_max_atlas_frame_order_cnt_lsb_minus4(bitstream.getUExpGolomb<uint8_t>());
  }
  x.aaps_extension_present_flag(bitstream.getFlag());

  if (x.aaps_extension_present_flag()) {
    x.aaps_vpcc_extension_present_flag(bitstream.getFlag());
    x.aaps_extension_7bits(bitstream.readBits<std::uint8_t>(7));
  }
  if (x.aaps_vpcc_extension_present_flag()) {
    x.aaps_vpcc_extension(AapsVpccExtension::decodeFrom(bitstream));
  }
  if (x.aaps_extension_7bits() != 0) {
    auto aapsExtensionData = std::vector<bool>{};
    while (bitstream.moreRbspData()) {
      aapsExtensionData.push_back(bitstream.getFlag());
    }
    x.aapsExtensionData(std::move(aapsExtensionData));
  }
  bitstream.rbspTrailingBits();

  return x;
}

void AtlasAdaptationParameterSetRBSP::encodeTo(std::ostream &stream) const {
  Common::OutputBitstream bitstream{stream};

  bitstream.putUExpGolomb(aaps_atlas_adaptation_parameter_set_id());
  bitstream.putFlag(aaps_log2_max_afoc_present_flag());

  if (aaps_log2_max_afoc_present_flag()) {
    bitstream.putUExpGolomb(aaps_log2_max_atlas_frame_order_cnt_lsb_minus4());
  }
  bitstream.putFlag(aaps_extension_present_flag());

  if (aaps_extension_present_flag()) {
    bitstream.putFlag(aaps_vpcc_extension_present_flag());
    bitstream.writeBits(aaps_extension_7bits(), 7);
  }
  if (aaps_vpcc_extension_present_flag()) {
    aaps_vpcc_extension().encodeTo(bitstream);
  }
  if (aaps_extension_7bits() != 0) {
    for (auto bit : aapsExtensionData()) {
      bitstream.putFlag(bit);
    }
  }
  bitstream.rbspTrailingBits();
}

} // namespace TMIV::MivBitstream
