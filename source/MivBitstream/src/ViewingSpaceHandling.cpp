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

#include <TMIV/MivBitstream/ViewingSpaceHandling.h>

#include <TMIV/Common/verify.h>

namespace TMIV::MivBitstream {
auto operator<<(std::ostream &stream, VhDeviceClass x) -> std::ostream & {
  switch (x) {
  case VhDeviceClass::VHDC_ALL:
    return stream << "VHDC_ALL";
  case VhDeviceClass::VHDC_HMD:
    return stream << "VHDC_HMD";
  case VhDeviceClass::VHDC_PHONE:
    return stream << "VHDC_PHONE";
  case VhDeviceClass::VHDC_ASD:
    return stream << "VHDC_ASD";
  default:
    return stream << "Unknown device class (" << static_cast<int>(x) << ")";
  }
}

auto operator<<(std::ostream &stream, VhApplicationClass x) -> std::ostream & {
  switch (x) {
  case VhApplicationClass::VHAC_ALL:
    return stream << "VHAC_ALL";
  case VhApplicationClass::VHAC_AR:
    return stream << "VHAC_AR";
  case VhApplicationClass::VHAC_VR:
    return stream << "VHAC_VR";
  case VhApplicationClass::VHAC_WEB:
    return stream << "VHAC_WEB";
  case VhApplicationClass::VHAC_SD:
    return stream << "VHAC_SD";
  default:
    return stream << "Unknown application class (" << static_cast<int>(x) << ")";
  }
}

auto operator<<(std::ostream &stream, VhMethod x) -> std::ostream & {
  switch (x) {
  case VhMethod::VHM_NULL:
    return stream << "VHM_NULL";
  case VhMethod::VHM_RENDER:
    return stream << "VHM_RENDER";
  case VhMethod::VHM_FADE:
    return stream << "VHM_FADE";
  case VhMethod::VHM_EXTRAP:
    return stream << "VHM_EXTRAP";
  case VhMethod::VHM_RESET:
    return stream << "VHM_RESET";
  case VhMethod::VHM_STRETCH:
    return stream << "VHM_STRETCH";
  case VhMethod::VHM_ROTATE:
    return stream << "VHM_ROTATE";
  default:
    return stream << "Unknown method (" << static_cast<int>(x) << ")";
  }
}

auto HandlingOption::operator==(const HandlingOption &other) const noexcept -> bool {
  return vs_handling_device_class == other.vs_handling_device_class &&
         vs_handling_application_class == other.vs_handling_application_class &&
         vs_handling_method == other.vs_handling_method;
}

auto HandlingOption::operator!=(const HandlingOption &other) const noexcept -> bool {
  return !operator==(other);
}

ViewingSpaceHandling::ViewingSpaceHandling(HandlingOptionList value)
    : m_handlingOptionList{std::move(value)} {}

auto ViewingSpaceHandling::vs_handling_options_count() const noexcept -> size_t {
  return m_handlingOptionList.size();
}

auto ViewingSpaceHandling::vs_handling_device_class(size_t h) const noexcept -> VhDeviceClass {
  VERIFY_MIVBITSTREAM(h < vs_handling_options_count());
  return m_handlingOptionList[h].vs_handling_device_class;
}

auto ViewingSpaceHandling::vs_handling_application_class(size_t h) const noexcept
    -> VhApplicationClass {
  VERIFY_MIVBITSTREAM(h < vs_handling_options_count());
  return m_handlingOptionList[h].vs_handling_application_class;
}

auto ViewingSpaceHandling::vs_handling_method(size_t h) const noexcept -> VhMethod {
  VERIFY_MIVBITSTREAM(h < vs_handling_options_count());
  return m_handlingOptionList[h].vs_handling_method;
}

auto operator<<(std::ostream &stream, const ViewingSpaceHandling &x) -> std::ostream & {
  stream << "vs_handling_options_count=" << x.vs_handling_options_count() << '\n';

  for (size_t h = 0; h < x.vs_handling_options_count(); ++h) {
    stream << "vs_handling_device_class( " << h << " )=" << x.vs_handling_device_class(h) << '\n';
    stream << "vs_handling_application_class( " << h << " )=" << x.vs_handling_application_class(h)
           << '\n';
    stream << "vs_handling_method( " << h << " )=" << x.vs_handling_method(h) << '\n';
  }
  return stream;
}

auto ViewingSpaceHandling::operator==(const ViewingSpaceHandling &other) const noexcept -> bool {
  return m_handlingOptionList == other.m_handlingOptionList;
}

auto ViewingSpaceHandling::operator!=(const ViewingSpaceHandling &other) const noexcept -> bool {
  return !operator==(other);
}

auto ViewingSpaceHandling::decodeFrom(Common::InputBitstream &bitstream) -> ViewingSpaceHandling {
  auto x = HandlingOptionList(bitstream.getUExpGolomb<size_t>());

  for (auto &el : x) {
    el.vs_handling_device_class = bitstream.readBits<VhDeviceClass>(6);
    el.vs_handling_application_class = bitstream.readBits<VhApplicationClass>(6);
    el.vs_handling_method = bitstream.readBits<VhMethod>(6);
  }

  return ViewingSpaceHandling{x};
}

void ViewingSpaceHandling::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putUExpGolomb(vs_handling_options_count());

  for (size_t h = 0; h < vs_handling_options_count(); ++h) {
    bitstream.writeBits(vs_handling_device_class(h), 6);
    bitstream.writeBits(vs_handling_application_class(h), 6);
    bitstream.writeBits(vs_handling_method(h), 6);
  }
}
} // namespace TMIV::MivBitstream
