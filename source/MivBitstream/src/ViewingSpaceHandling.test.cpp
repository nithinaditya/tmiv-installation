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

#include "test.h"

#include <TMIV/MivBitstream/ViewingSpaceHandling.h>

namespace TMIV::MivBitstream {
TEST_CASE("viewing_space_handling", "[Viewing space handling SEI payload syntax]") {
  SECTION("Null example") {
    const auto x = ViewingSpaceHandling{};
    REQUIRE(toString(x) == R"(vs_handling_options_count=0
)");
    REQUIRE(bitCodingTest(x, 1));
  }

  SECTION("Example 1") {
    const auto x = ViewingSpaceHandling{
        {{VhDeviceClass::VHDC_ALL, VhApplicationClass::VHAC_ALL, VhMethod::VHM_FADE}}};
    REQUIRE(toString(x) == R"(vs_handling_options_count=1
vs_handling_device_class( 0 )=VHDC_ALL
vs_handling_application_class( 0 )=VHAC_ALL
vs_handling_method( 0 )=VHM_FADE
)");
    REQUIRE(bitCodingTest(x, 21));
  }

  SECTION("Example 2") {
    const auto x = ViewingSpaceHandling{
        {{VhDeviceClass::VHDC_PHONE, VhApplicationClass::VHAC_ALL, VhMethod::VHM_ROTATE},
         {VhDeviceClass::VHDC_ALL, VhApplicationClass::VHAC_SD, VhMethod::VHM_NULL},
         {VhDeviceClass::VHDC_ALL, VhApplicationClass::VHAC_ALL, VhMethod::VHM_EXTRAP}}};
    REQUIRE(toString(x) == R"(vs_handling_options_count=3
vs_handling_device_class( 0 )=VHDC_PHONE
vs_handling_application_class( 0 )=VHAC_ALL
vs_handling_method( 0 )=VHM_ROTATE
vs_handling_device_class( 1 )=VHDC_ALL
vs_handling_application_class( 1 )=VHAC_SD
vs_handling_method( 1 )=VHM_NULL
vs_handling_device_class( 2 )=VHDC_ALL
vs_handling_application_class( 2 )=VHAC_ALL
vs_handling_method( 2 )=VHM_EXTRAP
)");
    REQUIRE(bitCodingTest(x, 59));
  }
}
} // namespace TMIV::MivBitstream
