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

#include <catch2/catch.hpp>

#include <TMIV/Common/verify.h>

using Catch::Matchers::Contains;
using Catch::Matchers::StartsWith;

TEST_CASE("Verify Macros") {
  SECTION("Pass on true conditions") {
    REQUIRE_NOTHROW(VERIFY_V3CBITSTREAM(true));
    REQUIRE_NOTHROW(VERIFY_MIVBITSTREAM(true));
    REQUIRE_NOTHROW(VERIFY_BITSTREAM(true));
    REQUIRE_NOTHROW(LIMITATION(true));
    REQUIRE_NOTHROW(CONSTRAIN_PTL(true));
  }

  SECTION("Raise a TMIV exception on false conditions") {
    REQUIRE_THROWS_AS(VERIFY_V3CBITSTREAM(false), TMIV::Common::Exception);
    REQUIRE_THROWS_WITH(VERIFY_V3CBITSTREAM(false),
                        StartsWith("ERROR: Failed to encode/decode V3C bitstream") &&
                            Contains(__FILE__));

    REQUIRE_THROWS_AS(VERIFY_MIVBITSTREAM(false), TMIV::Common::Exception);
    REQUIRE_THROWS_WITH(VERIFY_MIVBITSTREAM(false),
                        StartsWith("ERROR: Failed to encode/decode MIV bitstream") &&
                            Contains(__FILE__));

    REQUIRE_THROWS_AS(VERIFY_BITSTREAM(false), TMIV::Common::Exception);
    REQUIRE_THROWS_WITH(VERIFY_BITSTREAM(false),
                        StartsWith("ERROR: Failed to encode/decode bitstream") &&
                            Contains(__FILE__));

    REQUIRE_THROWS_AS(LIMITATION(false), TMIV::Common::Exception);
    REQUIRE_THROWS_WITH(LIMITATION(false),
                        StartsWith("ERROR: This aspect of MIV/3VC has not yet been implemented") &&
                            Contains(__FILE__));
  }

  SECTION("Special case: constrain only warns on stderr") { REQUIRE_NOTHROW(CONSTRAIN_PTL(false)); }
}
