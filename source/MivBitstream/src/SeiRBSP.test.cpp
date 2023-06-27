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

#include <TMIV/MivBitstream/SeiRBSP.h>

namespace TMIV::MivBitstream {
TEST_CASE("PayloadType", "[Supplemental Enhancement Information RBSP]") {
  SECTION("String conversion") {
    REQUIRE(toString(PayloadType::viewing_space_handling) == "viewing_space_handling");
    REQUIRE(toString(PayloadType::geometry_upscaling_parameters) ==
            "geometry_upscaling_parameters");
    REQUIRE(toString(PayloadType(42)) == "reserved_sei_message (42)");
  }

  SECTION("Integer conversion - as specified in V3C/V-PCC FDIS d224") {
    REQUIRE(0U == static_cast<unsigned int>(PayloadType::buffering_period));
    REQUIRE(1U == static_cast<unsigned int>(PayloadType::atlas_frame_timing));
    REQUIRE(2U == static_cast<unsigned int>(PayloadType::filler_payload));
    REQUIRE(3U == static_cast<unsigned int>(PayloadType::user_data_registered_itu_t_t35));
    REQUIRE(4U == static_cast<unsigned int>(PayloadType::user_data_unregistered));
    REQUIRE(5U == static_cast<unsigned int>(PayloadType::recovery_point));
    REQUIRE(6U == static_cast<unsigned int>(PayloadType::no_display));
    REQUIRE(7U == static_cast<unsigned int>(PayloadType::time_code));
    REQUIRE(8U == static_cast<unsigned int>(PayloadType::sei_manifest));
    REQUIRE(9U == static_cast<unsigned int>(PayloadType::sei_prefix_indication));
    REQUIRE(10U == static_cast<unsigned int>(PayloadType::active_sub_bitstreams));
    REQUIRE(11U == static_cast<unsigned int>(PayloadType::component_codec_mapping));
    REQUIRE(12U == static_cast<unsigned int>(PayloadType::scene_object_information));
    REQUIRE(13U == static_cast<unsigned int>(PayloadType::object_label_information));
    REQUIRE(14U == static_cast<unsigned int>(PayloadType::patch_information));
    REQUIRE(15U == static_cast<unsigned int>(PayloadType::volumetric_rectangle_information));
    REQUIRE(16U == static_cast<unsigned int>(PayloadType::atlas_object_association));
    REQUIRE(17U == static_cast<unsigned int>(PayloadType::viewport_camera_parameters));
    REQUIRE(18U == static_cast<unsigned int>(PayloadType::viewport_position));
    REQUIRE(19U == static_cast<unsigned int>(PayloadType::packed_independent_regions));
    REQUIRE(64U == static_cast<unsigned int>(PayloadType::attribute_transformation_params));
    REQUIRE(65U == static_cast<unsigned int>(PayloadType::occupancy_synthesis));
    REQUIRE(66U == static_cast<unsigned int>(PayloadType::geometry_smoothing));
    REQUIRE(67U == static_cast<unsigned int>(PayloadType::attribute_smoothing));
    REQUIRE(128U == static_cast<unsigned int>(PayloadType::viewing_space));
    REQUIRE(129U == static_cast<unsigned int>(PayloadType::viewing_space_handling));
    REQUIRE(130U == static_cast<unsigned int>(PayloadType::geometry_upscaling_parameters));
  }
}

TEST_CASE("sei_message", "[Supplemental Enhancement Information RBSP]") {
  SECTION("Default Constructor") {
    const auto message = SeiMessage{};
    REQUIRE(toString(message) == R"(payloadType=buffering_period
payloadSize=0
)");
    REQUIRE(byteCodingTest(message, 2));
  }

  SECTION("Time Code") {
    const auto message = SeiMessage{PayloadType::time_code, "Tick tock"};
    REQUIRE(toString(message) == R"(payloadType=time_code
payloadSize=9
)");
    REQUIRE(byteCodingTest(message, 11));
  }

  SECTION("Atlas Object Association") {
    const auto message = SeiMessage{PayloadType::atlas_object_association, "My Atlas"};
    REQUIRE(toString(message) == R"(payloadType=atlas_object_association
payloadSize=8
)");
    REQUIRE(byteCodingTest(message, 10));
  }
}

constexpr auto computeHeaderSizeFor(const std::size_t payload_size) -> std::size_t {
  const std::size_t bytes_to_signal_payload_size = (257 + payload_size) / 256;
  const std::size_t bytes_to_signal_payload_type = 1;
  return bytes_to_signal_payload_size + bytes_to_signal_payload_type;
}

constexpr auto computePayloadAndHeaderSizeFor(const std::size_t payload_size) -> std::size_t {
  return payload_size + computeHeaderSizeFor(payload_size);
}

TEST_CASE("sei_rbsp", "[Supplemental Enhancement Information RBSP]") {
  auto x = SeiRBSP{};

  REQUIRE(toString(x).empty());

  SECTION("Example 1") {
    x.messages().emplace_back();
    x.messages().emplace_back(PayloadType::sei_manifest, "Manifest");

    REQUIRE(toString(x) == R"(payloadType=buffering_period
payloadSize=0
payloadType=sei_manifest
payloadSize=8
)");
    REQUIRE(byteCodingTest(x, 13));
  }

  SECTION("Example 2") {
    const std::size_t number_of_bytes_of_atlas_object_association_payload = 7;

    x.messages().emplace_back(PayloadType::filler_payload, std::string(1000, 'x'));
    x.messages().emplace_back(PayloadType::filler_payload, std::string(254, 'a'));
    x.messages().emplace_back(PayloadType::filler_payload, std::string(255, 'b'));
    x.messages().emplace_back(PayloadType::filler_payload, std::string(256, 'c'));
    x.messages().emplace_back(PayloadType::filler_payload, std::string(257, 'd'));
    x.messages().emplace_back(PayloadType::user_data_unregistered, "Unregistered");
    x.messages().emplace_back(
        PayloadType::atlas_object_association,
        std::string(number_of_bytes_of_atlas_object_association_payload, 'e'));

    REQUIRE(toString(x) == R"(payloadType=filler_payload
payloadSize=1000
payloadType=filler_payload
payloadSize=254
payloadType=filler_payload
payloadSize=255
payloadType=filler_payload
payloadSize=256
payloadType=filler_payload
payloadSize=257
payloadType=user_data_unregistered
payloadSize=12
payloadType=atlas_object_association
payloadSize=7
)");
    const std::size_t trailing_byte = 1;
    const std::size_t expected_number_of_bytes =
        computePayloadAndHeaderSizeFor(1000) + computePayloadAndHeaderSizeFor(254) +
        computePayloadAndHeaderSizeFor(255) + computePayloadAndHeaderSizeFor(256) +
        computePayloadAndHeaderSizeFor(257) + computePayloadAndHeaderSizeFor(12) +
        computePayloadAndHeaderSizeFor(number_of_bytes_of_atlas_object_association_payload) +
        trailing_byte;

    REQUIRE(byteCodingTest(x, expected_number_of_bytes));
  }
}
} // namespace TMIV::MivBitstream
