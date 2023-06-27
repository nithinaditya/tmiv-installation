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

#include <TMIV/MivBitstream/SequenceConfig.h>

using namespace std::string_literals;

TEST_CASE("CameraConfig") {
  SECTION("Default construction with default values") {
    auto unit = TMIV::MivBitstream::CameraConfig{};
    CHECK(unit.viewParams == TMIV::MivBitstream::ViewParams{});
    CHECK(unit.bitDepthColor == 0);
    CHECK(unit.bitDepthTransparency == 0);
    CHECK(unit.bitDepthDepth == 0);
    CHECK(unit.bitDepthEntities == 0);
    CHECK(unit.colorspace == TMIV::MivBitstream::CameraConfig::Colorspace::yuv420);
    CHECK(unit.transparencyColorspace == TMIV::MivBitstream::CameraConfig::Colorspace::yuv420);
    CHECK(unit.depthColorspace == TMIV::MivBitstream::CameraConfig::Colorspace::yuv420);
    CHECK(unit.entitiesColorspace == TMIV::MivBitstream::CameraConfig::Colorspace::yuv420);
  }

  SECTION("Load from JSON, Equirectangular") {
    const auto json = TMIV::Common::Json::parse(R"(
{
    "BitDepthColor": 10,
    "BitDepthDepth": 16,
    "ColorSpace": "YUV420",
    "DepthColorSpace": "YUV420",
    "Depth_range": [ 0.1, 500.0 ],
    "Hor_range": [ -90.0, 90.0 ],
    "Name": "v2",
    "Position": [ -0.2878679633140564, -0.0878679633140564, 1.0 ],
    "Projection": "Equirectangular",
    "Resolution": [ 2048, 1048 ],
    "Rotation": [ 45.00000125223908, 19.3, 4.3 ],
    "Ver_range": [ -90.0, 90.0 ]
})");

    auto unit = TMIV::MivBitstream::CameraConfig(json);

    CHECK(unit.bitDepthColor == 10);
    CHECK(unit.bitDepthTransparency == 0);
    CHECK(unit.bitDepthDepth == 16);
    CHECK(unit.bitDepthEntities == 0);
    CHECK(unit.colorspace == TMIV::MivBitstream::CameraConfig::Colorspace::yuv420);
    CHECK(unit.transparencyColorspace == TMIV::MivBitstream::CameraConfig::Colorspace::yuv420);
    CHECK(unit.depthColorspace == TMIV::MivBitstream::CameraConfig::Colorspace::yuv420);
    CHECK(unit.entitiesColorspace == TMIV::MivBitstream::CameraConfig::Colorspace::yuv420);
    CHECK(unit.viewParams.name == "v2");

    SECTION("Save and load back") {
      auto newJson = TMIV::Common::Json{unit};
      auto y = TMIV::MivBitstream::CameraConfig{newJson};

      // NOTE(BK): Cannot use x == y because of floating-point conversions
      CHECK(unit.bitDepthColor == y.bitDepthColor);
      CHECK(unit.bitDepthTransparency == y.bitDepthTransparency);
      CHECK(unit.bitDepthDepth == y.bitDepthDepth);
      CHECK(unit.bitDepthEntities == y.bitDepthEntities);
      CHECK(unit.colorspace == y.colorspace);
      CHECK(unit.depthColorspace == y.depthColorspace);
      CHECK(unit.entitiesColorspace == y.entitiesColorspace);
      CHECK(unit.viewParams.name == y.viewParams.name);
    }
  }

  SECTION("Load from JSON, MPI camera") {
    const auto json = TMIV::Common::Json::parse(R"(
{
    "Background": 1,
    "BitDepthColor": 10,
    "BitDepthDepth": 0,
    "BitDepthTransparency": 8,
    "ColorSpace": "YUV420",
    "Depth_range": [ 0.4160835445, 12.86596775 ],
    "Depthmap": 0,
    "Focal": [ 1749.296875, 1749.296875 ],
    "HasInvalidDepth": false,
    "Name": "mpi",
    "nbMpiLayers": 423,
    "Position": [ -2.499999762, 1.649999857, 1.449999332 ],
    "Principle_point": [ 2088.0, 1012.0 ],
    "Projection": "Perspective",
    "Resolution": [ 4176, 2024 ],
    "Rotation": [ -33.91259003, 7.734590054, 0.0 ],
    "TransparencyColorSpace": "YUV420"
})");

    const auto unit = TMIV::MivBitstream::CameraConfig(json);
    CHECK(unit.viewParams.nbMpiLayers == 423);

    CHECK(unit.bitDepthColor == 10);
    CHECK(unit.bitDepthTransparency == 8);
    CHECK(unit.bitDepthDepth == 0);
    CHECK(unit.colorspace == TMIV::MivBitstream::CameraConfig::Colorspace::yuv420);
    CHECK(unit.transparencyColorspace == TMIV::MivBitstream::CameraConfig::Colorspace::yuv420);
    CHECK(unit.depthColorspace == TMIV::MivBitstream::CameraConfig::Colorspace::yuv420);
  }
}

TEST_CASE("SequenceConfig") {
  using TMIV::Common::Vec3d;
  using TMIV::MivBitstream::SequenceConfig;

  SECTION("Default construction with default values") {
    const auto unit = SequenceConfig{};
    CHECK(unit.boundingBoxCenter == Vec3d{});
    CHECK(unit.contentName.empty());
    CHECK(unit.frameRate == 0.);
    CHECK(unit.numberOfFrames == 0);
    CHECK(unit.cameras.empty());
    CHECK(unit.sourceCameraNames.empty());
    CHECK(unit.lengthsInMeters);
  }

  SECTION("Load from JSON") {
    const auto json = TMIV::Common::Json::parse(R"(
{
    "BoundingBox_center": [ -0.5, -0.5, 1.0 ],
    "Content_name": "Chess",
    "Fps": 30,
    "Frames_number": 300,
    "lengthsInMeters": false,
    "Informative": {
        "Cameras_number": 10,
        "RigRadius": 0.3,
        "Document": "MPEG128/m50787"
    },
    "cameras": [ {
        "Background": 1,
        "BitDepthColor": 10,
        "BitDepthDepth": 16,
        "ColorSpace": "YUV420",
        "DepthColorSpace": "YUV420",
        "Depth_range": [0.1, 500],
        "Depthmap": 1,
        "Name": "viewport",
        "Projection": "Perspective",
        "Position": [ -0.5, -0.5, 1.0 ],
        "Resolution": [2048, 2048],
        "Rotation": [ 0.0, 0.0, 0.0 ],
        "Focal": [1024, 1024],
        "Principle_point": [1024, 1024]
    },  {
        "BitDepthColor": 10,
        "BitDepthDepth": 16,
        "BitDepthTransparency": 13,
        "BitDepthEntitities": 15,
        "ColorSpace": "YUV420",
        "DepthColorSpace": "YUV420",
        "TransparencyColorSpace": "YUV420",
        "EntitiesColorSpace": "YUV420",
        "Depth_range": [ 0.1, 500.0 ],
        "Hor_range": [ -90.0, 90.0 ],
        "HasInvalidDepth": false,
        "Name": "v0",
        "Position": [ -0.5, -0.5, 1.2999999523162842 ],
        "Projection": "Equirectangular",
        "Resolution": [ 2048, 2048 ],
        "Rotation": [ -90.00000933466734, -90.0000161648565, 0.0 ],
        "Ver_range": [ -90.0, 90.0 ]
    } ]
}
)");
    const auto x = SequenceConfig{json};
    CHECK(x.boundingBoxCenter == Vec3d{-0.5, -0.5, 1.0});
    CHECK(x.contentName == "Chess"s);
    CHECK(x.frameRate == 30.);
    CHECK(x.numberOfFrames == 300);
    CHECK(x.cameras.size() == 2);
    CHECK(!x.cameras.back().viewParams.hasOccupancy);

    // Backwards compatible intelligent default is to recognize all v[0-9]+ as source cameras.
    // Otherwise existing sequence configuration files would have to be changed.
    CHECK(x.sourceCameraNames == std::vector{"v0"s});

    SECTION("sourceViewParams() obtains a VPL with only the source views") {
      const auto y = x.sourceViewParams();
      REQUIRE(y.size() == 1);
      REQUIRE(y.front() == x.cameras.back().viewParams);
    }

    SECTION("Save and load back") {
      const auto newJson = TMIV::Common::Json{x};
      const auto y = TMIV::MivBitstream::SequenceConfig{newJson};

      REQUIRE(x.cameras.size() == y.cameras.size());
      REQUIRE(x.contentName == y.contentName);
      REQUIRE(x.lengthsInMeters == y.lengthsInMeters);
      REQUIRE(x.numberOfFrames == y.numberOfFrames);

      // NOTE(BK): Cannot use x == y because of floating-point conversions
      for (std::size_t i = 0; i < x.cameras.size(); ++i) {
        CHECK(x.cameras[i].bitDepthColor == y.cameras[i].bitDepthColor);
        CHECK(x.cameras[i].bitDepthDepth == y.cameras[i].bitDepthDepth);
        CHECK(x.cameras[i].bitDepthTransparency == y.cameras[i].bitDepthTransparency);
        CHECK(x.cameras[i].bitDepthEntities == y.cameras[i].bitDepthEntities);
        CHECK(x.cameras[i].colorspace == y.cameras[i].colorspace);
        CHECK(x.cameras[i].depthColorspace == y.cameras[i].depthColorspace);
        CHECK(x.cameras[i].transparencyColorspace == y.cameras[i].transparencyColorspace);
        CHECK(x.cameras[i].entitiesColorspace == y.cameras[i].entitiesColorspace);
        CHECK(x.cameras[i].viewParams.name == y.cameras[i].viewParams.name);
      }
    }
  }

  SECTION("Load from JSON, minimal") {
    const auto json = TMIV::Common::Json::parse(R"(
{
    "BoundingBox_center": [ -0.5, -0.5, 1.0 ],
    "Content_name": "Example",
    "Fps": 25,
    "Frames_number": 97,
    "cameras": [ ]
}
)");
    const auto unit = SequenceConfig{json};
    CHECK(unit.boundingBoxCenter == Vec3d{-0.5, -0.5, 1.0});
    CHECK(unit.contentName == "Example"s);
    CHECK(unit.frameRate == 25);
    CHECK(unit.numberOfFrames == 97);
    CHECK(unit.cameras.empty());
    CHECK(unit.frameRanges.empty());

    SECTION("operator ==") {
      REQUIRE(unit == unit);
      REQUIRE(unit != SequenceConfig{});
    }
  }

  SECTION("Load from JSON, set sourceCameraNames") {
    const auto json = TMIV::Common::Json::parse(R"(
{
    "BoundingBox_center": [ -0.5, -0.5, 1.0 ],
    "Content_name": "Example",
    "Fps": 25,
    "Frames_number": 97,
    "sourceCameraNames": [ "some", "view", "names" ],
    "cameras": [ ]
}
)");
    const auto unit = SequenceConfig{json};

    // It is is better to specify source camera names in sequence configuration, instead of using
    // the v[0-9]+ convention (even when naming cameras that way) because that convention is
    // specific to MPEG while the direct specification of source camera names is more clear.
    CHECK(unit.sourceCameraNames == std::vector{"some"s, "view"s, "names"s});

    SECTION("sourceViewParams() throws a runtime_error unless all cameras are available") {
      REQUIRE_THROWS(unit.sourceViewParams());
    }

    SECTION("operator ==") {
      REQUIRE(unit == unit);
      REQUIRE(unit != SequenceConfig{});
    }
  }

  SECTION("Load from JSON, set frameRanges") {
    const auto json = TMIV::Common::Json::parse(R"(
{
    "BoundingBox_center": [ -0.5, -0.5, 1.0 ],
    "Content_name": "Example",
    "Fps": 25,
    "Frames_number": 100,
    "frameRanges": [{
        "maxNumberOfFrames": 10,
        "startFrame": 45
    }, {
        "maxNumberOfFrames": 81,
        "startFrame": 16
    }, {
        "maxNumberOfFrames": 97,
        "startFrame": 2
    }],
    "cameras": [ ]
}
)");
    const auto unit = SequenceConfig{json};
    const auto ref = std::vector<SequenceConfig::FrameRange>{{10, 45}, {81, 16}, {97, 2}};
    CHECK(unit.frameRanges == ref);
  }

  SECTION("SequenceConfig::startFrameGiven") {
    SECTION("Behavior without frame ranges") {
      auto unit = SequenceConfig{};
      unit.numberOfFrames = 100;

      SECTION("startFrameGiven with argument in full frame range returns zero") {
        const auto numberOfInputFrames = GENERATE(0, 1, 99, 100);
        CHECK(unit.startFrameGiven(numberOfInputFrames) == 0);
      }

      SECTION("startFrameGiven with argument out of range throws") {
        const auto numberOfInputFrames = GENERATE(-1, 101);
        REQUIRE_THROWS(unit.startFrameGiven(numberOfInputFrames));
      }
    }

    SECTION("Behavior with an example frame range list") {
      auto unit = SequenceConfig{};
      unit.numberOfFrames = 100;
      unit.frameRanges = {{10, 45}, {81, 16}, {97, 2}};

      SECTION("startFrameGiven with argument in first frame range returns that start frame") {
        const auto numberOfInputFrames = GENERATE(0, 1, 9, 10);
        CHECK(unit.startFrameGiven(numberOfInputFrames) == 45);
      }

      SECTION("startFrameGiven with argument in second frame range returns that start frame") {
        const auto numberOfInputFrames = GENERATE(11, 12, 80, 81);
        CHECK(unit.startFrameGiven(numberOfInputFrames) == 16);
      }

      SECTION("startFrameGiven with argument in third frame range returns that start frame") {
        const auto numberOfInputFrames = GENERATE(82, 83, 96, 97);
        CHECK(unit.startFrameGiven(numberOfInputFrames) == 2);
      }

      SECTION("startFrameGiven with argument in full frame range returns zero") {
        const auto numberOfInputFrames = GENERATE(98, 99, 100);
        CHECK(unit.startFrameGiven(numberOfInputFrames) == 0);
      }

      SECTION("startFrameGiven with argument out of range") {
        const auto numberOfInputFrames = GENERATE(-1, 101);
        REQUIRE_THROWS(unit.startFrameGiven(numberOfInputFrames));
      }
    }

    SECTION("The first matching frame range wins") {
      auto unit = SequenceConfig{};
      unit.numberOfFrames = 100;
      unit.frameRanges = {{81, 16}, {10, 45}, {97, 2}};

      SECTION("startFrameGiven with argument in first frame range returns that start frame") {
        const auto numberOfInputFrames = GENERATE(0, 1, 9, 10, 11, 12, 80, 81);
        CHECK(unit.startFrameGiven(numberOfInputFrames) == 16);
      }

      SECTION("startFrameGiven with argument in third frame range returns that start frame") {
        const auto numberOfInputFrames = GENERATE(82, 83, 96, 97);
        CHECK(unit.startFrameGiven(numberOfInputFrames) == 2);
      }

      SECTION("startFrameGiven with argument in full frame range returns zero") {
        const auto numberOfInputFrames = GENERATE(98, 99, 100);
        CHECK(unit.startFrameGiven(numberOfInputFrames) == 0);
      }

      SECTION("startFrameGiven with argument out of range") {
        const auto numberOfInputFrames = GENERATE(-1, 101);
        REQUIRE_THROWS(unit.startFrameGiven(numberOfInputFrames));
      }
    }

    SECTION("When a range itself is invalid, a runtime error is thrown even when the range is not "
            "selected (to promote finding such configuration errors quickly)") {
      auto unit = SequenceConfig{};
      unit.numberOfFrames = 96;
      unit.frameRanges = {{10, 45}, {81, 16}, {97, 2}};
      const auto numberOfInputFrames = GENERATE(0, 1, 30, 90, 99, 100);
      REQUIRE_THROWS(unit.startFrameGiven(numberOfInputFrames));
    }
  }

  SECTION("Load from JSON, lengthsInMeters key") {
    SECTION("The key is otpional for backwards compatibility") {
      const auto unit = SequenceConfig{TMIV::Common::Json::parse(R"(
{
    "BoundingBox_center": [ -0.5, -0.5, 1.0 ],
    "Content_name": "Example",
    "Fps": 25,
    "Frames_number": 97,
    "cameras": [ ]
}
)")};

      REQUIRE(unit.lengthsInMeters);
    }

    SECTION("Specify lengths are in meters") {
      const auto unit = SequenceConfig{TMIV::Common::Json::parse(R"(
{
    "BoundingBox_center": [ -0.5, -0.5, 1.0 ],
    "Content_name": "Example",
    "lengthsInMeters": false,
    "Fps": 25,
    "Frames_number": 97,
    "cameras": [ ]
}
)")};

      REQUIRE(!unit.lengthsInMeters);
    }

    SECTION("Specify arbitrary scene units") {
      const auto unit = SequenceConfig{TMIV::Common::Json::parse(R"(
{
    "BoundingBox_center": [ -0.5, -0.5, 1.0 ],
    "Content_name": "Example",
    "Fps": 25,
    "lengthsInMeters": true,
    "Frames_number": 97,
    "cameras": [ ]
}
)")};

      REQUIRE(unit.lengthsInMeters);
    }
  }
}
