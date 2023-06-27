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

#include <TMIV/Common/Json.h>

using namespace std::string_literals;
using namespace std::string_view_literals;

namespace TMIV::Common {
TEST_CASE("Json default construction corresponds to JSON null") {
  const auto json = Json{};
  REQUIRE(!json);
}

TEST_CASE("Json::explicit operator bool() return true if not null (even for false)") {
  REQUIRE(!Json{});
  REQUIRE(!Json::null);

  auto json = Json{false};
  REQUIRE(!!json);
}

TEST_CASE("There are value constructors for each of JSON value types") {
  REQUIRE(!Json{false}.as<bool>());
  REQUIRE(Json{true}.as<bool>());

  const auto s = "Hi!"s;
  const auto a = Json::Array{};
  const auto o = Json::Object{};

  REQUIRE(Json{s}.as<std::string>() == s);
  REQUIRE(Json{a}.as<Json::Array>().empty());
  REQUIRE(Json{o}.as<Json::Object>().empty());

  REQUIRE(Json{"Hello, world!"}.as<std::string>() == "Hello, world!"s);
  REQUIRE(Json{"Hello, world!"s}.as<std::string>() == "Hello, world!"s);
  REQUIRE(Json{"Hello, world!"sv}.as<std::string>() == "Hello, world!"s);
  REQUIRE(Json{Json::Array{}}.as<Json::Array>().empty());
  REQUIRE(Json{Json::Object{}}.as<Json::Object>().empty());

  SECTION("Arithmetic types are promoted") {
    REQUIRE(Json{42.F}.as<double>() == 42.);
    REQUIRE(Json{42}.as<int>() == 42);
    REQUIRE(Json{42.}.as<float>() == 42.F);
    REQUIRE(Json{std::uint32_t{42}}.as<short>() == 42);
  }

  SECTION("There is in-place value construction for JSON object") {
    const auto json =
        Json{std::in_place_type_t<Json::Object>{}, std::pair{"numberOfFrames"s, Json{97}},
             std::pair{"startFrame"s, Json{120}}};
    REQUIRE(json.as<Json::Object>().size() == 2);
  }

  SECTION("There is in-place value construction for JSON array") {
    const auto json = Json{std::in_place_type_t<Json::Array>{}, Json{3}, Json{"three"}};
    REQUIRE(json.as<Json::Array>().size() == 2);
  }
}

TEST_CASE("Json::Integer is a sufficiently large signed integer") {
  static_assert(std::is_integral_v<Json::Integer>);
  static_assert(std::is_signed_v<Json::Integer>);
  static_assert(std::is_same_v<std::common_type_t<Json::Integer, std::int64_t>, Json::Integer>);
  // Do not assume that Json::Integer is a specific type because that will break code when
  // Json::Integer is changed for an even larger type, e.g. std::intmax_t == std::int128_t
}

TEST_CASE("Json::as<T>() supports numeric conversion of integers") {
  auto json = Json{40};
  REQUIRE(json.as<char>() == '\x28');
  REQUIRE(json.as<std::int8_t>() == 40);
  REQUIRE(json.as<std::int16_t>() == 40);
  REQUIRE(json.as<std::int32_t>() == 40);
  REQUIRE(json.as<std::int64_t>() == 40);
  REQUIRE(json.as<std::uint8_t>() == 40);
  REQUIRE(json.as<std::uint16_t>() == 40);
  REQUIRE(json.as<std::uint32_t>() == 40);
  REQUIRE(json.as<std::uint64_t>() == 40);
  // NOTE(BK): Optional bounds checking to be added as part of #273

  SECTION("Converting an integer to a float is allowed because the user expects 40.0 and 40 to "
          "be the same number") {
    REQUIRE(json.as<float>() == 40.F);
    REQUIRE(json.as<double>() == 40.);
  }
}

TEST_CASE("Json::Number is a sufficiently large floating-point number") {
  static_assert(std::is_floating_point_v<Json::Number>);
  static_assert(std::is_signed_v<Json::Number>);
  static_assert(std::is_same_v<std::common_type_t<Json::Number, double>, Json::Number>);
}

TEST_CASE("Json::as<T>() supports numeric conversion of floats") {
  auto json = Json{30.4F};
  REQUIRE(json.as<float>() == Approx(30.4F));
  REQUIRE(json.as<double>() == Approx(30.4));
  // NOTE(BK): Optional bounds checking to be added as part of #273

  SECTION("Converting a float to an integer is not allowed") {
    REQUIRE_THROWS(json.as<int>());
    REQUIRE_THROWS(json.as<char>());

    // The exact message is implementation-defined, so the message is printed for visual inspection.
    try {
      json.as<int>();
    } catch (std::exception &e) {
      std::cout << e.what() << '\n';
    }
  }
}

TEST_CASE("Converting assignment operator with numeric promotion") {
  auto json = Json{};

  json = 1;
  REQUIRE(json.as<int>() == 1);

  json = 1.;
  REQUIRE(json.as<double>() == 1.);

  json = true;
  REQUIRE(json.as<bool>());

  json = false;
  REQUIRE(!json.as<bool>());

  json = "true";
  REQUIRE(json.as<std::string>() == "true"s);

  json = "true"s;
  REQUIRE(json.as<std::string>() == "true"s);

  json = "true"sv;
  REQUIRE(json.as<std::string>() == "true"s);

  json = Json::Object{};
  REQUIRE(json.as<Json::Object>().empty());

  json = Json::Array{};
  REQUIRE(json.as<Json::Array>().empty());
}

TEST_CASE("Represent infinity as \"inf\" in JSON") {
  SECTION("Json::as<T>() for floating-point type T converts \"inf\" to infinity") {
    REQUIRE(Json{"inf"}.as<float>() == std::numeric_limits<float>::infinity());
    REQUIRE(Json{"inf"}.as<double>() == std::numeric_limits<double>::infinity());
    REQUIRE(Json{"+inf"}.as<float>() == std::numeric_limits<float>::infinity());
    REQUIRE(Json{"+inf"}.as<double>() == std::numeric_limits<double>::infinity());
    REQUIRE(Json{"-inf"}.as<float>() == -std::numeric_limits<float>::infinity());
    REQUIRE(Json{"-inf"}.as<double>() == -std::numeric_limits<double>::infinity());
  }

  SECTION("Json::as<std::string>() returns \"inf\"") {
    REQUIRE(Json{"inf"}.as<std::string>() == "inf"s);
    REQUIRE(Json{"+inf"}.as<std::string>() == "+inf"s);
    REQUIRE(Json{"-inf"}.as<std::string>() == "-inf"s);
  }

  SECTION("Json{inf} is transformed to Json{\"inf\"}") {
    REQUIRE(Json{std::numeric_limits<float>::infinity()}.format() == "\"inf\""s);
    REQUIRE(Json{std::numeric_limits<double>::infinity()}.format() == "\"inf\""s);
  }
}

TEST_CASE("Json::as<std::filesytem::path> converts a string to a path") {
  REQUIRE(Json{"/some/path.txt"}.as<std::filesystem::path>() ==
          std::filesystem::path{"/some/path.txt"});
}

TEST_CASE("Parse a JSON") {
  SECTION("null") { REQUIRE(!Json::parse("null"sv)); }

  SECTION("whitespace") {
    REQUIRE(!Json::parse(" null"sv));
    REQUIRE(!Json::parse("null\n"sv));
    REQUIRE(!Json::parse("\tnull\r"sv));
  }

  SECTION("false") {
    REQUIRE(Json::parse("false"sv));
    REQUIRE(!Json::parse("false"sv).as<bool>());
  }

  SECTION("true") { REQUIRE(Json::parse("true"sv).as<bool>()); }

  SECTION("string") {
    REQUIRE(Json::parse("\"Hello, world!\""sv).as<std::string>() == "Hello, world!"s);
    REQUIRE(Json::parse("\"Escape \\\\ \\/ \\n \\r \\b \\f \\t \\r \\\" the string\""sv)
                .as<std::string>() == "Escape \\ / \n \r \b \f \t \r \" the string"s);
  }

  SECTION("number") {
    REQUIRE(Json::parse("0"sv).as<int>() == 0);
    REQUIRE(Json::parse("-1234"sv).as<int>() == -1234);
    REQUIRE(Json::parse("774"sv).as<int>() == 774);
    REQUIRE(Json::parse("-9223372036854775808"sv).as<std::int64_t>() == INT64_MIN);
    REQUIRE(Json::parse("9223372036854775807"sv).as<std::int64_t>() == INT64_MAX);
    REQUIRE(Json::parse("0.0"sv).as<double>() == 0.);
    REQUIRE(Json::parse("42E-002"sv).as<float>() == 0.42F);
    REQUIRE(Json::parse("2.4E+3"sv).as<double>() == 2400.);
    REQUIRE_THROWS(Json::parse("2."sv).as<double>()); // not allowed in JSON
    REQUIRE_THROWS(Json::parse("01"sv).as<int>());    // not allowed in JSON
  }

  SECTION("array") {
    REQUIRE(Json::parse("[]"sv).as<Json::Array>().empty());
    REQUIRE(Json::parse("\r[\r]\r"sv).as<Json::Array>().empty());
    REQUIRE(Json::parse("[4]"sv).as<Json::Array>().size() == 1);
    REQUIRE(Json::parse("[4]"sv).as<Json::Array>().front().as<int>() == 4);
    REQUIRE(Json::parse("[ 4, 2 ]"sv).as<Json::Array>().size() == 2);
    REQUIRE(Json::parse("[ 4, 2 ]"sv).as<Json::Array>().front().as<int>() == 4);
    REQUIRE(Json::parse("[ 4, 2.0 ]"sv).as<Json::Array>().back().as<double>() == 2.);
  }

  SECTION("object") {
    REQUIRE(Json::parse("{}"sv).as<Json::Object>().empty());
    REQUIRE(Json::parse("{ \"hello\": \"world\", \"goodby\": \"!\" }"sv)
                .as<Json::Object>()
                .find("hello"s)
                ->second.as<std::string>() == "world"s);
  }
}

TEST_CASE("Format a JSON") {
  SECTION("null") { REQUIRE(Json::null.format() == "null"s); }

  SECTION("false") { REQUIRE(Json{false}.format() == "false"s); }

  SECTION("true") { REQUIRE(Json{true}.format() == "true"s); }

  SECTION("string") {
    REQUIRE(Json{"Hello, world!"s}.format() == "\"Hello, world!\""s);

    REQUIRE(Json{"Escape \\ / \n \r \b \f \t \r \" the string"s}.format() ==
            "\"Escape \\\\ / \\n \\r \\b \\f \\t \\r \\\" the string\""s);
  }

  SECTION("number") {
    REQUIRE(Json{0}.format() == "0"s);
    REQUIRE(Json{-1234}.format() == "-1234"s);
    REQUIRE(Json{774}.format() == "774"s);
    REQUIRE(Json{INT64_MIN}.format() == "-9223372036854775808"s);
    REQUIRE(Json{INT64_MAX}.format() == "9223372036854775807"s);
    REQUIRE(Json{0.0}.format() == "0"s);
    REQUIRE(Json{42E-002}.format() == "0.41999999999999998"s);
    REQUIRE(Json{2.4E+3}.format() == "2400"s);
    REQUIRE(Json{M_PI}.format() == "3.1415926535897931"s);
  }

  SECTION("array") {
    REQUIRE(Json{Json::Array{}}.format() == "[ ]"s);
    REQUIRE(Json{std::in_place_type_t<Json::Array>{}, Json{4}}.format() == "[ 4 ]"s);
    REQUIRE(Json{std::in_place_type_t<Json::Array>{}, Json{4}, Json{2}}.format() == "[ 4, 2 ]"s);
    REQUIRE(Json{std::in_place_type_t<Json::Array>{}, Json{2}, Json{4.2}}.format() ==
            "[ 2, 4.2000000000000002 ]"s);
  }

  SECTION("object") {
    REQUIRE(Json{Json::Object{}}.format() == "{ }"s);
    REQUIRE(Json{std::in_place_type_t<Json::Object>{}, std::pair{"test"s, Json{true}}}.format() ==
            R"({
    "test": true
})");
    REQUIRE(Json{std::in_place_type_t<Json::Object>{}, std::pair{"fail"s, Json{}},
                 std::pair{"empty"s, Json{Json::Object{}}}}
                .format() ==
            R"({
    "empty": { },
    "fail": null
})");
  }

  SECTION("Indenting with object within array within object") {
    const auto inner = Json{std::in_place_type_t<Json::Object>{}, std::pair{"fail"s, Json{}},
                            std::pair{"empty"s, Json{Json::Object{}}}};
    const auto middle =
        Json{std::in_place_type_t<Json::Array>{}, inner, inner, Json{"break"}, inner};
    const auto outer =
        Json{std::in_place_type_t<Json::Object>{}, std::pair{"middle"s, middle},
             std::pair{"intermezzo"s, Json{Json::Array{}}}, std::pair{"first"s, middle}};
    REQUIRE(outer.format() == R"({
    "first": [ {
        "empty": { },
        "fail": null
    }, {
        "empty": { },
        "fail": null
    }, "break", {
        "empty": { },
        "fail": null
    } ],
    "intermezzo": [ ],
    "middle": [ {
        "empty": { },
        "fail": null
    }, {
        "empty": { },
        "fail": null
    }, "break", {
        "empty": { },
        "fail": null
    } ]
})");
  }
}

TEST_CASE("Save a JSON to a stream") {
  std::ostringstream stream;
  stream << '|';
  Json{}.saveTo(stream) << "|";
  REQUIRE(stream.str() == "|null|"s);
}

TEST_CASE("Load a JSON from a stream") {
  std::istringstream stream{"true"};
  const auto json = Json::loadFrom(stream);
  REQUIRE(json.as<bool>());
}

TEST_CASE("Update a JSON") {
  SECTION("For JSON null the strategy is to copy non-null over null") {
    auto a = Json{};
    REQUIRE(!a);

    a.update(Json{});
    REQUIRE(!a);

    a.update(Json{true});
    REQUIRE(a);

    a = 3;
    a.update(Json::null);
    REQUIRE(a.as<int>() == 3);
  }

  SECTION("For JSON objects the strategy is to merge") {
    auto a = Json{std::in_place_type_t<Json::Object>(), std::pair{"x"s, Json{3}},
                  std::pair{"y"s, Json{4}}};
    const auto b = Json{std::in_place_type_t<Json::Object>(), std::pair{"z"s, Json{5}}};
    a.update(b);

    REQUIRE(a.require("x"s).as<int>() == 3);
    REQUIRE(a.require("y"s).as<int>() == 4);
    REQUIRE(a.require("z"s).as<int>() == 5);

    SECTION("Updating an object with a non-object is not allowed (schema violation)") {
      REQUIRE_THROWS(a.update(Json{3}));
    }

    SECTION("Updating a non-object with an object is not allowed (schema violation)") {
      auto c = Json{3};
      REQUIRE_THROWS(c.update(b));
    }
  }

  SECTION("The default strategy is to copy") {
    auto a = Json{45};
    auto b = Json{"text"};
    a.update(b);
    REQUIRE(a.as<std::string>() == "text"s);
  }
}

TEST_CASE("Json::optional()") {
  auto json = Json{std::in_place_type_t<Json::Object>(), std::pair{"x"s, Json{3}},
                   std::pair{"y"s, Json{4}}};

  SECTION("Performs key-lookup") { REQUIRE(json.optional("x"s).as<int>() == 3); }

  SECTION("Returns null when a key does not exist") { REQUIRE(!json.optional("w"s)); }

  SECTION("Throws when not an JSON object") {
    json = 42;
    REQUIRE_THROWS(json.optional("meaning"s));
  }
}

TEST_CASE("Json::require()") {
  auto json = Json{std::in_place_type_t<Json::Object>(), std::pair{"x"s, Json{3}},
                   std::pair{"y"s, Json{4}}};

  SECTION("Performs key-lookup") { REQUIRE(json.require("x"s).as<int>() == 3); }

  SECTION("Throws when a key does not exist") { REQUIRE_THROWS(json.require("w"s)); }

  SECTION("Throws when not an JSON object") {
    json = 42;
    REQUIRE_THROWS(json.require("meaning"s));
  }
}

TEST_CASE("Json::asVector() converts a JSON array to a std::vector<T>") {
  auto json = Json{std::in_place_type_t<Json::Array>(), Json{1}, Json{2}, Json{3}};
  REQUIRE(json.asVector<int>() == std::vector{1, 2, 3});
}

TEST_CASE("Json::asVec() converts a JSON array to a Common::stack::Vector<T, M>") {
  auto json = Json{std::in_place_type_t<Json::Array>(), Json{1}, Json{2}, Json{3}};
  REQUIRE(json.asVec<std::uint16_t, 3>() == Vec3w{1, 2, 3});
}
} // namespace TMIV::Common
