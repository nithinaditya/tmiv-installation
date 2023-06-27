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

#include <TMIV/Common/Json.h>

#include <TMIV/Common/Common.h>

#include <regex>

using namespace std::string_view_literals;

namespace TMIV::Common {
namespace {
auto parseObject(std::string_view &text) -> Json::Object;
auto parseArray(std::string_view &text) -> Json::Array;
auto parseValue(std::string_view &text) -> Json;
auto parseString(std::string_view &text) -> std::string;
auto parseNumber(std::string_view &text) -> Json; // Json::Integer or Json::Number
void parseWhitespace(std::string_view &text);

[[nodiscard]] auto peekCharacter(std::string_view &text) -> char {
  if (text.empty()) {
    throw std::runtime_error("JSON parser: truncated input");
  }
  return text.front();
}

[[nodiscard]] auto parseCharacter(std::string_view &text) {
  const auto ch = peekCharacter(text);
  text.remove_prefix(1);
  return ch;
}

void parseFixedCharacter(std::string_view &text, char expected) {
  const auto actual = parseCharacter(text);

  if (actual != expected) {
    std::ostringstream what;
    what << "Expected '" << expected << "' but found '" << actual << "' (0x" << std::hex
         << int{actual} << ")";
    throw std::runtime_error(what.str());
  }
}

void parseFixedText(std::string_view &text, std::string_view expected) {
  for (const auto ch : expected) {
    parseFixedCharacter(text, ch);
  }
}

auto parseObject(std::string_view &text) -> Json::Object {
  auto x = Json::Object{};

  parseFixedCharacter(text, '{');
  parseWhitespace(text);

  auto first = true;

  while (peekCharacter(text) != '}') {
    if (!first) {
      parseFixedCharacter(text, ',');
      parseWhitespace(text);
    }
    first = false;

    auto key = parseString(text);
    parseWhitespace(text);
    parseFixedCharacter(text, ':');
    auto value = parseValue(text);

    if (!x.emplace(key, std::move(value)).second) {
      throw std::runtime_error(fmt::format("JSON parser: duplicate key '{}'", key));
    }
  }

  parseFixedCharacter(text, '}');
  return x;
}

auto parseArray(std::string_view &text) -> Json::Array {
  auto x = Json::Array{};

  parseFixedCharacter(text, '[');
  parseWhitespace(text);

  auto first = true;

  while (peekCharacter(text) != ']') {
    if (!first) {
      parseFixedCharacter(text, ',');
    }
    first = false;

    x.emplace_back(parseValue(text));
  }

  parseFixedCharacter(text, ']');
  return x;
}

// NOTE(BK): std::isdigit is codepage dependent
constexpr auto isDigit(char ch) noexcept -> bool {
  switch (ch) {
  case '0':
  case '1':
  case '2':
  case '3':
  case '4':
  case '5':
  case '6':
  case '7':
  case '8':
  case '9':
    return true;
  default:
    return false;
  }
}

auto parseValue(std::string_view &text) -> Json {
  auto x = Json{};

  parseWhitespace(text);

  const auto ch = peekCharacter(text);

  if (ch == '"') {
    x = parseString(text);
  } else if (ch == '-' || isDigit(ch)) {
    x = parseNumber(text);
  } else if (ch == '{') {
    x = parseObject(text);
  } else if (ch == '[') {
    x = parseArray(text);
  } else if (ch == 't') {
    parseFixedText(text, "true"sv);
    x = true;
  } else if (ch == 'f') {
    parseFixedText(text, "false"sv);
    x = false;
  } else {
    parseFixedText(text, "null"sv);
  }

  parseWhitespace(text);
  return x;
}

auto parseString(std::string_view &text) -> std::string {
  auto x = std::string{};

  parseFixedCharacter(text, '"');

  while (true) {
    const auto ch1 = parseCharacter(text);

    if (ch1 == '"') {
      return x;
    }
    if (ch1 == '\\') {
      const auto ch2 = parseCharacter(text);

      if (ch2 == '"' || ch2 == '\\' || ch2 == '/') {
        x.push_back(ch2);
      } else if (ch2 == 'b') {
        x.push_back('\b');
      } else if (ch2 == 'f') {
        x.push_back('\f');
      } else if (ch2 == 'n') {
        x.push_back('\n');
      } else if (ch2 == 'r') {
        x.push_back('\r');
      } else if (ch2 == 't') {
        x.push_back('\t');
      } else if (ch2 == 'u') {
        throw std::logic_error("JSON parser: unicode character codes are not yet supported");
      } else {
        throw std::runtime_error(
            fmt::format("JSON parser: invalid string escape character '{}'", ch2));
      }
    } else if ('\0' <= ch1 && ch1 < ' ') {
      throw std::runtime_error("JSON parser: control character within string");
    } else {
      x.push_back(ch1);
    }
  }
}

auto parseNumber(std::string_view &text) -> Json {
  constexpr auto number = 0;
  constexpr auto fraction = 2;
  constexpr auto exponent = 3;
  static const auto pattern = std::regex(R"(^(0|[\-1-9][0-9]*)(\.[0-9]+)?([eE][\-+]?[0-9]+)?)",
                                         std::regex_constants::optimize);

  auto match = std::match_results<std::string_view::const_iterator>{};

  if (std::regex_search(text.cbegin(), text.cend(), match, pattern)) {
    text.remove_prefix(static_cast<std::size_t>(match[0].length()));

    if (match[fraction].matched || match[exponent].matched) {
      static_assert(std::is_same_v<double, Json::Number>);
      return Json{std::stod(match[number])};
    }

    static_assert(std::is_same_v<long long, Json::Integer>);
    return Json{std::stoll(match[number])};
  }

  throw std::runtime_error("JSON parser: failed to parse number");
}

void parseWhitespace(std::string_view &text) {
  const auto n = text.find_first_not_of(" \n\r\t"sv);
  if (n == std::string_view::npos) {
    text.remove_prefix(text.size());
  } else {
    text.remove_prefix(n);
  }
}
} // namespace

const Json Json::null;

auto Json::parse(std::string_view text) -> Json {
  auto x = parseValue(text);

  if (!text.empty()) {
    throw std::runtime_error("JSON parser: stray characters at the end");
  }
  return x;
}

auto Json::loadFrom(std::istream &stream) -> Json {
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return parse(buffer.str());
}

auto Json::format() const -> std::string {
  std::ostringstream stream;
  stream.precision(std::numeric_limits<Json::Number>::max_digits10);
  saveTo(stream);
  return stream.str();
}

namespace {
auto saveValue(std::tuple<bool> /* tag */, std::ostream &stream, bool value, int /* level */)
    -> std::ostream & {
  return stream << std::boolalpha << value;
}

auto saveValue(std::tuple<Json::Integer> /* tag */, std::ostream &stream, Json::Integer value,
               int /* level */) -> std::ostream & {
  return stream << value;
}

auto saveValue(std::tuple<Json::Number> /* tag */, std::ostream &stream, Json::Number value,
               int /* level */) -> std::ostream & {
  if (std::isinf(value)) {
    if (0 < value) {
      return stream << "\"inf\"";
    }
    return stream << "\"-inf\"";
  }
  return stream << value;
}

auto saveValue(const std::tuple<std::string> & /* tag */, std::ostream &stream,
               const std::string &value, int /* level */) -> std::ostream & {
  stream << '"';
  for (const auto ch : value) {
    if (ch == '"') {
      stream << "\\\"";
    } else if (ch == '\\') {
      stream << "\\\\";
    } else if (ch == '\b') {
      stream << "\\b";
    } else if (ch == '\f') {
      stream << "\\f";
    } else if (ch == '\n') {
      stream << "\\n";
    } else if (ch == '\r') {
      stream << "\\r";
    } else if (ch == '\t') {
      stream << "\\t";
    } else if ('\0' <= ch && ch < ' ') {
      throw std::runtime_error("JSON formatter: control character within string");
    } else {
      stream << ch;
    }
  }
  return stream << '"';
}

auto saveValue(const std::tuple<Json::Object> & /* tag */, std::ostream &stream,
               const Json::Object &object, int level) -> std::ostream & {
  if (object.empty()) {
    return stream << "{ }";
  }
  const auto indent = std::string(std::size_t{4} * level, ' ');
  auto sep = "{\n"sv;
  for (const auto &[key, value] : object) {
    stream << sep << indent << "    ";
    Json{key}.saveTo(stream) << ": ";
    value.saveTo(stream, level + 1);
    sep = ",\n"sv;
  }
  return stream << "\n" << indent << '}';
}

auto saveValue(const std::tuple<Json::Array> & /* tag */, std::ostream &stream,
               const Json::Array &array, int level) -> std::ostream & {
  stream << '[';
  auto sep = " "sv;
  for (const auto &value : array) {
    stream << sep;
    value.saveTo(stream, level);
    sep = ", "sv;
  }
  return stream << " ]";
}

template <typename Type>
auto saveIf(const std::any &node, std::ostream &stream, int level) -> bool {
  if (const auto *value = std::any_cast<Type>(&node)) {
    return saveValue(std::tuple<Type>{}, stream, *value, level).good();
  }
  return false;
}

template <typename... Type>
auto saveAny(const std::any &node, std::ostream &stream, int level) -> std::ostream & {
  if ((saveIf<Type>(node, stream, level) || ...)) {
    return stream;
  }
  throw std::runtime_error("JSON: Failed to write JSON node to stream");
}
} // namespace

auto Json::saveTo(std::ostream &stream, int level) const -> std::ostream & {
  if (m_node.has_value()) {
    return saveAny<bool, Json::Integer, Json::Number, std::string, Json::Object, Json::Array>(
        m_node, stream, level);
  }
  return stream << "null";
}

auto Json::update(const Json &other) -> Json & {
  // Update strategy for JSON null: copy non-null over null
  if (!m_node.has_value() || !other.m_node.has_value()) {
    if (other.m_node.has_value()) {
      m_node = other.m_node;
    }
    return *this;
  }

  // Update strategy for JSON object: merge
  {
    auto *thisObject = std::any_cast<Object>(&m_node);
    const auto *otherObject = std::any_cast<Object>(&other.m_node);

    if ((thisObject != nullptr) && (otherObject != nullptr)) {
      mergeObject(*thisObject, *otherObject);
      return *this;
    }
    if ((thisObject != nullptr) || (otherObject != nullptr)) {
      throw std::runtime_error("JSON: Update objects with objects.");
    }
  }

  // Default strategy: copy
  m_node = other.m_node;
  return *this;
}

void Json::mergeObject(Json::Object &first, const Json::Object &second) {
  for (const auto &[key, value] : second) {
    first[key].update(value);
  }
}
} // namespace TMIV::Common
