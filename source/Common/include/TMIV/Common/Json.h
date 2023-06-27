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

#ifndef _TMIV_COMMON_JSON_H_
#define _TMIV_COMMON_JSON_H_

#include <TMIV/Common/Vector.h>

#include <any>
#include <iosfwd>
#include <map>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace TMIV::Common {
// https://www.json.org/
class Json {
private:
  // NOTE(BK): At first I tried to use std::variant and std::unique_ptr but any small mistake
  // resulted in pages of compiler errors so I gave up on that. The benefit of std::any is that it
  // is a copyable non-template class and thus Json is copyable (behaves like a value) and error
  // messages are readable. The small price to pay is that the type erasure moves some of the type
  // checking to runtime. This creates a class invariant and thus m_node had to be made private and
  // we need to have many constructors (instead of almost none).
  std::any m_node;

public:
  using Object = std::map<std::string, Json>;
  using Array = std::vector<Json>;
  using Integer = long long;
  using Number = double;

  // Default construction corresponds to JSON null
  Json() = default;

  // An lvalue Json that is default-constructed
  static const Json null;

  // Returns true unless the value is JSON null
  //  * This behaviour is different from previous releases whereby the JSON value `false` also
  //    returned false.
  explicit operator bool() const;

  // Rule of 5
  Json(const Json &other) = default;
  Json(Json &&other) = default;
  auto operator=(const Json &other) -> Json & = default;
  auto operator=(Json &&other) -> Json & = default;
  ~Json() = default;

  // Value constructors
  explicit Json(bool value);
  explicit Json(const char *value);
  explicit Json(std::string_view value);
  explicit Json(const std::string &value);
  explicit Json(const Object &value);
  explicit Json(const Array &value);
  explicit Json(Object &&value);
  explicit Json(std::string &&value);
  explicit Json(Array &&value);

  // Value construction with numeric promotion
  template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
  explicit Json(const T &value);

  // In-place value constructors
  template <typename... Args> explicit Json(std::in_place_type_t<Object> tag, Args &&...args);
  template <typename... Args> explicit Json(std::in_place_type_t<Array> tag, Args &&...args);

  // Converting assignment operators
  auto operator=(bool value) -> Json &;
  auto operator=(const char *value) -> Json &;
  auto operator=(std::string_view value) -> Json &;
  auto operator=(const std::string &value) -> Json &;
  auto operator=(const Object &value) -> Json &;
  auto operator=(const Array &value) -> Json &;
  auto operator=(Object &&value) -> Json &;
  auto operator=(std::string &&value) -> Json &;
  auto operator=(Array &&value) -> Json &;

  // Converting assignment with numeric promotion
  template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
  auto operator=(const T &value) -> Json &;

  // Parse a JSON
  static auto parse(std::string_view text) -> Json;

  // Load a JSON from a stream
  static auto loadFrom(std::istream &stream) -> Json;

  // Format a JSON
  auto format() const -> std::string;

  // Save a JSON to a stream
  auto saveTo(std::ostream &stream, int level = 0) const -> std::ostream &;

  // Update a JSON with another one
  //  * merges objects
  //  * copies non-null values
  auto update(const Json &other) -> Json &;

  // Merge of the second JSON object into the first
  //  1. Keys missing in first are added
  //  2. Matching keys are updated
  static void mergeObject(Object &first, const Object &second);

  // Access JSON value
  //  * with numeric conversions, e.g. as<uint16>(), as<float>(), etc.
  //  * When this node is not a JSON number, throws a `std::runtime_error`.
  //  * Numbers without mantissa and exponent are stored as Json::Integer, others as Json::Number
  //    which is a floating-point type.
  template <typename T> decltype(auto) as() const;

  // Access a JSON object by key
  //  * When a key is missing, returns `null`.
  //  * When this node is not a JSON object, throws a `std::runtime_error`.
  [[nodiscard]] auto optional(const std::string &key) const -> const Json &;

  // Access a JSON object by key
  //  * When a key is missing, throws a `std::runtime_error`.
  //  * When this node is not a JSON object, throws a `std::runtime_error`.
  [[nodiscard]] auto require(const std::string &key) const -> const Json &;

  // Copy JSON array of unknown length to std::vector<T> for given type T
  //  * When this node is not a JSON array, throws a `std::runtime_error`.
  template <typename T> [[nodiscard]] auto asVector() const -> std::vector<T>;

  // Copy JSON array of known length M to a stack::Vector<T, M> for given type T and M
  //  * When this node is not a JSON array, throws a `std::runtime_error`.
  template <typename T, std::size_t M> [[nodiscard]] auto asVec() const -> stack::Vector<T, M>;
};
} // namespace TMIV::Common

#include "Json.hpp"

#endif
