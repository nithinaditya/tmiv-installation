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
#error "Include the .h, not the .hpp"
#endif

#include <TMIV/Common/Common.h>

#include <fmt/format.h>
#include <fmt/printf.h>

#include <filesystem>

namespace TMIV::Common {
inline Json::operator bool() const { return m_node.has_value(); }

inline Json::Json(bool value) : m_node{value} {}

inline Json::Json(const char *value) : m_node{std::string{value}} {}

inline Json::Json(std::string_view value) : m_node{std::string{value}} {}

inline Json::Json(const std::string &value) : m_node{value} {}

inline Json::Json(const Object &value) : m_node{value} {}

inline Json::Json(const Array &value) : m_node{value} {}

inline Json::Json(Object &&value) : m_node{std::move(value)} {}

inline Json::Json(std::string &&value) : m_node{std::move(value)} {}

inline Json::Json(Array &&value) : m_node{std::move(value)} {}

template <typename T, typename> Json::Json(const T &value) {
  using ValueType = std::conditional_t<std::is_integral_v<T>, Json::Integer, Json::Number>;
  m_node = ValueType{value};
}

template <typename... Args>
Json::Json(std::in_place_type_t<Object> /* tag */, Args &&...args)
    : m_node{Object{std::forward<Args>(args)...}} {}

template <typename... Args>
Json::Json(std::in_place_type_t<Array> /* tag */, Args &&...args)
    : m_node{Array{std::forward<Args>(args)...}} {}

inline auto Json::operator=(bool value) -> Json & {
  m_node = value;
  return *this;
}

inline auto Json::operator=(const char *value) -> Json & {
  m_node = std::make_any<std::string>(value);
  return *this;
}

inline auto Json::operator=(std::string_view value) -> Json & {
  m_node = std::make_any<std::string>(value);
  return *this;
}

inline auto Json::operator=(const std::string &value) -> Json & {
  m_node = value;
  return *this;
}

inline auto Json::operator=(const Object &value) -> Json & {
  m_node = value;
  return *this;
}

inline auto Json::operator=(const Array &value) -> Json & {
  m_node = value;
  return *this;
}

inline auto Json::operator=(Object &&value) -> Json & {
  m_node = std::move(value);
  return *this;
}

inline auto Json::operator=(std::string &&value) -> Json & {
  m_node = std::move(value);
  return *this;
}

inline auto Json::operator=(Array &&value) -> Json & {
  m_node = std::move(value);
  return *this;
}

template <typename T, typename> inline auto Json::operator=(const T &value) -> Json & {
  using ValueType = std::conditional_t<std::is_integral_v<T>, Json::Integer, Json::Number>;
  m_node = ValueType{value};
  return *this;
}

template <typename T> decltype(auto) Json::as() const {
  try {
    if constexpr (std::is_same_v<T, bool>) {
      return std::any_cast<bool>(m_node);
    } else if constexpr (std::is_integral_v<T>) {
      return static_cast<T>(std::any_cast<Integer>(m_node)); // TODO(BK): check bounds (#273)
    } else if constexpr (std::is_floating_point_v<T>) {
      if (const auto *value = std::any_cast<Integer>(&m_node)) {
        return static_cast<T>(*value); // cast integer to float
      }
      if (const auto *value = std::any_cast<std::string>(&m_node)) {
        using namespace std::string_literals;
        if (*value == "inf"s || *value == "+inf"s) {
          return std::numeric_limits<T>::infinity();
        }
        if (*value == "-inf"s) {
          return -std::numeric_limits<T>::infinity();
        }
      }
      return static_cast<T>(std::any_cast<Number>(m_node)); // TODO(BK): check bounds (#273)
    } else if constexpr (std::is_same_v<T, std::filesystem::path>) {
      return std::filesystem::path{std::any_cast<std::string>(m_node)};
    } else {
      return std::any_cast<const T &>(m_node);
    }
  } catch (std::bad_any_cast & /* unused */) {
    throw std::runtime_error(fmt::format("JSON: value has wrong type:\n  * The expected type is "
                                         "{}\n  * The value is: {}",
                                         typeid(T).name(), format()));
  }
}

inline auto Json::optional(const std::string &key) const -> const Json & {
  const auto &object = as<Object>();
  const auto kvp = object.find(key);
  if (kvp == object.cend()) {
    return null;
  }
  return std::get<1>(*kvp);
}

inline auto Json::require(const std::string &key) const -> const Json & {
  if (const auto &m_node = optional(key)) {
    return m_node;
  }
  using namespace std::string_view_literals;
  throw std::runtime_error(fmt::format("JSON: Parameter '{}' is required but missing"sv, key));
}

template <typename T> auto Json::asVector() const -> std::vector<T> {
  const auto &a = as<Array>();
  auto v = std::vector<T>(a.size());
  std::transform(a.cbegin(), a.cend(), v.begin(),
                 [](const Json &m_node) { return m_node.as<T>(); });
  return v;
}

template <typename T, std::size_t M> auto Json::asVec() const -> stack::Vector<T, M> {
  stack::Vector<T, M> result;
  const auto &a = as<Array>();
  if (a.size() != M) {
    throw std::runtime_error("JSON int vector has wrong length");
  }
  for (stack::size_type i = 0; i != M; ++i) {
    result[i] = a[i].as<T>();
  }
  return result;
}
} // namespace TMIV::Common
