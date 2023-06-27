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

#ifndef _TMIV_COMMON_ARRAY_H_
#define _TMIV_COMMON_ARRAY_H_

#include "Traits.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <numeric>
#include <ostream>
#include <vector>

namespace TMIV::Common {
namespace Array {
using size_type = std::size_t;

template <typename T> class const_iterator {
public:
  using iterator_category = std::random_access_iterator_tag;
  using value_type = T;
  using difference_type = std::ptrdiff_t;
  using pointer = T *;
  using reference = T &;

protected:
  T *m_p;

public:
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
  explicit const_iterator(const T *x = nullptr) : m_p(const_cast<T *>(x)) {}
  const_iterator(const const_iterator &iter) = default;
  const_iterator(const_iterator &&iter) noexcept = default;
  ~const_iterator() = default;
  auto operator=(const const_iterator &rhs) -> const_iterator & = default;
  auto operator=(const_iterator &&rhs) noexcept -> const_iterator & = default;
  auto operator==(const const_iterator &rhs) const -> bool { return m_p == rhs.m_p; }
  auto operator!=(const const_iterator &rhs) const -> bool { return m_p != rhs.m_p; }
  auto operator*() const -> const T & { return *m_p; }
  auto operator->() const -> const T * { return m_p; }
  auto operator++() -> const_iterator & {
    ++m_p; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return *this;
  }
  auto operator++(int) -> const_iterator {
    const_iterator tmp(*this);
    operator++();
    return tmp;
  }
  auto operator--() -> const_iterator & {
    --m_p;
    return *this;
  }
  auto operator--(int) -> const_iterator {
    const_iterator tmp(*this);
    operator--();
    return tmp;
  }
  auto operator+(std::ptrdiff_t n) const -> const_iterator { return const_iterator(m_p + n); }
  auto operator+=(std::ptrdiff_t n) -> const_iterator & {
    m_p += n;
    return *this;
  }
  auto operator-(const const_iterator &iter) const -> std::ptrdiff_t { return m_p - iter.m_p; }
  auto operator-(std::ptrdiff_t n) const -> const_iterator { return const_iterator(m_p - n); }
  auto operator-=(std::ptrdiff_t n) -> const_iterator & {
    m_p -= n;
    return *this;
  }
  auto operator[](std::ptrdiff_t n) const -> const T & {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return m_p[n];
  }
  auto operator<(const const_iterator &rhs) const -> bool { return m_p < rhs.m_p; }
  auto operator<=(const const_iterator &rhs) const -> bool { return m_p <= rhs.m_p; }
  auto operator>(const const_iterator &rhs) const -> bool { return m_p > rhs.m_p; }
  auto operator>=(const const_iterator &rhs) const -> bool { return m_p >= rhs.m_p; }
  void swap(const_iterator &a, const_iterator &b) { std::swap(a, b); }
};

template <typename T>
auto operator+(std::ptrdiff_t n, const const_iterator<T> &rhs) -> const_iterator<T> {
  return rhs + n;
}

template <typename T> class iterator : public const_iterator<T> {
public:
  explicit iterator(T *x = nullptr) : const_iterator<T>(x) {}
  auto operator*() -> T & { return *this->m_p; }
  auto operator->() -> T * { return this->m_p; }
  auto operator++() -> iterator & {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    ++this->m_p;
    return *this;
  }
  auto operator++(int) -> iterator {
    iterator tmp(*this);
    operator++();
    return tmp;
  }
  auto operator--() -> iterator & {
    --this->m_p;
    return *this;
  }
  auto operator--(int) -> iterator {
    iterator tmp(*this);
    operator--();
    return tmp;
  }
  auto operator+(std::ptrdiff_t n) const -> iterator { return iterator(this->m_p + n); }
  auto operator+=(std::ptrdiff_t n) -> iterator & {
    this->m_p += n;
    return *this;
  }
  auto operator-(const iterator &iter) const -> std::ptrdiff_t { return this->m_p - iter.m_p; }
  auto operator-(std::ptrdiff_t n) const -> iterator { return iterator(this->m_p - n); }
  auto operator-=(std::ptrdiff_t n) -> iterator & {
    this->m_p -= n;
    return *this;
  }
  auto operator[](std::ptrdiff_t n) -> T & { return (this->m_p)[n]; }
};

template <typename T> auto operator+(std::ptrdiff_t n, const iterator<T> &rhs) -> iterator<T> {
  return rhs + n;
}

template <typename T> class const_dim_iterator {
public:
  using iterator_category = std::random_access_iterator_tag;
  using value_type = T;
  using difference_type = std::ptrdiff_t;
  using pointer = T *;
  using reference = T &;

protected:
  T *m_p{};
  std::ptrdiff_t m_step;

public:
  explicit const_dim_iterator(const T *x = nullptr, std::ptrdiff_t s = 0)
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
      : m_p(const_cast<T *>(x)), m_step(s) {}
  const_dim_iterator(const const_dim_iterator &iter) = default;
  const_dim_iterator(const_dim_iterator &&iter) noexcept = default;
  ~const_dim_iterator() = default;
  [[nodiscard]] auto n() const -> std::ptrdiff_t { return m_step; }
  auto operator=(const const_dim_iterator &rhs) -> const_dim_iterator & = default;
  auto operator=(const_dim_iterator &&rhs) noexcept -> const_dim_iterator & = default;
  auto operator==(const const_dim_iterator &rhs) const -> bool { return m_p == rhs.m_p; }
  auto operator!=(const const_dim_iterator &rhs) const -> bool { return m_p != rhs.m_p; }
  auto operator*() const -> const T & { return *m_p; }
  auto operator->() const -> const T * { return m_p; }
  auto operator++() -> const_dim_iterator & {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    m_p += m_step;
    return *this;
  }
  auto operator++(int) -> const_dim_iterator {
    const_dim_iterator out(*this);
    operator++();
    return out;
  }
  auto operator--() -> const_dim_iterator & {
    m_p -= m_step;
    return *this;
  }
  auto operator--(int) -> const_dim_iterator {
    const_dim_iterator out(*this);
    operator--();
    return out;
  }
  auto operator+(std::ptrdiff_t a) const -> const_dim_iterator {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return const_dim_iterator(m_p + a * m_step, this->m_step);
  }
  auto operator+=(std::ptrdiff_t a) -> const_dim_iterator & {
    m_p += (a * m_step);
    return *this;
  }
  auto operator-(const const_dim_iterator &iter) const -> std::ptrdiff_t {
    return (m_p - iter.m_p) / m_step;
  }
  auto operator-(std::ptrdiff_t a) const -> const_dim_iterator {
    return const_dim_iterator(m_p - a * m_step, this->m_step);
  }
  auto operator-=(std::ptrdiff_t a) -> const_dim_iterator & {
    m_p -= (a * m_step);
    return *this;
  }
  auto operator[](std::ptrdiff_t a) const -> const T & { return m_p[a * m_step]; }
  auto operator<(const const_dim_iterator &rhs) const -> bool { return m_p < rhs.m_p; }
  auto operator<=(const const_dim_iterator &rhs) const -> bool { return m_p <= rhs.m_p; }
  auto operator>(const const_dim_iterator &rhs) const -> bool { return m_p > rhs.m_p; }
  auto operator>=(const const_dim_iterator &rhs) const -> bool { return m_p >= rhs.m_p; }
  void swap(const_dim_iterator &a, const_dim_iterator &b) { std::swap(a, b); }
};

template <typename T>
auto operator+(std::ptrdiff_t a, const const_dim_iterator<T> &rhs) -> const_dim_iterator<T> {
  return rhs + a;
}

template <typename T> class dim_iterator : public const_dim_iterator<T> {
public:
  explicit dim_iterator(T *x = nullptr, std::ptrdiff_t s = 0) : const_dim_iterator<T>(x, s) {}
  auto operator*() -> T & { return *this->m_p; }
  auto operator->() -> T * { return this->m_p; }
  auto operator++() -> dim_iterator & {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    this->m_p += this->m_step;
    return *this;
  }
  auto operator++(int) -> dim_iterator {
    dim_iterator out(*this);
    operator++();
    return out;
  }
  auto operator--() -> dim_iterator & {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    this->m_p -= this->m_step;
    return *this;
  }
  auto operator--(int) -> dim_iterator {
    dim_iterator out(*this);
    operator--();
    return out;
  }
  auto operator+(std::ptrdiff_t a) const -> dim_iterator {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return dim_iterator(this->m_p + (a * this->m_step), this->m_step);
  }
  auto operator+=(std::ptrdiff_t a) -> dim_iterator & {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    this->m_p += (a * this->m_step);
    return *this;
  }
  auto operator-(const dim_iterator &iter) const -> std::ptrdiff_t {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return (this->m_p - iter.m_p) / this->m_step;
  }
  auto operator-(std::ptrdiff_t a) const -> dim_iterator {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return dim_iterator(this->m_p - a * this->m_step, this->m_step);
  }
  auto operator-=(std::ptrdiff_t a) -> dim_iterator & {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    this->m_p -= (a * this->m_step);
    return *this;
  }
  auto operator[](std::ptrdiff_t a) -> T & { return this->m_p[a * this->m_step]; }
};

template <typename T>
auto operator+(std::ptrdiff_t a, const dim_iterator<T> &rhs) -> dim_iterator<T> {
  return rhs + a;
}

} // namespace Array

namespace stack {
using size_type = TMIV::Common::Array::size_type;

template <size_type D, typename T, size_type... I> struct _Array {};

template <typename T, size_type M> struct _Array<1, T, M> {
protected:
  std::array<T, M> m_v{};

public:
  static constexpr auto size(size_type /*unused*/) -> size_type { return M; }
  static void sizes(size_type *iter) { *iter = M; }
  static constexpr auto size() -> size_type { return M; }
  static constexpr auto min_size() -> size_type { return M; }
  auto data() -> T * { return m_v.data(); }
  [[nodiscard]] auto data() const -> const T * { return m_v.data(); }
  template <size_type K>
  static constexpr auto offset(size_type i, size_type first = 0, size_type second = 0)
      -> size_type {
    return (i == K) ? second : first;
  }
  static constexpr auto step(size_type i) -> size_type { return i != 0U ? 1 : M; }
  static constexpr auto diag_step() -> size_type { return 1; }
  [[nodiscard]] auto get(size_type first) const -> T { return m_v[first]; }
  auto get(size_type first) -> T & { return m_v[first]; }
};

template <size_type D, typename T, size_type M, size_type N, size_type... I>
struct _Array<D, T, M, N, I...> {
protected:
  std::array<_Array<D - 1, T, N, I...>, M> m_v;

public:
  static constexpr auto size(size_type i) -> size_type {
    return i != 0U ? _Array<D - 1, T, N, I...>::size(i - 1) : M;
  }
  static void sizes(size_type *iter) {
    *iter = M;
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    _Array<D - 1, T, N, I...>::sizes(++iter);
  }
  static constexpr auto size() -> size_type { return M * _Array<D - 1, T, N, I...>::size(); }
  static auto min_size() -> size_type {
    return (std::min)(M, _Array<D - 1, T, N, I...>::min_size());
  }
  auto data() -> T * {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<T *>(m_v.data());
  }
  [[nodiscard]] auto data() const -> const T * {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    return reinterpret_cast<const T *>(m_v.data());
  }
  template <size_type K, typename... J>
  static constexpr auto offset(size_type i, size_type first, J... next) -> size_type {
    return (i == K) ? _Array<D - 1, T, N, I...>::template offset<K>(i + 1, first, next...)
                    : first * _Array<D - 1, T, N, I...>::size() +
                          _Array<D - 1, T, N, I...>::template offset<K>(i + 1, next...);
  }
  static constexpr auto step(size_type i) -> size_type {
    return i != 0U ? _Array<D - 1, T, N, I...>::step(i - 1)
                   : M * _Array<D - 1, T, N, I...>::step(i);
  }
  static constexpr auto diag_step() -> size_type {
    return step(1) + _Array<D - 1, T, N, I...>::diag_step();
  }
  template <typename... J> [[nodiscard]] auto get(size_type first, J... next) const -> T {
    return m_v[first].get(next...);
  }
  template <typename... J> auto get(size_type first, J... next) -> T & {
    return m_v[first].get(next...);
  }
};

template <typename T, size_type... I> class Array {
public:
  using value_type = T;
  using reference = T &;
  using const_reference = const T &;
  using iterator = TMIV::Common::Array::iterator<T>;
  using const_iterator = TMIV::Common::Array::const_iterator<T>;
  using dim_iterator = TMIV::Common::Array::dim_iterator<T>;
  using const_dim_iterator = TMIV::Common::Array::const_dim_iterator<T>;
  using diag_iterator = TMIV::Common::Array::dim_iterator<T>;
  using const_diag_iterator = TMIV::Common::Array::const_dim_iterator<T>;
  using difference_type = std::ptrdiff_t;
  using size_type = stack::size_type;
  using container_type = Array<T, I...>;
  using tuple_type = std::array<stack::size_type, sizeof...(I)>;
  template <typename U> using promoted_type = Array<std::common_type_t<T, U>, I...>;

protected:
  class Helper {
  protected:
    tuple_type m_sizes;

  public:
    Helper() { _Array<sizeof...(I), T, I...>::sizes(m_sizes.data()); };
    [[nodiscard]] auto sizes() const -> const tuple_type & { return m_sizes; }
  };

protected:
  static Helper m_helper;
  _Array<sizeof...(I), T, I...> m_v;

public:
  //! \brief Default constructor
  Array() = default;
  //! \brief Destructor.
  ~Array() = default;
  //! \brief Copy constructors.
  Array(const container_type &that) = default;
  explicit Array(T t) { std::fill(begin(), end(), t); }
  Array(std::initializer_list<T> v) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    std::copy(v.begin(), v.begin() + size(), begin());
  }
  template <typename OTHER, class = typename OTHER::dim_iterator>
  explicit Array(const OTHER &that) : Array() {
    if ((dim() == that.dim()) &&
        std::equal(that.sizes().begin(), that.sizes().end(), sizes().begin())) {
      std::transform(that.begin(), that.end(), begin(), [](auto v) { return static_cast<T>(v); });
    }
  }
  //! \brief Move constructor.
  Array(container_type &&that) noexcept = default;
  //! \brief Copy assignment.
  auto operator=(const Array &that) -> Array & = default;
  auto operator=(T t) -> Array & {
    std::fill(begin(), end(), t);
    return *this;
  }
  auto operator=(std::initializer_list<T> v) -> Array & {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    std::copy(v.begin(), v.begin() + size(), begin());
    return *this;
  }
  template <typename OTHER, class = typename OTHER::dim_iterator>
  auto operator=(const OTHER &that) -> Array & {
#ifndef NDEBUG
    size_t i = 0;
    for (; i < dim() && i < that.dim(); ++i) {
      assert(size(i) == that.size(i));
    }
    for (; i < dim(); ++i) {
      assert(size(i) == 1);
    }
    for (; i < that.dim(); ++i) {
      assert(that.size(i) == 1);
    }
#endif
    std::transform(that.begin(), that.end(), begin(), [](auto v) { return static_cast<T>(v); });
    return *this;
  }
  //! \brief Move assignment.
  auto operator=(Array &&that) noexcept -> Array & = default;
  //! \brief Equal operator.
  auto operator==(const Array &that) const -> bool {
    return std::equal(begin(), end(), that.begin());
  }
  //! \brief Different operator.
  auto operator!=(const Array &that) const -> bool {
    return !std::equal(begin(), end(), that.begin());
  }
  //! \brief Swap operator.
  void swap(Array &that) { std::swap(m_v, that.m_v); }
  //! \brief Resize operator.
  void resize(const tuple_type & /*unused*/) {}
  //! \brief Reshape operator.
  void reshape(const tuple_type & /*unused*/) {}
  //! \brief Returns the array dimension.
  static constexpr auto dim() -> size_type { return sizeof...(I); }
  //! \brief Returns the array size along the i-th dimension.
  static constexpr auto size(size_type i) -> size_type {
    return _Array<sizeof...(I), T, I...>::size(i);
  }
  //! \brief Returns the array sizes.
  static auto sizes() -> const tuple_type & {
    return m_helper.sizes();
  } // tuple_type out; _Array<sizeof...(I), T, I...>::sizes(out.data()); return
    // out; }
  //! \brief Returns the array total length
  static constexpr auto size() -> size_type { return _Array<sizeof...(I), T, I...>::size(); }
  //! \brief Returns the gap between 2 consecutive elements on the ith
  //! dimension.
  static constexpr auto step(size_type i) -> size_type {
    return _Array<sizeof...(I), T, I...>::step(i + 1);
  }
  //! \brief Returns true if the array is empty.
  static constexpr auto empty() -> bool { return (size() == 0); }
  //! \brief Data access, returns a pointer to the first element of the array.
  auto data() -> T * { return m_v.data(); }
  [[nodiscard]] auto data() const -> const T * { return m_v.data(); }
  //! \brief [] operator, returns the kth element of the array viewed as a one
  //! dimensional array.
  auto operator[](size_type k) const -> T {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return data()[k];
  }
  auto operator[](size_type k) -> T & {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return data()[k];
  }
  // TODO(CB) remove everything related to property, not used anywhere
  //! \brief Return the property of the array
  [[nodiscard]] auto getProperty() const -> int { return -1; }
  //! \brief Returns an iterator to the first element of the array.
  auto begin() -> iterator { return iterator(data()); }
  [[nodiscard]] auto begin() const -> const_iterator { return const_iterator(data()); }
  //! \brief Returns a const iterator to the first element of the array.
  [[nodiscard]] auto cbegin() const -> const_iterator { return const_iterator(data()); }
  //! \brief Returns an iterator to the first element after the end of the
  //! array.
  auto end() -> iterator {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return iterator(data() + size());
  }
  [[nodiscard]] auto end() const -> const_iterator {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return const_iterator(data() + size());
  }
  //! \brief Returns a const iterator to the first element after the end of the
  //! array.
  [[nodiscard]] auto cend() const -> const_iterator { return const_iterator(data() + size()); }
  //! \brief Returns an iterator along the Kth dimension to the first element of
  //! the hyperplane defined by next.
  template <size_type K, typename... J> auto dim_begin(J... next) const -> const_dim_iterator {
    return const_dim_iterator(
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        data() + _Array<sizeof...(I), T, I...>::template offset<K>(0, next...),
        _Array<sizeof...(I), T, I...>::step(K + 1));
  }
  template <size_type K, typename... J> auto dim_begin(J... next) -> dim_iterator {
    return dim_iterator(
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        data() + _Array<sizeof...(I), T, I...>::template offset<K>(0, next...),
        _Array<sizeof...(I), T, I...>::step(K + 1));
  }
  //! \brief Returns a const iterator along the Kth dimension to the first
  //! element of the hyperplane defined by next.
  template <size_type K, typename... J> auto cdim_begin(J... next) const -> const_dim_iterator {
    return const_dim_iterator(data() +
                                  _Array<sizeof...(I), T, I...>::template offset<K>(0, next...),
                              _Array<sizeof...(I), T, I...>::step(K + 1));
  }
  //! \brief Returns an iterator along the Kth dimension to the first element
  //! after the end of the hyperplane defined by next.
  template <size_type K, typename... J> auto dim_end(J... next) const -> const_dim_iterator {
    return const_dim_iterator(
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        data() + _Array<sizeof...(I), T, I...>::template offset<K>(0, next...) +
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            _Array<sizeof...(I), T, I...>::step(K),
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        _Array<sizeof...(I), T, I...>::step(K + 1));
  }
  template <size_type K, typename... J> auto dim_end(J... next) -> dim_iterator {
    return dim_iterator(data() + _Array<sizeof...(I), T, I...>::template offset<K>(0, next...) +
                            _Array<sizeof...(I), T, I...>::step(K),
                        _Array<sizeof...(I), T, I...>::step(K + 1));
  }
  //! \brief Returns a const iterator along the Kth dimension to the first
  //! element after the end of the hyperplane defined by next.
  template <size_type K, typename... J> auto cdim_end(J... next) const -> const_dim_iterator {
    return const_dim_iterator(data() + _Array<sizeof...(I), T, I...>::template offset<K>(next...) +
                                  _Array<sizeof...(I), T, I...>::step(K),
                              _Array<sizeof...(I), T, I...>::step(K + 1));
  }
  //! \brief Returns an iterator to the first diagonal element.
  [[nodiscard]] auto diag_begin() const -> const_diag_iterator {
    return const_diag_iterator(data(), _Array<sizeof...(I), T, I...>::diag_step());
  }
  auto diag_begin() -> diag_iterator {
    return diag_iterator(data(), _Array<sizeof...(I), T, I...>::diag_step());
  }
  //! \brief Returns a const iterator to the first diagonal element.
  [[nodiscard]] auto cdiag_begin() const -> const_diag_iterator {
    return const_diag_iterator(data(), _Array<sizeof...(I), T, I...>::diag_step());
  }
  //! \brief Returns an iterator to the first element afer the last diagonal
  //! element.
  [[nodiscard]] auto diag_end() const -> const_diag_iterator {
    return const_diag_iterator(data() + _Array<sizeof...(I), T, I...>::min_size() *
                                            _Array<sizeof...(I), T, I...>::diag_step(),
                               _Array<sizeof...(I), T, I...>::diag_step());
  }
  auto diag_end() -> diag_iterator {
    return diag_iterator(data() + _Array<sizeof...(I), T, I...>::min_size() *
                                      _Array<sizeof...(I), T, I...>::diag_step(),
                         _Array<sizeof...(I), T, I...>::diag_step());
  }
  //! \brief Returns a const iterator to the first element afer the last
  //! diagonal element.
  [[nodiscard]] auto cdiag_end() const -> const_diag_iterator {
    return const_diag_iterator(data() + _Array<sizeof...(I), T, I...>::min_size() *
                                            _Array<sizeof...(I), T, I...>::diag_step(),
                               _Array<sizeof...(I), T, I...>::diag_step());
  }
  //! \brief Returns m(i, j, k, ...)
  template <typename... J> auto operator()(J... idx) const -> T { return m_v.get(idx...); }
  template <typename... J> auto operator()(J... idx) -> T & { return m_v.get(idx...); }
  //! \brief Unary - operator.
  auto operator-() const -> container_type {
    container_type v;
    std::transform(begin(), end(), v.begin(), [](T x) { return -x; });
    return v;
  }
  //! \brief += scalar operator.
  auto operator+=(T v) -> container_type & {
    std::for_each(begin(), end(), [v](T &a) { a += v; });
    return *this;
  }
  //! \brief -= scalar operator.
  auto operator-=(T v) -> container_type & {
    std::for_each(begin(), end(), [v](T &a) { a -= v; });
    return *this;
  }
  //! \brief /= scalar operator.
  auto operator/=(T v) -> container_type & {
    std::for_each(begin(), end(), [v](T &a) { a /= v; });
    return *this;
  }
  //! \brief *= scalar operator.
  auto operator*=(T v) -> container_type & {
    std::for_each(begin(), end(), [v](T &a) { a *= v; });
    return *this;
  }
  //! \brief += operator.
  template <typename OTHER, class = typename OTHER::const_iterator>
  auto operator+=(const OTHER &that) -> container_type & {
    std::transform(begin(), end(), that.begin(), begin(),
                   [](T v1, typename OTHER::value_type v2) { return (v1 + v2); });
    return *this;
  }
  //! \brief += operator.
  template <typename OTHER, class = typename OTHER::const_iterator>
  auto operator-=(const OTHER &that) -> container_type & {
    std::transform(begin(), end(), that.begin(), begin(),
                   [](T v1, typename OTHER::value_type v2) { return (v1 - v2); });
    return *this;
  }
  //! \brief Return array filled with 0
  static auto zeros() -> container_type {
    container_type out;
    std::fill(out.begin(), out.end(), T{0});
    return out;
  }
  //! \brief Return array with diagonal filled with 1
  static auto eye() -> container_type {
    container_type out;

    std::fill(out.begin(), out.end(), T{0});
    std::fill(out.diag_begin(), out.diag_end(), T{1});

    return out;
  }
  template <typename OTHER> static auto from(const OTHER &other) -> container_type {
    container_type out;
    std::transform(other.begin(), other.end(), out.begin(),
                   [](auto v) { return static_cast<T>(v); });
    return out;
  }
};

template <typename T, size_type... I> typename Array<T, I...>::Helper Array<T, I...>::m_helper;

} // namespace stack

namespace heap {
using size_type = TMIV::Common::Array::size_type;

template <size_type D, typename T> class Array {
public:
  using value_type = T;
  using reference = T &;
  using const_reference = const T &;
  using iterator = TMIV::Common::Array::iterator<T>;
  using const_iterator = TMIV::Common::Array::const_iterator<T>;
  using dim_iterator = TMIV::Common::Array::dim_iterator<T>;
  using const_dim_iterator = TMIV::Common::Array::const_dim_iterator<T>;
  using diag_iterator = TMIV::Common::Array::dim_iterator<T>;
  using const_diag_iterator = TMIV::Common::Array::const_dim_iterator<T>;
  using difference_type = std::ptrdiff_t;
  using size_type = heap::size_type;
  using container_type = Array<D, T>;
  using tuple_type = std::array<heap::size_type, D>;
  template <typename U> using promoted_type = Array<D, std::common_type_t<T, U>>;

protected:
  std::array<size_type, D> m_size;
  std::array<size_type, D + 1> m_step;
  std::vector<T> m_v;
  int m_property = -1;

public:
  //! \brief Default constructors.
  Array() {
    m_size.fill(0);
    m_step.fill(0);
  }
  explicit Array(const tuple_type &sz) : Array() { this->resize(sz); }
  //! \brief Destructor.
  ~Array() = default;
  //! \brief Copy constructors.
  Array(const Array &that) = default;
  Array(const tuple_type &sz, T v) : Array() {
    this->resize(sz);
    std::fill(begin(), end(), v);
  }
  Array(const tuple_type &sz, std::initializer_list<T> v) : Array() {
    this->resize(sz);
    m_v = v;
  }
  template <typename OTHER, class = typename OTHER::dim_iterator>
  explicit Array(const OTHER &that) : Array() {
    tuple_type sz;

    std::copy(that.sizes().begin(), that.sizes().end(), sz.begin());
    std::fill(sz.begin() + that.sizes().size(), sz.end(), 1);

    this->resize(sz);
    std::transform(that.begin(), that.end(), begin(), [](auto v) { return static_cast<T>(v); });

    m_property = that.getProperty();
  }
  //! \brief Move constructor.
  Array(Array &&that) noexcept {
    m_size = that.m_size;
    m_step = that.m_step;
    m_v = std::move(that.m_v);
    m_property = that.m_property;

    that.m_size.fill(0);
    that.m_step.fill(0);
    that.m_property = -1;
  }
  //! \brief Copy assignment.
  auto operator=(const Array &that) -> Array & = default;
  template <typename OTHER, class = typename OTHER::dim_iterator>
  auto operator=(const OTHER &that) -> Array & {
    tuple_type sz;

    std::copy(that.sizes().begin(), that.sizes().end(), sz.begin());
    std::fill(sz.begin() + that.sizes().size(), sz.end(), 1);

    this->resize(sz);
    std::transform(that.begin(), that.end(), begin(), [](auto v) { return static_cast<T>(v); });

    m_property = that.getProperty();

    return *this;
  }
  auto operator=(T v) -> Array & {
    std::fill(begin(), end(), v);
    return *this;
  }
  //! \brief Move assignment.
  auto operator=(Array &&that) noexcept -> Array & {
    m_size = that.m_size;
    m_step = that.m_step;
    m_v = std::move(that.m_v);
    m_property = that.m_property;

    that.m_size.fill(0);
    that.m_step.fill(0);
    that.m_property = -1;

    return *this;
  }
  //! \brief Equal operator.
  auto operator==(const Array &that) const -> bool {
    return (std::equal(m_size.begin(), m_size.end(), that.m_size.begin()) &&
            std::equal(begin(), end(), that.begin()));
  }
  //! \brief Different operator.
  auto operator!=(const Array &that) const -> bool {
    return (!std::equal(m_size.begin(), m_size.end(), that.m_size.begin()) ||
            !std::equal(begin(), end(), that.begin()));
  }
  //! \brief Swap operator
  void swap(Array &that) {
    std::swap(m_size, that.m_size);
    std::swap(m_step, that.m_step);
    m_v.swap(that.m_v);
    std::swap(m_property, that.m_property);
  }
  //! \brief Resize operator.
  void resize(const tuple_type &sz) {
    if (std::equal(m_size.begin(), m_size.end(), sz.begin())) {
      return;
    }

    // Dimensions
    std::copy(sz.begin(), sz.end(), m_size.begin());

    // Lengths
    size_type l = 1;

    m_step.back() = 1;
    std::transform(m_size.rbegin(), m_size.rend(), m_step.rbegin() + 1, [&l](size_type s) {
      l *= s;
      return l;
    });

    // Data
    m_v.resize(m_step.front());
  }
  //! \brief Reshape operator.
  void reshape(const tuple_type &sz) {
    if (std::equal(m_size.begin(), m_size.end(), sz.begin())) {
      return;
    }

    // Dimensions
    std::copy(sz.begin(), sz.end(), m_size.begin());

    // Lengths
    size_type l = 1;

    m_step.back() = 1;
    std::transform(m_size.rbegin(), m_size.rend(), m_step.rbegin() + 1, [&l](size_type s) {
      l *= s;
      return l;
    });
  }
  //! \brief Returns the array dimension.
  static constexpr auto dim() -> size_type { return D; }
  //! \brief Returns the array size along the i-th dimension.
  [[nodiscard]] auto size(size_type i) const -> size_type { return m_size[i]; }
  //! \brief Returns the array sizes.
  [[nodiscard]] auto sizes() const -> const tuple_type & { return m_size; }
  //! \brief Returns the array total length.
  [[nodiscard]] auto size() const -> size_type { return m_step.front(); }
  //! \brief Returns the gap between 2 consecutive elements on the ith
  //! dimension.
  [[nodiscard]] auto step(size_type i) const -> size_type { return m_step[i + 1]; }
  //! \brief Returns the array steps
  [[nodiscard]] auto steps() const -> const std::array<size_type, D + 1> & { return m_step; }
  //! \brief Returns true if the array is empty.
  [[nodiscard]] auto empty() const -> bool { return (size() == 0); }
  //! \brief Data access, returns a pointer to the first element of the array.
  auto data() -> T * { return m_v.data(); }
  [[nodiscard]] auto data() const -> const T * { return m_v.data(); }
  //! \brief [] operator, returns the kth element of the array viewed as a one
  //! dimensional array.
  auto operator[](size_type k) const -> T { return m_v[k]; }
  auto operator[](size_type k) -> T & { return m_v[k]; }
  //! \brief Return the property of the array
  [[nodiscard]] auto getProperty() const -> int { return m_property; }
  // Set the property of the array
  void setProperty(int v) { m_property = v; }
  //! \brief Returns an iterator to the first element of the array.
  auto begin() -> iterator { return iterator(data()); }
  [[nodiscard]] auto begin() const -> const_iterator { return const_iterator(data()); }
  //! \brief Returns a const iterator to the first element of the array.
  [[nodiscard]] auto cbegin() const -> const_iterator { return const_iterator(data()); }
  //! \brief Returns an iterator to the first element after the end of the
  //! array.
  auto end() -> iterator {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return iterator(data() + size());
  }
  [[nodiscard]] auto end() const -> const_iterator {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return const_iterator(data() + size());
  }
  //! \brief Returns a const iterator to the first element after the end of the
  //! array.
  [[nodiscard]] auto cend() const -> const_iterator { return const_iterator(data() + size()); }
  //! \brief Returns an iterator along the Kth dimension to the first element of
  //! the hyperplane defined by next.
  template <size_type K, typename... I>
  [[nodiscard]] auto dim_begin(I... next) const -> const_dim_iterator {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return const_dim_iterator(data() + offset<K>(0, next...), m_step[K + 1]);
  }
  template <size_type K, typename... I> auto dim_begin(I... next) -> dim_iterator {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return dim_iterator(data() + offset<K>(0, next...), m_step[K + 1]);
  }
  //! \brief Returns a const iterator along the Kth dimension to the first
  //! element of the hyperplane defined by next.
  template <size_type K, typename... I> auto cdim_begin(I... next) const -> const_dim_iterator {
    return const_dim_iterator(data() + offset<K>(0, next...), m_step[K + 1]);
  }
  //! \brief Returns an iterator along the Kth dimension to the first element
  //! after the end of the hyperplane defined by next.
  template <size_type K, typename... I>
  [[nodiscard]] auto dim_end(I... next) const -> const_dim_iterator {
    return const_dim_iterator(data() + offset<K>(0, next...) + m_step[K], m_step[K + 1]);
  }
  template <size_type K, typename... I> auto dim_end(I... next) -> dim_iterator {
    return dim_iterator(data() + offset<K>(0, next...) + m_step[K], m_step[K + 1]);
  }
  //! \brief Returns a const iterator along the Kth dimension to the first
  //! element after the end of the hyperplane defined by next.
  template <size_type K, typename... I> auto cdim_end(I... next) const -> const_dim_iterator {
    return const_dim_iterator(data() + offset<K>(0, next...) + m_step[K], m_step[K + 1]);
  }
  //! \brief Returns an iterator to the first diagonal element.
  [[nodiscard]] auto diag_begin() const -> const_diag_iterator {
    return const_diag_iterator(data(), std::accumulate(m_step.begin() + 1, m_step.end(), size_t{}));
  }
  auto diag_begin() -> diag_iterator {
    return diag_iterator(data(), std::accumulate(m_step.begin() + 1, m_step.end(), size_t{}));
  }
  //! \brief Returns a const iterator to the first diagonal element.
  [[nodiscard]] auto cdiag_begin() const -> const_diag_iterator {
    return const_diag_iterator(data(), std::accumulate(m_step.begin() + 1, m_step.end(), size_t{}));
  }
  //! \brief Returns an iterator to the first element afer the last diagonal
  //! element.
  [[nodiscard]] auto diag_end() const -> const_diag_iterator {
    size_type d = std::accumulate(m_step.begin() + 1, m_step.end(), size_t{});
    return const_diag_iterator(data() + *std::min_element(m_size.begin(), m_size.end()) * d, d);
  }
  auto diag_end() -> diag_iterator {
    size_type d = std::accumulate(m_step.begin() + 1, m_step.end(), size_type{});
    return diag_iterator(data() + *std::min_element(m_size.begin(), m_size.end()) * d, d);
  }
  //! \brief Returns a const iterator to the first element afer the last
  //! diagonal element.
  [[nodiscard]] auto cdiag_end() const -> const_diag_iterator {
    size_type d = std::accumulate(m_step.begin() + 1, m_step.end(), size_t{});
    return const_diag_iterator(data() + *std::min_element(m_size.begin(), m_size.end()) * d, d);
  }
  //! \brief Returns m(i, j, k, ...)
  template <typename... I> auto operator()(size_type first, I... next) const -> T {
    return m_v[pos(1, first, next...)];
  }
  template <typename... I> auto operator()(size_type first, I... next) -> T & {
    return m_v[pos(1, first, next...)];
  }
  //! \brief Returns distance of m(i, j, k, ...) from m(0, 0, 0, ...)
  template <typename... I> auto distance(size_type first, I... next) const -> size_type {
    return pos(1, first, next...);
  }
  //! \brief Unary - operator.
  auto operator-() const -> Array {
    Array v(sizes());
    std::transform(begin(), end(), v.begin(), [](T x) { return -x; });
    return v;
  }
  //! \brief += scalar operator.
  auto operator+=(T v) -> Array & {
    std::for_each(begin(), end(), [v](T &a) { a += v; });
    return *this;
  }
  //! \brief -= scalar operator.
  auto operator-=(T v) -> Array & {
    std::for_each(begin(), end(), [v](T &a) { a -= v; });
    return *this;
  }
  //! \brief /= scalar operator.
  auto operator/=(T v) -> Array & {
    std::for_each(begin(), end(), [v](T &a) { a /= v; });
    return *this;
  }
  //! \brief *= scalar operator.
  auto operator*=(T v) -> Array & {
    std::for_each(begin(), end(), [v](T &a) { a *= v; });
    return *this;
  }
  //! \brief += operator.
  template <typename OTHER, class = typename OTHER::const_iterator>
  auto operator+=(const OTHER &that) -> Array & {
    std::transform(begin(), end(), that.begin(), begin(),
                   [](T v1, typename OTHER::value_type v2) { return (v1 + v2); });
    return *this;
  }
  //! \brief -= operator.
  template <typename OTHER, class = typename OTHER::const_iterator>
  auto operator-=(const OTHER &that) -> Array & {
    std::transform(begin(), end(), that.begin(), begin(),
                   [](T v1, typename OTHER::value_type v2) { return (v1 - v2); });
    return *this;
  }
  //! \brief Return array filled with 0
  static auto zeros(const tuple_type &sz) -> Array {
    Array out(sz);
    std::fill(out.begin(), out.end(), 0);
    return out;
  }
  //! \brief Return array with diagonal filled with 1
  static auto eye(const tuple_type &sz) -> Array {
    Array out(sz);

    std::fill(out.begin(), out.end(), T{0});
    std::fill(out.diag_begin(), out.diag_end(), T{1});

    return out;
  }

protected:
  template <size_type K>
  [[nodiscard]] auto offset(size_type i, size_type first = 0) const -> size_type {
    return (i == K) ? first : first * m_step[i + 1];
  }
  template <size_type K, typename... I>
  [[nodiscard]] auto offset(size_type i, size_type first, I... next) const -> size_type {
    return (i == K) ? offset<K>(i + 1, first, next...)
                    : first * m_step[i + 1] + offset<K>(i + 1, next...);
  }
  [[nodiscard]] auto pos(size_type /*unused*/, size_type first) const -> size_type { return first; }
  template <typename... I>
  [[nodiscard]] auto pos(size_type i, size_type first, I... next) const -> size_type {
    return first * m_step[i] + pos(i + 1, next...);
  }
};

} // namespace heap

namespace shallow {
using size_type = TMIV::Common::Array::size_type;

template <size_type D, typename T> class Array {
public:
  using value_type = T;
  using reference = T &;
  using const_reference = const T &;
  using iterator = TMIV::Common::Array::iterator<T>;
  using const_iterator = TMIV::Common::Array::const_iterator<T>;
  using dim_iterator = TMIV::Common::Array::dim_iterator<T>;
  using const_dim_iterator = TMIV::Common::Array::const_dim_iterator<T>;
  using diag_iterator = TMIV::Common::Array::dim_iterator<T>;
  using const_diag_iterator = TMIV::Common::Array::const_dim_iterator<T>;
  using difference_type = std::ptrdiff_t;
  using size_type = shallow::size_type;
  using container_type = Array<D, T>;
  using tuple_type = std::array<shallow::size_type, D>;
  template <typename U> using promoted_type = Array<D, std::common_type_t<T, U>>;

protected:
  std::array<size_type, D> m_size;
  std::array<size_type, D + 1> m_step;
  T *m_data = nullptr;
  int m_property = -1;

public:
  //! \brief Default constructors.
  Array() {
    m_size.fill(0);
    m_step.fill(0);
  }
  //! \brief Destructor.
  ~Array() = default;
  //! \brief Copy constructors.
  Array(const tuple_type &sz, T *src) : Array() {
    this->reshape(sz);
    m_data = src;
  }
  Array(const container_type &that) = default;
  template <typename OTHER, class = typename OTHER::dim_iterator>
  explicit Array(const OTHER &that,
                 SameTypeChecker<T, typename OTHER::value_type> * /*unused*/ = nullptr)
      : Array() {
    tuple_type sz;

    std::copy(that.sizes().begin(), that.sizes().end(), sz.begin());
    std::fill(sz.begin() + that.sizes().size(), sz.end(), 1);

    this->reshape(sz);
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    const auto *data = reinterpret_cast<const T *>(that.data());
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
    m_data = const_cast<T *>(data);

    m_property = that.getProperty();
  }
  //! \brief Move constructor.
  Array(Array &&that) noexcept {
    m_size = that.m_size;
    m_step = that.m_step;
    m_data = that.m_data;
    m_property = that.m_property;

    that.m_size.fill(0);
    that.m_step.fill(0);
    that.m_data = nullptr;
    that.m_property = -1;
  }
  //! \brief Copy assignment.
  auto operator=(const Array &that) -> Array & = default;
  template <typename OTHER, class = typename OTHER::dim_iterator>
  auto operator=(const OTHER &that) -> Array & {
    if (size() == that.size()) {
      tuple_type sz;

      std::copy(that.sizes().begin(), that.sizes().end(), sz.begin());
      std::fill(sz.begin() + that.sizes().size(), sz.end(), 1);

      this->reshape(sz);
      std::transform(that.begin(), that.end(), begin(), [](auto v) { return static_cast<T>(v); });

      m_property = that.getProperty();
    }

    return *this;
  }
  auto operator=(T v) -> Array & {
    std::fill(begin(), end(), v);
    return *this;
  }
  //! \brief Move assignment.
  auto operator=(Array &&that) noexcept -> Array & {
    m_size = that.m_size;
    m_step = that.m_step;
    m_data = that.m_data;
    m_property = that.m_property;

    that.m_size.fill(0);
    that.m_step.fill(0);
    that.m_data = nullptr;
    that.m_property = -1;

    return *this;
  }
  //! \brief Equal operator.
  auto operator==(const Array &that) const -> bool {
    return (std::equal(m_size.begin(), m_size.end(), that.m_size.begin()) &&
            std::equal(begin(), end(), that.begin()));
  }
  //! \brief Different operator.
  auto operator!=(const Array &that) const -> bool {
    return (!std::equal(m_size.begin(), m_size.end(), that.m_size.begin()) ||
            !std::equal(begin(), end(), that.begin()));
  }
  //! \brief Swap operator
  void swap(Array &that) {
    std::swap(m_size, that.m_size);
    std::swap(m_step, that.m_step);
    std::swap(m_data, that.m_data);
    std::swap(m_property, that.m_property);
  }
  //! \brief Resize operator.
  void resize(const tuple_type & /*unused*/) {}
  //! \brief Reshape operator.
  void reshape(const tuple_type &sz) {
    if (std::equal(m_size.begin(), m_size.end(), sz.begin())) {
      return;
    }

    // Dimensions
    std::copy(sz.begin(), sz.end(), m_size.begin());

    // Lengths
    size_type l = 1;

    m_step.back() = 1;
    std::transform(m_size.rbegin(), m_size.rend(), m_step.rbegin() + 1, [&l](size_type s) {
      l *= s;
      return l;
    });
  }
  //! \brief Returns the array dimension.
  static constexpr auto dim() -> size_type { return D; }
  //! \brief Returns the array size along the i-th dimension.
  [[nodiscard]] auto size(size_type i) const -> size_type { return m_size[i]; }
  //! \brief Returns the array sizes.
  [[nodiscard]] auto sizes() const -> const tuple_type & { return m_size; }
  //! \brief Returns the array total length.
  [[nodiscard]] auto size() const -> size_type { return m_step.front(); }
  //! \brief Returns the gap between 2 consecutive elements on the ith
  //! dimension.
  [[nodiscard]] auto step(size_type i) const -> size_type { return m_step[i + 1]; }
  //! \brief Returns true if the array is empty.
  [[nodiscard]] auto empty() const -> bool { return (size() == 0); }
  //! \brief Data access, returns a pointer to the first element of the array.
  auto data() -> T * { return m_data; }
  [[nodiscard]] auto data() const -> const T * { return m_data; }
  //! \brief [] operator, returns the kth element of the array viewed as a one
  //! dimensional array.
  auto operator[](int k) const -> T {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return m_data[k];
  }
  auto operator[](int k) -> T & {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return m_data[k];
  }
  //! \brief Return the property of the array
  [[nodiscard]] auto getProperty() const -> int { return m_property; }
  // \brief Set the property of the array
  void setProperty(int v) { m_property = v; }
  //! \brief Returns an iterator to the first element of the array.
  auto begin() -> iterator { return iterator(m_data); }
  [[nodiscard]] auto begin() const -> const_iterator { return const_iterator(m_data); }
  //! \brief Returns a const iterator to the first element of the array.
  [[nodiscard]] auto cbegin() const -> const_iterator { return const_iterator(m_data); }
  //! \brief Returns an iterator to the first element after the end of the
  //! array.
  auto end() -> iterator { return iterator(m_data + size()); }
  [[nodiscard]] auto end() const -> const_iterator { return const_iterator(m_data + size()); }
  //! \brief Returns a const iterator to the first element after the end of the
  //! array.
  [[nodiscard]] auto cend() const -> const_iterator { return const_iterator(m_data + size()); }
  //! \brief Returns an iterator along the Kth dimension to the first element of
  //! the hyperplane defined by next.
  template <size_type K, typename... I>
  [[nodiscard]] auto dim_begin(I... next) const -> const_dim_iterator {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return const_dim_iterator(m_data + offset<K>(0, next...), m_step[K + 1]);
  }
  template <size_type K, typename... I> auto dim_begin(I... next) -> dim_iterator {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return dim_iterator(m_data + offset<K>(0, next...), m_step[K + 1]);
  }
  //! \brief Returns a const iterator along the Kth dimension to the first
  //! element of the hyperplane defined by next.
  template <size_type K, typename... I> auto cdim_begin(I... next) const -> const_dim_iterator {
    return const_dim_iterator(m_data + offset<K>(0, next...), m_step[K + 1]);
  }
  //! \brief Returns an iterator along the Kth dimension to the first element
  //! after the end of the hyperplane defined by next.
  template <size_type K, typename... I>
  [[nodiscard]] auto dim_end(I... next) const -> const_dim_iterator {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return const_dim_iterator(m_data + offset<K>(0, next...) + m_step[K], m_step[K + 1]);
  }
  template <size_type K, typename... I> auto dim_end(I... next) -> dim_iterator {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return dim_iterator(m_data + offset<K>(0, next...) + m_step[K], m_step[K + 1]);
  }
  //! \brief Returns a const iterator along the Kth dimension to the first
  //! element after the end of the hyperplane defined by next.
  template <size_type K, typename... I> auto cdim_end(I... next) const -> const_dim_iterator {
    return const_dim_iterator(m_data + offset<K>(0, next...) + m_step[K], m_step[K + 1]);
  }
  //! \brief Returns an iterator to the first diagonal element.
  [[nodiscard]] auto diag_begin() const -> const_diag_iterator {
    return const_diag_iterator(m_data, std::accumulate(m_step.begin() + 1, m_step.end(), 0));
  }
  auto diag_begin() -> diag_iterator {
    return diag_iterator(m_data, std::accumulate(m_step.begin() + 1, m_step.end(), 0));
  }
  //! \brief Returns a const iterator to the first diagonal element.
  [[nodiscard]] auto cdiag_begin() const -> const_diag_iterator {
    return const_diag_iterator(m_data, std::accumulate(m_step.begin() + 1, m_step.end(), 0));
  }
  //! \brief Returns an iterator to the first element afer the last diagonal
  //! element.
  [[nodiscard]] auto diag_end() const -> const_diag_iterator {
    size_type d = std::accumulate(m_step.begin() + 1, m_step.end(), 0);
    return const_diag_iterator(m_data + *std::min_element(m_size.begin(), m_size.end()) * d, d);
  }
  auto diag_end() -> diag_iterator {
    size_type d = std::accumulate(m_step.begin() + 1, m_step.end(), 0);
    return diag_iterator(m_data + *std::min_element(m_size.begin(), m_size.end()) * d, d);
  }
  //! \brief Returns a const iterator to the first element afer the last
  //! diagonal element.
  [[nodiscard]] auto cdiag_end() const -> const_diag_iterator {
    size_type d = std::accumulate(m_step.begin() + 1, m_step.end(), 0);
    return const_diag_iterator(m_data + *std::min_element(m_size.begin(), m_size.end()) * d, d);
  }
  //! \brief Returns m(i, j, k, ...)
  template <typename... I> auto operator()(size_type first, I... next) const -> T {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return m_data[pos(1, first, next...)];
  }
  template <typename... I> auto operator()(size_type first, I... next) -> T & {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return m_data[pos(1, first, next...)];
  }
  //! \brief Returns distance of m(i, j, k, ...) from m(0, 0, 0, ...)
  template <typename... I> auto distance(size_type first, I... next) const -> size_type {
    return pos(1, first, next...);
  }
  //! \brief Unary - operator.
  auto operator-() const -> container_type {
    container_type v(sizes());
    std::transform(begin(), end(), v.begin(), [](T x) { return -x; });
    return v;
  }
  //! \brief += scalar operator.
  auto operator+=(T v) -> container_type & {
    std::for_each(begin(), end(), [v](T &a) { a += v; });
    return *this;
  }
  //! \brief -= scalar operator.
  auto operator-=(T v) -> container_type & {
    std::for_each(begin(), end(), [v](T &a) { a -= v; });
    return *this;
  }
  //! \brief /= scalar operator.
  auto operator/=(T v) -> container_type & {
    std::for_each(begin(), end(), [v](T &a) { a /= v; });
    return *this;
  }
  //! \brief *= scalar operator.
  auto operator*=(T v) -> container_type & {
    std::for_each(begin(), end(), [v](T &a) { a *= v; });
    return *this;
  }
  //! \brief += operator.
  template <typename OTHER, class = typename OTHER::const_iterator>
  auto operator+=(const OTHER &that) -> container_type & {
    std::transform(begin(), end(), that.begin(), begin(),
                   [](T v1, typename OTHER::value_type v2) { return (v1 + v2); });
    return *this;
  }
  //! \brief -= operator.
  template <typename OTHER, class = typename OTHER::const_iterator>
  auto operator-=(const OTHER &that) -> container_type & {
    std::transform(begin(), end(), that.begin(), begin(),
                   [](T v1, typename OTHER::value_type v2) { return (v1 - v2); });
    return *this;
  }

protected:
  template <size_type K>
  [[nodiscard]] auto offset(size_type i, size_type first = 0) const -> size_type {
    return (i == K) ? first : first * m_step[i + 1];
  }
  template <size_type K, typename... I>
  [[nodiscard]] auto offset(size_type i, size_type first, I... next) const -> size_type {
    return (i == K) ? offset<K>(i + 1, first, next...)
                    : first * m_step[i + 1] + offset<K>(i + 1, next...);
  }
  [[nodiscard]] auto pos(size_type /*unused*/, size_type first) const -> size_type { return first; }
  template <typename... I>
  [[nodiscard]] auto pos(size_type i, size_type first, I... next) const -> size_type {
    return first * m_step[i] + pos(i + 1, next...);
  }
};

} // namespace shallow

} // namespace TMIV::Common

//! \brief Send the array a to the stream os.
template <typename A, class = typename A::dim_iterator>
auto operator<<(std::ostream &os, const A &a) -> std::ostream & {
  typename A::size_type step = a.size(a.dim() - 1);
  typename A::const_iterator iter, iter1 = a.begin(), iter2 = a.end();

  for (iter = iter1; iter != iter2; iter += step) {
    std::for_each(iter, iter + step, [&os](typename A::value_type v) { os << v << " "; });
    if ((iter + step) != iter2) {
      os << '\n';
    }
  }

  return os;
}

//! \brief Load the array a from the stream is.
template <typename A, class = typename A::dim_iterator>
auto operator>>(std::istream &is, A &a) -> std::istream & {
  for (auto &e : a) {
    is >> e;
  }
  return is;
}

//! \brief Return true if a1 and a2 have the same size.
template <typename A1, typename A2> auto same_size(const A1 &a1, const A2 &a2) -> bool {
  if (a1.dim() != a2.dim()) {
    return false;
  }

  for (typename A1::size_type i = 0; i < a1.dim(); i++) {
    if (a1.size(i) != a2.size(i)) {
      return false;
    }
  }

  return true;
}

//! \brief array/scalar + operator.
template <typename A1, typename U, typename A2, class = typename A1::dim_iterator,
          class = typename TMIV::Common::NumericChecker<U>, class = typename A2::dim_iterator>
void add(const A1 &m, U u, A2 &out) {
  out.resize(m.sizes());
  std::transform(m.begin(), m.end(), out.begin(),
                 [u](typename A1::value_type v) -> typename A2::value_type { return (v + u); });
}
template <typename A1, typename U, typename A2, class = typename A1::dim_iterator,
          class = typename TMIV::Common::NumericChecker<U>, class = typename A2::dim_iterator>
void add(U u, const A1 &m, A2 &out) {
  out.resize(m.sizes());
  std::transform(m.begin(), m.end(), out.begin(),
                 [u](typename A1::value_type v) -> typename A2::value_type { return (u + v); });
}
template <typename A1, typename U, class = typename A1::diag_iterator,
          class = typename TMIV::Common::NumericChecker<U>>
auto operator+(const A1 &m, U u) -> typename A1::template promoted_type<U> {
  typename A1::template promoted_type<U> out;
  add(m, u, out);
  return out;
}
template <typename A1, typename U, class = typename A1::diag_iterator,
          class = typename TMIV::Common::NumericChecker<U>>
auto operator+(U u, const A1 &m) -> typename A1::template promoted_type<U> {
  typename A1::template promoted_type<U> out;
  add(u, m, out);
  return out;
}

//! \brief array/scalar - operator.
template <typename A1, typename U, typename A2, class = typename A1::dim_iterator,
          class = typename TMIV::Common::NumericChecker<U>, class = typename A2::dim_iterator>
void sub(const A1 &m, U u, A2 &out) {
  out.resize(m.sizes());
  std::transform(m.begin(), m.end(), out.begin(),
                 [u](typename A1::value_type v) -> typename A2::value_type { return (v - u); });
}
template <typename A1, typename U, typename A2, class = typename A1::dim_iterator,
          class = typename TMIV::Common::NumericChecker<U>, class = typename A2::dim_iterator>
void sub(U u, const A1 &m, A2 &out) {
  out.resize(m.sizes());
  std::transform(m.begin(), m.end(), out.begin(),
                 [u](typename A1::value_type v) -> typename A2::value_type { return (u - v); });
}
template <typename A1, typename U, class = typename A1::diag_iterator,
          class = typename TMIV::Common::NumericChecker<U>>
auto operator-(const A1 &m, U u) -> typename A1::template promoted_type<U> {
  typename A1::template promoted_type<U> out;
  sub(m, u, out);
  return out;
}
template <typename A1, typename U, class = typename A1::diag_iterator,
          class = typename TMIV::Common::NumericChecker<U>>
auto operator-(U u, const A1 &m) -> typename A1::template promoted_type<U> {
  typename A1::template promoted_type<U> out;
  sub(u, m, out);
  return out;
}

//! \brief array/scalar * operator.
template <typename A1, typename U, typename A2, class = typename A1::dim_iterator,
          class = typename TMIV::Common::NumericChecker<U>, class = typename A2::dim_iterator>
void mult(const A1 &m, U u, A2 &out) {
  out.resize(m.sizes());
  std::transform(m.begin(), m.end(), out.begin(),
                 [u](typename A1::value_type v) -> typename A2::value_type { return (v * u); });
}
template <typename A1, typename U, typename A2, class = typename A1::dim_iterator,
          class = typename TMIV::Common::NumericChecker<U>, class = typename A2::dim_iterator>
void mult(U u, const A1 &m, A2 &out) {
  out.resize(m.sizes());
  std::transform(m.begin(), m.end(), out.begin(),
                 [u](typename A1::value_type v) -> typename A2::value_type { return (u * v); });
}
template <typename A1, typename U, class = typename A1::diag_iterator,
          class = typename TMIV::Common::NumericChecker<U>>
auto operator*(const A1 &m, U u) -> typename A1::template promoted_type<U> {
  typename A1::template promoted_type<U> out;
  mult(m, u, out);
  return out;
}
template <typename A1, typename U, class = typename A1::diag_iterator,
          class = typename TMIV::Common::NumericChecker<U>>
auto operator*(U u, const A1 &m) -> typename A1::template promoted_type<U> {
  typename A1::template promoted_type<U> out;
  mult(u, m, out);
  return out;
}

//! \brief array/scalar / operator.
template <typename A1, typename U, typename A2, class = typename A1::dim_iterator,
          class = typename TMIV::Common::NumericChecker<U>, class = typename A2::dim_iterator>
void div(const A1 &m, U u, A2 &out) {
  out.resize(m.sizes());
  std::transform(m.begin(), m.end(), out.begin(),
                 [u](typename A1::value_type v) -> typename A2::value_type { return (v / u); });
}
template <typename A1, typename U, class = typename A1::diag_iterator,
          class = typename TMIV::Common::NumericChecker<U>>
auto operator/(const A1 &m, U u) -> typename A1::template promoted_type<U> {
  typename A1::template promoted_type<U> out;
  div(m, u, out);
  return out;
}

//! \brief array/array + operator.
template <typename A1, typename A2, typename A3, class = typename A1::dim_iterator,
          class = typename A2::dim_iterator, class = typename A3::dim_iterator>
void add(const A1 &m1, const A2 &m2, A3 &out) {
  out.resize(m1.sizes());
  std::transform(m1.begin(), m1.end(), m2.begin(), out.begin(),
                 [](typename A1::value_type v1, typename A2::value_type v2) ->
                 typename A3::value_type { return v1 + v2; });
}
template <typename A1, typename A2, class = typename A1::dim_iterator,
          class = typename A2::dim_iterator>
auto operator+(const A1 &m1, const A2 &m2) ->
    typename A1::template promoted_type<typename A2::value_type> {
  typename A1::template promoted_type<typename A2::value_type> out;
  add(m1, m2, out);
  return out;
}

//! \brief array/array - operator.
template <typename A1, typename A2, typename A3, class = typename A1::dim_iterator,
          class = typename A2::dim_iterator, class = typename A3::dim_iterator>
void sub(const A1 &m1, const A2 &m2, A3 &out) {
  out.resize(m1.sizes());
  std::transform(m1.begin(), m1.end(), m2.begin(), out.begin(),
                 [](typename A1::value_type v1, typename A2::value_type v2) ->
                 typename A3::value_type { return v1 - v2; });
}
template <typename A1, typename A2, class = typename A1::dim_iterator,
          class = typename A2::dim_iterator>
auto operator-(const A1 &m1, const A2 &m2) ->
    typename A1::template promoted_type<typename A2::value_type> {
  typename A1::template promoted_type<typename A2::value_type> out;
  sub(m1, m2, out);
  return out;
}

//! \brief Element-by-element multiplication operator.
template <typename A1, typename A2, typename A3, class = typename A1::dim_iterator,
          class = typename A2::dim_iterator, class = typename A3::dim_iterator>
void mult(const A1 &m1, const A2 &m2, A3 &out) {
  out.resize(m1.sizes());
  std::transform(m1.begin(), m1.end(), m2.begin(), out.begin(),
                 [](typename A1::value_type v1, typename A2::value_type v2) ->
                 typename A3::value_type { return v1 * v2; });
}
template <typename A1, typename A2, class = typename A1::dim_iterator,
          class = typename A2::dim_iterator>
auto mult(const A1 &m1, const A2 &m2) ->
    typename A1::template promoted_type<typename A2::value_type> {
  typename A1::template promoted_type<typename A2::value_type> out;
  mult(m1, m2, out);
  return out;
}

//! \brief Element-by-element division operator.
template <typename A1, typename A2, typename A3, class = typename A1::dim_iterator,
          class = typename A2::dim_iterator, class = typename A3::dim_iterator>
void div(const A1 &m1, const A2 &m2, A3 &out) {
  out.resize(m1.sizes());
  std::transform(m1.begin(), m1.end(), m2.begin(), out.begin(),
                 [](typename A1::value_type v1, typename A2::value_type v2) ->
                 typename A3::value_type { return v1 / v2; });
}
template <typename A1, typename A2, class = typename A1::dim_iterator,
          class = typename A2::dim_iterator>
auto div(const A1 &m1, const A2 &m2) ->
    typename A1::template promoted_type<typename A2::value_type> {
  typename A1::template promoted_type<typename A2::value_type> out;
  div(m1, m2, out);
  return out;
}

#endif
