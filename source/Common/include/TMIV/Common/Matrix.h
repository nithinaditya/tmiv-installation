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

#ifndef _TMIV_COMMON_MATRIX_H_
#define _TMIV_COMMON_MATRIX_H_

#include "Array.h"

namespace TMIV::Common {
template <typename A> class MatrixInterface : public A {
public:
  using size_type = typename A::size_type;
  using value_type = typename A::value_type;
  using const_row_iterator = typename A::const_dim_iterator;
  using row_iterator = typename A::dim_iterator;
  using const_column_iterator = typename A::const_dim_iterator;
  using column_iterator = typename A::dim_iterator;

  using A::A;
  MatrixInterface() : A() {}
  explicit MatrixInterface(const A &a) : A(a) {}
  explicit MatrixInterface(A &&a) : A(std::move(a)) {}
  using A::operator=;
  auto operator=(const A &a) -> MatrixInterface & {
    A::operator=(a);
    return *this;
  }
  auto operator=(A &&a) -> MatrixInterface & {
    A::operator=(std::move(a));
    return *this;
  }
  //! \brief Returns the number of rows of the matrix.
  [[nodiscard]] auto m() const -> size_type { return A::size(0); }
  //! \brief Returns the number of columns of the matrix.
  [[nodiscard]] auto n() const -> size_type { return A::size(1); }
  //! \brief Returns the number of rows of the matrix.
  [[nodiscard]] auto height() const -> size_type { return A::size(0); }
  //! \brief Returns the number of columns of the matrix.
  [[nodiscard]] auto width() const -> size_type { return A::size(1); }
  //! \brief Overloaded resize operator.
  using A::resize;
  void resize(size_type a, size_type b) { A::resize({a, b}); }
  //! \brief Returns an iterator to the first element of the ith row.
  [[nodiscard]] auto row_begin(size_type i) const -> const_row_iterator {
    return A::template dim_begin<1>(i);
  }
  auto row_begin(size_type i) -> row_iterator { return A::template dim_begin<1>(i); }
  //! \brief Returns a const iterator to the first element of the ith row.
  [[nodiscard]] auto crow_begin(size_type i) const -> const_row_iterator {
    return A::template cdim_begin<1>(i);
  }
  //! \brief Returns an iterator to the first element after the end of the ith
  //! row.
  [[nodiscard]] auto row_end(size_type i) const -> const_row_iterator {
    return A::template dim_end<1>(i);
  }
  auto row_end(size_type i) -> row_iterator { return A::template dim_end<1>(i); }
  //! \brief Returns a const iterator to the first element after the end of the
  //! ith row.
  [[nodiscard]] auto crow_end(size_type i) const -> const_row_iterator {
    return A::template cdim_end<1>(i);
  }
  //! \brief Returns an iterator to the first element of the jth column.
  [[nodiscard]] auto col_begin(size_type j) const -> const_column_iterator {
    return A::template dim_begin<0>(j);
  }
  auto col_begin(size_type j) -> column_iterator { return A::template dim_begin<0>(j); }
  //! \brief Returns a const iterator to the first element of the jth column.
  [[nodiscard]] auto ccol_begin(size_type j) const -> const_column_iterator {
    return A::template cdim_begin<0>(j);
  }
  //! \brief Returns an iterator to the first element after the end of the jth
  //! column.
  [[nodiscard]] auto col_end(size_type j) const -> const_column_iterator {
    return A::template dim_end<0>(j);
  }
  auto col_end(size_type j) -> column_iterator { return A::template dim_end<0>(j); }
  //! \brief Returns a const iterator to the first element after the end of the
  //! jth column.
  [[nodiscard]] auto ccol_end(size_type j) const -> const_column_iterator {
    return A::template cdim_end<0>(j);
  }
  //! \brief Returns true if the matrix is a row.
  [[nodiscard]] auto isRow() const -> bool { return (m() == 1); }
  //! \brief Returns true if the matrix is a column.
  [[nodiscard]] auto isColumn() const -> bool { return (n() == 1); }
  //! \brief Returns true if the matrix is positive.
  [[nodiscard]] auto isPositive() const -> bool;
};

namespace stack {
template <typename T, size_type M, size_type N> using Matrix = MatrixInterface<Array<T, M, N>>;
template <typename T> using Mat2x2 = Matrix<T, 2, 2>;
template <typename T> using Mat2x3 = Matrix<T, 2, 3>;
template <typename T> using Mat3x3 = Matrix<T, 3, 3>;
} // namespace stack

namespace heap {
template <typename T> using Matrix = MatrixInterface<Array<2, T>>;
}

namespace shallow {
template <typename T> using Matrix = MatrixInterface<Array<2, T>>;
}

// Additional definitions
using Mat2x2i = stack::Mat2x2<int32_t>;
using Mat3x3i = stack::Mat3x3<int32_t>;
using Mat2x2f = stack::Mat2x2<float>;
using Mat3x3f = stack::Mat3x3<float>;
template <typename T> using Mat = heap::Matrix<T>;

//! \brief Returns the type of the transpose of the matrix given as input.
template <typename T, Array::size_type M, Array::size_type N>
auto transpose_type(stack::Matrix<T, M, N>) -> stack::Matrix<T, N, M>;
template <typename T> auto transpose_type(heap::Matrix<T>) -> heap::Matrix<T>;

//! \brief Returns the transpose of the matrix given as input.
template <typename Mat1, typename Mat2> auto transpose(const Mat1 &in, Mat2 &out) -> Mat2 &;
template <typename Mat> auto transpose(const Mat &m) -> decltype(transpose_type(Mat()));

} // namespace TMIV::Common

#include "Matrix.hpp"

#endif
