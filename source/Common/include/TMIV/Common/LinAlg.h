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

#ifndef _TMIV_COMMON_LINALG_H_
#define _TMIV_COMMON_LINALG_H_

#include "Math.h"
#include "Matrix.h"
#include "Vector.h"

namespace TMIV::Common {
// Vector x Scalar and Scalar x Vector
template <typename T, typename X, typename = std::enable_if_t<std::is_arithmetic_v<X>>>
auto operator*(const heap::Vector<T> &A, X B) -> heap::Vector<std::common_type_t<T, X>>;
template <typename T, typename X, typename = std::enable_if_t<std::is_arithmetic_v<X>>>
auto operator*(X B, const heap::Vector<T> &A) -> heap::Vector<std::common_type_t<T, X>>;
template <typename T, size_t M, typename X, typename = std::enable_if_t<std::is_arithmetic_v<X>>>
auto operator*(const stack::Vector<T, M> &A, X B) -> stack::Vector<std::common_type_t<T, X>, M>;
template <typename T, size_t M, typename X, typename = std::enable_if_t<std::is_arithmetic_v<X>>>
auto operator*(X B, const stack::Vector<T, M> &A) -> stack::Vector<std::common_type_t<T, X>, M>;

// Vector-scalar & scalar-vector min
template <typename T> auto min(const heap::Vector<T> &A, T B) -> heap::Vector<T>;
template <typename T> auto min(T B, const heap::Vector<T> &A) -> heap::Vector<T>;
template <typename T, size_t M> auto min(const stack::Vector<T, M> &A, T B) -> stack::Vector<T, M>;
template <typename T, size_t M> auto min(T B, const stack::Vector<T, M> &A) -> stack::Vector<T, M>;

// Vector-scalar & scalar-vector max
template <typename T> auto max(const heap::Vector<T> &A, T B) -> heap::Vector<T>;
template <typename T> auto max(T B, const heap::Vector<T> &A) -> heap::Vector<T>;
template <typename T, size_t M> auto max(const stack::Vector<T, M> &A, T B) -> stack::Vector<T, M>;
template <typename T, size_t M> auto max(T B, const stack::Vector<T, M> &A) -> stack::Vector<T, M>;

// Vector-vector min
template <typename T>
auto min(const heap::Vector<T> &A, const heap::Vector<T> &B) -> heap::Vector<T>;
template <typename T, size_t M>
auto min(const stack::Vector<T, M> &A, const stack::Vector<T, M> &B) -> stack::Vector<T, M>;

// Vector-vector min
template <typename T>
auto max(const heap::Vector<T> &A, const heap::Vector<T> &B) -> heap::Vector<T>;
template <typename T, size_t M>
auto max(const stack::Vector<T, M> &A, const stack::Vector<T, M> &B) -> stack::Vector<T, M>;

// Vector abs
template <typename T> auto abs(const heap::Vector<T> &A) -> heap::Vector<T>;
template <typename T, size_t M> auto abs(const stack::Vector<T, M> &A) -> stack::Vector<T, M>;

// Matrix x Scalar and Scalar x Matrix
template <typename T, typename X, typename = std::enable_if_t<std::is_arithmetic_v<X>>>
auto operator*(const heap::Matrix<T> &A, X B) -> heap::Matrix<std::common_type_t<T, X>>;
template <typename T, typename X, typename = std::enable_if_t<std::is_arithmetic_v<X>>>
auto operator*(X B, const heap::Matrix<T> &A) -> heap::Matrix<std::common_type_t<T, X>>;
template <typename T, size_t M, size_t N, typename X,
          typename = std::enable_if_t<std::is_arithmetic_v<X>>>
auto operator*(const stack::Matrix<T, M, N> &A, X B)
    -> stack::Matrix<std::common_type_t<T, X>, M, N>;
template <typename T, size_t M, size_t N, typename X,
          typename = std::enable_if_t<std::is_arithmetic_v<X>>>
auto operator*(X B, const stack::Matrix<T, M, N> &A)
    -> stack::Matrix<std::common_type_t<T, X>, M, N>;

//! \brief Matrix product.
template <typename MAT1, typename MAT2, typename MAT3>
auto matprod(const MAT1 &A, char mA, const MAT2 &B, char mB, MAT3 &C) -> MAT3 &;
template <typename MAT> auto matprod(const MAT &A, char mA, const MAT &B, char mB) -> MAT;

//! \brief Matrix product operator
template <typename T, typename U>
auto operator*(const heap::Matrix<T> &A, const heap::Vector<U> &B)
    -> heap::Vector<std::common_type_t<T, U>>;
template <typename T, typename U, Array::size_type M>
auto operator*(const heap::Matrix<T> &A, const stack::Vector<U, M> &B)
    -> heap::Vector<std::common_type_t<T, U>>;
template <typename T, typename U>
auto operator*(const heap::Matrix<T> &A, const heap::Matrix<U> &B)
    -> heap::Matrix<std::common_type_t<T, U>>;
template <typename T, typename U, Array::size_type M, Array::size_type N>
auto operator*(const heap::Matrix<T> &A, const stack::Matrix<U, M, N> &B)
    -> heap::Matrix<std::common_type_t<T, U>>;

template <typename T, typename U, Array::size_type M, Array::size_type N>
auto operator*(const stack::Matrix<T, M, N> &A, const stack::Vector<U, N> &B)
    -> stack::Vector<std::common_type_t<T, U>, M>;
template <typename T, typename U, Array::size_type M, Array::size_type N>
auto operator*(const stack::Matrix<T, M, N> &A, const heap::Vector<U> &B)
    -> heap::Vector<std::common_type_t<T, U>>;
template <typename T, typename U, Array::size_type M, Array::size_type N>
auto operator*(const stack::Matrix<T, M, N> &A, const heap::Matrix<U> &B)
    -> heap::Matrix<std::common_type_t<T, U>>;
template <typename T, typename U, Array::size_type M, Array::size_type N, Array::size_type O>
auto operator*(const stack::Matrix<T, M, N> &A, const stack::Matrix<U, N, O> &B)
    -> stack::Matrix<std::common_type_t<T, U>, M, O>;

//! \brief Computes and returns A * A'.
template <typename T, Array::size_type M>
auto square_type(stack::Vector<T, M>) -> stack::Matrix<T, M, M>;
template <typename T, Array::size_type M, Array::size_type N>
auto square_type(stack::Matrix<T, M, N>) -> stack::Matrix<T, M, M>;
template <typename T> auto square_type(heap::Vector<T>) -> heap::Matrix<T>;
template <typename T> auto square_type(heap::Matrix<T>) -> heap::Matrix<T>;

template <typename MAT1, typename MAT2> void square(const MAT1 &A, MAT2 &out);
template <typename MAT> decltype(square_type(MAT())) square(const MAT &A);

//! \brief Computes and returns A' * A.
template <typename T, Array::size_type M, Array::size_type N>
auto transquare_type(stack::Matrix<T, M, N>) -> stack::Matrix<T, N, N>;
template <typename T> auto transquare_type(heap::Matrix<T>) -> heap::Matrix<T>;

template <typename MAT1, typename MAT2> void transquare(const MAT1 &A, MAT2 &out);
template <typename MAT> decltype(transquare_type(MAT())) transquare(const MAT &A);

//! \brief Computes the PLU factorization of the matrix A.
//! \param[in] A Matrix to decompose.
//! \param[out] LU LU factors (packed version).
//! \param[out] P Pivots indices.
//! \return Error code. EC = 0: Successfull exit.
template <typename MAT1, typename MAT2>
auto PLU(const MAT1 &A, MAT2 &LU, std::vector<int> &P) -> int;
template <typename MAT1, typename MAT2> auto PLU(const MAT1 &A, MAT2 &L, MAT2 &U, MAT2 &P) -> int;

//! \brief Computes the Cholesky factorization of a real symmetric or complex
//! hermitian positive definite matrix A. \param[in] A Real symmetric or complex
//! hermitian positive definite matrix to decompose. \param[out] info Pointer to
//! an error code. EC = 0: Successfull exit. EC < 0: Bad parameter. EC > 0: The
//! given matrix is not positive definite. \return New matrix containing the
//! Cholesky factorization of A.
template <typename MAT1, typename MAT2> auto chol(const MAT1 &A, MAT2 &out) -> int;
template <typename MAT> auto chol(const MAT &A, int *info = nullptr) -> MAT;

//! \brief Computes and returns the determinant of a square matrix A.
template <typename MAT> auto det(const MAT &A, int *info = nullptr) -> typename MAT::value_type;

//! \brief Solves the system AX = B (ie X = inv(A) * B) and return the solution
//! matrix. \param[in] A Square matrix A. \param[in] B Rectangular matrix B.
//! \param[out] info Optional pointer to an error code (default equal to
//! nullptr). EC = 0: Successfull exit. EC < 0: Bad parameter. \return New
//! matrix containing the solution of the system AX = B.
template <typename MAT1, typename MAT2, typename MAT3>
auto mldivide(const MAT1 &A, const MAT2 &B, MAT3 &out) -> int;
template <typename MAT1, typename MAT2>
auto mldivide(const MAT1 &A, const MAT2 &B, int *info = nullptr) -> MAT2;

//! \brief Solves the system XB = A (ie X = A * inv(B)) and returns the solution
//! matrix. \param[in] A Rectangular matrix A. \param[in] B Square matrix B.
//! \param[out] info Optional pointer to an error code (default equal to
//! nullptr). EC = 0: Successfull exit. EC < 0: Bad parameter. \return New
//! matrix containing the solution of the system XB = A.
template <typename MAT1, typename MAT2, typename MAT3>
auto mrdivide(const MAT1 &A, const MAT2 &B, MAT3 &out) -> int;
template <typename MAT1, typename MAT2>
auto mrdivide(const MAT1 &A, const MAT2 &B, int *info = nullptr) -> MAT1;

//! \brief Computes and return the inverse of A.
//! \param[in] A Square matrix A.
//! \param[out] info Optional pointer to an error code (default equal to
//! nullptr). EC = 0: Successfull exit. EC < 0: Bad parameter. EC > 0: The
//! matrix is singular. \return New matrix containing the inverse of A.
template <typename MAT1, typename MAT2> auto inv(const MAT1 &A, MAT2 &out) -> int;
template <typename MAT> auto inv(const MAT &A, int *info = nullptr) -> MAT;
} // namespace TMIV::Common

#include "LinAlg.hpp"

#endif
