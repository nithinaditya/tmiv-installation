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

#ifndef _TMIV_COMMON_BYTESTREAM_H_
#define _TMIV_COMMON_BYTESTREAM_H_

#include <cstdint>
#include <iosfwd>
#include <string>

namespace TMIV::Common {
auto readBytes(std::istream &stream, size_t bytes) -> uint64_t;
auto getUint8(std::istream &stream) -> uint8_t;
auto getUint16(std::istream &stream) -> uint16_t;
auto getUint32(std::istream &stream) -> uint32_t;
auto getUint64(std::istream &stream) -> uint64_t;
auto readString(std::istream &stream, size_t bytes) -> std::string;
auto moreRbspData(std::istream &stream) -> bool;
void rbspTrailingBits(std::istream &stream);

void writeBytes(std::ostream &stream, uint64_t value, size_t bytes);
void putUint8(std::ostream &stream, uint8_t value);
void putUint16(std::ostream &stream, uint8_t value);
void putUint32(std::ostream &stream, uint8_t value);
void putUint64(std::ostream &stream, uint8_t value);
void writeString(std::ostream &stream, const std::string &buffer);
void rbspTrailingBits(std::ostream &stream);
} // namespace TMIV::Common

#endif
