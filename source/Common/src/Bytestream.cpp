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

#include <TMIV/Common/Bytestream.h>

#include <TMIV/Common/Bitstream.h>

#include <iostream>

namespace {
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define VERIFY_BYTESTREAM(condition)                                                               \
  (void)(!!(condition) || verifyBytestreamFailed(#condition, __FILE__, __LINE__))

auto verifyBytestreamFailed(char const *condition, char const *file, int line) -> bool {
  std::cerr << "Failed to encode/decode byte stream: " << condition << " [" << file << "@" << line
            << std::endl;
  abort();
  return false;
}
} // namespace

namespace TMIV::Common {
auto readBytes(std::istream &stream, size_t bytes) -> uint64_t {
  VERIFY_BYTESTREAM(bytes <= 8);
  auto result = uint64_t{0};
  while (bytes-- > 0) {
    char buffer = 0;
    stream.get(buffer);
    result = (result << 8) | static_cast<uint8_t>(buffer);
  }
  VERIFY_BYTESTREAM(stream.good());
  return result;
}

auto getUint8(std::istream &stream) -> uint8_t {
  return static_cast<uint8_t>(readBytes(stream, 1));
}

auto getUint16(std::istream &stream) -> uint16_t {
  return static_cast<uint16_t>(readBytes(stream, 2));
}

auto getUint32(std::istream &stream) -> uint32_t {
  return static_cast<uint32_t>(readBytes(stream, 4));
}

auto getUint64(std::istream &stream) -> uint64_t { return readBytes(stream, 8); }

auto readString(std::istream &stream, size_t bytes) -> std::string {
  auto result = std::string(bytes, '\0');
  stream.read(result.data(), bytes);
  VERIFY_BYTESTREAM(stream.good());
  return result;
}

auto moreRbspData(std::istream &stream) -> bool {
  InputBitstream bitstream{stream};
  return bitstream.moreRbspData();
}

void rbspTrailingBits(std::istream &stream) {
  InputBitstream bitstream{stream};
  bitstream.rbspTrailingBits();
}

void writeBytes(std::ostream &stream, uint64_t value, size_t bytes) {
  VERIFY_BYTESTREAM(bytes <= 8);
  if (bytes > 1) {
    writeBytes(stream, value >> 8, bytes - 1);
  }
  if (bytes > 0) {
    stream.put(static_cast<char>(value));
  }
  VERIFY_BYTESTREAM(stream.good());
}

void putUint8(std::ostream &stream, uint8_t value) { writeBytes(stream, value, 1); }
void putUint16(std::ostream &stream, uint8_t value) { writeBytes(stream, value, 2); }
void putUint32(std::ostream &stream, uint8_t value) { writeBytes(stream, value, 4); }
void putUint64(std::ostream &stream, uint8_t value) { writeBytes(stream, value, 8); }

void writeString(std::ostream &stream, const std::string &buffer) {
  stream.write(buffer.data(), buffer.size());
  VERIFY_BYTESTREAM(stream.good());
}

void rbspTrailingBits(std::ostream &stream) {
  OutputBitstream bitstream{stream};
  bitstream.rbspTrailingBits();
}
} // namespace TMIV::Common
