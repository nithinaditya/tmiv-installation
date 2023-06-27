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
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
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

#ifndef _TMIV_VIDEODECODER_VIDEOSERVER_H_
#define _TMIV_VIDEODECODER_VIDEOSERVER_H_

#include <TMIV/VideoDecoder/IVideoDecoder.h>

namespace TMIV::VideoDecoder {
// The VideoServer uses a thread to change from push to pull mechanism.
class VideoServer {
public:
  explicit VideoServer(std::unique_ptr<IVideoDecoder> decoder, const std::string &bitstream);
  VideoServer(const VideoServer &) = delete;
  VideoServer(VideoServer &&) = default;
  auto operator=(const VideoServer &) -> VideoServer & = delete;
  auto operator=(VideoServer &&) -> VideoServer & = default;
  ~VideoServer();

  // Wait for the video server to block or stop
  void wait();

  // Get the next frame. If there are no more frames the result will be empty.
  auto getFrame() -> std::unique_ptr<Common::AnyFrame>;

private:
  class Impl;
  std::unique_ptr<Impl> m_impl;
};
} // namespace TMIV::VideoDecoder

#endif
