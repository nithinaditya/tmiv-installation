/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ISO/IEC
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

#include <TMIV/Decoder/EntityBasedPatchMapFilter.h>

#include <cassert>
#include <iostream>

namespace TMIV::Decoder {
EntityBasedPatchMapFilter::EntityBasedPatchMapFilter(const Common::Json & /*rootNode*/,
                                                     const Common::Json &componentNode) {
  m_entityFiltering = false;
  if (const auto &subnode = componentNode.optional("EntityDecodeRange")) {
    m_entityDecodeRange = subnode.asVec<int, 2>();
    m_entityFiltering = true;
  }
}

void EntityBasedPatchMapFilter::inplaceFilterBlockToPatchMaps(
    MivBitstream::AccessUnit &frame) const {
  if (m_entityFiltering) {
    for (auto &atlas : frame.atlas) {
      if (atlas.asps.asps_extension_present_flag() &&
          atlas.asps.asps_miv_extension_present_flag() &&
          0 < atlas.asps.asps_miv_extension().asme_max_entity_id()) {
        filterBlockToPatchMaps(atlas);
      }
    }
  }
}

void EntityBasedPatchMapFilter::filterBlockToPatchMaps(MivBitstream::AtlasAccessUnit &atlas) const {
  Common::Vec2i sz = atlas.blockToPatchMap.getSize();
  for (int y = 0; y < sz[1]; y++) {
    for (int x = 0; x < sz[0]; x++) {
      uint16_t patchId = atlas.blockToPatchMap.getPlane(0)(y, x);
      if (patchId != Common::unusedPatchId) {
        auto entityId = static_cast<int>(*atlas.patchParamsList[patchId].atlasPatchEntityId());
        if (entityId < m_entityDecodeRange[0] || m_entityDecodeRange[1] <= entityId) {
          atlas.blockToPatchMap.getPlane(0)(y, x) = Common::unusedPatchId;
        }
      }
    }
  }
}

} // namespace TMIV::Decoder
