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

#include <TMIV/Aggregator/Aggregator.h>
#include <TMIV/Common/Factory.h>
#include <TMIV/DepthQualityAssessor/DepthQualityAssessor.h>
#include <TMIV/Encoder/Encoder.h>
#include <TMIV/Encoder/GroupBasedEncoder.h>
#include <TMIV/Encoder/MpiEncoder.h>
#include <TMIV/GeometryQuantizer/ExplicitOccupancy.h>
#include <TMIV/GeometryQuantizer/GeometryQuantizer.h>
#include <TMIV/Packer/Packer.h>
#include <TMIV/Pruner/HierarchicalPruner.h>
#include <TMIV/Pruner/NoPruner.h>
#include <TMIV/Renderer/AdditiveSynthesizer.h>
#include <TMIV/Renderer/Inpainter.h>
#include <TMIV/Renderer/NoInpainter.h>
#include <TMIV/Renderer/PushPullInpainter.h>
#include <TMIV/Renderer/ViewWeightingSynthesizer.h>
#include <TMIV/ViewOptimizer/BasicViewAllocator.h>
#include <TMIV/ViewOptimizer/NoViewOptimizer.h>
#include <TMIV/ViewOptimizer/ServerSideInpainter.h>

namespace TMIV::Encoder {
void registerComponents() {
  auto &aggregators = Common::Factory<Aggregator::IAggregator>::getInstance();
  aggregators.registerAs<Aggregator::Aggregator>("Aggregator");

  auto &assesors = Common::Factory<DepthQualityAssessor::IDepthQualityAssessor>::getInstance();
  assesors.registerAs<DepthQualityAssessor::DepthQualityAssessor>("DepthQualityAssessor");

  auto &encoders = Common::Factory<IEncoder>::getInstance();
  encoders.registerAs<Encoder>("Encoder");
  encoders.registerAs<GroupBasedEncoder>("GroupBasedEncoder");

  auto &mpiEncoders = Common::Factory<IMpiEncoder>::getInstance();
  mpiEncoders.registerAs<MpiEncoder>("MpiEncoder");

  auto &geometryQuantizers = Common::Factory<GeometryQuantizer::IGeometryQuantizer>::getInstance();
  geometryQuantizers.registerAs<GeometryQuantizer::GeometryQuantizer>("GeometryQuantizer");
  geometryQuantizers.registerAs<GeometryQuantizer::ExplicitOccupancy>("ExplicitOccupancy");

  auto &packers = Common::Factory<Packer::IPacker>::getInstance();
  packers.registerAs<Packer::Packer>("Packer");

  auto &pruners = Common::Factory<Pruner::IPruner>::getInstance();
  pruners.registerAs<Pruner::HierarchicalPruner>("HierarchicalPruner");
  pruners.registerAs<Pruner::NoPruner>("NoPruner");

  auto &synthesizers = Common::Factory<Renderer::ISynthesizer>::getInstance();
  synthesizers.registerAs<Renderer::AdditiveSynthesizer>("AdditiveSynthesizer");
  synthesizers.registerAs<Renderer::ViewWeightingSynthesizer>("ViewWeightingSynthesizer");

  auto &inpainters = Common::Factory<Renderer::IInpainter>::getInstance();
  inpainters.registerAs<Renderer::Inpainter>("Inpainter");
  inpainters.registerAs<Renderer::NoInpainter>("NoInpainter");
  inpainters.registerAs<Renderer::PushPullInpainter>("PushPullInpainter");

  auto &viewOptimizers = Common::Factory<ViewOptimizer::IViewOptimizer>::getInstance();
  viewOptimizers.registerAs<ViewOptimizer::BasicViewAllocator>("BasicViewAllocator");
  viewOptimizers.registerAs<ViewOptimizer::NoViewOptimizer>("NoViewOptimizer");
  viewOptimizers.registerAs<ViewOptimizer::ServerSideInpainter>("ServerSideInpainter");
}
} // namespace TMIV::Encoder
