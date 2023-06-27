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

#include <TMIV/MivBitstream/NalUnit.h>
#include <TMIV/MivBitstream/V3cSampleStreamFormat.h>
#include <TMIV/MivBitstream/V3cUnit.h>

#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <variant>
#include <vector>

class StatisticalVariable {
public:
  auto operator<<(std::size_t value) -> auto & {
    ++m_count;
    m_sum += value;
    return *this;
  }

  friend auto operator<<(std::ostream &stream, const StatisticalVariable &x) -> std::ostream & {
    auto average = x.m_count > 0 ? static_cast<double>(x.m_sum) / x.m_count
                                 : std::numeric_limits<double>::quiet_NaN();
    return stream << x.m_count << ',' << x.m_sum << ',' << average;
  }

private:
  std::size_t m_count{};
  std::size_t m_sum{};
};

struct CompareVuh {
  auto operator()(const TMIV::MivBitstream::V3cUnitHeader &vuh1,
                  const TMIV::MivBitstream::V3cUnitHeader &vuh2) const -> bool {
    if (vuh1.vuh_unit_type() != vuh2.vuh_unit_type()) {
      return vuh1.vuh_unit_type() < vuh2.vuh_unit_type();
    }
    if (vuh1.vuh_unit_type() == TMIV::MivBitstream::VuhUnitType::V3C_VPS ||
        vuh1.vuh_unit_type() == TMIV::MivBitstream::VuhUnitType::V3C_CAD) {
      return false;
    }
    if (vuh1.vuh_atlas_id() != vuh2.vuh_atlas_id()) {
      return vuh1.vuh_atlas_id() < vuh2.vuh_atlas_id();
    }
    if (vuh1.vuh_unit_type() != TMIV::MivBitstream::VuhUnitType::V3C_GVD &&
        vuh1.vuh_unit_type() != TMIV::MivBitstream::VuhUnitType::V3C_AVD) {
      return false;
    }
    if (vuh1.vuh_map_index() != vuh2.vuh_map_index()) {
      return vuh1.vuh_map_index() < vuh2.vuh_map_index();
    }
    if (vuh1.vuh_unit_type() != TMIV::MivBitstream::VuhUnitType::V3C_AVD) {
      return false;
    }
    return vuh1.vuh_attribute_index() < vuh2.vuh_attribute_index();
  }
};

struct CompareNuh {
  auto operator()(const TMIV::MivBitstream::NalUnitHeader &nuh1,
                  const TMIV::MivBitstream::NalUnitHeader &nuh2) const -> bool {
    if (nuh1.nal_unit_type() != nuh2.nal_unit_type()) {
      return nuh1.nal_unit_type() < nuh2.nal_unit_type();
    }
    if (nuh1.nal_layer_id() != nuh2.nal_layer_id()) {
      return nuh1.nal_layer_id() < nuh2.nal_layer_id();
    }
    return nuh1.nal_temporal_id_plus1() < nuh2.nal_temporal_id_plus1();
  }
};

class BitrateReport {
public:
  void printTo(std::ostream &stream) const {
    stream << "vuh_unit_type,vuh_atlas_id,vuh_map_index,vuh_attribute_index,count,sum,average\n";
    for (const auto &[vuh, stats] : m_vuhStats) {
      stream << vuh.vuh_unit_type() << ',';
      switch (vuh.vuh_unit_type()) {
      case TMIV::MivBitstream::VuhUnitType::V3C_VPS:
      case TMIV::MivBitstream::VuhUnitType::V3C_CAD:
        stream << ",,";
        break;
      case TMIV::MivBitstream::VuhUnitType::V3C_AD:
      case TMIV::MivBitstream::VuhUnitType::V3C_OVD:
        stream << vuh.vuh_atlas_id() << ",,";
        break;
      case TMIV::MivBitstream::VuhUnitType::V3C_GVD:
        stream << vuh.vuh_atlas_id() << ',' << int{vuh.vuh_map_index()} << ',';
        break;
      case TMIV::MivBitstream::VuhUnitType::V3C_AVD:
        stream << vuh.vuh_atlas_id() << ',' << int{vuh.vuh_map_index()} << ','
               << int{vuh.vuh_attribute_index()};
        break;
      default:
        abort();
      }
      stream << ',' << stats << '\n';
    }

    stream << ",,,,,,\n";
    stream << "nal_unit_type,nal_layer_id,nal_temporal_id,,count,sum,average\n";

    for (const auto &[nuh, stats] : m_nuhStats) {
      stream << nuh.nal_unit_type() << ',' << int{nuh.nal_layer_id()} << ','
             << (nuh.nal_temporal_id_plus1() - 1) << ",," << stats << '\n';
    }
  }

  void add(const TMIV::MivBitstream::V3cUnitHeader &vuh, std::size_t size) {
    m_vuhStats[vuh] << size;
  }
  auto add(const TMIV::MivBitstream::NalUnitHeader &nuh, std::size_t size) {
    m_nuhStats[nuh] << size;
  }

private:
  std::map<TMIV::MivBitstream::V3cUnitHeader, StatisticalVariable, CompareVuh> m_vuhStats;
  std::map<TMIV::MivBitstream::NalUnitHeader, StatisticalVariable, CompareNuh> m_nuhStats;
};

class PartialParser {
public:
  void parseV3cSampleStream(std::istream &stream) {
    const auto ssvh = TMIV::MivBitstream::SampleStreamV3cHeader::decodeFrom(stream);

    while (stream.peek(), !stream.eof()) {
      const auto ssvu = TMIV::MivBitstream::SampleStreamV3cUnit::decodeFrom(stream, ssvh);
      std::istringstream substream{ssvu.ssvu_v3c_unit()};
      parseV3cUnit(substream, ssvu.ssvu_v3c_unit_size());
    }
  }

  void parseV3cUnit(std::istream &stream, std::size_t numBytesInV3CUnit) {
    auto vu = TMIV::MivBitstream::V3cUnit::decodeFrom(stream, numBytesInV3CUnit);
    m_report.add(vu.v3c_unit_header(), numBytesInV3CUnit);
    std::visit([this](const auto &x) { parseV3cUnitPayload(x); }, vu.v3c_unit_payload().payload());
  }

  void parseV3cUnitPayload(const std::monostate & /* unused */) {}

  void parseV3cUnitPayload(const TMIV::MivBitstream::V3cParameterSet & /* vps */) {}

  void parseV3cUnitPayload(const TMIV::MivBitstream::AtlasSubBitstream &asb) {
    for (const auto &nu : asb.nal_units()) {
      m_report.add(nu.nal_unit_header(), nu.size());
    }
  }

  void parseV3cUnitPayload(const TMIV::MivBitstream::VideoSubBitstream & /* unused */) {}

  [[nodiscard]] auto report() const -> auto & { return m_report; }

private:
  BitrateReport m_report;
};

auto main(int argc, char *argv[]) -> int {
  try {
    const auto args = std::vector(argv, argv + argc);

    if (args.size() != 3 || strcmp(args[1], "-b") != 0) {
      std::clog << "Usage: BitrateReport -b BITSTREAM" << std::endl;
      return 1;
    }

    std::ifstream stream{args[2], std::ios::binary};
    if (!stream.good()) {
      std::clog << "Failed to open bitstream for reading" << std::endl;
      return 1;
    }

    PartialParser parser;
    parser.parseV3cSampleStream(stream);
    parser.report().printTo(std::cout);
    return 0;
  } catch (std::exception &e) {
    std::clog << e.what() << std::endl;
    return 1;
  }
}
