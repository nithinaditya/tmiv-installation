cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
        DecoderLib
    SOURCES
        "src/AtlasDecoder.cpp"
        "src/CommonAtlasDecoder.cpp"
        "src/Decoder.cpp"
        "src/GeometryScaler.cpp"
        "src/OccupancyReconstructor.cpp"
        "src/EntityBasedPatchMapFilter.cpp"
        "src/MivDecoder.cpp"
        "src/NalUnitSemantics.h"
        "src/V3cSampleStreamDecoder.cpp"
        "src/V3cUnitBuffer.cpp"
        "include/TMIV/Decoder/IDecoder.h"
        "include/TMIV/Decoder/AtlasDecoder.h"
        "include/TMIV/Decoder/CommonAtlasDecoder.h"
        "include/TMIV/Decoder/Decoder.h"
        "include/TMIV/Decoder/GeometryScaler.h"
        "include/TMIV/Decoder/OccupancyReconstructor.h"
        "include/TMIV/Decoder/EntityBasedPatchMapFilter.h"
        "include/TMIV/Decoder/MivDecoder.h"
        "include/TMIV/Decoder/V3cSampleStreamDecoder.h"
        "include/TMIV/Decoder/V3cUnitBuffer.h"
    PUBLIC
        TMIV::VideoDecoderLib
    )

create_tmiv_executable(
    TARGET
        Decoder
    SOURCES
        "src/Decoder.main.cpp"
        "src/registerComponents.cpp"
    PRIVATE
        TMIV::DecoderLib
        TMIV::RendererFrontLib
    )
