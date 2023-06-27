cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
        EncoderLib
    SOURCES
        "src/Encoder.cpp"
        "src/Encoder_prepareSequence.cpp"
        "src/Encoder_prepareAccessUnit.cpp"
        "src/Encoder_pushFrame.cpp"
        "src/Encoder_completeAccessUnit.cpp"
        "src/Encoder_popFrame.cpp"
        "src/GroupBasedEncoder.cpp"
        "src/GeometryDownscaler.cpp"
        "src/MivEncoder.cpp"
        "src/MpiEncoder.cpp"
        "include/TMIV/Encoder/IEncoder.h"
        "include/TMIV/Encoder/IMpiEncoder.h"
        "include/TMIV/Encoder/Encoder.h"
        "include/TMIV/Encoder/GroupBasedEncoder.h"
        "include/TMIV/Encoder/GeometryDownscaler.h"
        "include/TMIV/Encoder/MivEncoder.h"
        "include/TMIV/Encoder/MpiEncoder.h"
    PUBLIC
        TMIV::ViewOptimizerLib
        TMIV::PrunerLib
        TMIV::AggregatorLib
        TMIV::PackerLib
        TMIV::GeometryQuantizerLib
        TMIV::DepthQualityAssessorLib
    )


create_tmiv_executable(
    TARGET
        Encoder
    SOURCES
        "src/Encoder.main.cpp"
        "src/registerComponents.cpp"
    PRIVATE
        TMIV::EncoderLib
        TMIV::IOLib
    )

create_tmiv_executable(
    TARGET
        MpiEncoder
    SOURCES
        "src/MpiEncoder.main.cpp"
        "src/registerComponents.cpp"
    PRIVATE
        TMIV::EncoderLib
        TMIV::IOLib
    )

create_tmiv_executable(
    TARGET
        Multiplexer
    SOURCES
        "src/Multiplexer.main.cpp"
    PRIVATE
        TMIV::MivBitstreamLib
        TMIV::IOLib
    )
