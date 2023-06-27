cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
        GeometryQuantizerLib
    SOURCES
        "src/GeometryQuantizer.cpp"
        "src/ExplicitOccupancy.cpp"
        "include/TMIV/GeometryQuantizer/ExplicitOccupancy.h"
        "include/TMIV/GeometryQuantizer/IGeometryQuantizer.h"
        "include/TMIV/GeometryQuantizer/GeometryQuantizer.h"
    PUBLIC
        TMIV::MivBitstreamLib
    )

create_catch2_unit_test(
    TARGET
        GeometryQuantizerTest
    SOURCES
        "src/GeometryQuantizer.test.cpp"
    PRIVATE
        TMIV::GeometryQuantizerLib
    )
