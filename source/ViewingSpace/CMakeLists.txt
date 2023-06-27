cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
        ViewingSpaceLib
    SOURCES
        "src/SignedDistance.cpp"
        "src/ViewingSpaceEvaluator.cpp"
        "include/TMIV/ViewingSpace/SignedDistance.h"
        "include/TMIV/ViewingSpace/ViewingSpaceEvaluator.h"
    PUBLIC
        TMIV::MivBitstreamLib
    )

create_catch2_unit_test(
    TARGET
        ViewingSpaceTest
    SOURCES
        "src/ViewingSpace.test.cpp"
    PRIVATE
        TMIV::ViewingSpaceLib
    )
