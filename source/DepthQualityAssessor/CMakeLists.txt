cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
        DepthQualityAssessorLib
    SOURCES
        "src/DepthQualityAssessor.cpp"
        "include/TMIV/DepthQualityAssessor/IDepthQualityAssessor.h"
        "include/TMIV/DepthQualityAssessor/DepthQualityAssessor.h"
    PUBLIC
        TMIV::RendererLib
    )
