cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
        ViewOptimizerLib
    SOURCES
        "src/AbstractViewSelector.cpp"
        "src/BasicViewAllocator.cpp"
        "src/KMedoidsCost.cpp"
        "src/ServerSideInpainter.cpp"
        "include/TMIV/ViewOptimizer/IViewOptimizer.h"
        "include/TMIV/ViewOptimizer/AbstractViewSelector.h"
        "include/TMIV/ViewOptimizer/BasicViewAllocator.h"
        "include/TMIV/ViewOptimizer/KMedoidsCost.h"
    PUBLIC
        TMIV::RendererLib
    )

create_catch2_unit_test(
    TARGET
        ViewOptimizerTest
    SOURCES
        "src/ServerSideInpainter.test.cpp"
    PRIVATE
        TMIV::ViewOptimizerLib
    )
