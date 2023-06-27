cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
        PrunerLib
    SOURCES
        "src/HierarchicalPruner.cpp"
        "src/IncrementalSynthesizer.h"
        "src/LumaStdDev.cpp"
        "src/LumaStdDev.h"
        "src/PrunedMesh.cpp"
        "src/PrunedMesh.h"
        "src/NoPruner.cpp"
        "include/TMIV/Pruner/IPruner.h"
        "include/TMIV/Pruner/HierarchicalPruner.h"
        "include/TMIV/Pruner/NoPruner.h"
    PUBLIC
        TMIV::RendererLib
    )

create_catch2_unit_test(
    TARGET
        PrunerTest
    SOURCES
        "src/LumaStdDev.test.cpp"
    PRIVATE
        TMIV::PrunerLib
    )
