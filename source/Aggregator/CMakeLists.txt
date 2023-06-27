cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
        AggregatorLib
    SOURCES
        "src/Aggregator.cpp"
        "include/TMIV/Aggregator/IAggregator.h"
        "include/TMIV/Aggregator/Aggregator.h"
    PUBLIC
        TMIV::CommonLib
    )
