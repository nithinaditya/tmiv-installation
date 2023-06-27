cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
        CommonLib
    SOURCES
        "src/Application.cpp"
        "src/Bitstream.cpp"
        "src/Bytestream.cpp"
        "src/Json.cpp"
        "src/Half.cpp"
        "src/Frame.cpp"
        "src/verify.cpp"
        "include/TMIV/Common/Bitstream.h"
        "include/TMIV/Common/Bitstream.hpp"
        "include/TMIV/Common/Bytestream.h"
        "include/TMIV/Common/Common.h"
        "include/TMIV/Common/Common.hpp"
        "include/TMIV/Common/Application.h"
        "include/TMIV/Common/Json.h"
        "include/TMIV/Common/Json.hpp"
        "include/TMIV/Common/Factory.h"
        "include/TMIV/Common/Filter.h"
        "include/TMIV/Common/Thread.h"
        "include/TMIV/Common/Frame.h"
        "include/TMIV/Common/Frame.hpp"
        "include/TMIV/Common/Half.h"
        "include/TMIV/Common/Half.hpp"
        "include/TMIV/Common/LinAlg.h"
        "include/TMIV/Common/LinAlg.hpp"
        "include/TMIV/Common/Vector.h"
        "include/TMIV/Common/Matrix.h"
        "include/TMIV/Common/Matrix.hpp"
        "include/TMIV/Common/Array.h"
        "include/TMIV/Common/Graph.h"
        "include/TMIV/Common/Graph.hpp"
        "include/TMIV/Common/Quaternion.h"
        "include/TMIV/Common/verify.h"
    PUBLIC
        fmt::fmt
    )

target_include_directories(CommonLib
    PUBLIC
        "$<INSTALL_INTERFACE:include>")

target_compile_features(CommonLib
    PUBLIC cxx_std_17)

install(
    TARGETS
        fmt
    EXPORT TMIVTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

create_catch2_unit_test(
    TARGET
        CommonTest
    SOURCES
        "src/Bitstream.test.cpp"
        "src/Common.test.cpp"
        "src/Filter.test.cpp"
        "src/Frame.test.cpp"
        "src/Graph.test.cpp"
        "src/Json.test.cpp"
        "src/Matrix.test.cpp"
        "src/Quaternion.test.cpp"
        "src/verify.test.cpp"
    PRIVATE
        TMIV::CommonLib
    )
