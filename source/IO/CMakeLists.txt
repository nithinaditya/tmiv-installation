cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

# Address this https://en.cppreference.com/w/cpp/filesystem note:
#   Using this library may require additional compiler/linker options. GNU implementation prior to
#   9.1 requires linking with -lstdc++fs and LLVM implementation prior to LLVM 9.0 requires linking
#   with -lc++fs
if(MSVC OR MSYS OR MINGW)
    message(STATUS "C++ filesystem does not require a library on this platform")
else()
    if (${CMAKE_CXX_COMPILER_ID} STREQUAL GNU AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS 9.1)
        message(STATUS "Adding stdc++fs for C++ filesystem")
        set(CppFilesystemLib stdc++fs)
    elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL Clang AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS 9.0)
        message(STATUS "Adding c++fs for C++ filesystem")
        set(CppFilesystemLib c++fs)
    endif()
endif()

create_tmiv_library(
    TARGET
        IOLib
    SOURCES
        "src/load.cpp"
        "src/save.cpp"
        "include/TMIV/IO/IO.h"
    PUBLIC
        TMIV::MivBitstreamLib
        ${CppFilesystemLib}
    )
