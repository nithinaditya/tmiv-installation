cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

if (HAVE_HM)
    set(hmSources
        "src/HmVideoDecoder.cpp"
        "include/TMIV/VideoDecoder/HmVideoDecoder.h")
endif()

create_tmiv_library(
    TARGET
        VideoDecoderLib
    SOURCES
        "src/IVideoDecoder.cpp"
        "src/VideoServer.cpp"
        "include/TMIV/VideoDecoder/IVideoDecoder.h"
        "include/TMIV/VideoDecoder/VideoServer.h"
        ${hmSources}
    PUBLIC
        TMIV::MivBitstreamLib
    PRIVATE
        Threads::Threads
    )

if (HAVE_HM)
    target_link_libraries(VideoDecoderLib PRIVATE TMIV::TLibDecoder)
    target_compile_definitions(VideoDecoderLib PRIVATE HAVE_HM)
endif()

option(BUILD_VideoDecoder "Build TMIV exectable VideoDecoder" FALSE)

if (BUILD_VideoDecoder)
    create_tmiv_executable(
        TARGET
            VideoDecoder
        SOURCES
            "src/VideoDecoder.main.cpp"
        PRIVATE
            VideoDecoderLib
        )
endif()
