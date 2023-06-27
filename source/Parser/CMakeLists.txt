cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_executable(
    TARGET
        Parser
    SOURCES
        "src/Parser.main.cpp"
    PRIVATE
        TMIV::MivBitstreamLib
    )

create_tmiv_executable(
    TARGET
        BitrateReport
    SOURCES
        "src/BitrateReport.main.cpp"
    PRIVATE
        TMIV::MivBitstreamLib
    )
