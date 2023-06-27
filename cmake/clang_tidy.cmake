cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

# NOTE(BK): In my experience MSVC is passing the wrong compiler options to a potentially older version of clang-tidy.
if (MSVC)
    # clang-tidy (when available) is opt-in for MSVC.
    set(ENABLE_CLANG_TIDY_DEFAULT OFF)
else()
    # clang-tidy (when available) is opt-out by default.
    set(ENABLE_CLANG_TIDY_DEFAULT ON)
endif()

option(ENABLE_CLANG_TIDY "Turn on clang_tidy processing if available" ${ENABLE_CLANG_TIDY_DEFAULT})

if (ENABLE_CLANG_TIDY)
    find_program(CLANG_TIDY_PATH NAMES "clang-tidy")

    if(CLANG_TIDY_PATH)
        set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_PATH}")
        message(STATUS "The clang-tidy path is set to ${CLANG_TIDY_PATH}")
    else()
        message(STATUS "clang-tidy could not be found")
    endif()
else()
    message(STATUS "clang-tidy is disabled")
endif()
