cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

option(BUILD_VVenC "Build VVenC" ON)
option(BUILD_VVdeC "Build VVdeC" ON)

if(BUILD_VVenC OR BUILD_VVdeC)
    include(FetchContent)
endif()

if(BUILD_VVenC)
    if(NO_INTERNET)
        set(LOCAL_VVENC_DIR ${CMAKE_SOURCE_DIR}/../vvenc-0.2.0.0 CACHE PATH "Path to the local VVenC directory" )
        message(STATUS "Looking for a local copy of VVenC in ${LOCAL_VVENC_DIR}")
        fetchcontent_declare(VVENC URL ${LOCAL_VVENC_DIR})
    else()
        fetchcontent_declare(VVENC
            GIT_REPOSITORY https://github.com/fraunhoferhhi/vvenc
            GIT_TAG "v0.2.0.0"
            GIT_PROGRESS TRUE
        )
    endif()

    set(vvenc_ADD_SUBDIRECTORIES "source/App/vvencFFapp")
    fetchcontent_makeavailable(VVENC)
    
    if (NOT MSVC)
        target_compile_options(vvenc PUBLIC "-w")        
    endif()
    
    set_property(TARGET vvenc vvencFFapp PROPERTY FOLDER "VVenC")
    install(TARGETS vvencFFapp EXPORT TMIVTargets RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

if(BUILD_VVdeC)
    if(NO_INTERNET)
        set(LOCAL_VVDEC_DIR ${CMAKE_SOURCE_DIR}/../vvdec-0.2.0.0 CACHE PATH "Path to the local VVdeC directory" )
        message(STATUS "Looking for a local copy of VVdeC in ${LOCAL_VVDEC_DIR}")
        fetchcontent_declare(VVDEC URL ${LOCAL_VVDEC_DIR})
    else()
        fetchcontent_declare(VVDEC
            GIT_REPOSITORY https://github.com/fraunhoferhhi/vvdec
            GIT_TAG "v0.2.0.0"
            GIT_PROGRESS TRUE
        )
    endif()

    set(vvdec_ADD_SUBDIRECTORIES "source/App/vvdecapp")
    fetchcontent_makeavailable(VVDEC)

    if (NOT MSVC)
        target_compile_options(vvdec PUBLIC "-w")
    endif()

    set_property(TARGET vvdec vvdecapp PROPERTY FOLDER "VVenC")
    install(TARGETS vvdecapp EXPORT TMIVTargets RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

