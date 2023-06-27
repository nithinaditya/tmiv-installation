function(create_tmiv_library)
    set(prefix TMIV_LIB_CREATOR)
    set(flags)
    set(singleValues TARGET)
    set(multiValues SOURCES PUBLIC PRIVATE)

    include(CMakeParseArguments)
    cmake_parse_arguments(${prefix}
        "${flags}"
        "${singleValues}"
        "${multiValues}"
        ${ARGN})

    add_library(${TMIV_LIB_CREATOR_TARGET} ${TMIV_LIB_CREATOR_SOURCES})
    add_library(TMIV::${TMIV_LIB_CREATOR_TARGET} ALIAS ${TMIV_LIB_CREATOR_TARGET})
    set_property(TARGET ${TMIV_LIB_CREATOR_TARGET} PROPERTY FOLDER "TMIV libraries")   
    target_link_libraries(${TMIV_LIB_CREATOR_TARGET} PUBLIC ${TMIV_LIB_CREATOR_PUBLIC})
    target_link_libraries(${TMIV_LIB_CREATOR_TARGET} PRIVATE ${TMIV_LIB_CREATOR_PRIVATE})
    
    target_include_directories(${TMIV_LIB_CREATOR_TARGET}
        PUBLIC
            "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

    install(
        TARGETS ${TMIV_LIB_CREATOR_TARGET}
        EXPORT TMIVTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

    install(
        DIRECTORY "include/"
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endfunction()

function(create_tmiv_executable)
    set(prefix TMIV_EXE_CREATOR)
    set(flags)
    set(singleValues TARGET)
    set(multiValues SOURCES PRIVATE)

    include(CMakeParseArguments)
    cmake_parse_arguments(${prefix}
        "${flags}"
        "${singleValues}"
        "${multiValues}"
        ${ARGN})

    add_executable(${TMIV_EXE_CREATOR_TARGET} ${TMIV_EXE_CREATOR_SOURCES})
    set_property(TARGET ${TMIV_EXE_CREATOR_TARGET} PROPERTY FOLDER "TMIV executables")
    target_link_libraries(${TMIV_EXE_CREATOR_TARGET} PRIVATE ${TMIV_EXE_CREATOR_PRIVATE})

    install(
        TARGETS ${TMIV_EXE_CREATOR_TARGET}
        EXPORT TMIVTargets
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endfunction()
