cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

if(CMAKE_TESTING_ENABLED)
    find_package(Python3 COMPONENTS Interpreter)

    if(Python3_Interpreter_FOUND)
        set(INTEGRATION_TEST_CONTENT_DIR "" CACHE PATH "Directory with test content")
        set(INTEGRATION_TEST_OUTPUT_DIR "${CMAKE_BINARY_DIR}/integration_test" CACHE PATH "Directory where the test output will be written")
        set(INTEGRATION_TEST_MAX_WORKERS CACHE STRING "Maximum number of worker threads (empty for default)")
        set(INTEGRATION_TEST_REFERENCE_DIR CACHE PATH "Directory with reference output for file comparison (leave empty if this build is the reference)")
        
        if(INTEGRATION_TEST_CONTENT_DIR STREQUAL "")
            message(STATUS "Integration testing is DISABLED because INTEGRATION_TEST_CONTENT_DIR is not provided")
        elseif(NOT BUILD_TAppEncoder)
            message(STATUS "Integration testing is DISABLED because BUILD_TAppEncoder is OFF")
        elseif(EXISTS ${INTEGRATION_TEST_CONTENT_DIR})
            # Nice-to-have but not a requirement:
            find_package(Git)
            if(Git_FOUND)
                set(GIT_ARG "-g")
            endif()

            if(NOT INTEGRATION_TEST_MAX_WORKERS STREQUAL "")
                set(PAR_ARG "-j")
            endif()
            
            if(NOT INTEGRATION_TEST_REFERENCE_DIR STREQUAL "")
                set(REF_ARG "-r")
            endif()

            add_custom_target(integration_test
                COMMAND
                    ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --config "$<CONFIG>"
                COMMAND
                    ${CMAKE_CTEST_COMMAND} --build-config "$<CONFIG>"
                COMMAND
                    ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --config "$<CONFIG>" --target install
                COMMAND
                    ${Python3_EXECUTABLE}
                    ${CMAKE_CURRENT_LIST_DIR}/integration_test.py
                    ${CMAKE_INSTALL_PREFIX}
                    ${CMAKE_SOURCE_DIR}
                    ${INTEGRATION_TEST_CONTENT_DIR}
                    ${INTEGRATION_TEST_OUTPUT_DIR}
                    ${GIT_ARG} ${GIT_EXECUTABLE}
                    ${PAR_ARG} ${INTEGRATION_TEST_MAX_WORKERS}
                    ${REF_ARG} ${INTEGRATION_TEST_REFERENCE_DIR})
            set_property(TARGET integration_test PROPERTY FOLDER "TMIV tests")
            message(STATUS "Integration testing is enabled through the integration_test target. This target will first execute the 'all', 'test' and 'install' targets.")
        else()
            message(STATUS "Integration testing is DISABLED because INTEGRATION_TEST_CONTENT_DIR (${INTEGRATION_TEST_CONTENT_DIR}) does not exist")
        endif()                
    else()
        message(STATUS "Integration testing is DISABLED because the Python 3 interpreter could not be found")
    endif()
endif()

# TODO(BK): Add comparison against a reference
# https://stackoverflow.com/questions/54353120/how-to-compare-files-in-cmake