cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

execute_process(
    COMMAND ${CMAKE_COMMAND} "--graphviz=module_graph" .
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

find_program(DOT_PATH NAMES "dot")
        
if(DOT_PATH)    
    execute_process(
        COMMAND ${DOT_PATH} "module_graph" "-Tsvg" "-o" "${OUTPUT_FILE}"
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
else()
    message(WARNING "The module graph has been generated but Graphviz is required to render the .dot-file as an image")
endif()
