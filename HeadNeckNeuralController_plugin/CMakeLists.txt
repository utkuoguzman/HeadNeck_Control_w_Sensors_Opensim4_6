cmake_minimum_required(VERSION 3.10)
project(osimHeadNeckNeural)

find_package(OpenSim REQUIRED)

set(SOURCE_FILES 
    HeadNeckNeuralController.cpp
    RegisterTypes_osimHeadNeckNeural.cpp
)

set(HEADER_FILES 
    HeadNeckNeuralController.h
    RegisterTypes_osimHeadNeckNeural.h
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE 
    ${OpenSim_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../vestibular_plugin
    ${CMAKE_CURRENT_SOURCE_DIR}/../plugin_source_code
)

target_compile_definitions(${PROJECT_NAME} PRIVATE OSIMHEADNECKNEURAL_EXPORTS)

# Link against OpenSim and the two other plugins
target_link_libraries(${PROJECT_NAME} PRIVATE 
    ${OpenSim_LIBRARIES}
    ${CMAKE_CURRENT_SOURCE_DIR}/../vestibular_plugin/build/Release/osimVestibular.lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../plugin_source_code/build/Release/osimMillard12EqWithAff.lib
)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "osimHeadNeckNeural")

add_executable(test_model test_model.cpp)
target_link_libraries(test_model PRIVATE ${PROJECT_NAME} ${OpenSim_LIBRARIES})
