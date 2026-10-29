cmake_minimum_required(VERSION 3.10)
project(osimHeadNeckNeuralAdaptive)

find_package(OpenSim REQUIRED)

set(SOURCE_FILES 
    HeadNeckNeuralControllerAdaptive.cpp
    RegisterTypes_osimHeadNeckNeuralAdaptive.cpp
)

set(HEADER_FILES 
    HeadNeckNeuralControllerAdaptive.h
    RegisterTypes_osimHeadNeckNeuralAdaptive.h
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE 
    ${OpenSim_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../vestibular_plugin
    ${CMAKE_CURRENT_SOURCE_DIR}/../proprioception_plugin
)

target_compile_definitions(${PROJECT_NAME} PRIVATE OSIMHEADNECKNEURALADAPTIVE_EXPORTS)

# Link against OpenSim and the two other plugins
target_link_libraries(${PROJECT_NAME} PRIVATE 
    ${OpenSim_LIBRARIES}
    ${CMAKE_CURRENT_SOURCE_DIR}/../vestibular_plugin/build/Release/osimVestibular.lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../proprioception_plugin/build/Release/osimMillard12EqWithAff.lib
)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "osimHeadNeckNeuralAdaptive")

