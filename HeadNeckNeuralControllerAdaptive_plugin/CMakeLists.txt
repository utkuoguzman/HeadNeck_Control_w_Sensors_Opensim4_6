cmake_minimum_required(VERSION 3.10)
project(osimHeadNeckNeuralAdaptive)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /O2 /Ob3 /arch:AVX2 /GL)
    target_link_options(${PROJECT_NAME} PRIVATE /LTCG)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -O3 -march=native -fPIC)
endif()

target_include_directories(${PROJECT_NAME} PRIVATE 
    ${OpenSim_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../vestibular_plugin
    ${CMAKE_CURRENT_SOURCE_DIR}/../proprioception_plugin
)

target_compile_definitions(${PROJECT_NAME} PRIVATE OSIMHEADNECKNEURALADAPTIVE_EXPORTS)

if(WIN32)
    set(VEST_LIB "${CMAKE_CURRENT_SOURCE_DIR}/../vestibular_plugin/build/Release/osimVestibular.lib")
    set(PROP_LIB "${CMAKE_CURRENT_SOURCE_DIR}/../proprioception_plugin/build/Release/osimMillard12EqWithAff.lib")
else()
    set(VEST_LIB "${CMAKE_CURRENT_SOURCE_DIR}/../vestibular_plugin/build/libosimVestibular.so")
    set(PROP_LIB "${CMAKE_CURRENT_SOURCE_DIR}/../proprioception_plugin/build/libosimMillard12EqWithAff.so")
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE 
    ${OpenSim_LIBRARIES}
    ${VEST_LIB}
    ${PROP_LIB}
)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "osimHeadNeckNeuralAdaptive")
