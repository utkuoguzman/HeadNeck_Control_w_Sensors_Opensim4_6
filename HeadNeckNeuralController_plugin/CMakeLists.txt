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
    ${CMAKE_CURRENT_SOURCE_DIR}/../proprioception_plugin
)

target_compile_definitions(${PROJECT_NAME} PRIVATE OSIMHEADNECKNEURAL_EXPORTS)

# Link against OpenSim and the two other plugins
if(WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE 
        ${OpenSim_LIBRARIES}
        ${CMAKE_CURRENT_SOURCE_DIR}/../vestibular_plugin/build/Release/osimVestibular.lib
        ${CMAKE_CURRENT_SOURCE_DIR}/../proprioception_plugin/build/Release/osimMillard12EqWithAff.lib
    )
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE 
        ${OpenSim_LIBRARIES}
        ${CMAKE_CURRENT_SOURCE_DIR}/../vestibular_plugin/build/libosimVestibular.so
        ${CMAKE_CURRENT_SOURCE_DIR}/../proprioception_plugin/build/libosimMillard12EqWithAff.so
    )
    set_target_properties(${PROJECT_NAME} PROPERTIES
        BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../vestibular_plugin/build;${CMAKE_CURRENT_SOURCE_DIR}/../proprioception_plugin/build"
        INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../vestibular_plugin/build;${CMAKE_CURRENT_SOURCE_DIR}/../proprioception_plugin/build"
    )
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "osimHeadNeckNeural")

if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /O2 /Ob3 /arch:AVX2 /GL)
    target_link_options(${PROJECT_NAME} PRIVATE /LTCG)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -O3 -march=native -flto)
    target_link_options(${PROJECT_NAME} PRIVATE -flto)
endif()

add_executable(test_model test_model.cpp)
target_link_libraries(test_model PRIVATE ${PROJECT_NAME} ${OpenSim_LIBRARIES})
