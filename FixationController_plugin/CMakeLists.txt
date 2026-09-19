cmake_minimum_required(VERSION 3.14)
project(osimFixation LANGUAGES CXX)

find_package(OpenSim REQUIRED)

set(SOURCE_FILES 
    FixationController.h 
    FixationController.cpp 
    RegisterTypes_osimFixation.h 
    RegisterTypes_osimFixation.cpp 
    osimFixationDLL.h
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE 
    ${OpenSim_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../vestibular_plugin
    ${CMAKE_CURRENT_SOURCE_DIR}/../proprioception_plugin
)
target_link_libraries(${PROJECT_NAME} PRIVATE 
    ${OpenSim_LIBRARIES}
    ${CMAKE_CURRENT_SOURCE_DIR}/../vestibular_plugin/build/Release/osimVestibular.lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../proprioception_plugin/build/Release/osimMillard12EqWithAff.lib
)
target_compile_definitions(${PROJECT_NAME} PRIVATE OSIMFIXATION_EXPORTS)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "osimFixation")

add_executable(attach_fixation attach_fixation.cpp)
target_link_libraries(attach_fixation ${PROJECT_NAME} ${OpenSim_LIBRARIES})

add_executable(testFixation testFixation.cpp)
target_link_libraries(testFixation ${PROJECT_NAME} ${OpenSim_LIBRARIES})
