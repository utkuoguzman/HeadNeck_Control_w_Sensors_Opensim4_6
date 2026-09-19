cmake_minimum_required(VERSION 3.14)
project(osimVestibular LANGUAGES CXX)
find_package(OpenSim REQUIRED)
set(SOURCE_FILES Schneider15VestibularAfferent.h Schneider15VestibularAfferent.cpp RegisterTypes_osimVestibular.h RegisterTypes_osimVestibular.cpp osimVestibularDLL.h)
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PRIVATE ${OpenSim_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenSim_LIBRARIES})
target_compile_definitions(${PROJECT_NAME} PRIVATE OSIMVESTIBULAR_EXPORTS)
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "osimVestibular")

add_executable(attach_sensor attach_sensor.cpp)
target_link_libraries(attach_sensor ${PROJECT_NAME} ${OpenSim_LIBRARIES})

# Test Program
add_executable(testVestibular testVestibular.cpp)
target_link_libraries(testVestibular ${PROJECT_NAME} ${OpenSim_LIBRARIES})
