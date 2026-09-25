###########################################
## For building with the rest of OpenSim ##
###########################################

cmake_minimum_required(VERSION 3.14)

# Define project
project(osimMillard12EqWithAff LANGUAGES CXX)

find_package(OpenSim REQUIRED)

file(GLOB SOURCE_FILES *.h *.cpp)

# --- 1. Remove mainAfferents from the DLL plugin ---
list(REMOVE_ITEM SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/mainAfferents.cpp")

set(SOURCE_FILES 
    Millard12EqMuscleWithAfferents.cpp
    Millard12EqMuscleWithIntermediateAfferent.cpp
    Millard12EqMuscleWithSimplifiedAfferent.cpp
    Mileusnic06Spindle.cpp
    Poppele70Spindle.cpp
    Lin02GolgiTendonOrgan.cpp
    SpindleController.cpp
    TugOfWarController.cpp
    AfferentAnalysis.cpp
    RegisterTypes_osimPlugin.cpp
)

set(HEADER_FILES 
    Millard12EqMuscleWithAfferents.h
    Millard12EqMuscleWithIntermediateAfferent.h
    Millard12EqMuscleWithSimplifiedAfferent.h
    Mileusnic06Spindle.h
    Poppele70Spindle.h
    Lin02GolgiTendonOrgan.h
    SpindleController.h
    TugOfWarController.h
    AfferentAnalysis.h
    RegisterTypes_osimPlugin.h
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES})

target_compile_definitions(${PROJECT_NAME} PRIVATE OSIMMILLARD12EQWITHAFF_EXPORTS)

target_include_directories(${PROJECT_NAME} PRIVATE ${OpenSim_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenSim_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "osimMillard12EqWithAff")

# --- 2. Build mainAfferents as an executable (compiling the sources directly to avoid DLL export issues on Windows) ---
add_executable(mainAfferents mainAfferents.cpp ${SOURCE_FILES} ${HEADER_FILES})
target_include_directories(mainAfferents PRIVATE ${OpenSim_INCLUDE_DIRS})
target_link_libraries(mainAfferents PRIVATE ${OpenSim_LIBRARIES})
