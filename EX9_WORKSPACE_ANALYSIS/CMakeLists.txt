# Test CMake version
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)

# The name of the project
PROJECT(EX9_WORKSPACE_ANALYSIS)
MESSAGE(${PROJECT_NAME} ":")

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

if( NOT DEFINED CMAKE_BUILD_TYPE )
    set(CMAKE_BUILD_TYPE Release)
endif()
# Use c++11
SET(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Set the RobWork root (edit in .bashrc if necessary)
SET(RW_ROOT $ENV{RW_ROOT})
SET(RWS_ROOT $ENV{RWS_ROOT})
SET(RWS_INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/../RwsInterface")
message(${RWS_INTERFACE})
# Verbose mode prints more info when doing make
set(CMAKE_VERBOSE_MAKEFILE false)

find_package(OpenCV 4.2 REQUIRED)
find_package( PCL REQUIRED )

# Set build type to release
SET(CMAKE_BUILD_TYPE Release)
MESSAGE("-- Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
# set(CMAKE_AUTOUIC ON)

# Use RobWork
SET(RobWork_DIR ${RW_ROOT}/cmake) 
FIND_PACKAGE(RobWork REQUIRED)
message("-- RobWork found: " ${RW_ROOT})

SET(RobWorkStudio_DIR ${RWS_ROOT}/cmake) 
FIND_PACKAGE(RobWorkStudio REQUIRED)
message("-- RobWorkStudio found: " ${RWS_ROOT})

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${RWS_INTERFACE} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS} ${RWS_INTERFACE})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

# Add the source files to build
SET(SrcFiles 
src/example_workspace_analysis.cpp 
${RWS_INTERFACE}/RwsInterface.h 
${RWS_INTERFACE}/RwsInterface.cpp
)
# src/SamplePlugin.cpp src/resources.qrc)

# The shared library to build:
#ADD_LIBRARY(EX2_REACHABILITY_ANALYSIS MODULE ${SrcFiles})
ADD_EXECUTABLE(EX9_WORKSPACE_ANALYSIS ${SrcFiles})
TARGET_LINK_LIBRARIES(EX9_WORKSPACE_ANALYSIS PUBLIC ${ROBWORK_LIBRARIES} ${OpenCV_LIBS} ${ROBWORKSTUDIO_LIBRARIES} ${PCL_LIBRARIES} ${RWS_INTERFACE})

MESSAGE(${PROJECT_NAME} " done!")
