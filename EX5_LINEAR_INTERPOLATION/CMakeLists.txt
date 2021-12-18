# Test CMake version
CMAKE_MINIMUM_REQUIRED(VERSION 3.1.0)

# The name of the project
PROJECT(EX5_LINEAR_INTERPOLATION)
MESSAGE(${PROJECT_NAME} ":")

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

if( NOT DEFINED CMAKE_BUILD_TYPE )
    set(CMAKE_BUILD_TYPE Release)
endif()

# Use c++11
SET(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

# Set the RobWork root (edit in .bashrc if necessary)
SET(RW_ROOT $ENV{RW_ROOT})
SET(RWS_ROOT $ENV{RWS_ROOT})
SET(RWS_INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/../RwsInterface")
message(${RWS_INTERFACE})
# Verbose mode prints more info when doing make
set(CMAKE_VERBOSE_MAKEFILE false)

find_package(OpenCV 4.2.0 REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})
add_definitions(${OPENCV_DEFINITIONS})


# Set build type to release
SET(CMAKE_BUILD_TYPE Release)
MESSAGE("-- Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
# set(CMAKE_AUTOUIC ON)

# Use RobWork
SET(RobWork_DIR ${RW_ROOT}/cmake) 
FIND_PACKAGE(RobWork REQUIRED)
include_directories( ${ROBWORK_INCLUDE_DIRS} )
message(STATUS "RW: ${ROBWORK_INCLUDE_DIRS}")
link_directories( ${ROBWORK_LIBRARY_DIRS} )
add_definitions(${ROBWORK_DEFINITIONS})

SET(RobWorkStudio_DIR ${RWS_ROOT}/cmake) 
FIND_PACKAGE(RobWorkStudio REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIRS} )
message(STATUS "RWS: ${ROBWORKSTUDIO_INCLUDE_DIRS}")
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS} )

find_package(PCL REQUIRED COMPONENTS common io visualization filters features)
include_directories(${PCL_SEARCH_INCLUDE_DIRS})
link_directories(${PCL_SEARCH_LIBRARY_DIRS})
add_definitions(${PCL_SEARCH_DEFINITIONS})

INCLUDE_DIRECTORIES(${RWS_INTERFACE})
LINK_DIRECTORIES(${RWS_INTERFACE})

# Add the source files to build
SET(SrcFiles 
src/example_linear_interpolation.cpp 
${RWS_INTERFACE}/RwsInterface.h 
${RWS_INTERFACE}/RwsInterface.cpp
)
# src/SamplePlugin.cpp src/resources.qrc)

# The shared library to build:
#ADD_LIBRARY(EX1_PICK_AND_PASTE MODULE ${SrcFiles})
ADD_EXECUTABLE(EX5_LINEAR_INTERPOLATION ${SrcFiles})
TARGET_LINK_LIBRARIES(EX5_LINEAR_INTERPOLATION ${ROBWORK_LIBRARIES} ${OpenCV_LIBS} ${ROBWORKSTUDIO_LIBRARIES} ${PCL_LIBRARIES} ${RWS_INTERFACE})

MESSAGE(${PROJECT_NAME} " done!")
