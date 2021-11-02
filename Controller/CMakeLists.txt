# Test CMake version
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)

# The name of the project
PROJECT(Controller)
MESSAGE(${PROJECT_NAME} ":")

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

if( NOT DEFINED CMAKE_BUILD_TYPE )
    set(CMAKE_BUILD_TYPE Release)
endif()

# Use c++11
SET(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Set the RobWork root (edit in .bashrc if necessary)
SET(RW_ROOT ~/Projects/RobWork/RobWork)
SET(RWS_ROOT ~/Projects/RobWork/RobWorkStudio)

# Verbose mode prints more info when doing make
set(CMAKE_VERBOSE_MAKEFILE false)

find_package(OpenCV REQUIRED)

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

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )



# Add the source files to build
SET(SrcFiles src/controller.cpp src/RwsInterface.h src/RwsInterface.cpp)
# src/SamplePlugin.cpp src/resources.qrc)


# The shared library to build:
#ADD_LIBRARY(Controller MODULE ${SrcFiles})
ADD_EXECUTABLE(Controller ${SrcFiles})
TARGET_LINK_LIBRARIES(Controller PUBLIC ${ROBWORK_LIBRARIES} ${OpenCV_LIBS} ${ROBWORKSTUDIO_LIBRARIES})

MESSAGE(${PROJECT_NAME} " done!")
