cmake_minimum_required(VERSION 2.6)
project(PoseSLAM CXX C)

find_package(GTSAMCMakeTools)
include(GtsamBuildTypes)
include(GtsamTesting) 
include(GtsamMatlabWrap)

include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
add_executable(${PROJECT_NAME} src/PoseSLAM.cpp)
target_link_libraries(${PROJECT_NAME} gtsam)
