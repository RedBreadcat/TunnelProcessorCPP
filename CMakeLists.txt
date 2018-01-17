cmake_minimum_required(VERSION 3.5)
project(TunnelProcessorCPP)
set (CMAKE_CXX_STANDARD 11)

find_package (Eigen3 REQUIRED NO_MODULE)

find_package (libpointmatcher 1.1.0 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} "${libpointmatcher_INCLUDE_DIRS}")

set(CMAKE_BUILD_TYPE RelWithDebInfo)
add_executable(tunnelproc src/Line.cpp src/LineFitter.cpp src/main.cpp src/OutlierRemover.cpp src/PointCloud.cpp src/Ring.cpp src/ScanMatcher.cpp)
target_link_libraries (tunnelproc Eigen3::Eigen ${libpointmatcher_LIBRARIES})

#set(CMAKE_BUILD_TYPE Debug)
#add_executable(tunnelprocdebug src/Line.cpp src/LineFitter.cpp src/main.cpp src/OutlierRemover.cpp src/PointCloud.cpp src/Ring.cpp src/ScanMatcher.cpp)
#target_link_libraries (tunnelprocdebug Eigen3::Eigen ${libpointmatcher_LIBRARIES})

