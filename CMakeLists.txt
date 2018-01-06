cmake_minimum_required(VERSION 3.9)
project(TunnelProcessorCPP)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

find_package (libpointmatcher 1.1.0 REQUIRED)
include_directories("${libpointmatcher_INCLUDE_DIRS}")

#add_executable(tunnelproc src/Line.cpp src/LineFitter.cpp src/main.cpp src/OutlierRemover.cpp src/PointCloud.cpp src/Ring.cpp src/ScanMatcher.cpp)
#target_link_libraries (tunnelproc Eigen3::Eigen ${libpointmatcher_LIBRARIES})

set(CMAKE_BUILD_TYPE RelWithDebInfo)
add_executable(tunnelprocdebug src/Line.cpp src/LineFitter.cpp src/main.cpp src/OutlierRemover.cpp src/PointCloud.cpp src/Ring.cpp src/ScanMatcher.cpp)
target_link_libraries (tunnelprocdebug Eigen3::Eigen ${libpointmatcher_LIBRARIES})
