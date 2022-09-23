cmake_minimum_required (VERSION 2.8)

project(PIALaserScan)
set(CMAKE_CXX_STANDARD 17)

file(GLOB catkin_LIBRARIES
     "/opt/ros/noetic/lib/*.so"
)

include_directories(
    include
    /opt/ros/noetic/include
)

add_library(pia_solver STATIC src/PIASolver.cpp)

add_executable(test src/test.cpp)
target_link_libraries(test ${catkin_LIBRARIES} pia_solver)