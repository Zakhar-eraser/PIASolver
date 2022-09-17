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

add_executable(PIA src/main.cpp src/PIASolver.cpp)
target_link_libraries(PIA ${catkin_LIBRARIES})