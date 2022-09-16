cmake_minimum_required (VERSION 2.8)

project(PIALaserScan)
set(CMAKE_CXX_STANDARD 17)

include_directories(
    include
    /opt/ros/noetic/include
)

add_executable(PIA src/main.cpp src/PIASolver.cpp)