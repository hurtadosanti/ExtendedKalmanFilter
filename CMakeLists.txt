cmake_minimum_required(VERSION 3.16)
project(KallmanFilter)

set(CMAKE_CXX_STANDARD 17)

add_executable(KallmanFilter main.cpp tracking.cpp kalman_filter.cpp)