cmake_minimum_required(VERSION 3.16)
project(KalmanFilter)

set(CMAKE_CXX_STANDARD 11)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")
include_directories(. src/include)

set(sources src/main.cpp src/tools.cpp src/FusionEKF.cpp src/kalman_filter.cpp)

add_executable(${PROJECT_NAME} ${sources})

target_link_libraries(${PROJECT_NAME} z ssl uv uWS)