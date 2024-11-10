cmake_minimum_required(VERSION 3.10)
project(RRTStar)

set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(rrt_star rrtstar.cpp)
target_link_libraries(rrt_star ${OpenCV_LIBS})
