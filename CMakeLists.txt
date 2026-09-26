cmake_minimum_required(VERSION 3.11)
project(a_star LANGUAGES CXX)

find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(
    a_star
    src/main.cpp
    src/a_star.cpp
    src/astar_visualizer.cpp
    src/map.cpp
)

target_include_directories(a_star PRIVATE include)
target_link_libraries(a_star PRIVATE ${OpenCV_LIBS})
