# cmake needs this line
cmake_minimum_required(VERSION 2.8)
project(find_edges)
add_definitions(-std=c++11)

find_package(OpenCV REQUIRED)

include_directories(include)
include_directories(include/c_fun.hpp)

aux_source_directory(./src  CPP_SOURCES)
add_executable(find_edges ${CPP_SOURCES})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} m)
