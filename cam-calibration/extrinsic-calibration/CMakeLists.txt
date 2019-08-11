cmake_minimum_required(VERSION 2.8)
add_definitions(-std=c++11)
project(extrinsic-calibration)
include_directories(./include)

aux_source_directory(./src  CPP_SOURCES)
find_package(OpenCV REQUIRED)
add_executable(extrinsic-calibration ${CPP_SOURCES})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} m)
