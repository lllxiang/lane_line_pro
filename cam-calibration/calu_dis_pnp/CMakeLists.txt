cmake_minimum_required(VERSION 2.8)
add_definitions(-std=c++11)
project(calu_dis_pnp)
include_directories(./include)

aux_source_directory(./src  CPP_SOURCES)
find_package(OpenCV REQUIRED)
add_executable(calu_dis_pnp ${CPP_SOURCES})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} m)
