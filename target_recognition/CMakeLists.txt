# cmake needs this line
cmake_minimum_required(VERSION 2.8)
project(target_recognition)
add_definitions(-std=c++11)

find_package(OpenCV REQUIRED)

include_directories(include)


aux_source_directory(./src  CPP_SOURCES)
add_executable(target_recognition ${CPP_SOURCES} src/main.cpp include/ransac_line2d.h src/ransac_line2d.cpp include/detect_parking_space.h src/detect_parking_space.cpp include/tools.h src/tools.cpp include/detect_line_space.h src/detect_line_space.cpp include/kalman_tracker.h src/kalman_tracker.cpp)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} m)
