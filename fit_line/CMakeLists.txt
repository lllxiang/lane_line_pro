# cmake needs this line
cmake_minimum_required(VERSION 2.8)
add_definitions(-std=c++11)
project(fit_line)
 
aux_source_directory(./src  CPP_SOURCES)
find_package(OpenCV REQUIRED)


include_directories(./include)


add_executable(fit_line ${CPP_SOURCES})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} m)
