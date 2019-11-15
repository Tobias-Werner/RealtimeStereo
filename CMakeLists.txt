cmake_minimum_required(VERSION 3.14)
project(SensorBox)

set(CMAKE_CXX_STANDARD 14)
find_package( OpenCV REQUIRED )
add_executable(SensorBox main.cpp)
target_link_libraries(SensorBox ${OpenCV_LIBS} )