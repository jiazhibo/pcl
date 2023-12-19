cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(read)

find_package(
	OpenCV REQUIRED
)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(read read.cpp)
target_link_libraries(read ${OpenCV_LIBS})
