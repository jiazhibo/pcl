cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pottilt)
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

find_package(PCL 1.2 REQUIRED)
find_package(
	OpenCV REQUIRED
)
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(pottilt pottilt.cpp)
target_link_libraries(pottilt ${PCL_LIBRARIES} ${OpenCV_LIBS})
