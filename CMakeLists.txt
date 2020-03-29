cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "-g -std=c++11")

add_executable(listener listener.cc)
target_link_libraries(listener 
  ${GAZEBO_LIBRARIES} 
  ${OpenCV_LIBRARIES} 
  ${PCL_LIBRARIES} 
  pthread )
