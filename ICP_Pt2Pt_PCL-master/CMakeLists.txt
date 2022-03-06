cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(icp_P2PT)

find_package(PCL 1.2 REQUIRED)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

remove_definitions(-DDISABLE_LIBUSB-1.0)

add_executable(icp_P2PT icp_P2PT.cpp)
target_link_libraries(icp_P2PT ${PCL_LIBRARIES})
