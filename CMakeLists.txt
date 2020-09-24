cmake_minimum_required(VERSION 3.17)
project(pcdtools)
find_package(PCL REQUIRED)
set(CMAKE_CXX_STANDARD 11)

include_directories(${PCL_INClUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcdtools main.cpp)
target_link_libraries(pcdtools ${PCL_LIBRARIES})
