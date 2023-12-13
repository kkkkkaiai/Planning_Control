cmake_minimum_required(VERSION 3.2)
project(traj_utils)


set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -Wno-dev")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED)

include_directories( 
    SYSTEM 
    include 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

add_library(traj_utils src/polynomial_traj.cpp)

target_link_libraries(traj_utils)  
