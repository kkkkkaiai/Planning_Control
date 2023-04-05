cmake_minimum_required(VERSION 2.8.3)
project(bspline)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories( 
    SYSTEM 
    traj_uitls
    include 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_library( bspline 
    src/uniform_bspline.cpp 
    # src/bspline_optimizer.cpp 
    src/gradient_descent_optimizer.cpp
    )
target_link_libraries( bspline
    traj_utils
    )  
