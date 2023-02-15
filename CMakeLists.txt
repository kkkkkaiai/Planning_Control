cmake_minimum_required(VERSION 3.8)
project(motion_planning)

include_directories(
    planning/trajectory_generator
    planning
)

# add CMakeLists files in subdirectories
add_subdirectory(planning/trajectory_generator/tinyspline)

# matplotlib-cpp
find_package(matplotlib_cpp REQUIRED)
add_library(matplotlib_cpp INTERFACE)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(matplotlib_cpp INTERFACE
  Python3::Python
  Python3::Module
)
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
  target_link_libraries(matplotlib_cpp INTERFACE
    Python3::NumPy
  )
else()
  target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
endif()


# Generate executable files
add_executable(constant_speed_test planning/trajectory_generator/constant_speed_test.cpp)
target_link_libraries(constant_speed_test PRIVATE tinysplinecxx matplotlib_cpp)
target_include_directories(constant_speed_test PRIVATE ${MATPLOTLIBCPP_INCLUDE_DIRS})

