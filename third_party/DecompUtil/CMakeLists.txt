cmake_minimum_required(VERSION 3.1)
project(decomp_util)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-O3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")


# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(BUILD_SHARED_LIBS ON)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} include python)

# PYBIND
if(CMAKE_HOST_APPLE)
  find_package(pybind11 REQUIRED)
elseif(CMAKE_HOST_UNIX)
  include(pybind11.cmake)
endif()

file(GLOB INCLUDE_FILES "include/*/*.h")
file(GLOB 2D_PYTHON_FILES "python/*_2D_*.cpp" )
file(GLOB 3D_PYTHON_FILES "python/*_3D_*.cpp" )
file(GLOB GEOMETRIC_UTILS_FILES "python/geometric_utils_python.cpp" )

pybind11_add_module(${PROJECT_NAME}_2d ${INCLUDE_FILES} ${2D_PYTHON_FILES})
pybind11_add_module(${PROJECT_NAME}_3d ${INCLUDE_FILES} ${3D_PYTHON_FILES})
pybind11_add_module(geometric_utils ${INCLUDE_FILES} ${GEOMETRIC_UTILS_FILES})

install(TARGETS ${PROJECT_NAME}_2d ${PROJECT_NAME}_3d geometric_utils
  COMPONENT python
  LIBRARY DESTINATION "${PYTHON_LIBRARY_DIR}"
)

# TEST
include(CTest)

add_executable(test_seed_decomp test/test_seed_decomp.cpp)
target_link_libraries(test_seed_decomp ${Boost_LIBRARIES})
add_test(test_seed_decomp test_seed_decomp)

add_executable(test_line_segment test/test_line_segment.cpp)
target_link_libraries(test_line_segment ${Boost_LIBRARIES})
add_test(test_line_segment test_line_segment ${CMAKE_SOURCE_DIR}/data/obstacles.txt)

add_executable(test_ellipsoid_decomp test/test_ellipsoid_decomp.cpp)
target_link_libraries(test_ellipsoid_decomp ${Boost_LIBRARIES})
add_test(test_ellipsoid_decomp test_ellipsoid_decomp ${CMAKE_SOURCE_DIR}/data/obstacles.txt)

add_executable(test_iterative_decomp test/test_iterative_decomp.cpp)
target_link_libraries(test_iterative_decomp ${Boost_LIBRARIES})
add_test(test_iterative_decomp test_iterative_decomp ${CMAKE_SOURCE_DIR}/data/obstacles.txt)

# install(FILES "${PROJECT_NAME}Config.cmake" "${PROJECT_NAME}ConfigVersion.cmake"
#         DESTINATION "share/${PROJECT_NAME}/cmake")

# install(TARGETS
#         RUNTIME DESTINATION bin
#         LIBRARY DESTINATION lib
#         ARCHIVE DESTINATION lib)

# install(DIRECTORY include/decomp_basis include/decomp_geometry include/decomp_util
#         DESTINATION include)





