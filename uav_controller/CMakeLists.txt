cmake_minimum_required(VERSION 2.8.3)
project(uav_controller)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -DEIGEN_DONT_PARALLELIZE")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  quadrotor_msgs
  message_filters
)

find_package(Eigen3)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES 
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(line src/line.cpp)
add_library(trajectory_generator src/trajectory_generator_waypoint.cpp)
target_link_libraries(line
  trajectory_generator
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${EIGEN3_INCLUDE_DIR}
  ooqpgensparse
  ooqpsparse
  ooqpgondzio
  ooqpbase
  blas
  ma27
  gfortran
)

add_executable(test_uav src/test_uav.cpp)
target_link_libraries(test_uav
  trajectory_generator
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${EIGEN3_INCLUDE_DIR}
  ooqpgensparse
  ooqpsparse
  ooqpgondzio
  ooqpbase
  blas
  ma27
  gfortran
)

add_executable(jiugang src/jiugang.cpp)
target_link_libraries(jiugang
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(proposed src/proposed.cpp)
target_link_libraries(proposed
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${EIGEN3_INCLUDE_DIR}
)
