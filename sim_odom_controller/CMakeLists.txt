cmake_minimum_required(VERSION 2.8.3)
project(sim_odom_controller)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  quadrotor_msgs
)

find_package(Eigen3)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sim_odom_controller
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

#add_dependencies(trajectory_generator ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(test_traj src/test_traj.cpp)
add_library(trajectory_generator_test src/trajectory_generator_waypoint.cpp)
target_link_libraries(test_traj trajectory_generator_test ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIR})
