cmake_minimum_required(VERSION 2.8.3)
project(tag_detector)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs cv_bridge)

find_package(OpenCV REQUIRED)

find_package(aruco REQUIRED)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(tag_detector src/tag_detector_node.cpp)

target_link_libraries(tag_detector ${catkin_LIBRARIES} ${OpenCV_LIBS} ${aruco_LIBS})