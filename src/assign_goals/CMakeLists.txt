cmake_minimum_required(VERSION 2.8.3)
project(assign_goals)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++11)
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  roscpp
  std_msgs
  uav_utils
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs quadrotor_msgs roscpp std_msgs uav_utils visualization_msgs
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(assign_goals_node src/assign_goals_node.cpp)
target_link_libraries(assign_goals_node ${catkin_LIBRARIES})