cmake_minimum_required(VERSION 2.8.3)
project(moving_obstacles)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++11)
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  traj_opt_v2
  traj_utils_v2
)

catkin_package(
  CATKIN_DEPENDS nav_msgs roscpp sensor_msgs std_msgs traj_opt_v2 traj_utils_v2
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(moving_obstacles_node src/moving_obstacles.cpp)
target_link_libraries(moving_obstacles_node ${catkin_LIBRARIES})