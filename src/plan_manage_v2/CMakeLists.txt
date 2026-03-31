cmake_minimum_required(VERSION 2.8.3)
project(ego_planner_v2)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  plan_env_v2
  path_searching_v2
  traj_opt_v2
  traj_utils_v2
  message_generation
  cv_bridge
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ego_planner_v2
 CATKIN_DEPENDS plan_env_v2 path_searching_v2 traj_opt_v2 traj_utils_v2
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(ego_planner_node_v2
  src/ego_planner_node.cpp 
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
  )
target_link_libraries(ego_planner_node_v2
  ${catkin_LIBRARIES}
  )
#add_dependencies(ego_planner_node_v2 ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(traj_server_v2 src/traj_server.cpp)
target_link_libraries(traj_server_v2 ${catkin_LIBRARIES})
# add_dependencies(traj_server_v2 ${${PROJECT_NAME}_EXPORTED_TARGETS})


