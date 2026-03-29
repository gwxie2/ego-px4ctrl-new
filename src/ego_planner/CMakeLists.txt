cmake_minimum_required(VERSION 2.8.3)
project(ego_planner)

option(EGO_ENABLE_MANUAL_TARGET_DIRECT_START "Manual target bypasses SEQUENTIAL_START in WAIT_TARGET" OFF)
option(EGO_ENABLE_GOAL_REACHABILITY_PROJECTION "Project manual goal to nearby free voxel if blocked" OFF)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

if(EGO_ENABLE_MANUAL_TARGET_DIRECT_START)
  add_definitions(-DEGO_ENABLE_MANUAL_TARGET_DIRECT_START=1)
else()
  add_definitions(-DEGO_ENABLE_MANUAL_TARGET_DIRECT_START=0)
endif()

if(EGO_ENABLE_GOAL_REACHABILITY_PROJECTION)
  add_definitions(-DEGO_ENABLE_GOAL_REACHABILITY_PROJECTION=1)
else()
  add_definitions(-DEGO_ENABLE_GOAL_REACHABILITY_PROJECTION=0)
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  plan_env
  path_searching
  bspline_opt
  traj_utils
  message_generation
  cv_bridge
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ego_planner
 CATKIN_DEPENDS plan_env path_searching bspline_opt traj_utils 
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(ego_planner_node
  src/ego_planner_node.cpp 
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
  )
target_link_libraries(ego_planner_node 
  ${catkin_LIBRARIES}
  )
#add_dependencies(ego_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})
#add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})


