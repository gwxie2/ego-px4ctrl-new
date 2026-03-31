cmake_minimum_required(VERSION 2.8.3)
project(swarm_bridge_v2)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)

set(ENABLE_TCP false) # requires zmq, zmqpp 

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  traj_utils_v2
  traj_opt_v2 # for traj2odom_node
  sensor_msgs
)

catkin_package(
 CATKIN_DEPENDS traj_utils_v2 traj_opt_v2
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(bridge_node_udp_v2
  src/bridge_node_udp.cpp 
  )
target_link_libraries(bridge_node_udp_v2
  ${catkin_LIBRARIES}
  )

if(ENABLE_TCP)

  add_executable(bridge_node_tcp_v2
    src/bridge_node_tcp.cpp 
    )
  target_link_libraries(bridge_node_tcp_v2
    ${catkin_LIBRARIES}
    zmq
    zmqpp
    )

endif(ENABLE_TCP)

add_executable(traj2odom_node_v2
  src/traj2odom_node.cpp 
  )
target_link_libraries(traj2odom_node_v2
  ${catkin_LIBRARIES}
  )


