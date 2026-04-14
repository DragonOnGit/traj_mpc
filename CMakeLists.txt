cmake_minimum_required(VERSION 3.0.2)
project(traj_mpc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  mavros_msgs
  roslib
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES traj_mpc
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs mavros_msgs roslib
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(traj_mpc_node src/traj_mpc_node.cpp src/mpc_controller.cpp src/trajectory_loader.cpp)

target_link_libraries(traj_mpc_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

install(TARGETS traj_mpc_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config trajectories
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)